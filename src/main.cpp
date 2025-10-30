#include <Arduino.h>
#include <AS5600.h>
#include <Wire.h>
#include "esp_sleep.h"
#include "./BleMouse.h"

#define LOGGING_ENABLED false

// Sleep and Power Management Settings
constexpr unsigned long DEEP_SLEEP_TIMEOUT_MS = 120 * 1000;
constexpr unsigned long DEEP_SLEEP_WAKE_INTERVAL_MS = 3 * 1000;
constexpr float ROTATION_CHANGE_THRESHOLD_DEGREES = 10.0f;

// Device Configuration
constexpr float SCROLL_RESOLUTION_MULTIPLIER = 128.0f;
constexpr float DEGREES_PER_SCROLL_STEP = -360.0f / 4096.0f * 128.0f;
constexpr int SERIAL_SPEED = 115200;
constexpr int ENCODER_POWER_PIN = 20;
constexpr int BATTERY_PIN = 4;
constexpr int BATTERY_AVERAGE_SAMPLES = 128;
constexpr float VOLTAGE_DIVIDER_RESISTOR_VALUE_PLUS = 214000.0f;  // Measure this!
constexpr float VOLTAGE_DIVIDER_RESISTOR_VALUE_MINUS = 218000.0f; // Measure this!
constexpr float VOLTAGE_DIVIDER_CORRECTION = (VOLTAGE_DIVIDER_RESISTOR_VALUE_MINUS + VOLTAGE_DIVIDER_RESISTOR_VALUE_PLUS) / VOLTAGE_DIVIDER_RESISTOR_VALUE_MINUS;
constexpr float VOLTAGE_CHANGE_THRESHOLD_MV = 5.0f;

// Operation Intervals
constexpr unsigned long ENCODER_READ_INTERVAL_MS = 15;  // Min 7.5ms for BLE
constexpr unsigned long BLE_CONNECTION_CHECK_INTERVAL_MS = 5000;
constexpr unsigned long BATTERY_CHECK_INTERVAL_MS = 10 * 1000;

// Battery voltage lookup tables (calibrate for your specific battery!)
constexpr int BATTERY_TABLE_SIZE = 11;
constexpr float BATTERY_CHARGING_VOLTAGES[BATTERY_TABLE_SIZE] = { 3545.0f, 3622.0f, 3715.0f, 3815.0f, 3876.0f, 3913.0f, 3945.0f, 3987.0f, 4035.0f, 4064.0f, 4100.0f };
constexpr float BATTERY_DISCHARGING_VOLTAGES[BATTERY_TABLE_SIZE] = { 3102.0f, 3442.0f, 3547.0f, 3673.0f, 3736.0f, 3776.0f, 3812.0f, 3880.0f, 3925.0f, 3952.0f, 4057.0f };
constexpr float BATTERY_PERCENTAGES[BATTERY_TABLE_SIZE] = { 0.0f, 10.0f, 20.0f, 30.0f, 40.0f, 50.0f, 60.0f, 70.0f, 80.0f, 90.0f, 100.0f };

// RTC Memory Storage (persists during deep sleep)
RTC_DATA_ATTR float rtcLastRotationAngle = 1000.0f;
RTC_DATA_ATTR float rtcBatteryVoltage = 100.0f;

// Global Objects
BleMouse bleMouse("Smooth scroller", "ESP32 - EbrithilNogare", rtcBatteryVoltage);
AS5600 as5600(&Wire);

// State Variables
unsigned long lastEncoderReadTimeMs = 0;
unsigned long lastBleConnectionCheckTimeMs = 0;
unsigned long lastBatteryCheckTimeMs = 0;
unsigned long lastActivityTimeMs = 0;
float previousEncoderAngle = 0.0f;
float currentEncoderAngle = 0.0f;
float lastStableVoltage = 0.0f;
bool isEncoderInitialized = false;
bool isAdvertising = false;
bool isCharging = false;

float calculateBatteryPercentageFromReading(uint32_t voltageReading) {
    float batteryVoltage = voltageReading * VOLTAGE_DIVIDER_CORRECTION;

    const float* voltageTable = isCharging ? BATTERY_CHARGING_VOLTAGES : BATTERY_DISCHARGING_VOLTAGES;

    // Clamp to min/max
    if (batteryVoltage <= voltageTable[0]) {
        return BATTERY_PERCENTAGES[0];
    }
    if (batteryVoltage >= voltageTable[BATTERY_TABLE_SIZE - 1]) {
        return BATTERY_PERCENTAGES[BATTERY_TABLE_SIZE - 1];
    }
    
    // Find interpolation range
    int lowerIndex = 0;
    for (int i = 0; i < BATTERY_TABLE_SIZE - 1; i++) {
        if (batteryVoltage >= voltageTable[i] && batteryVoltage <= voltageTable[i + 1]) {
            lowerIndex = i;
            break;
        }
    }
    
    const int upperIndex = lowerIndex + 1;
    
    // Linear interpolation
    const float voltage1 = voltageTable[lowerIndex];
    const float voltage2 = voltageTable[upperIndex];
    const float percentage1 = BATTERY_PERCENTAGES[lowerIndex];
    const float percentage2 = BATTERY_PERCENTAGES[upperIndex];

    return percentage1 + (batteryVoltage - voltage1) * (percentage2 - percentage1) / (voltage2 - voltage1);
}


#if LOGGING_ENABLED
void initializeSerialCommunication() {
    Serial.begin(SERIAL_SPEED);
    delay(1000);
    Serial.println("✓ Serial communication started");
}
#endif

// ========================================
// Bluetooth Management Functions
// ========================================

void initializeBluetooth() {
    bleMouse.begin();
    isAdvertising = true;
    lastBleConnectionCheckTimeMs = millis();
}

void terminateBluetooth() {
    bleMouse.end();
    isAdvertising = false;
    delay(100);
}

void handleBleConnectionManagement(unsigned long currentTimeMs) {
    if (currentTimeMs - lastBleConnectionCheckTimeMs < BLE_CONNECTION_CHECK_INTERVAL_MS) {
        return;
    }
    
    lastBleConnectionCheckTimeMs = currentTimeMs;
    const bool isCurrentlyConnected = bleMouse.isConnected();
    
    if (!isCurrentlyConnected) {
        bleMouse.end();
        delay(100);
        bleMouse.begin();
        isAdvertising = true;
    }

    if (isCurrentlyConnected && isAdvertising) {
        isAdvertising = false;
    }
}

// ========================================
// Encoder Management Functions
// ========================================

void initializeEncoder() {
    pinMode(ENCODER_POWER_PIN, OUTPUT);
    digitalWrite(ENCODER_POWER_PIN, HIGH);
    Wire.begin();
    as5600.begin(AS5600_SW_DIRECTION_PIN);
    
    // Wait for first valid register value
    const unsigned long startTime = millis();
    while (as5600.isConnected() && as5600.readAngle() == 0 && millis() - startTime < 10) {
        // Waiting for encoder to initialize
    }
    
    if (as5600.isConnected()) {
        isEncoderInitialized = true;
        currentEncoderAngle = as5600.readAngle() * AS5600_RAW_TO_DEGREES;
        
        if (rtcLastRotationAngle > 360.0f) {
            // First run after power-on
            previousEncoderAngle = currentEncoderAngle;
            rtcLastRotationAngle = currentEncoderAngle;
        } else {
            // Recovering from deep sleep
            previousEncoderAngle = rtcLastRotationAngle;
        }
    }
}

void terminateEncoder() {
    if (isEncoderInitialized) {
        isEncoderInitialized = false;
        Wire.end();
        digitalWrite(ENCODER_POWER_PIN, LOW);
    }
}

// ========================================
// Scroll Processing Functions
// ========================================

void processEncoderScrolling(unsigned long currentTimeMs) {
    if (currentTimeMs - lastEncoderReadTimeMs < ENCODER_READ_INTERVAL_MS || !isEncoderInitialized) {
        return;
    }
    
    lastEncoderReadTimeMs = currentTimeMs;
    currentEncoderAngle = as5600.readAngle() * AS5600_RAW_TO_DEGREES;
    float angleDifference = currentEncoderAngle - previousEncoderAngle;

    // Normalize angle difference to [-180, 180] range
    if (angleDifference > 180.0f) {
        angleDifference -= 360.0f;
    } else if (angleDifference < -180.0f) {
        angleDifference += 360.0f;
    }
    
    // Filter out invalid readings
    if (abs(angleDifference) >= 180.0f) {
        previousEncoderAngle = currentEncoderAngle;
        return;
    }

    const float rawScrollSteps = angleDifference / DEGREES_PER_SCROLL_STEP * SCROLL_RESOLUTION_MULTIPLIER;
    const int16_t scrollSteps = static_cast<int16_t>(constrain(rawScrollSteps, -32767.0f, 32767.0f));
    
    if (abs(scrollSteps) > 1) {
        previousEncoderAngle += (scrollSteps / SCROLL_RESOLUTION_MULTIPLIER) * DEGREES_PER_SCROLL_STEP;
        lastActivityTimeMs = currentTimeMs;

        if (bleMouse.isConnected()) {
            bleMouse.scroll(scrollSteps);
        }
    }
}

// ========================================
// Power Management Functions
// ========================================

void initBatteryReader() {
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);
    pinMode(BATTERY_PIN, INPUT);
}

void handleInactivityBasedSleep(unsigned long currentTimeMs) {
    if (currentTimeMs - lastActivityTimeMs < DEEP_SLEEP_TIMEOUT_MS) {
        return;
    }

    // Prepare for deep sleep
    terminateBluetooth();
    terminateEncoder();
    rtcLastRotationAngle = previousEncoderAngle;
    
    esp_sleep_enable_timer_wakeup(DEEP_SLEEP_WAKE_INTERVAL_MS * 1000);
    esp_deep_sleep_start();
}

void checkRotationAfterWakeup() {
    if (as5600.isConnected()) {
        const float currentAngle = as5600.readAngle() * AS5600_RAW_TO_DEGREES;
        const float angleDifference = abs(currentAngle - rtcLastRotationAngle);
        
        if (angleDifference >= ROTATION_CHANGE_THRESHOLD_DEGREES) {
            previousEncoderAngle = currentAngle;
            rtcLastRotationAngle = currentAngle;
            lastActivityTimeMs = millis();
            return; // Stay awake
        }
    }
    
    // No significant rotation detected, go back to sleep
    esp_sleep_enable_timer_wakeup(DEEP_SLEEP_WAKE_INTERVAL_MS * 1000);
    esp_deep_sleep_start();
}

// ========================================
// Battery Monitoring Functions
// ========================================

void checkBatteryStatus(unsigned long currentTimeMs) {
    if (currentTimeMs - lastBatteryCheckTimeMs < BATTERY_CHECK_INTERVAL_MS) {
        return;
    }
    
    lastBatteryCheckTimeMs = currentTimeMs;
    
    // Average multiple readings for stability
    uint32_t batteryVoltageSum = 0;
    for (int i = 0; i < BATTERY_AVERAGE_SAMPLES; i++) {
        batteryVoltageSum += analogReadMilliVolts(BATTERY_PIN);
    }
    const float batteryVoltage = batteryVoltageSum / static_cast<float>(BATTERY_AVERAGE_SAMPLES);

    // Detect charging state based on voltage trends
    if (lastStableVoltage == 0.0f) {
        lastStableVoltage = batteryVoltage;
        isCharging = true;
    }

    if (isCharging && lastStableVoltage - batteryVoltage > VOLTAGE_CHANGE_THRESHOLD_MV) {
        lastStableVoltage = batteryVoltage;
        isCharging = false;
    }

    if (!isCharging && batteryVoltage - lastStableVoltage > VOLTAGE_CHANGE_THRESHOLD_MV) {
        lastStableVoltage = batteryVoltage;
        isCharging = true;
    }

    const float batteryPercentage = calculateBatteryPercentageFromReading(batteryVoltage);

    #if LOGGING_ENABLED
        Serial.print("Battery: ");
        Serial.print(batteryVoltage * VOLTAGE_DIVIDER_CORRECTION, 0);
        Serial.print("mV (");
        Serial.print(batteryPercentage, 1);
        Serial.print("%) ");
        Serial.println(isCharging ? "Charging" : "Discharging");
    #endif

    rtcBatteryVoltage = batteryPercentage;
    if (bleMouse.isConnected()) {
        bleMouse.setBatteryLevel(constrain(batteryPercentage, 0.0f, 100.0f));
    }
}

// ========================================
// Main Setup and Loop Functions
// ========================================

void setup() {
    #if LOGGING_ENABLED
        initializeSerialCommunication();
    #endif
    
    initializeEncoder();

    // Check if waking from deep sleep
    const esp_sleep_wakeup_cause_t wakeupReason = esp_sleep_get_wakeup_cause();
    if (wakeupReason == ESP_SLEEP_WAKEUP_TIMER) {
        checkRotationAfterWakeup();
        // If we reach here, device should stay awake
    }
    
    initBatteryReader();
    initializeBluetooth();
    
    lastActivityTimeMs = millis();
}

void loop() {
    const unsigned long currentTimeMs = millis();

    processEncoderScrolling(currentTimeMs);
    handleBleConnectionManagement(currentTimeMs);
    checkBatteryStatus(currentTimeMs);
    handleInactivityBasedSleep(currentTimeMs);

    delay(ENCODER_READ_INTERVAL_MS);
}
