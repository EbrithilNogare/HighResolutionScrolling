#include <Arduino.h>
#include <AS5600.h>
#include <Wire.h>
#include "driver/temp_sensor.h"
#include "esp_sleep.h"
#include "./BleMouse.h"

#define LOGGING_ON false

// Sleep and Power Management Settings
const unsigned long DEEP_SLEEP_TIMEOUT_MS = 120 * 1000;
const unsigned long DEEP_SLEEP_WAKE_INTERVAL_MS = 3 * 1000;
const float ROTATION_CHANGE_THRESHOLD_DEGREES = 10.0;

// Device Configuration
const float SCROLL_RESOLUTION_MULTIPLIER = 128.0;
const float DEGREES_PER_SCROLL_STEP = -360.0 / 4096.0 * 128.0;
const float BATTERY_LOW_VOLTAGE = 3300; // mV
const float BATTERY_HIGH_VOLTAGE = 4100; // mV
const int SERIAL_SPEED = 115200;
const int ENCODER_POWER_PIN = 20;
const int BATTERY_PIN = 4;
const float VOLTAGE_DIVIDER_RESISTOR_VALUE_PLUS = 214000.0; // Measure this!
const float VOLTAGE_DIVIDER_RESISTOR_VALUE_MINUS = 218000.0; // Measure this!

// Operation Intervals
const unsigned long ENCODER_READ_INTERVAL_MS = 15; // BLE interval should not be less than 7.5 ms
const unsigned long BLE_CONNECTION_CHECK_INTERVAL_MS = 5000;
const unsigned long BATTERY_CHECK_INTERVAL_MS = 10 * 1000;

// RTC Memory Storage (persists during deep sleep)
RTC_DATA_ATTR float rtcLastRotationAngle = 1000.0;
RTC_DATA_ATTR float rtcBatteryVoltage = 100.0;

// Global Objects
BleMouse bleMouse("Smooth scroller", "ESP32 - EbrithilNogare", rtcBatteryVoltage);
AS5600 as5600(&Wire);

// State Variables
unsigned long lastStatusReportTimeMs = 0;
unsigned long lastEncoderReadTimeMs = 0;
unsigned long lastBleConnectionCheckTimeMs = 0;
unsigned long lastSuccessfulBatteryCheckTimeMs = 0;
unsigned long lastActivityTimeMs = 0;
float previousEncoderAngle = 0;
float currentEncoderAngle = 0;
bool isEncoderInitialized = false;
bool isAdvertising = false;
bool isCharging = false;
float lastStableVoltage = 0.0;

inline float calculateBatteryPercentageFromReading(uint32_t voltageReading) {
    const int tableSize = 11;
    const float chargingVoltages[tableSize] = {3545.0f, 3622.0f, 3715.0f, 3815.0f, 3876.0f, 3913.0f, 3945.0f, 3987.0f, 4035.0f, 4064.0f, 4100.0f}; // Measure this!
    const float dischargingVoltages[tableSize] = {3102.0f, 3442.0f, 3547.0f, 3673.0f, 3736.0f, 3776.0f, 3812.0f, 3880.0f, 3925.0f, 3952.8f, 4057.0f}; // Measure this!
    const float percentages[tableSize] = {0.0f, 10.0f, 20.0f, 30.0f, 40.0f, 50.0f, 60.0f, 70.0f, 80.0f, 90.0f, 100.0f};

    float voltageDividerCorrection = (VOLTAGE_DIVIDER_RESISTOR_VALUE_MINUS + VOLTAGE_DIVIDER_RESISTOR_VALUE_PLUS) / VOLTAGE_DIVIDER_RESISTOR_VALUE_MINUS;
    float batteryVoltage = voltageReading * voltageDividerCorrection;
    
    const float* voltageTable = isCharging ? chargingVoltages : dischargingVoltages;
    
    if (batteryVoltage <= voltageTable[0]) {
        return percentages[0];
    }
    if (batteryVoltage >= voltageTable[tableSize - 1]) {
        return percentages[tableSize - 1];
    }
    
    int lowerIndex = 0;
    for (int i = 0; i < tableSize - 1; i++) {
        if (batteryVoltage >= voltageTable[i] && batteryVoltage <= voltageTable[i + 1]) {
            lowerIndex = i;
            break;
        }
    }
    
    int upperIndex = lowerIndex + 1;
    
    // Linear interpolation between the two closest points
    float voltage1 = voltageTable[lowerIndex];
    float voltage2 = voltageTable[upperIndex];
    float percentage1 = percentages[lowerIndex];
    float percentage2 = percentages[upperIndex];
    
    float interpolatedPercentage = percentage1 + (batteryVoltage - voltage1) * (percentage2 - percentage1) / (voltage2 - voltage1);
    
    return interpolatedPercentage;
}


#if LOGGING_ON
void initializeSerialCommunication() {
    Serial.begin(SERIAL_SPEED);
    delay(1000);

    Serial.println("✓ Serial communication started");
}
#endif // LOGGING_ON

void initializeTemperatureSensor() {
    temp_sensor_config_t temperatureSensorConfig = TSENS_CONFIG_DEFAULT();
    temperatureSensorConfig.dac_offset = TSENS_DAC_L2;
    temp_sensor_set_config(temperatureSensorConfig);
    temp_sensor_start();
}

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
    if (currentTimeMs - lastBleConnectionCheckTimeMs < BLE_CONNECTION_CHECK_INTERVAL_MS)
        return;
    
    lastBleConnectionCheckTimeMs = currentTimeMs;
    
    bool isCurrentlyConnected = bleMouse.isConnected();
    
    if (!isCurrentlyConnected) {
        bleMouse.end();
        delay(100);
        bleMouse.begin();
        isAdvertising = true;
    }

    if(isCurrentlyConnected && isAdvertising) {
        isAdvertising = false;
    }
}

// ========================================
// Encoder Management Functions
// ========================================

void initializeEncoder() {
    pinMode(ENCODER_POWER_PIN, OUTPUT);
    digitalWrite(ENCODER_POWER_PIN, HIGH); // power ON
    Wire.begin();
    as5600.begin(AS5600_SW_DIRECTION_PIN);
    
    // wait for first register value
    unsigned long time = millis();
    while(as5600.isConnected() && as5600.readAngle() == 0 && millis() - time < 10) { }
    
    if (as5600.isConnected()) {
        isEncoderInitialized = true;
        currentEncoderAngle = as5600.readAngle() * AS5600_RAW_TO_DEGREES;
        
        if (rtcLastRotationAngle > 360.0) {
            // First run
            previousEncoderAngle = currentEncoderAngle;
            rtcLastRotationAngle = currentEncoderAngle;
        } else {
            // Deep sleep recovery
            previousEncoderAngle = rtcLastRotationAngle;
        }
    }
}

void terminateEncoder(){
    if (isEncoderInitialized) {
        isEncoderInitialized = false;
        Wire.end();
        digitalWrite(ENCODER_POWER_PIN, LOW); // power OFF
    }
}

// ========================================
// Scroll Processing Functions
// ========================================

void processEncoderScrolling(unsigned long currentTimeMs) {
    if (currentTimeMs - lastEncoderReadTimeMs < ENCODER_READ_INTERVAL_MS || !isEncoderInitialized)
        return;
    
    lastEncoderReadTimeMs = currentTimeMs;
    
    currentEncoderAngle = as5600.readAngle() * AS5600_RAW_TO_DEGREES;
    float angleDifferenceInDegrees = currentEncoderAngle - previousEncoderAngle;

    if (angleDifferenceInDegrees > 180) {
        angleDifferenceInDegrees -= 360;
    } else if (angleDifferenceInDegrees < -180) {
        angleDifferenceInDegrees += 360;
    }
    
    if(abs(angleDifferenceInDegrees) >= 180){
        previousEncoderAngle = currentEncoderAngle;
        return; // often getting number 5000 out of this
    }

    float rawScrollSteps = angleDifferenceInDegrees / DEGREES_PER_SCROLL_STEP * SCROLL_RESOLUTION_MULTIPLIER;
    signed short scrollSteps = static_cast<signed short>(min(max(rawScrollSteps, -32767.0f), 32767.0f));
    
    if (abs(scrollSteps) > 1) {
        previousEncoderAngle += (scrollSteps / SCROLL_RESOLUTION_MULTIPLIER) * DEGREES_PER_SCROLL_STEP;
        lastActivityTimeMs = currentTimeMs;

        if(bleMouse.isConnected()){
            bleMouse.scroll(scrollSteps);
        }
    }
}

// ========================================
// Power Management Functions
// ========================================

void initBatteryReader() {
    analogReadResolution(12); // 12-bit resolution (0-4095)
    analogSetAttenuation(ADC_11db); // Set attenuation for 0-3.3V range
    pinMode(BATTERY_PIN, INPUT);
}

void handleInactivityBasedSleep(unsigned long currentTimeMs) {
    if (currentTimeMs - lastActivityTimeMs < DEEP_SLEEP_TIMEOUT_MS)
        return;

    // Entering deep sleep
    terminateBluetooth();
    terminateEncoder();
    
    rtcLastRotationAngle = previousEncoderAngle;
    
    esp_sleep_enable_timer_wakeup(DEEP_SLEEP_WAKE_INTERVAL_MS * 1000);
    esp_deep_sleep_start();
}

void checkRotationAfterWakeup() {    
    if (as5600.isConnected()) {
        float currentAngle = as5600.readAngle() * AS5600_RAW_TO_DEGREES;
        float savedAngle = rtcLastRotationAngle;
        float angleDifference = abs(currentAngle - savedAngle);
        
        if (angleDifference >= ROTATION_CHANGE_THRESHOLD_DEGREES) {
            previousEncoderAngle = currentAngle;
            rtcLastRotationAngle = currentAngle;
            lastActivityTimeMs = millis();
            return; // Exit function to stay awake
        }
    }
    
    esp_sleep_enable_timer_wakeup(DEEP_SLEEP_WAKE_INTERVAL_MS * 1000);
    esp_deep_sleep_start();
}

// ========================================
// Utility Functions
// ========================================

void checkBatteryStatus(unsigned long currentTimeMs)
{
    if (currentTimeMs - lastSuccessfulBatteryCheckTimeMs < BATTERY_CHECK_INTERVAL_MS)
        return;
    
    lastSuccessfulBatteryCheckTimeMs = currentTimeMs;
    
    const float numReadings = 128;
    
    uint32_t batteryVoltageSum = 0;
    
    for (int i = 0; i < numReadings; i++)
        batteryVoltageSum += analogReadMilliVolts(BATTERY_PIN);
    float batteryVoltage = batteryVoltageSum / numReadings;
    
    float voltageDividerCorrection = (VOLTAGE_DIVIDER_RESISTOR_VALUE_MINUS + VOLTAGE_DIVIDER_RESISTOR_VALUE_PLUS) / VOLTAGE_DIVIDER_RESISTOR_VALUE_MINUS;

    const float threshold = 5.0f; // mV

    if(lastStableVoltage == 0.0){
        lastStableVoltage = batteryVoltage;
        isCharging = BATTERY_HIGH_VOLTAGE - (BATTERY_HIGH_VOLTAGE - BATTERY_LOW_VOLTAGE) / 5 < batteryVoltage;
    }
    if(isCharging && lastStableVoltage - batteryVoltage > threshold){
        lastStableVoltage = batteryVoltage;
        isCharging = false;
    }
    if(!isCharging && batteryVoltage - lastStableVoltage > threshold){
        lastStableVoltage = batteryVoltage;
        isCharging = true;
    }

    float batteryPercentage = calculateBatteryPercentageFromReading(batteryVoltage);

    #if LOGGING_ON
            Serial.print("Battery voltage: ");
            Serial.print(batteryVoltage * voltageDividerCorrection, 0);
            Serial.print("mV (");
            Serial.print(batteryPercentage, 1);
            Serial.print("%) ");
            Serial.println("Charging: " + String(isCharging ? "Yes" : "No"));
    #endif

    rtcBatteryVoltage = batteryPercentage;
    if(bleMouse.isConnected())
        bleMouse.setBatteryLevel(constrain(batteryPercentage, 0.0f, 100.0f));
}

// ========================================
// Main Setup and Loop Functions
// ========================================

void setup() {
    #if LOGGING_ON
        initializeSerialCommunication();
    #endif
    
    initializeEncoder();

    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    if (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER) {
        checkRotationAfterWakeup();
        // If we reach here, powering up
    }
    
    initBatteryReader();
    initializeTemperatureSensor();
    initializeBluetooth();
    
    lastActivityTimeMs = millis();
}

unsigned long lastHelloTimeMs = 0;

void loop() {
    unsigned long currentTimeMs = millis();

    processEncoderScrolling(currentTimeMs);
    handleBleConnectionManagement(currentTimeMs);
    checkBatteryStatus(currentTimeMs);
    handleInactivityBasedSleep(currentTimeMs);

    delay(ENCODER_READ_INTERVAL_MS);
}
