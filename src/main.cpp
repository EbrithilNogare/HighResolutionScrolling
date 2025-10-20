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
const float BATTERY_LOW_VOLTAGE = 3.3;
const float BATTERY_HIGH_VOLTAGE = 4.2;
const int SERIAL_SPEED = 115200;
const int ENCODER_POWER_PIN = 20;
const int BATTERY_PIN = 4;
const float VOLTAGE_DIVIDER_RESISTOR_VALUE_LIVE = 217.4; // Measure this!
const float VOLTAGE_DIVIDER_RESISTOR_VALUE_GROUND = 221.7; // Measure this!

// Operation Intervals
const unsigned long ENCODER_READ_INTERVAL_MS = 15; // BLE interval should not be less than 7.5 ms
const unsigned long BLE_CONNECTION_CHECK_INTERVAL_MS = 5000;
const unsigned long BATTERY_CHECK_INTERVAL_MS = 300 * 1000;

// Global Objects
BleMouse bleMouse("Smooth scroller", "ESP32 - EbrithilNogare", 69);
AS5600 as5600(&Wire);

// RTC Memory Storage (persists during deep sleep)
RTC_DATA_ATTR float rtcLastRotationAngle = 1000.0;

// State Variables
unsigned long lastStatusReportTimeMs = 0;
unsigned long lastEncoderReadTimeMs = 0;
unsigned long lastBleConnectionCheckTimeMs = 0;
unsigned long lastSuccessfulBleConnectionTimeMs = 0;
unsigned long lastActivityTimeMs = 0;
float previousEncoderAngle = 0;
float currentEncoderAngle = 0;
bool isEncoderInitialized = false;
bool isAdvertising = false;


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
    lastSuccessfulBleConnectionTimeMs = millis();
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
    if (currentTimeMs - lastSuccessfulBleConnectionTimeMs < BATTERY_CHECK_INTERVAL_MS)
        return;
    
    lastSuccessfulBleConnectionTimeMs = currentTimeMs;
    
    const float numReadings = 32;
    
    uint32_t batteryVoltageSum = 0;
    
    for (int i = 0; i < numReadings; i++)
        batteryVoltageSum += analogReadMilliVolts(BATTERY_PIN);
    
    float voltageDividerCorrection = (VOLTAGE_DIVIDER_RESISTOR_VALUE_GROUND + VOLTAGE_DIVIDER_RESISTOR_VALUE_LIVE) / VOLTAGE_DIVIDER_RESISTOR_VALUE_GROUND;
    float batteryVoltage = (batteryVoltageSum / numReadings / 1000.0f) * voltageDividerCorrection;
    float batteryPercentage = ((batteryVoltage - BATTERY_LOW_VOLTAGE) / (BATTERY_HIGH_VOLTAGE - BATTERY_LOW_VOLTAGE)) * 100.0;

    #if LOGGING_ON
            Serial.print("Battery voltage: ");
            Serial.print(batteryVoltage, 2);
            Serial.print("V (");
            Serial.print(batteryPercentage, 1);
            Serial.println("%)");
    #endif

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
