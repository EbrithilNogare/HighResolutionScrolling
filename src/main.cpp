#include <Arduino.h>
#include "./BleMouse.h"
#include <AS5600.h>
#include <Wire.h>
#include "driver/temp_sensor.h"
#include "esp_sleep.h"

#define LOGGING_ON true

// settings
const float SCROLL_RESOLUTION_MULTIPLIER = 128.0;
const float DEGREES_PER_SCROLL_STEP = 90.0;

// config
const float BATTERY_VOLTAGE_DIVIDER_RATIO = 2.0;
const float BATTERY_LOW_VOLTAGE = 3.0;
const float BATTERY_HIGH_VOLTAGE = 4.2;

// timers
const unsigned long STATUS_REPORT_INTERVAL_MS = 1000;
const unsigned long ENCODER_READ_INTERVAL_MS = 16; // ble interval should not be less than 7.5 ms
const unsigned long BLE_CONNECTION_CHECK_INTERVAL_MS = 5000;
const unsigned long BATTERY_CHECK_INTERVAL_MS = 5000; // todo rise this number
const unsigned long LIGHT_SLEEP_TIMEOUT_MS = 60000;
const unsigned long LIGHT_SLEEP_WAKE_INTERVAL_MS = 1000;
const unsigned long INACTIVITY_LIGHT_SLEEP_MS = 50000;

BleMouse bleMouse("Smooth scroller", "ESP32 - EbrithilNogare", 69);
AS5600 as5600(&Wire);

unsigned long lastStatusReportTimeMs = 0;
unsigned long lastEncoderReadTimeMs = 0;
unsigned long lastBleConnectionCheckTimeMs = 0;
unsigned long lastSuccessfulBleConnectionTimeMs = 0;
unsigned long lastScrollEventTimeMs = 0;
unsigned long lastAngleChangeTimeMs = 0;
float previousEncoderAngle = 0;
float currentEncoderAngle = 0;
float lastInactiveAngle = 0;
bool isEncoderInitialized = false;
bool isAdvertising = false;


void reportDeviceStatus() {
    Serial.print(" | ");
    Serial.print(bleMouse.isConnected() ? "BLE: ON" : "BLE: OFF");
    
    float temperatureCelsius = 0;
    temp_sensor_read_celsius(&temperatureCelsius);
    Serial.print(" | ");
    Serial.print(temperatureCelsius);    
    Serial.print("°C");

    if (isEncoderInitialized) {
        currentEncoderAngle = as5600.readAngle() * AS5600_RAW_TO_DEGREES;
        Serial.print(" | Encoder: ");
        Serial.print(currentEncoderAngle, 1);
        Serial.print("° | Magnet: ");
        Serial.print(as5600.detectMagnet() ? "YES" : " NO");
    } else {
        Serial.print(" | Encoder: OFF");
    }

    Serial.println();
}

void initializeSerialCommunication() {
    Serial.begin(115200);
    delay(100);

    Serial.println("✓ Serial communication started");
}

void initializeTemperatureSensor() {
    temp_sensor_config_t temperatureSensorConfig = TSENS_CONFIG_DEFAULT();
    temperatureSensorConfig.dac_offset = TSENS_DAC_L2;
    temp_sensor_set_config(temperatureSensorConfig);
    temp_sensor_start();

    #if LOGGING_ON
        Serial.println("✓ Temperature sensor initialized");
    #endif
}

void initializeBluetoothMouse() {
    bleMouse.begin();

    isAdvertising = true;
    lastSuccessfulBleConnectionTimeMs = millis();
    lastBleConnectionCheckTimeMs = millis();

    #if LOGGING_ON
        Serial.println("✓ BLE Mouse service started");
    #endif
}

void initializeEncoder() {
    Wire.begin();
    delay(100);
    as5600.begin(4);
    delay(100);
    
    if (as5600.isConnected()) {
        isEncoderInitialized = true;
        currentEncoderAngle = as5600.readAngle() * AS5600_RAW_TO_DEGREES;
    }

    #if LOGGING_ON
        if(as5600.isConnected())
            Serial.println("✓ AS5600 magnetic encoder connected successfully");
        else
            Serial.println("Error: AS5600 sensor not found!");
    #endif
}

void handleBleConnectionManagement(unsigned long currentTimeMs) {
    bool isCurrentlyConnected = bleMouse.isConnected();
    
    if (!isCurrentlyConnected && !isAdvertising) {
        bleMouse.end();
        delay(100);
        bleMouse.begin();
        isAdvertising = true;

        #if LOGGING_ON
            Serial.println("Info: Starting BLE advertising");
        #endif
    }

    if(isCurrentlyConnected && isAdvertising) {
        isAdvertising = false;
        
        #if LOGGING_ON
           Serial.println("Info: BLE connection established");
        #endif
    }
}

void processEncoderScrolling(unsigned long currentTimeMs) {
    currentEncoderAngle = as5600.readAngle() * AS5600_RAW_TO_DEGREES;
    float angleDifferenceInDegrees = currentEncoderAngle - previousEncoderAngle;

    if (angleDifferenceInDegrees > 180) {
        angleDifferenceInDegrees -= 360;
    } else if (angleDifferenceInDegrees < -180) {
        angleDifferenceInDegrees += 360;
    }

    float rawScrollSteps = angleDifferenceInDegrees / DEGREES_PER_SCROLL_STEP * SCROLL_RESOLUTION_MULTIPLIER;
    signed short scrollSteps = static_cast<signed short>(min(max(rawScrollSteps, -32767.0f), 32767.0f));
    
    if (abs(scrollSteps) > 1) {
        previousEncoderAngle += (scrollSteps / SCROLL_RESOLUTION_MULTIPLIER) * DEGREES_PER_SCROLL_STEP;
    } else {
        previousEncoderAngle = currentEncoderAngle;
    }

    if (abs(scrollSteps) > 1) {
        lastScrollEventTimeMs = currentTimeMs;
        bleMouse.scroll(scrollSteps);

        #if LOGGING_ON
            Serial.print("Info: Scroll command sent: ");
            Serial.println(scrollSteps);
        #endif
    }
}

void handleInactivityBasedSleep(unsigned long currentTimeMs) {
    if (currentTimeMs - lastScrollEventTimeMs >= INACTIVITY_LIGHT_SLEEP_MS) {
        currentEncoderAngle = as5600.readAngle() * AS5600_RAW_TO_DEGREES;
        float angleDifference = abs(currentEncoderAngle - lastInactiveAngle);
    
        #if LOGGING_ON
           Serial.println("Info: Entering light sleep");
        #endif

        // esp_sleep_enable_timer_wakeup(LIGHT_SLEEP_WAKE_INTERVAL_MS * 1000);
        // esp_light_sleep_start(); // not working

        #if LOGGING_ON
           Serial.println("Info: Waking up from light sleep");
        #endif

    }
}

void checkBatteryStatus(unsigned long currentTimeMs)
{
    uint8_t batteryLevel = currentTimeMs % 100; // Simulated battery level

    if(bleMouse.isConnected())
        bleMouse.setBatteryLevel(batteryLevel);
}

void setup() {
    #if LOGGING_ON
        initializeSerialCommunication();
    #endif
    
    initializeTemperatureSensor();
    initializeEncoder();
    initializeBluetoothMouse();
    
    unsigned long currentTimeMs = millis();
    lastScrollEventTimeMs = currentTimeMs;
    lastAngleChangeTimeMs = currentTimeMs;
    if (isEncoderInitialized) {
        lastInactiveAngle = as5600.readAngle() * AS5600_RAW_TO_DEGREES;
    }
}

void loop() {
    unsigned long currentTimeMs = millis();

    #if LOGGING_ON
        if (currentTimeMs - lastStatusReportTimeMs >= STATUS_REPORT_INTERVAL_MS) {
            lastStatusReportTimeMs = currentTimeMs;
            reportDeviceStatus();
        }
    #endif
    
    if (currentTimeMs - lastEncoderReadTimeMs >= ENCODER_READ_INTERVAL_MS && bleMouse.isConnected() && isEncoderInitialized) {
        lastEncoderReadTimeMs = currentTimeMs;
        processEncoderScrolling(currentTimeMs);
    }
    
    if (currentTimeMs - lastBleConnectionCheckTimeMs >= BLE_CONNECTION_CHECK_INTERVAL_MS) {
        lastBleConnectionCheckTimeMs = currentTimeMs;
        handleBleConnectionManagement(currentTimeMs);
    }

    if( currentTimeMs - lastSuccessfulBleConnectionTimeMs >= BATTERY_CHECK_INTERVAL_MS) {
        lastSuccessfulBleConnectionTimeMs = currentTimeMs;
        checkBatteryStatus(currentTimeMs);
    }
    
    // handleInactivityBasedSleep(currentTimeMs);

    

    delay(3);
}
