#include <Arduino.h>
#include "./BleMouse.h"
#include <AS5600.h>
#include <Wire.h>
#include "driver/temp_sensor.h"
#include "esp_sleep.h"

#define LOGGING_ON true

// settings
const float SCROLL_RESOLUTION_MULTIPLIER = 128.0;
const float DEGREES_PER_SCROLL_STEP = -9.0;

// config
const float BATTERY_VOLTAGE_DIVIDER_RATIO = 2.0;
const float BATTERY_LOW_VOLTAGE = 3.0;
const float BATTERY_HIGH_VOLTAGE = 4.2;
const int SERIAL_SPEED = 115200;
const int ENCODER_POWER_PIN = 20;
const int ENCODER_SCL_PIN = 4;

// timers
const unsigned long STATUS_REPORT_INTERVAL_MS = 1000;
const unsigned long ENCODER_READ_INTERVAL_MS = 16; // ble interval should not be less than 7.5 ms
const unsigned long BLE_CONNECTION_CHECK_INTERVAL_MS = 5000;
const unsigned long BATTERY_CHECK_INTERVAL_MS = 60 * 1000; // todo rise this number
const unsigned long LIGHT_SLEEP_WAKE_INTERVAL_MS = 1000;
const unsigned long LIGHT_SLEEP_TIMEOUT_MS = 10 * 1000; // todo rise this number

BleMouse bleMouse("Smooth scroller", "ESP32 - EbrithilNogare", 69);
AS5600 as5600(&Wire);

unsigned long lastStatusReportTimeMs = 0;
unsigned long lastEncoderReadTimeMs = 0;
unsigned long lastBleConnectionCheckTimeMs = 0;
unsigned long lastSuccessfulBleConnectionTimeMs = 0;
unsigned long lastScrollEventTimeMs = 0;
float previousEncoderAngle = 0;
float currentEncoderAngle = 0;
bool isEncoderInitialized = false;
bool isAdvertising = false;


#if LOGGING_ON
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

void Log2DGraph()
{
    Serial.print('#');
    Serial.print(as5600.readAngle() * AS5600_RAW_TO_DEGREES);
    Serial.print(' ');
    Serial.println(as5600.readMagnitude());
}

void initializeSerialCommunication() {
    Serial.begin(SERIAL_SPEED);
    delay(100);

    Serial.println("✓ Serial communication started");
}
#endif // LOGGING_ON


void initializeTemperatureSensor() {
    temp_sensor_config_t temperatureSensorConfig = TSENS_CONFIG_DEFAULT();
    temperatureSensorConfig.dac_offset = TSENS_DAC_L2;
    temp_sensor_set_config(temperatureSensorConfig);
    temp_sensor_start();

    #if LOGGING_ON
        Serial.println("✓ Temperature sensor initialized");
    #endif
}

void initializeBluetooth() {
    bleMouse.begin();

    isAdvertising = true;
    lastSuccessfulBleConnectionTimeMs = millis();
    lastBleConnectionCheckTimeMs = millis();

    #if LOGGING_ON
        Serial.println("✓ BLE Mouse service started");
    #endif
}

void terminateBluetooth() {
    bleMouse.end();
    isAdvertising = false;
    delay(50);
}

void initializeEncoder() {
    digitalWrite(ENCODER_POWER_PIN, HIGH); // power ON
    delay(1);
    Wire.begin();
    delay(2);
    as5600.begin(ENCODER_SCL_PIN);
    delay(10);
    
    if (as5600.isConnected()) {
        isEncoderInitialized = true;
        currentEncoderAngle = as5600.readAngle() * AS5600_RAW_TO_DEGREES;
        previousEncoderAngle = currentEncoderAngle;
    }

    #if LOGGING_ON
        if(as5600.isConnected())
            Serial.println("✓ AS5600 magnetic encoder connected successfully");
        else
            Serial.println("Error: AS5600 sensor not found!");
    #endif
}

void terminateEncoder(){
    if (isEncoderInitialized) {
        isEncoderInitialized = false;
        Wire.end();
        digitalWrite(ENCODER_POWER_PIN, LOW); // power OFF
        delay(1);
        
        #if LOGGING_ON
            Serial.println("✓ AS5600 magnetic encoder terminated");
        #endif
    }
}

void handleBleConnectionManagement(unsigned long currentTimeMs) {
    bool isCurrentlyConnected = bleMouse.isConnected();
    
    if (!isCurrentlyConnected) {
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

    if(abs(angleDifferenceInDegrees) >= 180){
        previousEncoderAngle = currentEncoderAngle;
        return; // often getting number 5000 out of this
    }

    if (angleDifferenceInDegrees > 180) {
        angleDifferenceInDegrees -= 360;
    } else if (angleDifferenceInDegrees < -180) {
        angleDifferenceInDegrees += 360;
    }

    float rawScrollSteps = angleDifferenceInDegrees / DEGREES_PER_SCROLL_STEP * SCROLL_RESOLUTION_MULTIPLIER;
    signed short scrollSteps = static_cast<signed short>(min(max(rawScrollSteps, -32767.0f), 32767.0f));
    
    if (abs(scrollSteps) > 1) {
        previousEncoderAngle += (scrollSteps / SCROLL_RESOLUTION_MULTIPLIER) * DEGREES_PER_SCROLL_STEP;
        lastScrollEventTimeMs = currentTimeMs;

        if(bleMouse.isConnected())
            bleMouse.scroll(scrollSteps);

        #if LOGGING_ON
            Serial.print("Info: Scroll command sent: ");
            Serial.println(scrollSteps);
        #endif
    }
}

void handleInactivityBasedSleep(unsigned long now) {
    if (now - lastScrollEventTimeMs < LIGHT_SLEEP_TIMEOUT_MS)
        return;
    
    #if LOGGING_ON
        Serial.println("Info: Entering light sleep");
        Serial.flush();
        Serial.end();
    #endif

    terminateBluetooth();
    
    int lastInactiveAngle = as5600.readAngle() * AS5600_RAW_TO_DEGREES;
    float newAngle;
    do {
        // Go sleep
        esp_sleep_enable_timer_wakeup(LIGHT_SLEEP_WAKE_INTERVAL_MS * 1000);
        esp_light_sleep_start();
        // Wake up
        sleep(5);
        newAngle = as5600.readAngle() * AS5600_RAW_TO_DEGREES;
    } while (abs(newAngle - lastInactiveAngle) < 5.0);
    
    #if LOGGING_ON
        Serial.begin(SERIAL_SPEED);
        delay(100);
        Serial.println("Info: Movement detected, resuming full power");
    #endif

    initializeBluetooth();
    
    previousEncoderAngle = newAngle;
    lastScrollEventTimeMs = millis();
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
    
    pinMode(ENCODER_POWER_PIN, OUTPUT);
    
    initializeTemperatureSensor();
    initializeEncoder();
    initializeBluetooth();
    
    lastScrollEventTimeMs = millis();
}

void loop() {
    unsigned long currentTimeMs = millis();

    #if LOGGING_ON
        if (currentTimeMs - lastStatusReportTimeMs >= STATUS_REPORT_INTERVAL_MS) {
            lastStatusReportTimeMs = currentTimeMs;
            reportDeviceStatus();
        }
        //Log2DGraph();
    #endif
    
    if (currentTimeMs - lastEncoderReadTimeMs >= ENCODER_READ_INTERVAL_MS && isEncoderInitialized) {
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
    
    handleInactivityBasedSleep(currentTimeMs);

    

    delay(ENCODER_READ_INTERVAL_MS);
}
