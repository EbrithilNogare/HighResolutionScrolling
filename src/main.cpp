#include <Arduino.h>
#include "./BleMouse.h"
#include <AS5600.h>
#include <Wire.h>
#include "driver/temp_sensor.h"


const unsigned long HEARTBEAT_INTERVAL = 100; // ms
const unsigned long ENCODER_READ_INTERVAL = 1000; // ms
const unsigned int RESOLUTION_MULTIPLIER = 4; // Higher values = more sensitive scrolling
const float BASE_DEGREES_PER_SCROLL = 15.0; // Degrees of rotation per scroll step

BleMouse bleMouse("ESP32-C3 Scroll Mouse", "ESP32", 69);
AS5600 as5600(&Wire);

unsigned long lastHeartbeat = 0;
unsigned long lastEncoderRead = 0;
float lastAngle = 0;
float currentAngle = 0;
bool encoderInitialized = false;
float totalRotation = 0; // Track total rotation for better precision

bool bleSetupCompleted = false;


void reportLog(){
        // Bluetooth
        Serial.print(" | ");
        Serial.print(bleMouse.isConnected() ? "BLE: ON" : "BLE: OFF");
        
        // Temperature
        float result = 0;
        temp_sensor_read_celsius(&result);
        Serial.print(" | ");
        Serial.print(result);    
        Serial.print("°C");

        // Encoder
        if (encoderInitialized) {
            currentAngle = as5600.readAngle() * AS5600_RAW_TO_DEGREES;
            Serial.print(" | Encoder: ");
            Serial.print(currentAngle, 1);
            Serial.print("°");
        } else {
            Serial.print(" | Encoder: OFF");
        }

        Serial.println();
}

void initSerial(){
    Serial.begin(9600);
    delay(100);
    Serial.println("✓ Serial communication started");
}

void initTempSensor(){
    temp_sensor_config_t temp_sensor = TSENS_CONFIG_DEFAULT();
    temp_sensor.dac_offset = TSENS_DAC_L2;
    temp_sensor_set_config(temp_sensor);
    temp_sensor_start();
}

void initBLE(){
    bleMouse.begin();
    Serial.println("✓ BLE Mouse service started");
}

void setupBLE(){
    if (bleMouse.isConnected()) {
        bleSetupCompleted = true;
        std::string output = "Nothing";
        bleMouse.setResolutionMultiplier(RESOLUTION_MULTIPLIER, &output);
        Serial.print("✓ Resolution multiplier set to ");
        Serial.println(RESOLUTION_MULTIPLIER);
        Serial.println(output.c_str());
    }
}

void initEncoder() {
    Wire.begin();
    delay(100);
    as5600.begin(4);
    delay(100);
    
    if (as5600.isConnected()) {
        Serial.println("✓ AS5600 magnetic encoder connected successfully");
        encoderInitialized = true;
        currentAngle = as5600.readAngle() * AS5600_RAW_TO_DEGREES;
    } else {
        Serial.println("Error: AS5600 sensor not found! Check I2C connections.");
        Serial.println("       Expected connections:");
        Serial.println("       - SDA to GPIO6");
        Serial.println("       - SCL to GPIO7");
        Serial.println("       - DIR to GPIO4");
        Serial.println("       - VCC to 3.3V");
        Serial.println("       - GND to GND");
    }
}

void setup()
{
    initSerial();
    initTempSensor();
    initEncoder();
    initBLE();
}

void loop()
{
    unsigned long currentTime = millis();
    
    if(!bleSetupCompleted){
        setupBLE();
    }

    if(currentTime - lastEncoderRead >= ENCODER_READ_INTERVAL && bleMouse.isConnected() && encoderInitialized) {
        lastEncoderRead = currentTime;
        bleMouse.scroll(1);
        Serial.println("Info: Simulated scroll command sent (16 units)");
    }

    if (currentTime - lastHeartbeat >= HEARTBEAT_INTERVAL) {
        lastHeartbeat = currentTime;
        reportLog();
    }
    
    delay(10); // Small delay to prevent overwhelming the system
}
