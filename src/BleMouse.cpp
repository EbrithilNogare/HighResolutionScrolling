#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include "BLE2902.h"
#include "BLEHIDDevice.h"
#include "HIDTypes.h"
#include "HIDKeyboardTypes.h"
#include <driver/adc.h>
#include "sdkconfig.h"

#include "BleConnectionStatus.h"
#include "BleMouse.h"

#if defined(CONFIG_ARDUHAL_ESP_LOG)
  #include "esp32-hal-log.h"
  #define LOG_TAG ""
#else
  #include "esp_log.h"
  static const char* LOG_TAG = "BLEDevice";
#endif

static const uint8_t _hidReportDescriptor[] = {
  USAGE_PAGE(1),       0x01, // USAGE_PAGE (Generic Desktop)
  USAGE(1),            0x02, // USAGE (Mouse)
  COLLECTION(1),       0x01, // COLLECTION (Application)
  USAGE_PAGE(1),       0x01, //   USAGE_PAGE (Generic Desktop)
  USAGE(1),            0x02, //   USAGE (Mouse)
  COLLECTION(1),       0x02, //   COLLECTION (Logical)
  REPORT_ID(1),        0x01, //     REPORT_ID (1)
  USAGE(1),            0x01, //     USAGE (Pointer)
  COLLECTION(1),       0x00, //     COLLECTION (Physical)
  // --------------------------------------------------- Padding
  REPORT_SIZE(1),      0x08, //       REPORT_SIZE (8)
  REPORT_COUNT(1),     0x01, //       REPORT_COUNT (1)
  HIDINPUT(1),         0x03, //       INPUT (Constant, Variable, Absolute) ;8 bit padding
  // --------------------------  ----------------------- Padding
  REPORT_SIZE(1),      0x08, //       REPORT_SIZE (8)
  REPORT_COUNT(1),     0x01, //       REPORT_COUNT (1)
  HIDINPUT(1),         0x03, //       INPUT (Constant, Variable, Absolute) ;8 bit padding
  // --------------------------------------------------- Resolution Multiplier
  COLLECTION(1),       0x02, //       COLLECTION (Logical)
  REPORT_ID(1),        0x02, //         REPORT_ID (2)
  USAGE(1),            0x48, //         USAGE (Resolution Multiplier)
  REPORT_COUNT(1),     0x01, //         REPORT_COUNT (1)
  REPORT_SIZE(1),      0x04, //         REPORT_SIZE (4)
  LOGICAL_MINIMUM(1),  0x00, //         LOGICAL_MINIMUM (0)
  LOGICAL_MAXIMUM(1),  0x0f, //         LOGICAL_MAXIMUM (15)
  PHYSICAL_MINIMUM(1), 0x01, //         PHYSICAL_MINIMUM (1)
  PHYSICAL_MAXIMUM(1), 0x10, //         PHYSICAL_MAXIMUM (16)
  FEATURE(1),          0x06, //         FEATURE (Data, Var, Abs)
  // --------------------------------------------------- Wheel
  REPORT_ID(1),        0x01, //         REPORT_ID (1)
  USAGE(1),            0x38, //         USAGE (Wheel)
  PHYSICAL_MINIMUM(1), 0x00, //         PHYSICAL_MINIMUM (0)
  PHYSICAL_MAXIMUM(1), 0x00, //         PHYSICAL_MAXIMUM (0)
  LOGICAL_MINIMUM(1),  0x81, //         LOGICAL_MINIMUM (-127)
  LOGICAL_MAXIMUM(1),  0x7f, //         LOGICAL_MAXIMUM (127)
  REPORT_SIZE(1),      0x08, //         REPORT_SIZE (8)
  HIDINPUT(1),         0x06, //         INPUT (Data, Variable, Relative)
  END_COLLECTION(0),         //       END_COLLECTION (Logical)
  // --------------------------------------------------- Horizontal wheel
  REPORT_SIZE(1),      0x08, //       REPORT_SIZE (8)
  REPORT_COUNT(1),     0x01, //       REPORT_COUNT (1)
  HIDINPUT(1),         0x03, //       INPUT (Constant, Variable, Absolute) ;8 bit padding
  END_COLLECTION(0),         //     END_COLLECTION (Physical)
  END_COLLECTION(0),         //   END_COLLECTION (Logical)
  END_COLLECTION(0),         // END_COLLECTION (Application)
};

BleMouse::BleMouse(std::string deviceName, std::string deviceManufacturer, uint8_t batteryLevel) : 
    _buttons(0),
    hid(0)
{
  this->deviceName = deviceName;
  this->deviceManufacturer = deviceManufacturer;
  this->batteryLevel = batteryLevel;
  this->connectionStatus = new BleConnectionStatus();
}

void BleMouse::begin(void)
{
  xTaskCreate(this->taskServer, "server", 20000, (void *)this, 5, NULL);
}

void BleMouse::end(void){}

void BleMouse::scroll(signed char wheel)
{
  if (this->isConnected())
  {
    uint8_t m[5];
    m[0] = 0;
    m[1] = 0;
    m[2] = 0;
    m[3] = wheel;
    m[4] = 0;
    this->inputMouse->setValue(m, 5);
    this->inputMouse->notify();
  }
}

void BleMouse::setResolutionMultiplier(uint8_t multiplier)
{
  if (this->isConnected() && this->featureResolution)
  {
    if (multiplier < 1) multiplier = 1;
    if (multiplier > 16) multiplier = 16;
    
    uint8_t featureValue = multiplier - 1; // 0-15 range
    this->featureResolution->setValue(&featureValue, 1);
    ESP_LOGD(LOG_TAG, "Resolution multiplier set to %d", multiplier);
  }
}

bool BleMouse::isConnected(void) {
  return this->connectionStatus->connected;
}

void BleMouse::setBatteryLevel(uint8_t level) {
  this->batteryLevel = level;
  if (hid != 0)
      this->hid->setBatteryLevel(this->batteryLevel);
}

void BleMouse::taskServer(void* pvParameter) {
  BleMouse* bleMouseInstance = (BleMouse *) pvParameter; //static_cast<BleMouse *>(pvParameter);
  BLEDevice::init(bleMouseInstance->deviceName);
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(bleMouseInstance->connectionStatus);
  

  bleMouseInstance->hid = new BLEHIDDevice(pServer);
  bleMouseInstance->inputMouse = bleMouseInstance->hid->inputReport(1); // <-- input REPORTID from report map
  bleMouseInstance->featureResolution = bleMouseInstance->hid->featureReport(2); // <-- feature REPORTID for resolution multiplier
  bleMouseInstance->connectionStatus->inputMouse = bleMouseInstance->inputMouse;

  bleMouseInstance->hid->manufacturer()->setValue(bleMouseInstance->deviceManufacturer);

  bleMouseInstance->hid->pnp(0x02, 0xe502, 0xa111, 0x0210);
  bleMouseInstance->hid->hidInfo(0x00,0x02);

  BLESecurity *pSecurity = new BLESecurity();

  pSecurity->setAuthenticationMode(ESP_LE_AUTH_BOND);

  bleMouseInstance->hid->reportMap((uint8_t*)_hidReportDescriptor, sizeof(_hidReportDescriptor));
  bleMouseInstance->hid->startServices();

  bleMouseInstance->onStarted(pServer);

  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->setAppearance(HID_MOUSE);
  pAdvertising->addServiceUUID(bleMouseInstance->hid->hidService()->getUUID());
  pAdvertising->start();
  bleMouseInstance->hid->setBatteryLevel(bleMouseInstance->batteryLevel);

  ESP_LOGD(LOG_TAG, "Advertising started!");
  vTaskDelay(portMAX_DELAY); //delay(portMAX_DELAY);
}
