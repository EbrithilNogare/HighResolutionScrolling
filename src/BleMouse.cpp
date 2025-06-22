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
  // ------------------------------------------------- X/Y position
  USAGE_PAGE(1),       0x01, //       USAGE_PAGE (Generic Desktop)
  USAGE(1),            0x30, //       USAGE (X)
  USAGE(1),            0x31, //       USAGE (Y)
  LOGICAL_MINIMUM(1),  0x81, //       LOGICAL_MINIMUM (-127)
  LOGICAL_MAXIMUM(1),  0x7f, //       LOGICAL_MAXIMUM (127)
  REPORT_SIZE(1),      0x08, //       REPORT_SIZE (8)
  REPORT_COUNT(1),     0x02, //       REPORT_COUNT (2)
  HIDINPUT(1),         0x06, //       INPUT (Data, Variable, Relative) ;3 bytes (X,Y,Wheel)
  // --------------------------------------------------- Wheel
  USAGE(1),            0x38, //       USAGE (Wheel)
  PHYSICAL_MINIMUM(1), 0x00, //       PHYSICAL_MINIMUM (0)
  PHYSICAL_MAXIMUM(1), 0x00, //       PHYSICAL_MAXIMUM (0)
  LOGICAL_MINIMUM(1),  0x81, //       LOGICAL_MINIMUM (-127)
  LOGICAL_MAXIMUM(1),  0x7f, //       LOGICAL_MAXIMUM (127)
  REPORT_SIZE(1),      0x08, //       REPORT_SIZE (8)
  REPORT_COUNT(1),     0x01, //       REPORT_COUNT (1)
  HIDINPUT(1),         0x06, //       INPUT (Data, Variable, Relative)
  // ------------------------------------------------- Resolution Multiplier
  USAGE_PAGE(1),       0x01, //       Generic Desktop
  USAGE(1),            0x48, //       Usage: Resolution Multiplier
  LOGICAL_MINIMUM(1),  0x00, //       Logical Min = 0
  LOGICAL_MAXIMUM(1),  0x01, //       Logical Max = 1
  PHYSICAL_MINIMUM(1), 0x01, //       Physical Min = 1
  PHYSICAL_MAXIMUM(1), 0x80, //       Physical Max = 128
  REPORT_SIZE(1),      0x08, //       REPORT_SIZE (8)
  REPORT_COUNT(1),     0x01, //       REPORT_COUNT (1)
  FEATURE(1),          0x02, //       Feature (Data,Var,Abs)
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
    uint8_t m[3];
    m[0] = 0; // X
    m[1] = 0; // Y
    m[2] = wheel; // vertical wheel
    this->inputMouse->setValue(m, 3);
    this->inputMouse->notify();
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
  bleMouseInstance->inputMouse = bleMouseInstance->hid->inputReport(0x01); // <-- input REPORTID from report map
  bleMouseInstance->featureResolution = bleMouseInstance->hid->featureReport(0x01); // <-- feature REPORTID for resolution multiplier
  bleMouseInstance->connectionStatus->inputMouse = bleMouseInstance->inputMouse;
  
  bleMouseInstance->featureResolution->setValue(new uint8_t{0x80}, 1);
  
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
