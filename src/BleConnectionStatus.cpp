#include "BleConnectionStatus.h"

BleConnectionStatus::BleConnectionStatus(void) {
}

void BleConnectionStatus::onConnect(BLEServer* pServer)
{
  this->connected = true;
}

void BleConnectionStatus::onDisconnect(BLEServer* pServer)
{
  this->connected = false;
}
