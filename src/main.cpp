#include <Arduino.h>
#include <NimBLEDevice.h>

#define LOG_TAG_BLESERVER "BleServer"

int MTU_SIZE = 128;
int PACKET_SIZE = MTU_SIZE - 3;
NimBLEServer *pServer = nullptr;
NimBLEService *pServiceVesc = nullptr;
NimBLEService *pServiceRescue = nullptr;
NimBLECharacteristic *pCharacteristicVescTx = nullptr;
NimBLECharacteristic *pCharacteristicVescRx = nullptr;

bool deviceConnected = false;
bool oldDeviceConnected = false;
uint8_t txValue = 0;

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define VESC_SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define VESC_CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define VESC_CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

// DIAGNOSTICS: set to 1 to test with RX/TX physically swapped, in case the
// wiring orientation to the VESC turns out to be reversed. No rewiring needed,
// just flip this and re-flash.
#define VESC_UART_SWAP_PINS 0
#if VESC_UART_SWAP_PINS
#define VESC_RX_PIN 16 // GPIO16 (D6)
#define VESC_TX_PIN 17 // GPIO17 (D7)
#else
#define VESC_RX_PIN 17 // GPIO17 (D7)
#define VESC_TX_PIN 16 // GPIO16 (D6)
#endif

/**  None of these are required as they will be handled by the library with defaults. **
 **                       Remove as you see fit for your needs                        */
class MyServerCallbacks : public NimBLEServerCallbacks
{
  void onConnect(NimBLEServer *pServer, NimBLEConnInfo &connInfo) override
  {
    ESP_LOGI(LOG_TAG_BLESERVER, "Client connected: %s", connInfo.getAddress().toString().c_str());
    ESP_LOGI(LOG_TAG_BLESERVER, "Multi-connect support: start advertising");
    deviceConnected = true;
    NimBLEDevice::startAdvertising();
  }

  void onDisconnect(NimBLEServer *pServer, NimBLEConnInfo &connInfo, int reason) override
  {
    ESP_LOGI(LOG_TAG_BLESERVER, "Client disconnected - start advertising");
    deviceConnected = false;
    NimBLEDevice::startAdvertising();
  }

  void onMTUChange(uint16_t MTU, NimBLEConnInfo &connInfo) override
  {
    ESP_LOGI(LOG_TAG_BLESERVER, "MTU changed - new size %d, peer %s", MTU, connInfo.getAddress().toString().c_str());
    MTU_SIZE = MTU;
    PACKET_SIZE = MTU_SIZE - 3;
  }
};

char tmpbuf[1024]; // CAUTION: always use a global buffer, local buffer will flood the stack

void dumpBuffer(std::string header, std::string buffer)
{
  if (esp_log_level_get("BleServer") < ESP_LOG_DEBUG)
  {
    return;
  }
  int length = snprintf(tmpbuf, 50, "%s : len = %d / ", header.c_str(), buffer.length());
  for (char i : buffer)
  {
    length += snprintf(tmpbuf + length, 1024 - length, "%02x ", i);
  }
  ESP_LOGD(LOG_TAG_BLESERVER, "%s", tmpbuf);
}

// DIAGNOSTICS: minimal VESC UART protocol implementation, just enough to
// send a COMM_GET_VALUES request directly over Serial1. The VESC stays
// silent until it is asked for something, so periodically sending this
// (while no BLE client is connected, to avoid colliding with real traffic)
// proves whether the UART link to the VESC actually works in both
// directions, independent of BLE/VESC Tool.
uint16_t vescCrc16(const uint8_t *buf, size_t len)
{
  uint16_t crc = 0;
  for (size_t i = 0; i < len; i++)
  {
    crc ^= (uint16_t)buf[i] << 8;
    for (int b = 0; b < 8; b++)
    {
      crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
  }
  return crc;
}

void sendVescGetValuesPing()
{
  const uint8_t COMM_GET_VALUES = 4;
  uint8_t packet[6];
  packet[0] = 0x02; // short packet start byte
  packet[1] = 1;    // payload length
  packet[2] = COMM_GET_VALUES;
  uint16_t crc = vescCrc16(&packet[2], 1);
  packet[3] = (crc >> 8) & 0xFF;
  packet[4] = crc & 0xFF;
  packet[5] = 0x03; // stop byte
  Serial1.write(packet, sizeof(packet));
  ESP_LOGI(LOG_TAG_BLESERVER, "Diagnostic ping sent to VESC (COMM_GET_VALUES)");
}

class MyCallbacks : public NimBLECharacteristicCallbacks
{
  void onWrite(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo) override
  {
    ESP_LOGD(LOG_TAG_BLESERVER, "onWrite to characteristics: %s", pCharacteristic->getUUID().toString().c_str());
    std::string rxValue = pCharacteristic->getValue();
    if (rxValue.length() > 0)
    {
      if (pCharacteristic->getUUID().equals(pCharacteristicVescRx->getUUID()))
      {
        dumpBuffer("BLE/UART => VESC: ", rxValue);
        for (int i = 0; i < rxValue.length(); i++)
        {
          Serial1.write(rxValue[i]);
        }
      }
    }
  }
};

void setup()
{
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, VESC_RX_PIN, VESC_TX_PIN);
  esp_log_level_set(LOG_TAG_BLESERVER, ESP_LOG_DEBUG); // verbose UART/BLE byte dumps on Serial monitor

  // Create the BLE Device
  NimBLEDevice::init("VescBLEBridge");
  NimBLEDevice::setPower(9); // +9 dBm

  // Create the BLE Server
  pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  NimBLEDevice::setSecurityAuth(true, false, false); // bonding, mitm, secure connections

  // Create the BLE Service
  NimBLEService *pService = pServer->createService(VESC_SERVICE_UUID);

  // Create a BLE TX Characteristic
  pCharacteristicVescTx = pService->createCharacteristic(
      VESC_CHARACTERISTIC_UUID_TX,
      NIMBLE_PROPERTY::NOTIFY |
          NIMBLE_PROPERTY::READ);

  // Create a BLE RX Characteristic
  pCharacteristicVescRx = pService->createCharacteristic(
      VESC_CHARACTERISTIC_UUID_RX,
      NIMBLE_PROPERTY::WRITE |
          NIMBLE_PROPERTY::WRITE_NR);

  pCharacteristicVescRx->setCallbacks(new MyCallbacks());

  // Start advertising
  NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(VESC_SERVICE_UUID);
  //    pAdvertising->setAppearance(0x00);
  //    pAdvertising->setScanResponse(true);
  //    pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter

  pAdvertising->start();
  ESP_LOGI(LOG_TAG_BLESERVER, "waiting a client connection to notify...");
}

std::string vescBuffer;
std::string updateBuffer;
unsigned long lastVescPingMs = 0;

void loop()
{

  if (Serial1.available())
  {
    int oneByte;
    while (Serial1.available())
    {
      oneByte = Serial1.read();
      vescBuffer.push_back(oneByte);
    }

    // Always dump raw bytes received from the VESC, regardless of BLE
    // connection state, so wiring/UART issues are visible on the Serial
    // monitor even without VESC Tool connected.
    dumpBuffer("VESC => UART (raw)", vescBuffer);

    if (deviceConnected)
    {
      while (vescBuffer.length() > 0)
      {
        if (vescBuffer.length() > PACKET_SIZE)
        {
          pCharacteristicVescTx->setValue(vescBuffer.substr(0, PACKET_SIZE));
          vescBuffer = vescBuffer.substr(PACKET_SIZE);
        }
        else
        {
          pCharacteristicVescTx->setValue(vescBuffer);
          vescBuffer.clear();
        }
        pCharacteristicVescTx->notify();
        delay(5); // bluetooth stack will go into congestion, if too many packets are sent
      }
    }
    else
    {
      vescBuffer.clear(); // avoid unbounded growth while nobody is connected
    }
  }

  // DIAGNOSTICS: while no BLE client is connected, periodically probe the
  // VESC directly over UART. Remove once the link is confirmed working.
  if (!deviceConnected && millis() - lastVescPingMs > 2000)
  {
    lastVescPingMs = millis();
    sendVescGetValuesPing();
  }

  // disconnecting
  if (!deviceConnected && oldDeviceConnected)
  {
    delay(500);                  // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising(); // restart advertising
    ESP_LOGI(LOG_TAG_BLESERVER, "start advertising");
    oldDeviceConnected = deviceConnected;
  }
  // connecting
  if (deviceConnected && !oldDeviceConnected)
  {
    // do stuff here on connecting
    oldDeviceConnected = deviceConnected;
  }
}