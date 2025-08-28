#include "bluetooth_service.h"
#include "sensor_manager.h"
#include <ArduinoBLE.h>


// --- UUIDs ---
BLEService waterSoftenerService("12345678-1234-5678-9abc-def012345678");

BLECharacteristic tdsCharacteristic(
  "12345678-1234-5678-9abc-def012345679",
  BLERead | BLENotify, 4
);

BLECharacteristic temperatureCharacteristic(
  "12345678-1234-5678-9abc-def01234567a",
  BLERead | BLENotify, 4
);

// Descriptors for characteristic names
BLEDescriptor tdsDescriptor("2901", "TDS (ppm)");
BLEDescriptor temperatureDescriptor("2901", "Temperature (°C)");

// Connection status
static volatile bool centralConnected = false;

// Forward-declare handlers
void onBLEConnected(BLEDevice central);
void onBLEDisconnected(BLEDevice central);

bool initBluetoothService(const char* deviceName)
{
  // Start BLE right away
  if (!BLE.begin()) {
    Serial.println(F("[BLE] Failed to initialize BLE!"));
    // Blink fast forever so you can *see* this case
    pinMode(LED_BUILTIN, OUTPUT);
    while (1) { digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); delay(150); }
    return false;
  }

  // Short, ASCII-only name strongly recommended
  BLE.setLocalName(deviceName);
  BLE.setDeviceName(deviceName);

  // Service + characteristics
  BLE.setAdvertisedService(waterSoftenerService);
  
  // Add descriptors to characteristics
  tdsCharacteristic.addDescriptor(tdsDescriptor);
  temperatureCharacteristic.addDescriptor(temperatureDescriptor);
  
  waterSoftenerService.addCharacteristic(tdsCharacteristic);
  waterSoftenerService.addCharacteristic(temperatureCharacteristic);
  BLE.addService(waterSoftenerService);

  // Seed values
  float initialTDS  = 0.0f;
  float initialTemp = 25.0f;
  tdsCharacteristic.writeValue((byte*)&initialTDS,  sizeof(float));
  temperatureCharacteristic.writeValue((byte*)&initialTemp, sizeof(float));

  // Event handlers (simpler than polling for central)
  BLE.setEventHandler(BLEConnected,    onBLEConnected);
  BLE.setEventHandler(BLEDisconnected, onBLEDisconnected);

  // Advertise now
  BLE.advertise();

  Serial.print(F("[BLE] Advertising as: ")); Serial.println(deviceName);
  Serial.println(F("[BLE] Service UUID:       12345678-1234-5678-9abc-def012345678"));
  Serial.println(F("[BLE] TDS Char UUID:      12345678-1234-5678-9abc-def012345679"));
  Serial.println(F("[BLE] Temp Char UUID:     12345678-1234-5678-9abc-def01234567a"));

  return true;
}

// Keep this small and quick; call it very frequently
void handleBluetoothEvents()
{
  // Track poll() timing
  static unsigned long lastPollTime = 0;
  static unsigned long pollCount = 0;
  static unsigned long maxInterval = 0;
  static unsigned long totalInterval = 0;
  
  unsigned long currentTime = millis();
  if (lastPollTime > 0) {
    unsigned long interval = currentTime - lastPollTime;
    totalInterval += interval;
    if (interval > maxInterval) {
      maxInterval = interval;
    }
  }
  
  BLE.poll(); // keep the stack breathing
  
  pollCount++;
  lastPollTime = currentTime;
  
  // Report stats every 5 seconds
  static unsigned long lastReport = 0;
  if (currentTime - lastReport >= 5000) {
    lastReport = currentTime;
    if (pollCount > 0) {
      unsigned long avgInterval = totalInterval / (pollCount - 1);
      Serial.print(F("[BLE_TIMING] Poll count: ")); Serial.print(pollCount);
      Serial.print(F(", Avg interval: ")); Serial.print(avgInterval); Serial.print(F("ms"));
      Serial.print(F(", Max interval: ")); Serial.print(maxInterval); Serial.print(F("ms"));
      Serial.print(F(", Frequency: ~")); Serial.print(1000.0 / avgInterval); Serial.println(F("Hz"));
    }
    // Reset stats for next period
    pollCount = 1;
    totalInterval = 0;
    maxInterval = 0;
  }

  // Heartbeat while advertising (1 Hz)
  static unsigned long last = 0;
  if (!centralConnected && (millis() - last > 1000)) {
    last = millis();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }

  // Push data only when connected
  if (centralConnected) {
    extern volatile SharedVariable sv;
    updateTDSCharacteristic(sv.tds_reading);
    updateTemperatureCharacteristic(sv.temperature);
  }
}

// Event handlers
void onBLEConnected(BLEDevice central) {
  centralConnected = true;
  digitalWrite(LED_BUILTIN, HIGH); // solid on when connected
  Serial.print(F("[BLE] Connected to central: "));
  Serial.println(central.address());
}

void onBLEDisconnected(BLEDevice central) {
  centralConnected = false;
  digitalWrite(LED_BUILTIN, LOW);
  Serial.println(F("[BLE] Disconnected"));
  BLE.advertise(); // restart advertising immediately
  Serial.println(F("[BLE] Restarted advertising"));
}

// Characteristic updates (unchanged)
void updateTDSCharacteristic(float tdsValue) {
  if (centralConnected) {
    tdsCharacteristic.writeValue((byte*)&tdsValue, sizeof(float));
  }
}

void updateTemperatureCharacteristic(float temperature) {
  if (centralConnected) {
    temperatureCharacteristic.writeValue((byte*)&temperature, sizeof(float));
  }
}
