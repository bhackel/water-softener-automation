#ifndef BLUETOOTH_SERVICE_H
#define BLUETOOTH_SERVICE_H

#include <Arduino.h>

// Initialize Bluetooth service
bool initBluetoothService(const char* deviceName);

// Update sensor data for Bluetooth characteristics
void updateTDSCharacteristic(float tdsValue);
void updateTemperatureCharacteristic(float temperature);

// Handle Bluetooth events (call in main loop)
void handleBluetoothEvents();

// Check if a central device is connected
bool isBluetoothConnected();

#endif // BLUETOOTH_SERVICE_H