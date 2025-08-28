#ifndef DATABASE_H
#define DATABASE_H

#include <Arduino.h>
#include <ArduinoJson.h>

// Reading structure for circular buffer storage
struct Reading {
    uint32_t timestamp;
    float value;
};

// Buffer size: 288 readings = 24 hours at 5-minute intervals
static const size_t BUFFER_SIZE = 288;

// Initializes the SD card and RTC. Returns true on success.
bool init_database();

// No-op for SD, but here if you want to close files early.
void close_database();

// Append a line "timestamp,temperature" to temp.csv
bool add_temperature_reading(float temperature);

// Append a line "timestamp,tdsValue" to tds.csv
bool add_tds_reading(float tdsValue);

// Reads the last 24 hours of entries from temp.csv into a JSON array.
// jsonBuffer must be at least maxSize bytes. Returns true on success.
bool get_last_24h_data(char *jsonBuffer, size_t maxSize);

// Same for tds.csv
bool get_last_24h_tds_data(char *jsonBuffer, size_t maxSize);

#endif // DATABASE_H
