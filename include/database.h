#ifndef DATABASE_H
#define DATABASE_H

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <ArduinoJson.h>
#include <RTClib.h>    // for timestamping; see https://github.com/adafruit/RTClib

// todo: Change this to your SD chip-select pin:
static const uint8_t SD_CS_PIN = 4;

// Filenames on the SD card:
static const char* TEMP_LOG_FILENAME = "temp.csv";
static const char* TDS_LOG_FILENAME  = "tds.csv";

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
