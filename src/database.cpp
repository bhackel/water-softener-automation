/**
 * database.cpp – SD-card logging back-end for Arduino Nano 33 IoT
 *    - Stores temperature and TDS readings in two CSV files
 *    - Can retrieve the last 24 h of data as a JSON array
 *
 *  Hardware notes
 *  --------------
 *  • SD module wired to SPI        MOSI=11  MISO=12  SCK=13  CS=4 (default)
 *  • A time source is required.  If you do NOT have an RTC, call
 *      syncUnixTime(<now>) from your Wi-Fi-based NTP routine once at boot.
 */

#include "database.h"
#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <ArduinoJson.h>


// SRAM–friendly JSON buffer (≈ 2 kB)
static const size_t   JSON_DOC_CAPACITY  = 2048;

// -----------------------------------------------------------------------------
// SIMPLE TIME SERVICE (RTC-less fallback)
// -----------------------------------------------------------------------------
static time_t g_unixTimeOffset = 0;   // seconds to add to millis()/1000

static time_t nowUnix()
{
    return g_unixTimeOffset + millis() / 1000UL;
}

void syncUnixTime(time_t currentUtc)
{
    g_unixTimeOffset = currentUtc - (millis() / 1000UL);
}

// -----------------------------------------------------------------------------
// INTERNAL HELPERS
// -----------------------------------------------------------------------------
static bool ensureFileExists(const char* path, const char* headerLine)
{
    if (SD.exists(path)) {
        return true;
    }
    File f = SD.open(path, FILE_WRITE);
    if (!f) return false;
    f.print(headerLine);   // e.g. "timestamp,value\n"
    f.close();
    return true;
}

static bool appendReading(const char* path, float value)
{
    File f = SD.open(path, FILE_WRITE);
    if (!f) {
        Serial.println(F("[DB] ERROR: cannot open file for append"));
        return false;
    }
    f.print(nowUnix());
    f.print(',');
    f.println(value, 2);   // two decimal places
    f.close();
    return true;
}

static bool buildLast24hJson(const char* path,
                             const char* valueKey,
                             char* jsonBuffer,
                             size_t maxSize)
{
    File f = SD.open(path, FILE_READ);
    if (!f) {
        Serial.println(F("[DB] ERROR: cannot open file for read"));
        return false;
    }

    StaticJsonDocument<JSON_DOC_CAPACITY> doc;
    JsonArray arr = doc.to<JsonArray>();

    const time_t cutoff = nowUnix() - 24UL * 60UL * 60UL;

    // Re-use one heap string to save RAM
    String line;
    while (f.available()) {
        line = f.readStringUntil('\n');
        // line format:   <timestamp>,<value>
        char* cstr = line.begin();
        char* comma = strchr(cstr, ',');
        if (!comma) continue;

        *comma = '\0';
        unsigned long ts = strtoul(cstr, nullptr, 10);
        if (ts < cutoff) continue;               // skip old rows

        float val = atof(comma + 1);
        JsonObject obj = arr.createNestedObject();
        obj["time"] = ts;
        obj[valueKey] = val;
    }
    f.close();

    // Serialise to supplied buffer
    const size_t written = serializeJson(doc, jsonBuffer, maxSize);
    if (written == 0) {
        Serial.println(F("[DB] WARNING: JSON buffer too small"));
        return false;
    }
    return true;
}

// -----------------------------------------------------------------------------
// PUBLIC API
// -----------------------------------------------------------------------------
bool init_database()
{
    if (!SD.begin(SD_CS_PIN)) {
        Serial.println(F("[DB] SD.begin failed"));
        return false;
    }

    const bool ok1 = ensureFileExists(TEMP_LOG_FILENAME, "timestamp,temperature\n");
    const bool ok2 = ensureFileExists(TDS_LOG_FILENAME,  "timestamp,tds\n");
    return ok1 && ok2;
}

void close_database()
{
    // Nothing to do for SD – kept for interface parity.
}

bool add_temperature_reading(float temperature)
{
    return appendReading(TEMP_LOG_FILENAME, temperature);
}

bool add_tds_reading(float tdsValue)
{
    return appendReading(TDS_LOG_FILENAME, tdsValue);
}

bool get_last_24h_data(char* jsonBuffer, size_t maxSize)
{
    return buildLast24hJson(TEMP_LOG_FILENAME, "temperature",
                            jsonBuffer, maxSize);
}

bool get_last_24h_tds_data(char* jsonBuffer, size_t maxSize)
{
    return buildLast24hJson(TDS_LOG_FILENAME, "tds",
                            jsonBuffer, maxSize);
}
