/**
 * database.cpp – In-memory logging back-end for Arduino Nano 33 BLE
 *    - Stores temperature and TDS readings in circular buffers
 *    - Can retrieve the last 24 h of data as a JSON array
 *    - Uses SRAM storage (data lost on reset, but dependency-free)
 */

#include "data_logger.h"
#include <Arduino.h>
#include <ArduinoJson.h>

// -----------------------------------------------------------------------------
// CIRCULAR BUFFER STORAGE
// -----------------------------------------------------------------------------
static Reading tempBuffer[BUFFER_SIZE];
static Reading tdsBuffer[BUFFER_SIZE];
static size_t tempWriteIndex = 0;
static size_t tdsWriteIndex = 0;
static size_t tempCount = 0;
static size_t tdsCount = 0;

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
static void addReading(Reading* buffer, size_t& writeIndex, size_t& count, float value)
{
    buffer[writeIndex].timestamp = nowUnix();
    buffer[writeIndex].value = value;
    
    writeIndex = (writeIndex + 1) % BUFFER_SIZE;
    if (count < BUFFER_SIZE) {
        count++;
    }
}

static bool buildLast24hJson(const Reading* buffer, size_t writeIndex, size_t count,
                             const char* valueKey, char* jsonBuffer, size_t maxSize)
{
    JsonDocument doc;
    JsonArray arr = doc.to<JsonArray>();
    
    const time_t cutoff = nowUnix() - 24UL * 60UL * 60UL;
    
    // Read from newest to oldest
    for (size_t i = 0; i < count; i++) {
        // Calculate actual index (newest first)
        size_t actualIndex;
        if (count < BUFFER_SIZE) {
            // Buffer not full yet, start from beginning
            actualIndex = (count - 1 - i);
        } else {
            // Buffer is full, start from writeIndex-1 (newest)
            actualIndex = (writeIndex - 1 - i + BUFFER_SIZE) % BUFFER_SIZE;
        }
        
        const Reading& reading = buffer[actualIndex];
        
        // Skip readings older than 24 hours
        if (reading.timestamp < cutoff) continue;
        
        JsonObject obj = arr.add<JsonObject>();
        obj["time"] = reading.timestamp;
        obj[valueKey] = reading.value;
    }
    
    // Serialize to supplied buffer
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
    // Initialize circular buffer indices
    tempWriteIndex = 0;
    tdsWriteIndex = 0;
    tempCount = 0;
    tdsCount = 0;
    
    // Clear buffers
    memset(tempBuffer, 0, sizeof(tempBuffer));
    memset(tdsBuffer, 0, sizeof(tdsBuffer));
    
    Serial.println(F("[DB] In-memory database initialized"));
    Serial.print(F("[DB] Buffer size: "));
    Serial.print(BUFFER_SIZE);
    Serial.println(F(" readings per sensor"));
    
    return true;
}

void close_database()
{
    // Nothing to do for in-memory storage
}

bool add_temperature_reading(float temperature)
{
    addReading(tempBuffer, tempWriteIndex, tempCount, temperature);
    return true;
}

bool add_tds_reading(float tdsValue)
{
    addReading(tdsBuffer, tdsWriteIndex, tdsCount, tdsValue);
    return true;
}

bool get_last_24h_data(char* jsonBuffer, size_t maxSize)
{
    return buildLast24hJson(tempBuffer, tempWriteIndex, tempCount, 
                            "temperature", jsonBuffer, maxSize);
}

bool get_last_24h_tds_data(char* jsonBuffer, size_t maxSize)
{
    return buildLast24hJson(tdsBuffer, tdsWriteIndex, tdsCount, 
                            "tds", jsonBuffer, maxSize);
}