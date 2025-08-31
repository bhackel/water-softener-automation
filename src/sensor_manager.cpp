/*  assignment1.cpp  –  Arduino Nano 33 IoT port
 *  ------------------------------------------------
 *  Build with:  Arduino IDE / arduino-cli
 *  Required libs:  Wire, ArduinoJson (for db code), ADS1X15 (optional)
 */

#include "sensor_manager.h"
#include "data_logger.h"
#include <Arduino.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>

/* ------------------------------------------------------------------ */
/*  ───── helpers: ADS1115 (bare-metal, uses 0x48)                     */
/* ------------------------------------------------------------------ */
namespace {
    constexpr uint8_t ADS_ADDR      = 0x48;
    constexpr uint8_t ADS_REG_CONV  = 0x00;
    constexpr uint8_t ADS_REG_CFG   = 0x01;

    void adsWrite16(uint8_t reg, uint16_t val)
    {
        Wire.beginTransmission(ADS_ADDR);
        Wire.write(reg);
        Wire.write(val >> 8);
        Wire.write(val & 0xFF);
        Wire.endTransmission();
    }

    uint16_t adsRead16(uint8_t reg)
    {
        Wire.beginTransmission(ADS_ADDR);
        Wire.write(reg);
        Wire.endTransmission(false);          // restart
        Wire.requestFrom(ADS_ADDR, (uint8_t)2);

        uint16_t hi = Wire.read();
        uint16_t lo = Wire.read();
        return (hi << 8) | lo;
    }
}

/* ------------------------------------------------------------------ */
/*  ───── OneWire Temperature Sensor Setup                            */
/* ------------------------------------------------------------------ */
namespace {
    OneWire oneWire(PIN_TEMP_ONEWIRE);
    DallasTemperature temperatureSensor(&oneWire);
    DeviceAddress tempSensorAddress;
    bool tempSensorFound = false;
}

/* ------------------------------------------------------------------ */
/*  Shared state                                                       */
/* ------------------------------------------------------------------ */
volatile SharedVariable sv;

/* ------------------------------------------------------------------ */
/*  Initialisation                                                     */
/* ------------------------------------------------------------------ */
void init_shared_variable(volatile SharedVariable &sv)
{
    sv.bProgramExit           = 0;
    sv.sequenceState          = SEQ_IDLE;
    sv.sequenceTimestamp      = 0;
    sv.cancelSequence         = 0;
    sv.waterLevelAboveThreshold = false;
    sv.waterLevelBelowThreshold = false;
    sv.temperature            = 0.0f;
    sv.tds_reading            = 0.0f;
    sv.detectedHardWater      = false;
}

static void ledInit()
{
    pinMode(PIN_SMD_RED, OUTPUT);
    pinMode(PIN_SMD_GRN, OUTPUT);
    pinMode(PIN_SMD_BLU, OUTPUT);
}

void init_sensors(volatile SharedVariable &sv)
{
    static_cast<void>(sv);  // unused for now
    ledInit();

    pinMode(PIN_BUTTON,               INPUT_PULLUP);

    pinMode(PIN_ULTRASONIC_TRIG,   OUTPUT);
    pinMode(PIN_ULTRASONIC_ECHO,   INPUT);

    pinMode(PIN_RELAY_3_WAY_VALVE,    OUTPUT);
    pinMode(PIN_RELAY_N_O_VALVE,      OUTPUT);
    pinMode(PIN_RELAY_PUMP,           OUTPUT);

    TURN_OFF(PIN_RELAY_3_WAY_VALVE);
    TURN_OFF(PIN_RELAY_N_O_VALVE);
    TURN_OFF(PIN_RELAY_PUMP);

    Wire.begin();       // start I²C for ADS1115
    
    // Initialize OneWire temperature sensor
    temperatureSensor.begin();
    temperatureSensor.setWaitForConversion(false); // Non-blocking mode
    
    // Search for temperature sensor
    if (temperatureSensor.getDeviceCount() > 0) {
        if (temperatureSensor.getAddress(tempSensorAddress, 0)) {
            temperatureSensor.setResolution(tempSensorAddress, 12); // 12-bit precision
            tempSensorFound = true;
            Serial.println(F("[TEMP] DS18B20 sensor found and configured"));
        } else {
            Serial.println(F("[TEMP] DS18B20 sensor found but address read failed"));
        }
    } else {
        Serial.println(F("[TEMP] No DS18B20 sensors found on OneWire bus"));
    }
}

/* ------------------------------------------------------------------ */
/*  ───── BUTTON TASK                                                 */
/* ------------------------------------------------------------------ */
void body_button(volatile SharedVariable &sv)
{
    static int  lastState            = HIGH;
    static bool longPressHandled     = false;
    static unsigned long pressStart  = 0;

    int current = digitalRead(PIN_BUTTON);

    if (current == LOW && lastState == HIGH) {          // press began
        pressStart        = millis();
        longPressHandled  = false;
    }

    if (current == LOW) {                               // holding
        if (!longPressHandled && millis() - pressStart >= 1000) {
            sv.cancelSequence     = 1;
            sv.detectedHardWater  = false;
            longPressHandled      = true;
            Serial.println(F("[BTN] Long-press → cancel."));
        }
    }

    if (current == HIGH && lastState == LOW) {          // released
        if (!longPressHandled) {                        // short press
            if (sv.sequenceState == SEQ_IDLE) {
                sv.sequenceState     = SEQ_STEP1;
                sv.sequenceTimestamp = millis();
                sv.cancelSequence    = 0;
                sv.detectedHardWater = false;
                Serial.println(F("[BTN] Sequence started."));
            } else if (sv.sequenceState == SEQ_FINISHED) {
                sv.cancelSequence = 1;
            } else {
                sv.sequenceState     = (SequenceState)(sv.sequenceState + 1);
                sv.sequenceTimestamp = millis();
                Serial.print  (F("[BTN] Advance to step "));
                Serial.println((int)sv.sequenceState);
            }
        }
        delay(50);      // debounce
    }

    lastState = current;
}

/* ------------------------------------------------------------------ */
/*  ───── LED                                                         */
/* ------------------------------------------------------------------ */
static void setLed(uint8_t r, uint8_t g, uint8_t b, uint8_t w = 0)
{
    analogWrite(PIN_SMD_RED, r);
    analogWrite(PIN_SMD_GRN, g);
    analogWrite(PIN_SMD_BLU, b);
    analogWrite(PIN_SMD_WHT, w);
}

void body_led(volatile SharedVariable &sv)
{
    switch (sv.sequenceState) {
        case SEQ_IDLE:
            if (sv.detectedHardWater)
                setLed(180, 140, 140, 80);  // light pinkish-white
            else
                setLed(0, 0, 0, 40);        // small white for idle
            break;
        case SEQ_STEP1:      setLed(  0, 180,  50, 50); break;  // light green
        case SEQ_STEP2:
        case SEQ_STEP3:      setLed(  0,  50, 180, 50); break;  // light blue
        case SEQ_STEP4:      setLed(180, 180,   0, 60); break;  // light yellow
        case SEQ_STEP5:      setLed(  0, 180,  50, 50); break;  // light green
        case SEQ_STEP6:
        case SEQ_STEP7:      setLed(180, 120,   0, 40); break;  // light orange
        case SEQ_FINISHED:   setLed(  0, 200,   0, 80); break;  // bright light green
        case SEQ_FAILURE_DETECTED:
                             setLed(200,   0,   0, 30); break;  // light red
        default: break;
    }
}

/* ------------------------------------------------------------------ */
/*  ───── SEQUENCE STATE-MACHINE                                     */
/* ------------------------------------------------------------------ */
static void reset_sequence(volatile SharedVariable &sv)
{
    TURN_OFF(PIN_RELAY_3_WAY_VALVE);
    TURN_OFF(PIN_RELAY_N_O_VALVE);
    TURN_OFF(PIN_RELAY_PUMP);
    sv.sequenceState  = SEQ_IDLE;
    sv.cancelSequence = 0;
    Serial.println(F("[SEQ] Reset."));
}

void body_sequence(volatile SharedVariable &sv)
{
    const unsigned long now = millis();

    if (sv.cancelSequence) {
        reset_sequence(sv);
        return;
    }

    switch (sv.sequenceState) {
        case SEQ_IDLE:
            TURN_OFF(PIN_RELAY_3_WAY_VALVE);
            TURN_OFF(PIN_RELAY_N_O_VALVE);
            TURN_OFF(PIN_RELAY_PUMP);
            break;

        case SEQ_STEP1:
            TURN_ON(PIN_RELAY_3_WAY_VALVE);
            if ((now - sv.sequenceTimestamp >=  1 * MINUTES) || sv.waterLevelAboveThreshold) {
                sv.sequenceState     = SEQ_STEP2;
                sv.sequenceTimestamp = now;
            }
            break;

        case SEQ_STEP2:
            TURN_ON(PIN_RELAY_N_O_VALVE);
            if (now - sv.sequenceTimestamp >=  3 * SECONDS) {
                sv.sequenceState     = SEQ_STEP3;
                sv.sequenceTimestamp = now;
            }
            break;

        case SEQ_STEP3:
            TURN_ON(PIN_RELAY_PUMP);
            if (now - sv.sequenceTimestamp >= 12 * MINUTES) {
                sv.sequenceState     = SEQ_STEP4;
                sv.sequenceTimestamp = now;
            }
            break;

        case SEQ_STEP4:
            TURN_OFF(PIN_RELAY_3_WAY_VALVE);
            if ((now - sv.sequenceTimestamp >=  5 * MINUTES) || sv.waterLevelBelowThreshold) {
                sv.sequenceState     = SEQ_STEP5;
                sv.sequenceTimestamp = now;
            }
            break;

        case SEQ_STEP5:
            TURN_OFF(PIN_RELAY_N_O_VALVE);
            TURN_OFF(PIN_RELAY_PUMP);
            TURN_ON (PIN_RELAY_3_WAY_VALVE);
            if ((now - sv.sequenceTimestamp >=  1 * MINUTES) || sv.waterLevelAboveThreshold) {
                sv.sequenceState     = SEQ_STEP6;
                sv.sequenceTimestamp = now;
            }
            break;

        case SEQ_STEP6:
            TURN_ON(PIN_RELAY_N_O_VALVE);
            TURN_ON(PIN_RELAY_3_WAY_VALVE);
            if (now - sv.sequenceTimestamp >=  5 * SECONDS) {
                sv.sequenceState     = SEQ_STEP7;
                sv.sequenceTimestamp = now;
            }
            break;

        case SEQ_STEP7:
            TURN_OFF(PIN_RELAY_3_WAY_VALVE);
            TURN_ON (PIN_RELAY_PUMP);
            if ((now - sv.sequenceTimestamp >=  5 * MINUTES) || sv.waterLevelBelowThreshold) {
                sv.sequenceState     = SEQ_FINISHED;
                sv.sequenceTimestamp = now;
            }
            break;

        case SEQ_FINISHED:
            TURN_OFF(PIN_RELAY_3_WAY_VALVE);
            TURN_ON (PIN_RELAY_N_O_VALVE);
            TURN_OFF(PIN_RELAY_PUMP);
            if (now - sv.sequenceTimestamp >= 15 * MINUTES) {
                sv.cancelSequence = 1;
            }
            break;

        case SEQ_FAILURE_DETECTED:
            TURN_OFF(PIN_RELAY_3_WAY_VALVE);
            TURN_OFF(PIN_RELAY_N_O_VALVE);
            TURN_OFF(PIN_RELAY_PUMP);
            break;
    }
}

/* ------------------------------------------------------------------ */
/*  ───── TDS SENSOR                                                 */
/* ------------------------------------------------------------------ */
void body_tds(volatile SharedVariable &sv)
{
    // Read directly from Arduino analog pin A0 (0-3.3V, 12-bit resolution)
    uint16_t raw = analogRead(PIN_TDS_ANALOG);
    
    // Convert to voltage (Nano 33 BLE: 12-bit ADC, 3.3V reference)
    const float voltage = (float)raw * 3.3f / 4095.0f;
    
    // Convert voltage to TDS - this needs calibration with your 330PPM solution
    float tds = voltage * 1000.0f;  // Starting conversion factor - needs calibration

    // Apply temperature compensation (when temperature data is valid)
    if (sv.temperature >= 0.0f && sv.temperature <= 50.0f) {
        const float coeff = 1.0f + 0.02f * (sv.temperature - 25.0f);
        tds /= coeff;
    }

    sv.tds_reading = tds;
    if (tds > HARD_WATER_TDS_THRESHOLD) sv.detectedHardWater = true;
}

/* ------------------------------------------------------------------ */
/*  ───── ULTRASONIC BUCKET LEVEL                                    */
/* ------------------------------------------------------------------ */
enum UltrasonicState { ULTRASONIC_IDLE=0, ULTRASONIC_WAIT_HIGH=1, ULTRASONIC_WAIT_LOW=2 };
static UltrasonicState ultrasonic_state = ULTRASONIC_IDLE;
static unsigned long ultrasonic_next_ms = 0;
static unsigned long ultrasonic_t_start = 0;
static unsigned long ultrasonic_rise = 0;

void body_ultrasonic(volatile SharedVariable &sv) {
  const unsigned long now_ms = millis();
  const unsigned long now_us = micros();

  switch (ultrasonic_state) {
    case ULTRASONIC_IDLE: {
      if (now_ms < ultrasonic_next_ms) return;

      // 10 µs trigger (no busy loops beyond 12 µs)
      digitalWrite(PIN_ULTRASONIC_TRIG, LOW);  delayMicroseconds(2);
      digitalWrite(PIN_ULTRASONIC_TRIG, HIGH); delayMicroseconds(10);
      digitalWrite(PIN_ULTRASONIC_TRIG, LOW);

      ultrasonic_t_start = micros();
      ultrasonic_state   = ULTRASONIC_WAIT_HIGH;
      ultrasonic_next_ms = now_ms + US_RATE_MS;  // schedule next measurement cycle
      return;
    }

    case ULTRASONIC_WAIT_HIGH: {
      if (digitalRead(PIN_ULTRASONIC_ECHO)) {
        ultrasonic_rise  = micros();
        ultrasonic_state = ULTRASONIC_WAIT_LOW;
        return;
      }
      if ((unsigned long)(micros() - ultrasonic_t_start) >= MAX_ECHO_ULTRASONIC) {
        // No echo started within expected window → skip this cycle (don’t fail immediately)
        ultrasonic_state = ULTRASONIC_IDLE;
      }
      return;
    }

    case ULTRASONIC_WAIT_LOW: {
      if (!digitalRead(PIN_ULTRASONIC_ECHO)) {
        const unsigned long fall = micros();
        const unsigned long width = (fall >= ultrasonic_rise) ? (fall - ultrasonic_rise) : 0UL;

        // Convert µs → cm with integer math (rounded): cm ≈ width/58
        const unsigned long cm = (width + 29UL) / 58UL;

        // Update your flags (no floats needed)
        sv.waterLevelAboveThreshold = (cm <  WATER_LEVEL_HIGH_THRESHOLD_CM);
        sv.waterLevelBelowThreshold = (cm >  WATER_LEVEL_LOW_THRESHOLD_CM);

        // Optional: treat near-limit as a soft warning instead of instant failure
        // if (cm > (MAX_DISTANCE_CM * 9UL) / 10UL) { /* warn */ }

        ultrasonic_state = ULTRASONIC_IDLE;
        return;
      }

      // Echo stayed high too long; clamp to max and finish the cycle
      if ((unsigned long)(micros() - ultrasonic_rise) >= MAX_ECHO_ULTRASONIC) {
        const unsigned long cm = (MAX_ECHO_ULTRASONIC + 29UL) / 58UL;
        sv.waterLevelAboveThreshold = (cm <  WATER_LEVEL_HIGH_THRESHOLD_CM);
        sv.waterLevelBelowThreshold = (cm >  WATER_LEVEL_LOW_THRESHOLD_CM);
        ultrasonic_state = ULTRASONIC_IDLE;
      }
      return;
    }
  }
}

/* ------------------------------------------------------------------ */
/*  ───── TEMPERATURE (non-blocking OneWire DS18B20)                  */
/* ------------------------------------------------------------------ */
enum TemperatureState { TEMP_IDLE = 0, TEMP_REQUEST = 1, TEMP_WAIT = 2, TEMP_READ = 3 };
static TemperatureState temp_state = TEMP_IDLE;
static unsigned long temp_next_ms = 0;
static unsigned long temp_conversion_start = 0;

void body_persistent_temperature(volatile SharedVariable &sv)
{
    const unsigned long now = millis();
    
    // If no sensor found, use fallback temperature
    if (!tempSensorFound) {
        sv.temperature = 25.0f; // Default room temperature
        return;
    }
    
    switch (temp_state) {
        case TEMP_IDLE:
            // Wait for next reading cycle (every 2 seconds)
            if (now >= temp_next_ms) {
                temp_state = TEMP_REQUEST;
                temp_next_ms = now + 2000; // Schedule next reading in 2 seconds
            }
            break;
            
        case TEMP_REQUEST:
            // Start temperature conversion
            temperatureSensor.requestTemperaturesByAddress(tempSensorAddress);
            temp_conversion_start = now;
            temp_state = TEMP_WAIT;
            break;
            
        case TEMP_WAIT:
            // Wait for conversion to complete (750ms for 12-bit resolution)
            if (now - temp_conversion_start >= 750) {
                temp_state = TEMP_READ;
            }
            break;
            
        case TEMP_READ:
            // Read the temperature value
            if (temperatureSensor.isConversionComplete()) {
                float tempC = temperatureSensor.getTempC(tempSensorAddress);
                
                // Validate temperature reading
                if (tempC != DEVICE_DISCONNECTED_C && tempC >= -55.0f && tempC <= 125.0f) {
                    sv.temperature = tempC;
                } else {
                    // Keep previous temperature on invalid reading
                    Serial.println(F("[TEMP] Invalid reading, keeping previous value"));
                }
            }
            temp_state = TEMP_IDLE;
            break;
    }
}

/* ------------------------------------------------------------------ */
/*  ───── PERIODIC DATABASE LOGGING                                  */
/* ------------------------------------------------------------------ */
void body_persistent_log_to_database(volatile SharedVariable &sv)
{
    static unsigned long last = 0;
    if (millis() - last < 30000) return;            // every 30 s
    last = millis();

    add_tds_reading(sv.tds_reading);
    add_temperature_reading(sv.temperature);

    Serial.print  (F("[DB] Logged – TDS: "));
    Serial.print  (sv.tds_reading, 2);
    Serial.print  (F(" ppm, Temp: "));
    Serial.println(sv.temperature, 2);
}
