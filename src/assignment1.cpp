/*  assignment1.cpp  –  Arduino Nano 33 IoT port
 *  ------------------------------------------------
 *  Build with:  Arduino IDE / arduino-cli
 *  Required libs:  Wire, ArduinoJson (for db code), ADS1X15 (optional)
 */

#include "assignment1.h"
#include "database.h"
#include <Arduino.h>
#include <Wire.h>

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
    (void)sv;            // unused for now
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
static void setLed(uint8_t r, uint8_t g, uint8_t b)
{
    analogWrite(PIN_SMD_RED, r);
    analogWrite(PIN_SMD_GRN, g);
    analogWrite(PIN_SMD_BLU, b);
}

void body_led(volatile SharedVariable &sv)
{
    switch (sv.sequenceState) {
        case SEQ_IDLE:
            if (sv.detectedHardWater)
                setLed(192, 192, 192);  // dim white
            else
                setLed(0, 0, 0);
            break;
        case SEQ_STEP1:      setLed( 64, 255,  64); break;  // green
        case SEQ_STEP2:
        case SEQ_STEP3:      setLed( 64,  64, 255); break;  // blue
        case SEQ_STEP4:      setLed(255, 255,   0); break;  // yellow
        case SEQ_STEP5:      setLed( 64, 255,  64); break;  // green
        case SEQ_STEP6:
        case SEQ_STEP7:      setLed(255, 165,   0); break;  // orange
        case SEQ_FINISHED:   setLed(  0, 255,   0); break;  // bright green
        case SEQ_FAILURE_DETECTED:
                             setLed(255,   0,   0); break;  // red
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
    /*  ADS1115 single-shot, AIN0-GND, ±2.048 V, 128 SPS                 */
    constexpr uint16_t cfg = 0b1100010110000011;   // OS=1, MUX=100, PGA=010, MODE=1, DR=100, COMP_QUE=11

    adsWrite16(ADS_REG_CFG, cfg);
    delay(10);                                      // wait for conversion
    uint16_t raw = adsRead16(ADS_REG_CONV);

    /*  swap bytes (ADS is big-endian)                                   */
    raw = (raw >> 8) | (raw << 8);
    if (raw & 0x8000) raw = 0;                      // exclude negatives

    const float voltage = (float)raw * 2.048f / 32767.0f;
    float tds           = voltage * 1221.0f;

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
void body_ultrasonic(volatile SharedVariable &sv)
{
    /*  Send 10 µs pulse  */
    digitalWrite(PIN_ULTRASONIC_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(PIN_ULTRASONIC_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_ULTRASONIC_TRIG, LOW);

    const unsigned long dur = pulseIn(PIN_ULTRASONIC_ECHO, HIGH, 30000UL); // 30 ms max
    if (dur == 0) return;                           // timeout → ignore

    const double distance = dur * 0.0343 / 2.0;     // cm

    sv.waterLevelAboveThreshold = (distance < WATER_LEVEL_HIGH_THRESHOLD_CM);
    sv.waterLevelBelowThreshold = (distance > WATER_LEVEL_LOW_THRESHOLD_CM);

    if (distance > MAX_DISTANCE_CM * 0.9)
        sv.sequenceState = SEQ_FAILURE_DETECTED;
}

/* ------------------------------------------------------------------ */
/*  ───── TEMPERATURE (placeholder)                                   */
/* ------------------------------------------------------------------ */
void body_persistent_temperature(volatile SharedVariable &sv)
{
    /*  TODO: Replace with OneWire + DallasTemperature                   */
    static unsigned long last = 0;
    if (millis() - last < 2000) return;             // every 2 s
    last = millis();

    sv.temperature = 25.0f;                         // stub
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
