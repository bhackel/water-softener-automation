#ifndef ASSIGNMENT1_H
#define ASSIGNMENT1_H

#include <Arduino.h>
#include "data_logger.h"

#define DEBUG 1

// Time macros
#define SECONDS    1000UL
#define MINUTES    (60UL * SECONDS)

// Relay control macros
#define TURN_ON(pin)   digitalWrite((pin), HIGH)
#define TURN_OFF(pin)  digitalWrite((pin), LOW)

// === Pin definitions ===
// Inputs
static const uint8_t PIN_BUTTON             = 12;

static const uint8_t PIN_ULTRASONIC_TRIG    = 6;
static const uint8_t PIN_ULTRASONIC_ECHO    = 7;

static const uint8_t PIN_TEMP_ONEWIRE       = 5;   // OneWire bus for temperature sensor

static const uint8_t PIN_TDS_ANALOG         = A0;  // TDS sensor analog input

// Outputs
static const uint8_t PIN_SMD_RED            = 9;   // PWM Supported
static const uint8_t PIN_SMD_GRN            = 8;   // PWM Supported
static const uint8_t PIN_SMD_BLU            = 10;  // PWM Supported
static const uint8_t PIN_SMD_WHT            = 11;  // PWM Supported

static const uint8_t PIN_RELAY_3            = 4;
static const uint8_t PIN_RELAY_2            = 3;
static const uint8_t PIN_RELAY_1            = 2;
static const uint8_t PIN_RELAY_N_O_VALVE    = PIN_RELAY_3;
static const uint8_t PIN_RELAY_3_WAY_VALVE  = PIN_RELAY_1;
static const uint8_t PIN_RELAY_PUMP         = PIN_RELAY_2;

// Water-level and TDS thresholds
#define WATER_LEVEL_HIGH_THRESHOLD_CM   10  
#define WATER_LEVEL_LOW_THRESHOLD_CM    25  
#define WATER_LEVEL_MAX_DISTANCE_CM     50UL
#define US_RATE_MS                      250UL
#define MAX_ECHO_ULTRASONIC             ((WATER_LEVEL_MAX_DISTANCE_CM * 58UL * 12UL) / 10UL)  // ceil(cm*58*1.2) (20% margin)
#define HARD_WATER_TDS_THRESHOLD        1500  

// Sequence states
enum SequenceState {
    SEQ_IDLE,
    SEQ_STEP1,
    SEQ_STEP2,
    SEQ_STEP3,
    SEQ_STEP4,
    SEQ_STEP5,
    SEQ_STEP6,
    SEQ_STEP7,
    SEQ_FINISHED,
    SEQ_FAILURE_DETECTED
};

// Shared data between tasks
struct SharedVariable {
    volatile bool   bProgramExit;
    volatile SequenceState sequenceState;
    volatile unsigned long sequenceTimestamp;
    volatile bool   cancelSequence;
    volatile bool   waterLevelAboveThreshold;
    volatile bool   waterLevelBelowThreshold;
    volatile float  temperature;
    volatile float  tds_reading;
    volatile bool   detectedHardWater;
};

extern volatile SharedVariable sv;

// Function prototypes
void init_shared_variable(volatile SharedVariable &sv);
void init_sensors(volatile SharedVariable &sv);

void body_button(volatile SharedVariable &sv);
void body_led(volatile SharedVariable &sv);
void body_sequence(volatile SharedVariable &sv);
void body_tds(volatile SharedVariable &sv);
void body_ultrasonic(volatile SharedVariable &sv);
void body_persistent_temperature(volatile SharedVariable &sv);
void body_persistent_log_to_database(volatile SharedVariable &sv);

#endif // ASSIGNMENT1_H
