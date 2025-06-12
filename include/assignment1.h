#ifndef ASSIGNMENT1_H
#define ASSIGNMENT1_H

#include <Arduino.h>
#include "database.h"    // you’ll need to port this too

#define DEBUG 1

// Time macros
#define SECONDS    1000UL
#define MINUTES    (60UL * SECONDS)

// Relay control macros
#define TURN_ON(pin)   digitalWrite((pin), HIGH)
#define TURN_OFF(pin)  digitalWrite((pin), LOW)

// === Pin definitions ===
// todo change according to pcb
static const uint8_t PIN_BUTTON             = 2;

static const uint8_t PIN_ULTRASONIC_TRIG    = 3;
static const uint8_t PIN_ULTRASONIC_ECHO    = 4;

static const uint8_t PIN_SMD_RED            = 9;   // PWM
static const uint8_t PIN_SMD_GRN            = 10;  // PWM
static const uint8_t PIN_SMD_BLU            = 11;  // PWM

static const uint8_t PIN_RELAY_3            = 5;
static const uint8_t PIN_RELAY_2            = 6;
static const uint8_t PIN_RELAY_1            = 7;
static const uint8_t PIN_RELAY_N_O_VALVE    = PIN_RELAY_3;
static const uint8_t PIN_RELAY_3_WAY_VALVE  = PIN_RELAY_1;
static const uint8_t PIN_RELAY_PUMP         = PIN_RELAY_2;

// Water-level and TDS thresholds
#define WATER_LEVEL_HIGH_THRESHOLD_CM   10  
#define WATER_LEVEL_LOW_THRESHOLD_CM    25  
#define MAX_DISTANCE_CM                 50
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
