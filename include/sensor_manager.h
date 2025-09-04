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
static const uint8_t PIN_BUTTON             = D12;

static const uint8_t PIN_ULTRASONIC_TRIG    = D6;
static const uint8_t PIN_ULTRASONIC_ECHO    = D7;

static const uint8_t PIN_TEMP_ONEWIRE       = D5;  // Onewire, Nano ESP32 D5 converted to GPIO 8

static const uint8_t PIN_TDS_ANALOG         = A0;  // TDS sensor analog input

// Outputs
static const uint8_t PIN_SMD_RED            = D9;   // PWM Supported
static const uint8_t PIN_SMD_GRN            = D8;   // PWM Supported
static const uint8_t PIN_SMD_BLU            = D10;  // PWM Supported
static const uint8_t PIN_SMD_WHT            = D11;  // PWM Supported

static const uint8_t PIN_RELAY_3            = D4;
static const uint8_t PIN_RELAY_2            = D3;
static const uint8_t PIN_RELAY_1            = D2;
static const uint8_t PIN_RELAY_N_O_VALVE    = PIN_RELAY_3;
static const uint8_t PIN_RELAY_3_WAY_VALVE  = PIN_RELAY_1;
static const uint8_t PIN_RELAY_PUMP         = PIN_RELAY_2;

// Water-level and TDS thresholds
#define WATER_LEVEL_HIGH_THRESHOLD_CM   10  
#define WATER_LEVEL_LOW_THRESHOLD_CM    25  
#define WATER_LEVEL_MAX_DISTANCE_CM     50UL
#define ULTRASONIC_READ_INTERVAL_MS     250UL
#define MAX_ECHO_ULTRASONIC             ((WATER_LEVEL_MAX_DISTANCE_CM * 58UL * 12UL) / 10UL)  // ceil(cm*58*1.2) (20% margin)
#define HARD_WATER_TDS_THRESHOLD        1500
#define ULTRASONIC_FAILURE_THRESHOLD    10   // Switch to failure state after 10 consecutive failures  

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
    volatile uint16_t ultrasonicFailureCount;
};

extern volatile SharedVariable sv;

// Function prototypes
void init_shared_variable(volatile SharedVariable &sv);
void init_sensors(volatile SharedVariable &sv);
void ultrasonic_init();

void body_button(volatile SharedVariable &sv);
void body_led(volatile SharedVariable &sv);
void body_sequence(volatile SharedVariable &sv);
void body_tds(volatile SharedVariable &sv);
void body_ultrasonic(volatile SharedVariable &sv);
void body_temperature(volatile SharedVariable &sv);
void body_persistent_log_to_database(volatile SharedVariable &sv);

#endif // ASSIGNMENT1_H
