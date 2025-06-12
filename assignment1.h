#ifndef _ASSIGNMENT_BODY_
#define _ASSIGNMENT_BODY_

#include <stdint.h>
#include <wiringPi.h>
#include <softPwm.h>
#include <sys/time.h>
#include <stdbool.h>
#include <string.h>
#include <glob.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "database.h"

#define DEBUG 1

// Macros for simple time
#define SECONDS 1000
#define MINUTES 60 * SECONDS

// Macros for relay control
#define TURN_ON(pin)  digitalWrite((pin), 1)
#define TURN_OFF(pin) digitalWrite((pin), 0)

// Pin definitions
#define PIN_BUTTON              0

#define PIN_ULTSON_BUCKET_TRIG  11
#define PIN_ULTSON_BUCKET_ECHO  31

#define PIN_SMD_RED             13  // artifically swapped with green
#define PIN_SMD_GRN             14  // artifically swapped with red
#define PIN_SMD_BLU             12

#define PIN_RELAY_3             29          // do not change
#define PIN_RELAY_2             28          // do not change
#define PIN_RELAY_1             25          // do not change
#define PIN_RELAY_N_O_VALVE     PIN_RELAY_3
#define PIN_RELAY_3_WAY_VALVE   PIN_RELAY_1
#define PIN_RELAY_PUMP          PIN_RELAY_2

// Unsorted definitions
#define WATER_LEVEL_HIGH_THRESHOLD_CM 10 // distance in CM from sensor (smaller -> more water)
#define WATER_LEVEL_LOW_THRESHOLD_CM 25 // distance in CM from sensor (larger -> less water)
#define HARD_WATER_TDS_THRESHOLD 1500 // tds


// Sequence states for the relay sequence state machine.
typedef enum {
    SEQ_IDLE,      // No sequence running.
    SEQ_STEP1,     // 3-way valve enabled.
    SEQ_STEP2,     // NO valve enabled.
    SEQ_STEP3,     // Pump enabled.
    SEQ_STEP4,     // 3-way valve off for 20 sec.
    SEQ_STEP5,     // NO valve & pump off; 3-way valve on for 30 sec.
    SEQ_STEP6,     // NO valve and 3-way valve on for 5 sec.
    SEQ_STEP7,     // Pump on for 20 sec.
    SEQ_FINISHED,  // keep blocker on 
    SEQ_FAILURE_DETECTED    // when a fault is detected
} SequenceState;

// Shared variable structure for inter-task communication.
typedef struct shared_variable {
    int bProgramExit;               // Program termination flag.
    SequenceState sequenceState;    // Current state of the sequence.
    unsigned long sequenceTimestamp;// Timestamp when the current state started.
    int cancelSequence;             // Flag to cancel the sequence.
    bool waterLevelAboveThreshold;
    bool waterLevelBelowThreshold;
    float temperature;
    float tds_reading;
    bool detectedHardWater;
} SharedVariable;

// Declare the shared variable (use volatile if needed).
extern volatile SharedVariable sv;

// Function declarations.
void init_shared_variable(volatile SharedVariable* sv);
void init_sensors(volatile SharedVariable* sv);
void body_button(volatile SharedVariable* sv);   // Button: start or cancel sequence.
void body_led(volatile SharedVariable* sv);      // RGB LED: display sequence state.
void body_sequence(volatile SharedVariable* sv); // State machine for relay sequence.
void body_tds(volatile SharedVariable* sv);      // TDS sensor reading
void body_ultrasonic(volatile SharedVariable* sv); // ultrasonic bucket water level height reading
void body_persistent_temperature(volatile SharedVariable* sv); // temperature sensor
void body_persistent_log_to_database(volatile SharedVariable* sv); // persistent logging

#endif



