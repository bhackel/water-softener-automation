#include "assignment1.h"
#include <stdio.h>
#include <wiringPi.h>
#include <softPwm.h>
#include <wiringPiI2C.h>
#include <time.h>

// Initialize shared variables.
void init_shared_variable(volatile SharedVariable* sv) {
    sv->bProgramExit = 0;
    sv->sequenceState = SEQ_IDLE;
    sv->sequenceTimestamp = 0;
    sv->cancelSequence = 0;
}

// Initialize the RGB LED via software PWM.
void ledInit(void) {
    softPwmCreate(PIN_SMD_RED, 0, 255);
    softPwmCreate(PIN_SMD_GRN, 0, 255);
    softPwmCreate(PIN_SMD_BLU, 0, 255);
}

// Initialize button input and relay outputs.
void init_sensors(volatile SharedVariable* sv) {
    ledInit();
    pinMode(PIN_BUTTON, INPUT);

    pinMode(PIN_ULTSON_BUCKET_TRIG, OUTPUT);
    pinMode(PIN_ULTSON_BUCKET_ECHO, INPUT);

    pinMode(PIN_RELAY_3_WAY_VALVE, OUTPUT);
    pinMode(PIN_RELAY_N_O_VALVE, OUTPUT);
    pinMode(PIN_RELAY_PUMP, OUTPUT);

    // Ensure all relays are off.
    digitalWrite(PIN_RELAY_3_WAY_VALVE, LOW);
    digitalWrite(PIN_RELAY_N_O_VALVE, LOW);
    digitalWrite(PIN_RELAY_PUMP, LOW);
}


void body_button(volatile SharedVariable* sv) {
    static int lastButtonState = HIGH;
    static unsigned long buttonPressStartTime = 0;
    static bool longPressTriggered = false;  // Flag to avoid double-triggering on long press

    int currentButtonState = digitalRead(PIN_BUTTON);

    // Detect when the button is pressed down (transition from HIGH to LOW)
    if (currentButtonState == LOW && lastButtonState == HIGH) {
        buttonPressStartTime = millis();   // Record the time when the button was pressed
        longPressTriggered = false;          // Reset the long press flag
    }

    // While the button is held down, check if it exceeds 1 second
    if (currentButtonState == LOW) {
        // Only check for long press if the sequence is running (i.e. not idle)
        if (sv->sequenceState != SEQ_IDLE && !longPressTriggered &&
            (millis() - buttonPressStartTime >= 1000)) {
            sv->cancelSequence = 1;          // Cancel the sequence immediately
            longPressTriggered = true;         // Mark that long press has been handled
            printf("Sequence cancellation requested (long press).\n");
        }
    }

    // Detect when the button is released (transition from LOW to HIGH)
    if (currentButtonState == HIGH && lastButtonState == LOW) {
        // If long press wasn't triggered, handle as a short press
        if (!longPressTriggered) {
            if (sv->sequenceState == SEQ_IDLE) {
                // Start the sequence if idle
                sv->sequenceState = SEQ_STEP1;
                sv->sequenceTimestamp = millis();
                sv->cancelSequence = 0;
                sv->detectedHardWater = false;
                printf("Sequence started.\n");
            } else {
                // Advance to the next step in the sequence for a short press 💯
                // (Assuming your state machine is set up to handle an incremented state)
                sv->sequenceState++;  
                sv->sequenceTimestamp = millis();
                printf("Sequence advanced to step %d.\n", sv->sequenceState);
            }
        }
        delay(50);  // Simple debounce after release
    }

    lastButtonState = currentButtonState;
}


// LED task: set the RGB LED color based on the current sequence state.
void body_led(volatile SharedVariable* sv) {
    switch (sv->sequenceState) {
        case SEQ_IDLE:
            // Idle
            if (sv->detectedHardWater) {
                // LED dim white.
                softPwmWrite(PIN_SMD_RED, 192);
                softPwmWrite(PIN_SMD_GRN, 192);
                softPwmWrite(PIN_SMD_BLU, 192);
            } else {
                softPwmWrite(PIN_SMD_RED, 0);
                softPwmWrite(PIN_SMD_GRN, 0);
                softPwmWrite(PIN_SMD_BLU, 0);
            }
            break;
        case SEQ_STEP1:
            // 3-way valved switched to fill bucket
            // Green
            softPwmWrite(PIN_SMD_RED, 64);
            softPwmWrite(PIN_SMD_GRN, 255);
            softPwmWrite(PIN_SMD_BLU, 64);
            break;
        case SEQ_STEP2:
            // NO Closed, no more shower water
        case SEQ_STEP3:
            // Pump Enabled
            // Blue
            softPwmWrite(PIN_SMD_RED, 64);
            softPwmWrite(PIN_SMD_GRN, 64);
            softPwmWrite(PIN_SMD_BLU, 255);
            break;
        case SEQ_STEP4:
            // 3-way switched to shower head, dumping water
            // Yellow
            softPwmWrite(PIN_SMD_RED, 255);
            softPwmWrite(PIN_SMD_GRN, 255);
            softPwmWrite(PIN_SMD_BLU, 0);
            break;
        case SEQ_STEP5:
            // Refilling bucket
            // Green
            softPwmWrite(PIN_SMD_RED, 64);
            softPwmWrite(PIN_SMD_GRN, 255);
            softPwmWrite(PIN_SMD_BLU, 64);
            break;
        case SEQ_STEP6:
            // NO valve closed and 3-way in shower mode.
        case SEQ_STEP7:
            // Orange: Pump on to dump waste water
            softPwmWrite(PIN_SMD_RED, 255);
            softPwmWrite(PIN_SMD_GRN, 165);
            softPwmWrite(PIN_SMD_BLU, 0);
            break;
        case SEQ_FAILURE_DETECTED:
            softPwmWrite(PIN_SMD_RED, 255);
            softPwmWrite(PIN_SMD_GRN, 0);
            softPwmWrite(PIN_SMD_BLU, 0);
            break;
        default:
            break;
    }
}

// Helper function: reset the sequence and turn off all relays.
void reset_sequence(volatile SharedVariable* sv) {
    TURN_OFF(PIN_RELAY_3_WAY_VALVE);
    TURN_OFF(PIN_RELAY_N_O_VALVE);
    TURN_OFF(PIN_RELAY_PUMP);
    sv->sequenceState = SEQ_IDLE;
    sv->cancelSequence = 0;
    printf("Sequence reset.\n");
}

// Sequence state machine: handles the timed relay operations.
void body_sequence(volatile SharedVariable* sv) {
    unsigned long now = millis();

    if (sv->cancelSequence) {
        reset_sequence(sv);
        return;
    }

    switch (sv->sequenceState) {
        case SEQ_IDLE:
            // Do nothing.
            TURN_OFF(PIN_RELAY_3_WAY_VALVE);
            TURN_OFF(PIN_RELAY_N_O_VALVE);
            TURN_OFF(PIN_RELAY_PUMP);
            break;
        case SEQ_STEP1:
            // Immediately: enable the 3-way valve.
            // This will fill the reservoir with shower water
            // Stop after time period or when sensor detects full
            TURN_ON(PIN_RELAY_3_WAY_VALVE);
            if ((now - sv->sequenceTimestamp >= (1 * MINUTES)) || sv->waterLevelAboveThreshold) {
                sv->sequenceState = SEQ_STEP2;
                sv->sequenceTimestamp = now;
                printf("Transition to SEQ_STEP2\n");
            }
            break;
        case SEQ_STEP2:
            // Enable the NO valve.
            // This will prevent shower water from continuing into the reservoir
            TURN_ON(PIN_RELAY_N_O_VALVE);
            if (now - sv->sequenceTimestamp >= (3 * SECONDS)) {
                sv->sequenceState = SEQ_STEP3;
                sv->sequenceTimestamp = now;
                printf("Transition to SEQ_STEP3\n");
            }
            break;
        case SEQ_STEP3:
            // Enable the pump.
            // This will begin circulating salt water through the softener
            TURN_ON(PIN_RELAY_PUMP);
            if (now - sv->sequenceTimestamp >= (5 * MINUTES)) {
                sv->sequenceState = SEQ_STEP4;
                sv->sequenceTimestamp = now;
                printf("Transition to SEQ_STEP4\n");
            }
            break;
        case SEQ_STEP4:
            // Turn off the 3-way valve.
            // This will dump the waste water through the shower
            TURN_OFF(PIN_RELAY_3_WAY_VALVE);
            if (now - sv->sequenceTimestamp >= (5 * MINUTES) || sv->waterLevelBelowThreshold) {
                sv->sequenceState = SEQ_STEP5;
                sv->sequenceTimestamp = now;
                printf("Transition to SEQ_STEP5\n");
            }
            break;
        case SEQ_STEP5:
            // Turn off NO valve and pump; turn on the 3-way valve.
            // This will refill the reservoir with fresh water
            TURN_OFF(PIN_RELAY_N_O_VALVE);
            TURN_OFF(PIN_RELAY_PUMP);
            TURN_ON(PIN_RELAY_3_WAY_VALVE);
            if ((now - sv->sequenceTimestamp >= (1 * MINUTES)) || sv->waterLevelAboveThreshold) {
                sv->sequenceState = SEQ_STEP6;
                sv->sequenceTimestamp = now;
                printf("Transition to SEQ_STEP6\n");
            }
            break;
        case SEQ_STEP6:
            // Turn on both NO valve and 3-way valve.
            TURN_ON(PIN_RELAY_N_O_VALVE);
            TURN_ON(PIN_RELAY_3_WAY_VALVE);
            if (now - sv->sequenceTimestamp >= (5 * SECONDS)) {
                sv->sequenceState = SEQ_STEP7;
                sv->sequenceTimestamp = now;
                printf("Transition to SEQ_STEP7\n");
            }
            break;
        case SEQ_STEP7:
            // Turn the pump on to dump the waste water
            TURN_OFF(PIN_RELAY_3_WAY_VALVE);
            TURN_ON(PIN_RELAY_PUMP);
            if (now - sv->sequenceTimestamp >= (5 * MINUTES) || sv->waterLevelBelowThreshold) {
                reset_sequence(sv);
            }
            break;
        case SEQ_FAILURE_DETECTED:
            TURN_OFF(PIN_RELAY_3_WAY_VALVE);
            TURN_OFF(PIN_RELAY_N_O_VALVE);
            TURN_OFF(PIN_RELAY_PUMP);
            break;
        default:
            break;
    }
}

void body_tds(volatile SharedVariable* sv) {
    // Use a static file descriptor so the I2C device is only opened once.
    static int fd = -1;
    if (fd == -1) {
        fd = wiringPiI2CSetup(0x48);  // Default I2C address for ADS1115.
        if (fd == -1) {
            printf("Failed to initialize ADS1115.\n");
            return;
        }
    }
    
    // Configure the ADS1115 to start a single conversion on AIN0.
    // New configuration for 3.3V operation:
    //  - OS (bit15) = 1: Start single conversion.
    //  - MUX (bits 14-12) = 100: AIN0 relative to GND.
    //  - PGA (bits 11-9) = 010: ±2.048V range (instead of ±4.096V).
    //  - MODE (bit8) = 1: Single-shot mode.
    //  - DR (bits 7-5) = 100: 128 samples per second.
    //  - COMP_QUE (bits 1-0) = 11: Disable comparator.
    int config = 0b1100010110000011; // Changed from 0xC383: PGA now set for ±2.048V.
    wiringPiI2CWriteReg16(fd, 0x01, config);
    
    delay(10);  // Wait ~10ms for the conversion to complete.
    
    // Read the conversion result from register 0x00.
    int raw = wiringPiI2CReadReg16(fd, 0x00);
    
    // The ADS1115 outputs data in big-endian format.
    // Perform manual byte swapping.
    int swapped = ((raw & 0xFF) << 8) | ((raw >> 8) & 0xFF);
    
    // Convert from two's complement if necessary.
    if (swapped > 32767) {
        swapped -= 65536;
    }
    if (swapped < 0) {
        swapped = 0;  // For single-ended measurements, negative values shouldn't occur.
    }
    
    // Convert ADC value to voltage using the ±2.048V range.
    float voltage = (float)swapped * 2.048 / 32767.0;
    
    // Convert voltage to TDS (ppm). Adjust the conversion factor based on calibration.
    // printf("Raw voltage: %d\n", voltage);
    float tds = voltage * 1221.0;

    // Apply temperature compensation only if the temperature is between 0 and 50°C.
    if (sv->temperature >= 0.0 && sv->temperature <= 50.0) {
        float temperatureCoefficient = 0.02;  // 2% per °C adjustment factor.
        float compensationFactor = 1.0 + temperatureCoefficient * (sv->temperature - 25.0);
        tds /= compensationFactor;
    }

    // Store the compensated TDS reading.
    sv->tds_reading = tds;
    
    // Print the TDS sensor reading.
    // printf("TDS Sensor Reading: %.2f ppm (Voltage: %.3f V, Raw: %d, Swapped: %d)\n", tds, voltage, raw, swapped);
    sv->tds_reading = tds;

    if (tds > HARD_WATER_TDS_THRESHOLD) {
        sv->detectedHardWater = true;
    }
}


#define MAX_DISTANCE_CM 50
#define TIMEOUT_US 2915            // Timeout corresponding to 50 cm ( (50cm * 2) / 0.0343 )
#define SOUND_SPEED_CM_PER_US 0.0343  // Speed of sound in cm/µs

// Returns the current time in microseconds.
static inline long get_microseconds() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec * 1000000L + tv.tv_usec;
}


// Wait for a specific pin to reach the desired state within the timeout.
// Returns the time (in µs) when the state is reached, or -1 if timeout occurs.
static long wait_for_pin_state(int pin, int desired_state, long timeout_us) {
    long start_time = get_microseconds();
    while (digitalRead(pin) != desired_state) {
        if ((get_microseconds() - start_time) > timeout_us) {
            return start_time + timeout_us;
        }
    }
    return get_microseconds();
}

// Sends the trigger pulse and measures the echo duration in microseconds.
// Returns the duration or -1 if a timeout occurs.
static long measure_echo_duration() {
    long start, end;

    // Ensure the trigger is LOW, then send a 10µs HIGH pulse.
    digitalWrite(PIN_ULTSON_BUCKET_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(PIN_ULTSON_BUCKET_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_ULTSON_BUCKET_TRIG, LOW);

    // Wait for the echo pin to go HIGH.
    start = wait_for_pin_state(PIN_ULTSON_BUCKET_ECHO, HIGH, TIMEOUT_US);

    // Wait for the echo pin to go LOW.
    end = wait_for_pin_state(PIN_ULTSON_BUCKET_ECHO, LOW, TIMEOUT_US);

    return end - start;
}

// Convert duration (in µs) to distance in centimeters.
static double calculate_distance_cm(long duration) {
    return (duration * SOUND_SPEED_CM_PER_US) / 2.0;
}


// This function is called repeatedly by the main loop.
void body_ultrasonic(volatile SharedVariable* sv) {
    // Static variable for exponential smoothing.
    static long filtered_duration = 0;
    long duration = measure_echo_duration();

    // Apply exponential smoothing: 9X% previous value, X% new reading for slower response.
    if (filtered_duration == 0) {
        filtered_duration = duration;
    } else {
        filtered_duration = (filtered_duration * 95 + duration * 5) / 100;
    }


    // Calculate the distance using the filtered duration.
    double distance = calculate_distance_cm(filtered_duration);
    // printf("Distance: %.2f cm\n", distance);

    // Update the shared variable
    sv->waterLevelAboveThreshold = (distance < WATER_LEVEL_HIGH_THRESHOLD_CM);
    sv->waterLevelBelowThreshold = (distance > WATER_LEVEL_LOW_THRESHOLD_CM);

    // Force failure state if distance is way too high (no echo detected, bad connection)
    if (distance > (MAX_DISTANCE_CM * 0.9)) {
        sv->sequenceState = SEQ_FAILURE_DETECTED;
    }
}


void body_persistent_temperature(volatile SharedVariable* sv) {
    while (true) {
        glob_t glob_result;
        float temperature = -1000.0;  // Default error value
        int valid = 0;                // Flag to indicate if we got a valid reading
        long timeout = 50000;         // Timeout of 50ms (50,000 microseconds)
        long overall_start = get_microseconds();

        // Locate the DS18B20 sensor (directory names start with "28-")
        if (glob("/sys/bus/w1/devices/28-*", 0, NULL, &glob_result) != 0 || glob_result.gl_pathc < 1) {
            fprintf(stderr, "No DS18B20 sensor found.\n");
            globfree(&glob_result);
            return;
        }
        
        // Build the full file path to the sensor's data file.
        char sensor_file[256];
        snprintf(sensor_file, sizeof(sensor_file), "%s/w1_slave", glob_result.gl_pathv[0]);
        globfree(&glob_result);

        // Try to read a valid sensor output within 50ms
        while (get_microseconds() - overall_start < timeout) {
            long iter_start = get_microseconds();

            FILE *fp = fopen(sensor_file, "r");
            if (!fp) {
                perror("Failed to open sensor file");
                return;
            }
            long open_time = get_microseconds();
            // printf("[Log] Time to open sensor file: %ld us\n", open_time - iter_start);

            char line[256];

            // Read the first line; it must contain "YES" if the reading is valid.
            if (fgets(line, sizeof(line), fp) == NULL) {
                fclose(fp);
                // printf("[Log] Failed to read first line.\n");
                continue;
            }
            long first_line_time = get_microseconds();
            // printf("[Log] Time to read first line: %ld us\n", first_line_time - open_time);

            if (strstr(line, "YES") == NULL) {
                fclose(fp);
                // printf("[Log] First line invalid (no 'YES'), elapsed: %ld us\n", get_microseconds() - iter_start);
                usleep(5000);  // 5ms delay before retrying
                continue;
            }

            // Read the second line that contains the temperature data.
            if (fgets(line, sizeof(line), fp) == NULL) {
                fclose(fp);
                // printf("[Log] Failed to read second line.\n");
                continue;
            }
            long second_line_time = get_microseconds();
            // printf("[Log] Time to read second line: %ld us\n", second_line_time - first_line_time);
            fclose(fp);

            // Extract temperature value after "t="
            char *t_ptr = strstr(line, "t=");
            if (t_ptr != NULL) {
                t_ptr += 2;  // Skip "t="
                temperature = atof(t_ptr) / 1000.0;  // Convert from millidegrees to °C
                valid = 1;
                long iter_end = get_microseconds();
                // printf("[Log] Valid reading obtained in %ld us\n", iter_end - iter_start);
                break;
            }
        }

        if (!valid) {
            // printf("[Log] Sensor reading timed out (over 50ms). Using last valid reading: %.2f °C\n", sv->temperature);
        } else {
            sv->temperature = temperature;
            
            //printf("Temperature: %.2f °C\n", temperature);
        }
    }
}


void body_persistent_log_to_database(volatile SharedVariable* sv) {
    while (true) {
        delay(30000);

        add_tds_reading(sv->tds_reading);
        add_temperature_reading(sv->temperature);


        time_t now = time(NULL);
        struct tm *tm_info = localtime(&now);
        char timestampStr[26];

        // Format timestamp as "YYYY-MM-DD HH:MM:SS"
        strftime(timestampStr, sizeof(timestampStr), "%Y-%m-%d %H:%M:%S", tm_info);
        printf("[%s] Data logged to database: tds %.2f, temp %.2f.\n", timestampStr, sv->tds_reading, sv->temperature);
    }
}