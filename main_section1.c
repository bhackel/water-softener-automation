#include <stdio.h>
#include <pthread.h>
#include <wiringPi.h>
#include <softPwm.h>
#include "assignment1.h"
#include "http_server.c"
#include "database.h"


volatile SharedVariable sv;

// Macro to declare a thread for a given body function.
#define thread_decl(NAME) \
void* thread_##NAME(void* param) { \
    SharedVariable* pV = (SharedVariable*) param; \
    body_##NAME(pV); \
    return NULL; \
}

// Only the tasks needed for the new project.
thread_decl(button)
thread_decl(led)
thread_decl(sequence)
thread_decl(tds)
thread_decl(ultrasonic)
thread_decl(persistent_temperature)
thread_decl(persistent_log_to_database)

// Macros for thread creation and joining.
#define thread_create(NAME) pthread_create(&t_##NAME, NULL, thread_##NAME, (void *)&sv);
#define thread_join(NAME)   pthread_join(t_##NAME, NULL);
#define thread_detach(NAME) pthread_detach(t_##NAME);


void handle_sigint(int sig) {
    server_exit = 1;
}



int main(int argc, char* argv[]) {
    init_database();

    // Setup WiringPi.
    if (wiringPiSetup() == -1) {
        printf("Failed to setup wiringPi.\n");
        return 1;
    }

    // Initialize shared variable and hardware (button, LED, relay outputs).
    init_shared_variable(&sv);
    init_sensors(&sv);

    // Thread identifiers for our three tasks.
    pthread_t t_button, 
              t_led, 
              t_sequence, 
              t_tds, 
              t_ultrasonic, 
              t_persistent_temperature,
              t_persistent_log_to_database,
              server_thread;

    // ---------- concurrent threads -----------
    // temperature sensor
    thread_create(persistent_temperature)
    thread_detach(persistent_temperature)
    // html server
    pthread_create(&server_thread, NULL, (void *(*)(void *))server_main, NULL);
    pthread_detach(server_thread);
    // sensor logging to database
    thread_create(persistent_log_to_database)
    thread_detach(persistent_log_to_database)

    // Set up the SIGINT handler.
    struct sigaction sa;
    sa.sa_handler = handle_sigint;
    sa.sa_flags = 0;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGINT, &sa, NULL);


    // Main loop: repeatedly spawn and join the threads.
    while (!sv.bProgramExit) {
        thread_create(button)
        thread_create(led)
        thread_create(sequence)
        thread_create(tds)
        thread_create(ultrasonic)

        thread_join(button)
        thread_join(led)
        thread_join(sequence)
        thread_join(tds)
        thread_join(ultrasonic)

        delay(10); // Small delay between iterations.

        if (server_exit) {
            sv.bProgramExit = 1;
        }
    }

    close_database();
    printf("Program finished.\n");

    return 0;
}
