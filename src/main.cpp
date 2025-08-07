/***********************************************************************
 * main.ino  –  Water-Softener controller for Arduino Nano 33 IoT
 * --------------------------------------------------------------------
 *  • Single-threaded, cooperative loop (no pthreads).
 *  • Tasks are the non-blocking body_*() functions you already ported.
 *  • Replace WIFI_SSID / WIFI_PASS with your credentials.
 ***********************************************************************/

#include <Arduino.h>
#include "assignment1.h"
#include "database.h"
#include "http_server.h"

/* ---------- Wi-Fi credentials (update these) ---------- */
static const char *WIFI_SSID = "YOUR_SSID";
static const char *WIFI_PASS = "YOUR_WIFI_PASSWORD";

/* ------------------------------------------------------------------ */
/*  STARTUP DELAY FUNCTION                                            */
/* ------------------------------------------------------------------ */
void waitForUserInput()
{
    Serial.println(F("[STARTUP] Water Softener Controller"));
    Serial.println(F("[STARTUP] Press any key and hit Enter to continue..."));
    
    unsigned long lastMessage = 0;
    const unsigned long messageInterval = 2000; // 2 seconds
    
    while (true) {
        // Check if user sent any input
        if (Serial.available() > 0) {
            // Clear the input buffer
            while (Serial.available() > 0) {
                Serial.read();
            }
            Serial.println(F("[STARTUP] Starting system..."));
            delay(500);
            return;
        }
        
        // Send periodic reminder messages
        unsigned long currentTime = millis();
        if (currentTime - lastMessage >= messageInterval) {
            Serial.println(F("[STARTUP] Waiting for input... (press any key)"));
            lastMessage = currentTime;
        }
        
        delay(100); // Small delay to prevent busy waiting
    }
}

/* ------------------------------------------------------------------ */
/*  SETUP                                                             */
/* ------------------------------------------------------------------ */
void setup()
{
    Serial.begin(115200);
    
    // Wait for user input before continuing
    waitForUserInput();

    /* core initialisation ------------------------------------------ */
    init_shared_variable(sv);
    init_sensors(sv);

    // /* persistent storage ------------------------------------------- */
    // if (!init_database()) {
    //     Serial.println(F("[BOOT] SD-card logger init failed - check wiring"));
    // }

    /* network + HTTP server ---------------------------------------- */
    startWebServer(WIFI_SSID, WIFI_PASS);   // defined in http_server.cpp
}

/* ------------------------------------------------------------------ */
/*  LOOP                                                              */
/* ------------------------------------------------------------------ */
void loop()
{
    /* cooperative “multitasking” ----------------------------------- */
    body_button(sv);
    body_sequence(sv);
    body_led(sv);
    body_ultrasonic(sv);
    body_tds(sv);
    body_persistent_temperature(sv);
    body_persistent_log_to_database(sv);

    /* handle incoming web requests --------------------------------- */
    // handleWebClient();                     // from http_server.cpp

    delay(5);                              // small yield to Wi-Fi stack
}
