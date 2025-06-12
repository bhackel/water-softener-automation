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
/*  SETUP                                                             */
/* ------------------------------------------------------------------ */
void setup()
{
    Serial.begin(115200);

    /* core initialisation ------------------------------------------ */
    init_shared_variable(sv);
    init_sensors(sv);

    /* persistent storage ------------------------------------------- */
    if (!init_database()) {
        Serial.println(F("[BOOT] SD-card logger init failed - check wiring"));
    }

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
    handleWebClient();                     // from http_server.cpp

    delay(5);                              // small yield to Wi-Fi stack
}
