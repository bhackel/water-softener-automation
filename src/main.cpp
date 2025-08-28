/***********************************************************************
 * main.cpp  –  Water-Softener controller for Arduino Nano 33 BLE
 * --------------------------------------------------------------------
 *  • Single-threaded, cooperative loop (no pthreads).
 *  • Tasks are the non-blocking body_*() functions you already ported.
 *  • Exposes TDS and temperature data via Bluetooth Low Energy.
 ***********************************************************************/

#include <Arduino.h>
#include "sensor_manager.h"
#include "data_logger.h"
#include "bluetooth_service.h"

/* ---------- Bluetooth device name ---------- */
static const char *BLE_DEVICE_NAME = "Water";


/* ------------------------------------------------------------------ */
/*  SETUP                                                             */
/* ------------------------------------------------------------------ */
void setup()
{
    Serial.begin(115200);

    if (!initBluetoothService(BLE_DEVICE_NAME)) {
        Serial.println(F("[BOOT] Bluetooth service init failed"));
    }

    /* core initialisation ------------------------------------------ */
    init_shared_variable(sv);
    init_sensors(sv);
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

    /* handle Bluetooth events ------------------------------------- */
    handleBluetoothEvents();               // from bluetooth_service.cpp

    delay(5);                              // small yield for BLE stack
}
