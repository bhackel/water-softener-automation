#ifndef DATABASE_H
#define DATABASE_H

#include <sqlite3.h>
#include <stddef.h>

// Initializes the SQLite database (opens file, creates table if not exists).
// Returns 0 on success, non-zero on error.
int init_database(void);

// Closes the SQLite database.
void close_database(void);

// Inserts a new temperature reading into the database.
// Returns 0 on success, non-zero on error.
int add_temperature_reading(float temperature);

// Retrieves JSON for the last 24 hours of data (time + temperature),
// storing it in jsonBuffer. Returns 0 on success, non-zero on error.
int get_last_24h_data(char *jsonBuffer, size_t maxSize);

// New: TDS
int add_tds_reading(float tdsValue);
int get_last_24h_tds_data(char *jsonBuffer, size_t maxSize);

#endif // DATABASE_H
