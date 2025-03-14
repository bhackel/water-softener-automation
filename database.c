#include "database.h"
#include <stdio.h>
#include <time.h>
#include <string.h>

static sqlite3 *db = NULL;

int init_database(void) {
    int rc = sqlite3_open("sensor_data.db", &db);
    if (rc != SQLITE_OK) {
        fprintf(stderr, "Cannot open database: %s\n", sqlite3_errmsg(db));
        return rc;
    }

    // Create temperature table if not exists
    const char *sql_temp =
        "CREATE TABLE IF NOT EXISTS temperature_data ("
        "id INTEGER PRIMARY KEY AUTOINCREMENT, "
        "timestamp INTEGER, "
        "temperature REAL"
        ");";

    char *errmsg = NULL;
    rc = sqlite3_exec(db, sql_temp, NULL, NULL, &errmsg);
    if (rc != SQLITE_OK) {
        fprintf(stderr, "Failed to create temperature_data table: %s\n", errmsg);
        sqlite3_free(errmsg);
        return rc;
    }

    // Create TDS table if not exists
    const char *sql_tds =
        "CREATE TABLE IF NOT EXISTS tds_data ("
        "id INTEGER PRIMARY KEY AUTOINCREMENT, "
        "timestamp INTEGER, "
        "tds REAL"
        ");";

    rc = sqlite3_exec(db, sql_tds, NULL, NULL, &errmsg);
    if (rc != SQLITE_OK) {
        fprintf(stderr, "Failed to create tds_data table: %s\n", errmsg);
        sqlite3_free(errmsg);
        return rc;
    }

    return 0; // success
}

void close_database(void) {
    if (db) {
        sqlite3_close(db);
        db = NULL;
    }
}

//--------------------- Temperature Functions (already present) ---------------------//

int add_temperature_reading(float temperature) {
    if (!db) {
        fprintf(stderr, "Database not initialized.\n");
        return -1;
    }
    time_t now = time(NULL);

    const char *sql =
        "INSERT INTO temperature_data (timestamp, temperature) VALUES (?, ?);";

    sqlite3_stmt *stmt;
    int rc = sqlite3_prepare_v2(db, sql, -1, &stmt, NULL);
    if (rc != SQLITE_OK) {
        fprintf(stderr, "Failed to prepare insert statement: %s\n", sqlite3_errmsg(db));
        return rc;
    }

    sqlite3_bind_int64(stmt, 1, (sqlite3_int64)now);
    sqlite3_bind_double(stmt, 2, (double)temperature);

    rc = sqlite3_step(stmt);
    if (rc != SQLITE_DONE) {
        fprintf(stderr, "Insert failed: %s\n", sqlite3_errmsg(db));
        sqlite3_finalize(stmt);
        return rc;
    }
    sqlite3_finalize(stmt);
    return 0;
}

int get_last_24h_data(char *jsonBuffer, size_t maxSize) {
    if (!db) {
        fprintf(stderr, "Database not initialized.\n");
        return -1;
    }
    time_t one_day_ago = time(NULL) - (24 * 60 * 60);

    const char *sql =
        "SELECT timestamp, temperature "
        "FROM temperature_data "
        "WHERE timestamp >= ? "
        "ORDER BY timestamp ASC;";

    sqlite3_stmt *stmt;
    int rc = sqlite3_prepare_v2(db, sql, -1, &stmt, NULL);
    if (rc != SQLITE_OK) {
        fprintf(stderr, "Failed to prepare SELECT: %s\n", sqlite3_errmsg(db));
        return rc;
    }
    sqlite3_bind_int64(stmt, 1, (sqlite3_int64)one_day_ago);

    size_t offset = 0;
    offset += snprintf(jsonBuffer + offset, maxSize - offset, "[");
    int rowCount = 0;

    while ((rc = sqlite3_step(stmt)) == SQLITE_ROW) {
        if (rowCount > 0) {
            offset += snprintf(jsonBuffer + offset, maxSize - offset, ",");
        }
        time_t t = (time_t)sqlite3_column_int64(stmt, 0);
        double temp = sqlite3_column_double(stmt, 1);

        offset += snprintf(jsonBuffer + offset, maxSize - offset,
                           "{\"time\":%ld,\"temperature\":%.2f}",
                           (long)t, temp);

        rowCount++;
    }
    offset += snprintf(jsonBuffer + offset, maxSize - offset, "]");
    sqlite3_finalize(stmt);

    if (rc != SQLITE_DONE) {
        fprintf(stderr, "SELECT step error: %s\n", sqlite3_errmsg(db));
        return rc;
    }
    return 0;
}

//--------------------- TDS Functions (new) ---------------------//

int add_tds_reading(float tdsValue) {
    if (!db) {
        fprintf(stderr, "Database not initialized.\n");
        return -1;
    }
    time_t now = time(NULL);

    const char *sql =
        "INSERT INTO tds_data (timestamp, tds) VALUES (?, ?);";

    sqlite3_stmt *stmt;
    int rc = sqlite3_prepare_v2(db, sql, -1, &stmt, NULL);
    if (rc != SQLITE_OK) {
        fprintf(stderr, "Failed to prepare insert statement for TDS: %s\n", sqlite3_errmsg(db));
        return rc;
    }

    sqlite3_bind_int64(stmt, 1, (sqlite3_int64)now);
    sqlite3_bind_double(stmt, 2, (double)tdsValue);

    rc = sqlite3_step(stmt);
    if (rc != SQLITE_DONE) {
        fprintf(stderr, "Insert failed for TDS: %s\n", sqlite3_errmsg(db));
        sqlite3_finalize(stmt);
        return rc;
    }
    sqlite3_finalize(stmt);
    return 0; // success
}

int get_last_24h_tds_data(char *jsonBuffer, size_t maxSize) {
    if (!db) {
        fprintf(stderr, "Database not initialized.\n");
        return -1;
    }
    time_t one_day_ago = time(NULL) - (24 * 60 * 60);

    const char *sql =
        "SELECT timestamp, tds "
        "FROM tds_data "
        "WHERE timestamp >= ? "
        "ORDER BY timestamp ASC;";

    sqlite3_stmt *stmt;
    int rc = sqlite3_prepare_v2(db, sql, -1, &stmt, NULL);
    if (rc != SQLITE_OK) {
        fprintf(stderr, "Failed to prepare SELECT for TDS: %s\n", sqlite3_errmsg(db));
        return rc;
    }

    sqlite3_bind_int64(stmt, 1, (sqlite3_int64)one_day_ago);

    size_t offset = 0;
    offset += snprintf(jsonBuffer + offset, maxSize - offset, "[");
    int rowCount = 0;

    while ((rc = sqlite3_step(stmt)) == SQLITE_ROW) {
        if (rowCount > 0) {
            offset += snprintf(jsonBuffer + offset, maxSize - offset, ",");
        }
        time_t t = (time_t)sqlite3_column_int64(stmt, 0);
        double tdsVal = sqlite3_column_double(stmt, 1);

        offset += snprintf(jsonBuffer + offset, maxSize - offset,
                           "{\"time\":%ld,\"tds\":%.2f}",
                           (long)t, tdsVal);

        rowCount++;
    }
    offset += snprintf(jsonBuffer + offset, maxSize - offset, "]");
    sqlite3_finalize(stmt);

    if (rc != SQLITE_DONE) {
        fprintf(stderr, "SELECT step error for TDS: %s\n", sqlite3_errmsg(db));
        return rc;
    }
    return 0; // success
}
