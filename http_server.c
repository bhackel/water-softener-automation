#include <microhttpd.h>
#include <stdio.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include "assignment1.h"
#include "database.h"

#define PORT 8888

volatile sig_atomic_t server_exit = 0;


char *load_template(const char *filename) {
    FILE *file = fopen(filename, "rb");
    if (!file) return NULL;
    fseek(file, 0, SEEK_END);
    long filesize = ftell(file);
    rewind(file);

    char *buffer = malloc(filesize + 1);
    if (buffer) {
        fread(buffer, 1, filesize, file);
        buffer[filesize] = '\0';
    }
    fclose(file);
    return buffer;
}

char *build_page(const char *template, float temperature) {
    // This is a very naive implementation.
    // Find the token "{{temperature}}"
    const char *token = "{{temperature}}";
    char *pos = strstr(template, token);
    if (!pos) {
        // Token not found, return a copy of the original template.
        return strdup(template);
    }
    
    // Calculate sizes for the new string.
    size_t pre_size = pos - template;
    char temp_str[32];
    snprintf(temp_str, sizeof(temp_str), "%.2f", temperature);
    size_t temp_str_size = strlen(temp_str);
    size_t post_size = strlen(template) - pre_size - strlen(token);
    size_t new_size = pre_size + temp_str_size + post_size;
    
    char *result = malloc(new_size + 1);
    if (!result) return NULL;
    
    // Build the new string.
    memcpy(result, template, pre_size);
    memcpy(result + pre_size, temp_str, temp_str_size);
    memcpy(result + pre_size + temp_str_size, pos + strlen(token), post_size);
    result[new_size] = '\0';
    
    return result;
}



static enum MHD_Result answer_to_connection(void *cls,
                                            struct MHD_Connection *connection,
                                            const char *url,
                                            const char *method,
                                            const char *version,
                                            const char *upload_data,
                                            size_t *upload_data_size,
                                            void **con_cls)
{
    // Serve "/data" with JSON (existing code).
    if (strcmp(url, "/data") == 0) {
        char jsonBuffer[262144];
        int rc = get_last_24h_data(jsonBuffer, sizeof(jsonBuffer));
        if (rc != SQLITE_OK) {
            const char *errorMsg = "{\"error\":\"Failed to get data\"}";
            struct MHD_Response *errorResp = 
                MHD_create_response_from_buffer(strlen(errorMsg), 
                                                (void *)errorMsg,
                                                MHD_RESPMEM_PERSISTENT);
            return MHD_queue_response(connection, MHD_HTTP_INTERNAL_SERVER_ERROR, errorResp);
        }
        // Serve JSON
        struct MHD_Response *response =
            MHD_create_response_from_buffer(strlen(jsonBuffer),
                                            (void *)jsonBuffer,
                                            MHD_RESPMEM_MUST_COPY);
        return MHD_queue_response(connection, MHD_HTTP_OK, response);
    }

    // New "/data_tds" endpoint
    if (strcmp(url, "/data_tds") == 0) {
        char jsonBuffer[262144];
        int rc = get_last_24h_tds_data(jsonBuffer, sizeof(jsonBuffer));
        if (rc != 0) {
            const char *errorMsg = "{\"error\":\"Failed to get TDS data\"}";
            struct MHD_Response *errorResp = 
                MHD_create_response_from_buffer(strlen(errorMsg),
                                                (void *)errorMsg,
                                                MHD_RESPMEM_PERSISTENT);
            return MHD_queue_response(connection, MHD_HTTP_INTERNAL_SERVER_ERROR, errorResp);
        }

        struct MHD_Response *response =
            MHD_create_response_from_buffer(strlen(jsonBuffer),
                                            (void *)jsonBuffer,
                                            MHD_RESPMEM_MUST_COPY);
        return MHD_queue_response(connection, MHD_HTTP_OK, response);
    }

    // Serve "/" with index.html
    if (strcmp(url, "/") == 0) {
        FILE *file = fopen("index.html", "rb");
        if (!file) {
            // Fallback response if file not found
            const char *error = "<html><body><h1>File not found</h1></body></html>";
            struct MHD_Response *resp =
                MHD_create_response_from_buffer(strlen(error),
                                                (void *)error,
                                                MHD_RESPMEM_PERSISTENT);
            return MHD_queue_response(connection, MHD_HTTP_NOT_FOUND, resp);
        }

        // Read file into a buffer
        fseek(file, 0, SEEK_END);
        long fileSize = ftell(file);
        rewind(file);

        char *buffer = malloc(fileSize + 1);
        if (!buffer) {
            fclose(file);
            const char *error = "<html><body><h1>Memory error</h1></body></html>";
            struct MHD_Response *resp =
                MHD_create_response_from_buffer(strlen(error),
                                                (void *)error,
                                                MHD_RESPMEM_PERSISTENT);
            return MHD_queue_response(connection, MHD_HTTP_INTERNAL_SERVER_ERROR, resp);
        }

        fread(buffer, 1, fileSize, file);
        buffer[fileSize] = '\0';
        fclose(file);

        struct MHD_Response *resp =
            MHD_create_response_from_buffer(strlen(buffer),
                                            (void *)buffer,
                                            MHD_RESPMEM_MUST_FREE);
        return MHD_queue_response(connection, MHD_HTTP_OK, resp);
    }

    // If unrecognized route, you could return a 404 or redirect.
    const char *notFound = "<html><body><h1>404 Not Found</h1></body></html>";
    struct MHD_Response *resp =
        MHD_create_response_from_buffer(strlen(notFound),
                                        (void *)notFound,
                                        MHD_RESPMEM_PERSISTENT);
    return MHD_queue_response(connection, MHD_HTTP_NOT_FOUND, resp);
}



int server_main() {

    struct MHD_Daemon *daemon;

    daemon = MHD_start_daemon(MHD_USE_INTERNAL_POLLING_THREAD,
                              PORT,
                              NULL,
                              NULL,
                              &answer_to_connection,
                              NULL,
                              MHD_OPTION_END);
    if (NULL == daemon) {
        fprintf(stderr, "Failed to start the HTTP server.\n");
        return 1;
    }
    printf("Server is running on http://localhost:%d/\n", PORT);

    while (!server_exit) {
        sleep(1);  // Sleep for 1 second to reduce CPU usage.
    }

    MHD_stop_daemon(daemon);
    return 0;
}