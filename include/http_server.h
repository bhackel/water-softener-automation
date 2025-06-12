#ifndef HTTP_SERVER_H
#define HTTP_SERVER_H

#include <Arduino.h>

// call once
void startWebServer(const char* ssid, const char* pass);

// call every loop()
void handleWebClient(void);

#endif  // HTTP_SERVER_H
