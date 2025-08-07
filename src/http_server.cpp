#include "http_server.h"
#include "assignment1.h"
#include "database.h"

#include <WiFiNINA.h>
#include <avr/dtostrf.h>   // tiny helper for float→string

// ---------- Wi-Fi ----------
static WiFiServer server(80);

// pick reasonable sizes for AVR SAM-D21
static constexpr size_t RX_LINE_BUF = 128;
static constexpr size_t JSON_BUF    = 2048;  // fits in SRAM

static char lineBuf[RX_LINE_BUF];

/* ------------------------------------------------------------------ */
// INITIALISE

void startWebServer(const char* ssid, const char* pass)
{
	// 1. connect to Wi-Fi
	Serial.print(F("[WIFI] Begin connecting to "));
	Serial.print(ssid);
	Serial.print(F("... "));
	Serial.println();
	WiFi.begin(ssid, pass);
	while (WiFi.status() != WL_CONNECTED) {
		Serial.print(F("[WIFI] Connecting to "));
		Serial.print(ssid);
		Serial.print(F("... "));
		Serial.print(WiFi.status());
		Serial.println();
		delay(500);
	}
	Serial.print(F("[WIFI] Connected to "));
	Serial.print(ssid);
	Serial.print(F(" with IP: "));
	Serial.println(WiFi.localIP());

	// 2. launch server
	server.begin();
	Serial.print(F("Web server ready: http://"));
	Serial.print(WiFi.localIP());
	Serial.println(F("/"));
}

/* ------------------------------------------------------------------ */
// ROUTE HELPERS

static void send_404(WiFiClient& c)
{
	c.println(F("HTTP/1.1 404 Not Found"));
	c.println(F("Content-Type: text/html\r\nConnection: close\r\n"));
	c.println(F("<html><body><h1>404 Not Found</h1></body></html>"));
}

static void send_json(WiFiClient& c, const char* json)
{
	c.println(F("HTTP/1.1 200 OK"));
	c.println(F("Content-Type: application/json"));
	c.println(F("Connection: close\r\n"));
	c.println(json);
}

static void send_simple_html(WiFiClient& c)
{
	c.println(F("HTTP/1.1 200 OK"));
	c.println(F("Content-Type: text/html"));
	c.println(F("Connection: close\r\n"));
	
	c.println(F("<!DOCTYPE html><html><head><title>Water Softener</title></head><body>"));
	c.println(F("<h1>Water Softener Controller</h1>"));
	c.println(F("<p><a href='/api/temp'>Temperature Data</a></p>"));
	c.println(F("<p><a href='/api/tds'>TDS Data</a></p>"));
	c.println(F("</body></html>"));
}

/* ------------------------------------------------------------------ */
// MAIN CLIENT HANDLER  – call in loop()

void handleWebClient(void)
{
	WiFiClient client = server.available();
	if (!client) return;

	// ----- read the request line (GET /path HTTP/1.1) -----
	size_t idx = 0;
	unsigned long t0 = millis();
	while (client.connected() && millis() - t0 < 1000) {
	if (!client.available()) continue;
	char c = client.read();
	if (c == '\r') continue;        // ignore CR
	if (c == '\n') break;           // end of line
	if (idx < RX_LINE_BUF - 1) lineBuf[idx++] = c;
	}
	lineBuf[idx] = '\0';

	// very naïve tokenisation
	char* method = strtok(lineBuf, " ");
	char* path   = strtok(nullptr, " ");

	if (!method || !path) { send_404(client); return; }

	/* -------------------------------------------------- */
	// ------------ ROUTING ------------------------------
	/* -------------------------------------------------- */

	if (strcmp(path, "/") == 0) {
	send_simple_html(client);                    // Simple HTML homepage
	}

	else if (strcmp(path, "/data") == 0) {
	static char jsonBuf[JSON_BUF];
	if (get_last_24h_data(jsonBuf, sizeof(jsonBuf)))
		send_json(client, jsonBuf);
	else
		send_json(client, "{\"error\":\"db read failed\"}");
	}

	else if (strcmp(path, "/data_tds") == 0) {
	static char jsonBuf[JSON_BUF];
	if (get_last_24h_tds_data(jsonBuf, sizeof(jsonBuf)))
		send_json(client, jsonBuf);
	else
		send_json(client, "{\"error\":\"db read failed\"}");
	}

	else {
	send_404(client);
	}

	delay(1);       // make sure data is sent
	client.stop();
}
