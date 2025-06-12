#include "http_server.h"
#include "assignment1.h"
#include "database.h"

#include <WiFiNINA.h>
#include <SD.h>            // for index.html if you store it on SD
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
	WiFi.begin(ssid, pass);
	while (WiFi.status() != WL_CONNECTED) {
	delay(500);
	}

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

static void stream_file(WiFiClient& c, const char* path)
{
	File f = SD.open(path, FILE_READ);
	if (!f) { send_404(c); return; }

	c.println(F("HTTP/1.1 200 OK"));
	c.println(F("Content-Type: text/html"));
	c.println(F("Connection: close\r\n"));

	static char buf[64];
	int n;
	while ((n = f.read(buf, sizeof(buf))) > 0) {
	c.write(buf, n);        // chunk out to client
	}
	f.close();
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
	stream_file(client, "/index.html");          // SD-card copy of your page
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
