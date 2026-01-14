#include <Arduino.h>
#include <WiFi.h>

const char *ssid = "Aiphone";
const char *password = "finelikewine";

WiFiServer server(80);

void setup() {
    Serial.begin(115200);

    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);

    Serial.print("Connecting");
    while (WiFi.status() != WL_CONNECTED) {
        delay(300);
        Serial.print(".");
    }

    Serial.println();
    Serial.print("Connected! IP: ");
    Serial.println(WiFi.localIP());

    server.begin();
}

void loop() {
    WiFiClient client = server.available();
    if (!client)
        return;

    while (!client.available()) {
        delay(1);
    }

    String req = client.readStringUntil('\r');
    client.readStringUntil('\n');

    Serial.println("Request: " + req);

    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/plain");
    client.println("Connection: close");
    client.println();
    client.println("Hello from ESP32!");

    delay(1);
    client.stop();
}
