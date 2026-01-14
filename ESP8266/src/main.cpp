#include <Arduino.h>
#include <SoftwareSerial.h>

SoftwareSerial esp32Serial(14, 12);

void setup() {
    Serial.begin(115200);
    esp32Serial.begin(9600);
    Serial.println("System online. Listening for the Captain...");
}

void loop() {
    if (esp32Serial.available()) {
        String incoming = esp32Serial.readStringUntil('\n');

        if (incoming.length() > 0) {
            Serial.print("Internal Log (Captain said): ");
            Serial.println(incoming);
        }
    }
}
