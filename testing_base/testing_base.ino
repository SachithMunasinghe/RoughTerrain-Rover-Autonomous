#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>

// LoRa transceiver pins for ESP8266
#define NSS_PIN  D8  // Chip Select
#define RST_PIN  D3  // Reset
#define DIO0_PIN D1  // IRQ pin

// LoRa frequency & sync
const long   LORA_FREQUENCY = 433E6; 
const uint8_t LORA_SYNC_WORD = 0xF2;

void setup() {
  Serial.begin(115200);
  Serial.println("Starting ESP8266 + LoRa Base Station...");

  LoRa.setPins(NSS_PIN, RST_PIN, DIO0_PIN);
  while (!LoRa.begin(LORA_FREQUENCY)) {
    Serial.println("LoRa init failed. Retrying...");
    delay(500);
  }
  LoRa.setSyncWord(LORA_SYNC_WORD);

  Serial.println("LoRa Initialized Successfully!");
}

void loop() {
  // 1) Check for incoming LoRa data
  int packetSize = LoRa.parsePacket();
  if (packetSize > 0) {
    String incoming_data;
    while (LoRa.available()) {
      incoming_data += (char)LoRa.read();
    }
    incoming_data.trim();

    // Print each telemetry data as a separate line
    Serial.println(incoming_data);
  }

  // 2) Check for commands from the GUI over USB Serial
  while (Serial.available() > 0) {
    String gui_command = Serial.readStringUntil('\n');
    gui_command.trim();
    if (gui_command.length() > 0) {
      // Forward to Rover via LoRa
      LoRa.beginPacket();
      LoRa.println(gui_command); // Send as one line
      LoRa.endPacket();

      // Debug
      Serial.print("Command sent: ");
      Serial.println(gui_command);
    }
  }

  delay(5);
}
