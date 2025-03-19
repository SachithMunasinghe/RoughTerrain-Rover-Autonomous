/*************************************************************
   Base Station (ESP8266) - Method A
   Forwards GUI commands via LoRa, receives telemetry.
*************************************************************/
#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>

// LoRa transceiver pins for ESP8266
#define NSS_PIN  D8  // Chip Select
#define RST_PIN  D3  // Reset
#define DIO0_PIN D1  // IRQ pin

// LoRa frequency for your region
const long LORA_FREQUENCY = 433E6; 
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
  // 1. Check for incoming LoRa packets (telemetry or else)
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String incoming_data = "";
    while (LoRa.available()) {
      incoming_data += (char)LoRa.read();
    }
    incoming_data.trim();
    // Print to Serial so the Python GUI sees it
    Serial.println(incoming_data);
  }

  // 2. Check for GUI commands via Serial
  //    (The Python GUI sends lines like "forward\n", "stop\n", etc.)
  while (Serial.available() > 0) {
    String gui_command = Serial.readStringUntil('\n');
    gui_command.trim();
    if (gui_command.length() > 0) {
      // Forward this command via LoRa
      LoRa.beginPacket();
      LoRa.print(gui_command);
      LoRa.endPacket();
      // For debug
      Serial.println("Command sent: " + gui_command);
    }
  }

  delay(5);
}
