#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>

// Define the pins used by the LoRa transceiver module
#define NSS_PIN  D8  // Chip Select (CS)
#define RST_PIN  D3  // Reset pin
#define DIO0_PIN D1  // IRQ pin (DIO0)

// LoRa frequency, adjust for your region (e.g., 433E6, 868E6, 915E6)
const long LORA_FREQUENCY = 433E6;

// Set your sync word (must match the rover)
const uint8_t LORA_SYNC_WORD = 0xF2;

// Optional: Configure LoRa parameters for optimized communication
// Uncomment and adjust as needed
// const int SPREADING_FACTOR = 7;
// const long SIGNAL_BANDWIDTH = 250E3;
// const int CODING_RATE = 5;

void setup() {
  // Initialize Serial for communication with the GUI
  Serial.begin(115200);

  // Wait for Serial to be ready (optional, depends on board)
  // while (!Serial) { ; }

  Serial.println("Initializing ESP8266 + LoRa Base Station...");

  // Initialize LoRa with specified pins
  LoRa.setPins(NSS_PIN, RST_PIN, DIO0_PIN);

  // Attempt to begin LoRa at the chosen frequency
  while (!LoRa.begin(LORA_FREQUENCY)) {
    Serial.println("LoRa initialization failed. Retrying...");
    delay(500);
  }

  // Optionally set sync word
  LoRa.setSyncWord(LORA_SYNC_WORD);

  // Optional: Set LoRa parameters for optimized communication
  // Uncomment and adjust based on requirements
  /*
  LoRa.setSpreadingFactor(SPREADING_FACTOR);
  LoRa.setSignalBandwidth(SIGNAL_BANDWIDTH);
  LoRa.setCodingRate4(CODING_RATE);
  */

  Serial.println("LoRa Initialized Successfully!");
}

void loop() {
  // Handle incoming telemetry data from the rover via LoRa
  receiveTelemetry();

  // Handle incoming commands from the GUI via Serial
  sendCommands();

  // Small delay for stability
  delay(5);
}

void receiveTelemetry() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String incomingData = "";
    while (LoRa.available()) {
      incomingData += (char)LoRa.read();
    }
    incomingData.trim(); // Remove any leading/trailing whitespace
    if (incomingData.length() > 0) {
      Serial.println(incomingData); // Forward telemetry data to GUI
    }
  }
}

void sendCommands() {
  while (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim(); // Remove any leading/trailing whitespace
    if (command.length() > 0) {
      sendCommandToRover(command);
    }
  }
}

void sendCommandToRover(const String& command) {
  LoRa.beginPacket();
  LoRa.print(command);
  LoRa.endPacket();
  Serial.println("Command forwarded to rover: " + command);
}
