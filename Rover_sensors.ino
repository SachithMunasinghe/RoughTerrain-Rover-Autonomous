#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>  
#include <ESP32Servo.h>  
#include <Wire.h>          
#include <Adafruit_VL53L0X.h> 
#include <TinyGPSPlus.h>      

// Define I2C pins (default for ESP32)
#define I2C_SDA 21
#define I2C_SCL 22

// GPS UART pins: for example, use UART2 on pins 16 (RX) and 17 (TX)
#define GPS_RX_PIN 16
#define GPS_TX_PIN 17

// Define the pins for the LoRa transceiver module
#define NSS 4
#define RST 5
#define DI0 2

// Define the pins for the motor controllers
#define IN1 25  // Left side motors
#define IN2 26
#define IN3 27  // Right side motors
#define IN4 14
#define ENA 12  // PWM for left motors
#define ENB 13  // PWM for right motors

// Define the pins for the servo motors (gimbal control)
#define SERVO_VERTICAL_PIN 32  // Vertical servo motor
#define SERVO_HORIZONTAL_PIN 33  // Horizontal servo motor

// Define the pin for the head light
#define LIGHT_PIN 15

Adafruit_VL53L0X distanceSensor = Adafruit_VL53L0X();
TinyGPSPlus gps;            // GPS parser
HardwareSerial SerialGPS(2); 


unsigned long lastDataSendTime = 0;
const unsigned long DataInterval = 1000; // 1 second

// Variable to store incoming LoRa data
String LoraData; 

// Track current operating mode; default is "manual"
String currentMode = "manual";

// Servo objects for gimbal control
Servo verticalServo;
Servo horizontalServo;

// The vertical servo will operate only between 120 (max upward) and 170 (max downward)
const int VERTICAL_MIN = 120;
const int VERTICAL_MAX = 160;

// Define horizontal servo limits for the gimbal (in degrees)
const int HORIZONTAL_MIN = 20;
const int HORIZONTAL_MAX = 135;

// Gimbal positions (initialize vertical to center = 35°, horizontal to center = 90°)
int verticalAngle = (VERTICAL_MIN + VERTICAL_MAX) / 2;  // 35
int horizontalAngle = (HORIZONTAL_MIN + HORIZONTAL_MAX) / 2;  // 90

// Forward declarations for rover control
void controlRover(String command);
void goForward(unsigned int speed);
void goBackward(unsigned int speed);
void turnLeft(unsigned int speed);
void turnRight(unsigned int speed);
void stopRover();
void controlGimbal(String command);
void centerGimbal();
void updateServoPositions();
void sendSensorData();
void readGPS();
void autoModeLogic();
uint16_t readDistance();
uint16_t readDistanceAtAngle(int angle);
bool waitWhileCheckingMode(unsigned long totalDelayMs);

void setup() {
  Serial.begin(115200);

  Wire.begin(I2C_SDA, I2C_SCL);
  LoRa.setPins(NSS, RST, DI0);

  // Attempt to begin LoRa at 433 MHz (for Asia region)
  while (!LoRa.begin(433E6)) {
    Serial.println("LoRa initialization failed. Retrying...");
    delay(500);
  }
  LoRa.setSyncWord(0xF2);
  Serial.println("LoRa Initialized Successfully..");

  // Initialize motor controller pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Ensure motors are off initially
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);

  Serial.println("Default mode: MANUAL");

  // Initialize the head light
  pinMode(LIGHT_PIN, OUTPUT);

  // Servo Initialization
  verticalServo.attach(SERVO_VERTICAL_PIN);
  horizontalServo.attach(SERVO_HORIZONTAL_PIN);

  // Center the gimbal at startup
  centerGimbal();

  // Initialize VL53L0X Sensor
  if (!distanceSensor.begin()) {
    Serial.println("Failed to boot VL53L0X sensor!");
    while (1) {
      delay(10);
    }
  }
  //distanceSensor.setTimeout(500);
  Serial.println("VL53L0X Initialized.");

  // Initialize GPS (GY-GPS6M) on UART2
  SerialGPS.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.println("GPS serial started at 9600 baud.");

}

void loop() {
  // Check if there's an incoming LoRa packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    // Read the entire LoRa data as a string
    while (LoRa.available()) {
      LoraData = LoRa.readString();
    }
    LoraData.trim();

    // Handle gimbal commands separately
    if (LoraData.startsWith("cam")) {
      controlGimbal(LoraData);
    } else {
      // Pass the command to controlRover
      controlRover(LoraData);
    }
  }
  readGPS();

  if (currentMode == "auto") {
    autoModeLogic();
  }
  // 2) Periodically read and send obstacle data
  unsigned long now = millis();
  if (now - lastDataSendTime >= DataInterval) {
    lastDataSendTime = now;
    sendSensorData(); 
  }
  delay(1);
}

void readGPS() {
  while (SerialGPS.available() > 0) {
    char c = SerialGPS.read();
    gps.encode(c);
  }
}

void sendSensorData() {
  // 1) Read distance from VL53L0X
  VL53L0X_RangingMeasurementData_t measure;
  distanceSensor.rangingTest(&measure, false);

  // if measurement is out of range, set distance to 9999
  uint16_t distance_mm = (measure.RangeStatus != 4) ? measure.RangeMilliMeter : 9999;

  // 2) Get GPS location
  double latitude  = gps.location.isValid() ? gps.location.lat() : 0.0;
  double longitude = gps.location.isValid() ? gps.location.lng() : 0.0;


  String obstacleData = String("OBSTACLE:") + distance_mm;
  String gpsData = String("GPS:") + String(latitude, 6) + "," + String(longitude, 6);
  
  LoRa.beginPacket();
  LoRa.println(obstacleData);
  LoRa.println(gpsData);
  LoRa.endPacket();

  Serial.print("Sent: ");
  Serial.print(obstacleData);
  Serial.print(" | ");
  Serial.println(gpsData);
}

//GIMBAL CONTROL LOGIC
void controlGimbal(String command) {
  
  if (command == "camdown") {
    verticalAngle = constrain(verticalAngle - 10, VERTICAL_MIN, VERTICAL_MAX);
  } else if (command == "camup") {
    verticalAngle = constrain(verticalAngle + 10, VERTICAL_MIN, VERTICAL_MAX);
  }
  else if (command == "camright") {
    horizontalAngle = constrain(horizontalAngle - 10, HORIZONTAL_MIN, HORIZONTAL_MAX);
  } else if (command == "camleft") {
    horizontalAngle = constrain(horizontalAngle + 10, HORIZONTAL_MIN, HORIZONTAL_MAX);
  } else if (command == "camcenter") {
    centerGimbal();
    return;
  } else {
    Serial.println("Unknown gimbal command: " + command);
    return;
  }

  updateServoPositions();
  Serial.println("Updated Gimbal Position: Vertical=" + String(verticalAngle) + ", Horizontal=" + String(horizontalAngle));
}

void centerGimbal() {
  verticalAngle = (VERTICAL_MIN + VERTICAL_MAX) / 2;
  horizontalAngle = (HORIZONTAL_MIN + HORIZONTAL_MAX) / 2;  
  updateServoPositions();
  Serial.println("Gimbal Centered");
}

void updateServoPositions() {
  verticalServo.write(verticalAngle);
  horizontalServo.write(horizontalAngle);
}

// Decide what to do based on the command and the current mode
void controlRover(String command) {
  command.trim();  // Remove extra spaces/newlines if any

  // 1) Mode switching
  if (command == "manual") {
    currentMode = "manual";
    stopRover(); // stop motors upon switching
    Serial.println("Switched to MANUAL mode");
    return;
  } 
  else if (command == "auto") {
    currentMode = "auto";
    stopRover(); // stop motors upon switching
    Serial.println("Switched to AUTO mode");
    return;
  }

  // If we are in AUTO mode, ignore manual commands
  if (currentMode == "auto") {
    Serial.println("Ignoring command - currently in AUTO mode");
    return;
  }

  // 2) MANUAL mode => interpret movement commands
  if (command == "forward") {
    goForward(255);
  }
  else if (command == "backward") {
    goBackward(255);
  }
  else if (command == "left") {
    turnLeft(255);
  }
  else if (command == "right") {
    turnRight(255);
  }
  else if (command == "stop") {
    stopRover();
  }
  else if (command == "lighton") {
    // Example: turn on LED or headlight
    Serial.println("Light ON (implement as needed).");
    digitalWrite(LIGHT_PIN, HIGH);
  }
  else if (command == "lightoff") {
    // Example: turn off LED or headlight
    Serial.println("Light OFF (implement as needed).");
    digitalWrite(LIGHT_PIN, LOW);
  }
  else {
    Serial.println("Unknown rover command: " + command);
  }
}

// Rover movement functions
void goForward(unsigned int speed) {
  analogWrite(ENA, speed);  // For the left motors
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  analogWrite(ENB, speed);  // For the right motors
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  Serial.println("Rover moving FORWARD");
}

void goBackward(unsigned int speed) {
  analogWrite(ENA, speed);  // For the left motors
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  analogWrite(ENB, speed);  // For the right motors
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  Serial.println("Rover moving BACKWARD");
}

void turnLeft(unsigned int speed) {
  // For a left turn: left side reversed, right side forward
  analogWrite(ENA, 200);   // left side at a slower or reversed speed
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  analogWrite(ENB, speed); // right side normal speed forward
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  Serial.println("Rover turning LEFT");
}

void turnRight(unsigned int speed) {
  // For a right turn: left side forward, right side reversed
  analogWrite(ENA, speed);  // left side normal speed forward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  analogWrite(ENB, 200);    // right side reversed or slower
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  Serial.println("Rover turning RIGHT");
}

void stopRover() {
  // Stop all motors
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);

  Serial.println("Rover STOPPED");
}

void autoModeLogic() {
  if (currentMode != "auto") return;
  uint16_t threshold_mm = 300;

  // 1) Read distance on the left side
  uint16_t distLeft = readDistanceAtAngle(HORIZONTAL_MIN);

  // 3) Read distance in the MIDDLE
  int midAngle = (HORIZONTAL_MIN + HORIZONTAL_MAX) / 2;
  uint16_t distMid = readDistanceAtAngle(midAngle);

  // 2) Read distance on the right side
  uint16_t distRight = readDistanceAtAngle(HORIZONTAL_MAX);

  Serial.println("AUTO SCAN => "
                 "Left: "  + String(distLeft) +
                 " mm, Mid: " + String(distMid) +
                 " mm, Right: " + String(distRight) + " mm");

  bool blockedLeft  = (distLeft  < threshold_mm);
  bool blockedMid   = (distMid   < threshold_mm);
  bool blockedRight = (distRight < threshold_mm);

  if (blockedLeft && blockedMid && blockedRight) {
    // All three blocked -> e.g., turn around or do something
    Serial.println("AUTO => All sides blocked, turning LEFT");
    turnLeft(200);
    if (!waitWhileCheckingMode(600)) {
      stopRover();
      return;
    }
    stopRover();
  }
  else if (blockedMid) {
    // Middle is blocked, so see if left or right is open
    if (blockedLeft && blockedRight) {
      // Both sides are also blocked -> same as above
      turnLeft(200);
      if (!waitWhileCheckingMode(600)) {
      stopRover();
      return;
    }
      stopRover();
    }
    else if (blockedLeft) {
      // Left blocked, mid blocked => try turning RIGHT
      turnRight(200);
      if (!waitWhileCheckingMode(500)) {
      stopRover();
      return;
    }
      stopRover();
    }
    else if (blockedRight) {
      // Right blocked, mid blocked => try turning LEFT
      turnLeft(200);
      if (!waitWhileCheckingMode(500)) {
      stopRover();
      return;
    }
      stopRover();
    }
    else {
      // Mid is blocked, but left or right is open, pick one
      // e.g. pick the side with the greater distance
      if (distLeft > distRight) {
        turnLeft(200);  
      } else {
        turnRight(200);
      }
      if (!waitWhileCheckingMode(500)) {
        stopRover();
        return;
      }
      stopRover();
    }
  }
  else {
    // Middle is clear => go forward
    Serial.println("AUTO => Middle is clear, going FORWARD");
    goForward(200);
    if (!waitWhileCheckingMode(600)) {
      stopRover();
      return;
    }
    stopRover();
  }
}

uint16_t readDistance() {
  VL53L0X_RangingMeasurementData_t measure;
  distanceSensor.rangingTest(&measure, false);
  uint16_t dist_mm = (measure.RangeStatus != 4) 
                     ? measure.RangeMilliMeter 
                     : 9999;
  return dist_mm;
}

uint16_t readDistanceAtAngle(int angle) {
  // Move servo
  horizontalAngle = constrain(angle, HORIZONTAL_MIN, HORIZONTAL_MAX);
  horizontalServo.write(horizontalAngle);

  // Wait for servo to settle
  if (!waitWhileCheckingMode(300)) {
    // If user pressed manual in mid-scan, return a large distance to avoid blocking
    return 9999;
  }

  // Read distance
  return readDistance();
}

bool waitWhileCheckingMode(unsigned long totalDelayMs) {
  unsigned long start = millis();
  while (millis() - start < totalDelayMs) {
    delay(25);  // Short delay to allow real-time LoRa checking

    // Check for new LoRa messages while waiting
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
      LoraData = "";
      while (LoRa.available()) {
        LoraData += LoRa.readString();
      }
      LoraData.trim();
      Serial.print("Received command: '");
      Serial.print(LoraData);
      Serial.println("'");

      if (LoraData == "manual") {
        currentMode = "manual";  // Immediately update mode
        stopRover();
        Serial.println("Switched to MANUAL mode from AUTO");
        return false;  // Exit waiting early
      }
    }

    // If mode changed from another function, exit immediately
    if (currentMode != "auto") {
      Serial.println("Mode changed during wait; exiting early.");
      return false;
    }
  }
  return true;  // Completed the full wait time in auto mode
}

