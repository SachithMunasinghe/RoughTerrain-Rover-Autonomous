#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>  // Using the LoRa library by Sandeep Mistry
#include <ESP32Servo.h>  // Servo library for controlling gimbal motors
#include <VL53L0X.h>

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
#define SERVO_VERTICAL_PIN 16  // Vertical servo motor
#define SERVO_HORIZONTAL_PIN 17  // Horizontal servo motor

const int OBSTACLE_THRESHOLD = 100; // mm; if measured distance is below this, obstacle detected.
const int SCAN_MIN = 20;            // Minimum horizontal servo angle (predefined above)
const int SCAN_MAX = 135;           // Maximum horizontal servo angle (predefined above)
const int SCAN_STEP = 15;           // Angle step when sweeping the horizontal servo

// Variable to store incoming LoRa data
String LoraData; 

// Track current operating mode; default is "manual"
String currentMode = "manual";

// Servo objects for gimbal control
Servo verticalServo;
Servo horizontalServo;

VL53L0X tofSensor;
// Timer for auto obstacle avoidance routines
unsigned long lastAutoActionTime = 0;
const unsigned long autoActionInterval = 200; // Milliseconds between each auto action

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
void scanAndDecideObstacle();

void setup() {
  Serial.begin(115200);

  // Setup the LoRa transceiver pins
  LoRa.setPins(NSS, RST, DI0);

  // Attempt to begin LoRa at 433 MHz (for Asia region)
  while (!LoRa.begin(433E6)) {
    Serial.println("LoRa initialization failed. Retrying...");
    delay(500);
  }

  // Set sync word (ensures communication between paired devices only)
  LoRa.setSyncWord(0xF2);

  // OPTIONAL: If you want a faster data rate (lower SF, higher BW):
  // LoRa.setSpreadingFactor(7);    // faster transmissions, less range
  // LoRa.setSignalBandwidth(250E3);
  // LoRa.setCodingRate4(5);

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

  // Servo Initialization
  verticalServo.attach(SERVO_VERTICAL_PIN);
  horizontalServo.attach(SERVO_HORIZONTAL_PIN);

  // Center the gimbal at startup
  centerGimbal();

  // Initialize VL53L0X Sensor
  Wire.begin();
  tofSensor.setTimeout(500);
  if (!tofSensor.init()) {
    Serial.println("VL53L0X initialization failed!");
    while (1);  // Halt execution if sensor is not detected.
  }
  tofSensor.startContinuous();
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
  /// If in AUTO mode, run obstacle avoidance routine
  if (currentMode == "auto") {
    if (millis() - lastAutoActionTime >= autoActionInterval) {
      scanAndDecideObstacle();
      lastAutoActionTime = millis();
    }
  }
  // Shorter delay for near-real-time checks
  // was delay(50); now delay(5)
  delay(5);
}

//GIMBAL CONTROL LOGIC
void controlGimbal(String command) {
  // Vertical movement:
  // "camup" decreases the vertical angle to tilt the camera up (but not below VERTICAL_MIN)
  // "camdown" increases the vertical angle to tilt the camera down (but not above VERTICAL_MAX)
  if (command == "camdown") {
    verticalAngle = constrain(verticalAngle - 10, VERTICAL_MIN, VERTICAL_MAX);
  } else if (command == "camup") {
    verticalAngle = constrain(verticalAngle + 10, VERTICAL_MIN, VERTICAL_MAX);
  }
  /// Horizontal movement adjustments:
  // "camleft" decreases horizontal angle but not below HORIZONTAL_MIN
  // "camright" increases horizontal angle but not above HORIZONTAL_MAX
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
  // Center vertical at the midpoint between VERTICAL_MIN and VERTICAL_MAX (145°)
  verticalAngle = (VERTICAL_MIN + VERTICAL_MAX) / 2;
  horizontalAngle = (HORIZONTAL_MIN + HORIZONTAL_MAX) / 2;  // Center horizontal remains unchanged
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
  else {
    Serial.print("Unknown manual command: ");
    Serial.println(command);
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

void scanAndDecideObstacle() {
  // Before starting, check if we're still in auto mode
  if (currentMode != "auto") return;
  
  // Center the gimbal using the defined limits
  centerGimbal();
  delay(200);  // Allow time for the servo to reach the center
  
  int bestAngle = horizontalAngle;  // Start with the current centered angle
  int maxDistance = 0;
  
  // Sweep through horizontal servo angles from SCAN_MIN to SCAN_MAX
  for (int angle = SCAN_MIN; angle <= SCAN_MAX; angle += SCAN_STEP) {
    // If mode has changed during the scan, exit immediately.
    if (currentMode != "auto") return;

    horizontalServo.write(angle);
    delay(300);  // Allow time for servo movement
    
    int distance = tofSensor.readRangeContinuousMillimeters();
    if (tofSensor.timeoutOccurred()) {
      Serial.print("Sensor timeout at angle: ");
      Serial.println(angle);
      continue;  // Skip this reading if sensor times out
    }
    
    Serial.print("Angle ");
    Serial.print(angle);
    Serial.print(" -> Distance: ");
    Serial.println(distance);
    
    // Record this angle if the sensor reading is greater than previous maxDistance
    if (distance > maxDistance) {
      maxDistance = distance;
      bestAngle = angle;
    }
  }
  
  // Point the horizontal servo to the best angle found
  horizontalServo.write(bestAngle);
  horizontalAngle = bestAngle;
  Serial.print("Best Angle: ");
  Serial.print(bestAngle);
  Serial.print(" with Distance: ");
  Serial.println(maxDistance);
  
  // Decide on movement based on the measured maximum distance
  if (maxDistance > OBSTACLE_THRESHOLD) {
    goForward(255);
    Serial.println("Auto Mode: Clear path detected. Moving forward.");
  } else {
    turnRight(255);
    Serial.println("Auto Mode: Obstacle detected. Turning right.");
  }
}
