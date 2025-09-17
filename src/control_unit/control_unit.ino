/*
  Arduino Robot System - Control Unit
  
  This Arduino serves as the user interface and command center for the robot system.
  It handles user input, displays sensor data, and sends movement commands to the motor unit.
  
  Hardware Requirements:
  - LCD I2C Display (16x2, address 0x27)
  - 4 Control Buttons (pins 2, 3, 4, 5) 
  - Potentiometer (A1) - Speed control
  - Slide Switch (A0) - Override mode
  
  Author: Arduino Robot System Project
  Version: 2.0
*/

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// === CONFIGURATION ===
const int LCD_ADDRESS = 0x27;
const int LCD_COLS = 16;
const int LCD_ROWS = 2;

// Pin definitions
const int BUTTON_PINS[] = {4, 2, 3, 5};  // Forward, Right, Reverse, Left
const int NUM_BUTTONS = 4;
const int POT_PIN = A1;
const int OVERRIDE_SWITCH_PIN = A0;

// Timing constants
const unsigned long DISPLAY_UPDATE_INTERVAL = 500;  // ms
const unsigned long COMMAND_SEND_INTERVAL = 200;    // ms
const unsigned long SERIAL_TIMEOUT = 1000;          // ms

// === GLOBAL VARIABLES ===
LiquidCrystal_I2C lcd(LCD_ADDRESS, LCD_COLS, LCD_ROWS);

// Sensor data from motor unit
struct SensorData {
  float temperature = 0.0;
  int gasValues[3] = {0, 0, 0};
  float distances[2] = {0.0, 0.0};
  unsigned long lastUpdate = 0;
  bool dataValid = false;
};

SensorData sensors;

// Control state
struct ControlState {
  bool overrideActive = false;
  int potValue = 0;
  bool buttonPressed[NUM_BUTTONS] = {false, false, false, false};
  String lastCommand = "CMD:OFF";
  unsigned long lastCommandSent = 0;
  unsigned long lastDisplayUpdate = 0;
};

ControlState control;

// === INITIALIZATION ===
void setup() {
  Serial.begin(9600);
  
  // Initialize LCD
  lcd.init();
  lcd.backlight();
  displayStartupMessage();
  
  // Configure input pins
  setupInputPins();
  
  // Initialize control state
  control.lastCommandSent = millis();
  control.lastDisplayUpdate = millis();
  
  Serial.println("Control Unit initialized");
}

void setupInputPins() {
  for (int i = 0; i < NUM_BUTTONS; i++) {
    pinMode(BUTTON_PINS[i], INPUT_PULLUP);
  }
  pinMode(OVERRIDE_SWITCH_PIN, INPUT_PULLUP);
}

void displayStartupMessage() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Robot Control");
  lcd.setCursor(0, 1);
  lcd.print("System v2.0");
  delay(2000);
  lcd.clear();
}

// === MAIN LOOP ===
void loop() {
  unsigned long currentTime = millis();
  
  // Read inputs
  readInputs();
  
  // Process serial data from motor unit
  processSerialData();
  
  // Send commands to motor unit
  if (currentTime - control.lastCommandSent >= COMMAND_SEND_INTERVAL) {
    sendMovementCommand();
    control.lastCommandSent = currentTime;
  }
  
  // Update display
  if (currentTime - control.lastDisplayUpdate >= DISPLAY_UPDATE_INTERVAL) {
    updateDisplay();
    control.lastDisplayUpdate = currentTime;
  }
  
  // Check for communication timeout
  checkCommunicationTimeout(currentTime);
}

// === INPUT HANDLING ===
void readInputs() {
  // Read potentiometer
  control.potValue = analogRead(POT_PIN);
  
  // Read override switch (LOW when active due to INPUT_PULLUP)
  control.overrideActive = (digitalRead(OVERRIDE_SWITCH_PIN) == LOW);
  
  // Read buttons
  for (int i = 0; i < NUM_BUTTONS; i++) {
    control.buttonPressed[i] = (digitalRead(BUTTON_PINS[i]) == LOW);
  }
}

// === COMMAND PROCESSING ===
String determineMovementCommand() {
  if (control.overrideActive) {
    return "CMD:ALL_FORWARD";
  }
  
  // Priority: Forward > Right > Reverse > Left
  if (control.buttonPressed[0]) return "CMD:ALL_FORWARD";
  if (control.buttonPressed[1]) return "CMD:PARTIAL_RIGHT"; 
  if (control.buttonPressed[2]) return "CMD:ALL_REVERSE";
  if (control.buttonPressed[3]) return "CMD:PARTIAL_LEFT";
  
  return "CMD:OFF";
}

void sendMovementCommand() {
  String command = determineMovementCommand();
  
  // Only send if command changed or it's been a while
  if (command != control.lastCommand || 
      millis() - control.lastCommandSent > 1000) {
    
    Serial.println(command + ":" + String(control.potValue));
    control.lastCommand = command;
  }
}

// === SERIAL COMMUNICATION ===
void processSerialData() {
  if (Serial.available()) {
    String data = Serial.readStringUntil('\n');
    data.trim();
    
    if (data.startsWith("SENSORS:")) {
      parseSensorData(data);
    }
  }
}

void parseSensorData(const String& data) {
  // Parse enhanced format: SENSORS:T:XX.X:G0:Y1:G1:Y2:G2:Y3:D0:Z1:D1:Z2
  sensors.dataValid = false;
  
  int tempIdx = data.indexOf(":T:") + 3;
  if (tempIdx > 2) {
    int nextIdx = data.indexOf(":", tempIdx);
    if (nextIdx > tempIdx) {
      sensors.temperature = data.substring(tempIdx, nextIdx).toFloat();
      
      // Parse gas sensors
      for (int i = 0; i < 3; i++) {
        String gasTag = ":G" + String(i) + ":";
        int gasIdx = data.indexOf(gasTag);
        if (gasIdx >= 0) {
          gasIdx += gasTag.length();
          int gasEnd = data.indexOf(":", gasIdx);
          if (gasEnd > gasIdx) {
            sensors.gasValues[i] = data.substring(gasIdx, gasEnd).toInt();
          }
        }
      }
      
      // Parse distance sensors
      for (int i = 0; i < 2; i++) {
        String distTag = ":D" + String(i) + ":";
        int distIdx = data.indexOf(distTag);
        if (distIdx >= 0) {
          distIdx += distTag.length();
          int distEnd = (i == 1) ? data.length() : data.indexOf(":", distIdx);
          if (distEnd > distIdx) {
            sensors.distances[i] = data.substring(distIdx, distEnd).toFloat();
          }
        }
      }
      
      sensors.lastUpdate = millis();
      sensors.dataValid = true;
    }
  }
}

void checkCommunicationTimeout(unsigned long currentTime) {
  if (sensors.dataValid && 
      currentTime - sensors.lastUpdate > SERIAL_TIMEOUT) {
    sensors.dataValid = false;
  }
}

// === DISPLAY MANAGEMENT ===
void updateDisplay() {
  lcd.clear();
  
  if (!sensors.dataValid) {
    displayConnectionStatus();
    return;
  }
  
  // Display sensor data
  displaySensorData();
}

void displayConnectionStatus() {
  lcd.setCursor(0, 0);
  lcd.print("Connecting...");
  lcd.setCursor(0, 1);
  
  if (control.overrideActive) {
    lcd.print("OVERRIDE ACTIVE");
  } else {
    lcd.print("Speed: ");
    lcd.print(map(control.potValue, 0, 1023, 0, 100));
    lcd.print("%");
  }
}

void displaySensorData() {
  // Line 1: Temperature and highest gas reading
  lcd.setCursor(0, 0);
  lcd.print("T:");
  lcd.print(sensors.temperature, 1);
  lcd.print((char)223);  // Degree symbol
  lcd.print("C G:");
  
  int maxGas = max(max(sensors.gasValues[0], sensors.gasValues[1]), sensors.gasValues[2]);
  lcd.print(maxGas);
  
  // Line 2: Minimum distance and status
  lcd.setCursor(0, 1);
  float minDist = min(sensors.distances[0], sensors.distances[1]);
  lcd.print("Dist:");
  lcd.print(minDist, 0);
  lcd.print("cm ");
  
  // Status indicator
  if (control.overrideActive) {
    lcd.print("OVR");
  } else if (control.lastCommand != "CMD:OFF") {
    lcd.print("MOV");
  } else {
    lcd.print("STP");
  }
}