/*
  Arduino Robot System - Motor/Sensor Unit
  
  This Arduino controls the robot's movement and monitors environmental conditions.
  It receives commands from the control unit and sends back sensor data.
  
  Hardware Requirements:
  - 3 Motors with L293D drivers (pins 5-13, PWM on 9-11)
  - Temperature sensor TMP36 (A1)
  - 3 Gas sensors (A0, A2, A3)
  - 2 Ultrasonic sensors single-pin (A4, A5)
  - Emergency LEDs (pins 2, 4)
  
  Author: Arduino Robot System Project
  Version: 2.0
*/

// === CONFIGURATION ===
// Motor configuration
const int NUM_MOTORS = 3;
const int MOTOR_EN[NUM_MOTORS]  = {10, 11, 9};   // PWM enable pins
const int MOTOR_IN1[NUM_MOTORS] = {7, 12, 6};    // Direction control 1
const int MOTOR_IN2[NUM_MOTORS] = {8, 13, 5};    // Direction control 2

// LED configuration
const int LED_EMERGENCY_1 = 4;
const int LED_EMERGENCY_2 = 2;
const int LED_STATUS = 3;

// Sensor configuration
const int TEMP_PIN = A1;
const int GAS_PINS[3] = {A0, A2, A3};
const int ULTRA_PINS[2] = {A4, A5};
const int NUM_GAS_SENSORS = 3;
const int NUM_ULTRA_SENSORS = 2;

// Safety thresholds (configurable)
const float TEMP_EMERGENCY_THRESHOLD = 100.0;    // °C
const int GAS_EMERGENCY_THRESHOLD = 150;         // analog reading
const float DISTANCE_WARNING_THRESHOLD = 30.0;   // cm
const float DISTANCE_EMERGENCY_THRESHOLD = 10.0; // cm

// Timing configuration
const unsigned long SENSOR_READ_INTERVAL = 200;   // ms
const unsigned long DATA_SEND_INTERVAL = 300;     // ms
const unsigned long EMERGENCY_BLINK_INTERVAL = 250; // ms

// === GLOBAL VARIABLES ===
// System state
struct SystemState {
  bool emergencyMode = false;
  bool emergencyLedState = false;
  unsigned long lastSensorRead = 0;
  unsigned long lastDataSent = 0;
  unsigned long lastEmergencyBlink = 0;
  int currentSpeed = 255;
  String currentCommand = "CMD:OFF";
};

SystemState system;

// Sensor data storage
struct SensorReadings {
  float temperature = 0.0;
  int gasValues[NUM_GAS_SENSORS] = {0};
  float distances[NUM_ULTRA_SENSORS] = {0.0};
  bool temperatureOk = true;
  bool gasOk = true;
  bool distanceOk = true;
};

SensorReadings sensors;

// === INITIALIZATION ===
void setup() {
  Serial.begin(9600);
  
  // Configure motor pins
  setupMotorPins();
  
  // Configure LED pins
  setupLEDPins();
  
  // Initialize system state
  system.lastSensorRead = millis();
  system.lastDataSent = millis();
  system.lastEmergencyBlink = millis();
  
  // Startup sequence
  performStartupSequence();
  
  Serial.println("Motor/Sensor Unit initialized");
}

void setupMotorPins() {
  for (int i = 0; i < NUM_MOTORS; i++) {
    pinMode(MOTOR_EN[i], OUTPUT);
    pinMode(MOTOR_IN1[i], OUTPUT);
    pinMode(MOTOR_IN2[i], OUTPUT);
    
    // Ensure motors start in off state
    digitalWrite(MOTOR_EN[i], LOW);
    digitalWrite(MOTOR_IN1[i], LOW);
    digitalWrite(MOTOR_IN2[i], LOW);
  }
}

void setupLEDPins() {
  pinMode(LED_EMERGENCY_1, OUTPUT);
  pinMode(LED_EMERGENCY_2, OUTPUT);
  pinMode(LED_STATUS, OUTPUT);
  
  // Turn off all LEDs
  digitalWrite(LED_EMERGENCY_1, LOW);
  digitalWrite(LED_EMERGENCY_2, LOW);
  digitalWrite(LED_STATUS, LOW);
}

void performStartupSequence() {
  // LED test sequence
  digitalWrite(LED_STATUS, HIGH);
  delay(200);
  digitalWrite(LED_EMERGENCY_1, HIGH);
  delay(200);
  digitalWrite(LED_EMERGENCY_2, HIGH);
  delay(200);
  
  // Turn off test LEDs
  digitalWrite(LED_STATUS, LOW);
  digitalWrite(LED_EMERGENCY_1, LOW);
  digitalWrite(LED_EMERGENCY_2, LOW);
  
  // Brief motor test (very low speed)
  testMotors();
}

void testMotors() {
  for (int i = 0; i < NUM_MOTORS; i++) {
    digitalWrite(MOTOR_IN1[i], HIGH);
    digitalWrite(MOTOR_IN2[i], LOW);
    analogWrite(MOTOR_EN[i], 50);  // Very low speed
    delay(100);
    analogWrite(MOTOR_EN[i], 0);
    digitalWrite(MOTOR_IN1[i], LOW);
  }
}

// === MAIN LOOP ===
void loop() {
  unsigned long currentTime = millis();
  
  // Read sensors at regular intervals
  if (currentTime - system.lastSensorRead >= SENSOR_READ_INTERVAL) {
    readAllSensors();
    system.lastSensorRead = currentTime;
  }
  
  // Check safety conditions
  checkSafetyConditions();
  
  // Send sensor data
  if (currentTime - system.lastDataSent >= DATA_SEND_INTERVAL) {
    sendSensorData();
    system.lastDataSent = currentTime;
  }
  
  // Process incoming commands
  processIncomingCommands();
  
  // Handle emergency LED blinking
  handleEmergencyLEDs(currentTime);
  
  // Execute movement commands (only if safe)
  if (!system.emergencyMode) {
    executeCurrentCommand();
    digitalWrite(LED_STATUS, HIGH);
  } else {
    stopAllMotors();
    digitalWrite(LED_STATUS, LOW);
  }
}

// === SENSOR READING ===
void readAllSensors() {
  // Read temperature
  sensors.temperature = readTemperature();
  
  // Read gas sensors
  for (int i = 0; i < NUM_GAS_SENSORS; i++) {
    sensors.gasValues[i] = analogRead(GAS_PINS[i]);
  }
  
  // Read ultrasonic sensors
  for (int i = 0; i < NUM_ULTRA_SENSORS; i++) {
    sensors.distances[i] = readUltrasonicDistance(ULTRA_PINS[i]);
  }
}

float readTemperature() {
  int reading = analogRead(TEMP_PIN);
  float voltage = reading * (5.0 / 1023.0);
  float temperatureC = (voltage - 0.5) * 100.0;
  
  // Basic range validation
  if (temperatureC < -50 || temperatureC > 150) {
    temperatureC = sensors.temperature; // Keep last valid reading
  }
  
  return temperatureC;
}

float readUltrasonicDistance(int pin) {
  // Single-pin ultrasonic sensor technique
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  delayMicroseconds(2);
  digitalWrite(pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(pin, LOW);
  
  pinMode(pin, INPUT);
  long duration = pulseIn(pin, HIGH, 30000); // 30ms timeout
  
  if (duration == 0) {
    return 0.0; // No echo received
  }
  
  float distance = duration * 0.034 / 2.0;
  
  // Range validation (typical HC-SR04 range: 2-400cm)
  if (distance < 2.0 || distance > 400.0) {
    return 0.0;
  }
  
  return distance;
}

// === SAFETY MONITORING ===
void checkSafetyConditions() {
  // Check temperature
  sensors.temperatureOk = (sensors.temperature <= TEMP_EMERGENCY_THRESHOLD);
  
  // Check gas levels
  sensors.gasOk = true;
  for (int i = 0; i < NUM_GAS_SENSORS; i++) {
    if (sensors.gasValues[i] > GAS_EMERGENCY_THRESHOLD) {
      sensors.gasOk = false;
      break;
    }
  }
  
  // Check distances
  sensors.distanceOk = true;
  for (int i = 0; i < NUM_ULTRA_SENSORS; i++) {
    if (sensors.distances[i] > 0 && sensors.distances[i] <= DISTANCE_EMERGENCY_THRESHOLD) {
      sensors.distanceOk = false;
      break;
    }
  }
  
  // Determine emergency state
  bool previousEmergency = system.emergencyMode;
  system.emergencyMode = !sensors.temperatureOk || !sensors.gasOk || !sensors.distanceOk;
  
  // Log emergency state changes
  if (system.emergencyMode && !previousEmergency) {
    Serial.println("EMERGENCY: Safety threshold exceeded!");
    logEmergencyConditions();
  } else if (!system.emergencyMode && previousEmergency) {
    Serial.println("RECOVERY: Safety conditions restored");
  }
}

void logEmergencyConditions() {
  if (!sensors.temperatureOk) {
    Serial.print("Temperature critical: ");
    Serial.println(sensors.temperature);
  }
  
  if (!sensors.gasOk) {
    Serial.print("Gas levels critical: ");
    for (int i = 0; i < NUM_GAS_SENSORS; i++) {
      Serial.print("G");
      Serial.print(i);
      Serial.print(":");
      Serial.print(sensors.gasValues[i]);
      Serial.print(" ");
    }
    Serial.println();
  }
  
  if (!sensors.distanceOk) {
    Serial.print("Obstacle detected: ");
    for (int i = 0; i < NUM_ULTRA_SENSORS; i++) {
      Serial.print("D");
      Serial.print(i);
      Serial.print(":");
      Serial.print(sensors.distances[i]);
      Serial.print("cm ");
    }
    Serial.println();
  }
}

// === COMMUNICATION ===
void sendSensorData() {
  // Enhanced format: SENSORS:T:XX.X:G0:Y1:G1:Y2:G2:Y3:D0:Z1:D1:Z2
  Serial.print("SENSORS:T:");
  Serial.print(sensors.temperature, 1);
  
  for (int i = 0; i < NUM_GAS_SENSORS; i++) {
    Serial.print(":G");
    Serial.print(i);
    Serial.print(":");
    Serial.print(sensors.gasValues[i]);
  }
  
  for (int i = 0; i < NUM_ULTRA_SENSORS; i++) {
    Serial.print(":D");
    Serial.print(i);
    Serial.print(":");
    Serial.print(sensors.distances[i], 1);
  }
  
  Serial.println();
}

void processIncomingCommands() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    // Parse command and speed
    int separatorIndex = command.lastIndexOf(':');
    if (separatorIndex != -1) {
      String cmd = command.substring(0, separatorIndex);
      int speedValue = command.substring(separatorIndex + 1).toInt();
      
      if (cmd.startsWith("CMD:")) {
        system.currentCommand = cmd;
        system.currentSpeed = map(speedValue, 0, 1023, 50, 255); // Min speed 50
        
        // Clamp speed to valid range
        system.currentSpeed = constrain(system.currentSpeed, 0, 255);
      }
    }
  }
}

// === MOTOR CONTROL ===
void executeCurrentCommand() {
  if (system.currentCommand == "CMD:ALL_FORWARD") {
    moveAllMotors(true, system.currentSpeed);
  } else if (system.currentCommand == "CMD:ALL_REVERSE") {
    moveAllMotors(false, system.currentSpeed);
  } else if (system.currentCommand == "CMD:PARTIAL_RIGHT") {
    turnRight(system.currentSpeed);
  } else if (system.currentCommand == "CMD:PARTIAL_LEFT") {
    turnLeft(system.currentSpeed);
  } else {
    stopAllMotors();
  }
}

void moveAllMotors(bool forward, int speed) {
  for (int i = 0; i < NUM_MOTORS; i++) {
    digitalWrite(MOTOR_IN1[i], forward ? HIGH : LOW);
    digitalWrite(MOTOR_IN2[i], forward ? LOW : HIGH);
    analogWrite(MOTOR_EN[i], speed);
  }
}

void turnRight(int speed) {
  // Motors 0 and 2 active for right turn
  digitalWrite(MOTOR_IN1[0], HIGH);
  digitalWrite(MOTOR_IN2[0], LOW);
  analogWrite(MOTOR_EN[0], speed);
  
  digitalWrite(MOTOR_IN1[2], HIGH);
  digitalWrite(MOTOR_IN2[2], LOW);
  analogWrite(MOTOR_EN[2], speed);
  
  // Motor 1 off
  digitalWrite(MOTOR_IN1[1], LOW);
  digitalWrite(MOTOR_IN2[1], LOW);
  analogWrite(MOTOR_EN[1], 0);
}

void turnLeft(int speed) {
  // Motors 1 and 2 active for left turn
  digitalWrite(MOTOR_IN1[1], HIGH);
  digitalWrite(MOTOR_IN2[1], LOW);
  analogWrite(MOTOR_EN[1], speed);
  
  digitalWrite(MOTOR_IN1[2], HIGH);
  digitalWrite(MOTOR_IN2[2], LOW);
  analogWrite(MOTOR_EN[2], speed);
  
  // Motor 0 off
  digitalWrite(MOTOR_IN1[0], LOW);
  digitalWrite(MOTOR_IN2[0], LOW);
  analogWrite(MOTOR_EN[0], 0);
}

void stopAllMotors() {
  for (int i = 0; i < NUM_MOTORS; i++) {
    digitalWrite(MOTOR_IN1[i], LOW);
    digitalWrite(MOTOR_IN2[i], LOW);
    analogWrite(MOTOR_EN[i], 0);
  }
}

// === LED MANAGEMENT ===
void handleEmergencyLEDs(unsigned long currentTime) {
  if (system.emergencyMode) {
    // Blink emergency LEDs
    if (currentTime - system.lastEmergencyBlink >= EMERGENCY_BLINK_INTERVAL) {
      system.emergencyLedState = !system.emergencyLedState;
      digitalWrite(LED_EMERGENCY_1, system.emergencyLedState);
      digitalWrite(LED_EMERGENCY_2, system.emergencyLedState);
      system.lastEmergencyBlink = currentTime;
    }
  } else {
    // Turn off emergency LEDs
    digitalWrite(LED_EMERGENCY_1, LOW);
    digitalWrite(LED_EMERGENCY_2, LOW);
    system.emergencyLedState = false;
  }
}