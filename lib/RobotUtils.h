/*
  Arduino Robot System - Utility Functions
  
  This header file contains utility functions and helper classes
  for sensor processing, data validation, and system operations.
  
  Version: 2.0
*/

#ifndef ROBOT_UTILS_H
#define ROBOT_UTILS_H

#include <Arduino.h>
#include "RobotConfig.h"

// === DATA STRUCTURES ===

struct SensorReading {
  float value;
  unsigned long timestamp;
  bool valid;
  
  SensorReading() : value(0.0), timestamp(0), valid(false) {}
  SensorReading(float v, bool isValid = true) : value(v), timestamp(millis()), valid(isValid) {}
};

struct Vector3D {
  float x, y, z;
  Vector3D() : x(0), y(0), z(0) {}
  Vector3D(float _x, float _y, float _z) : x(_x), y(_y), z(_z) {}
};

// === UTILITY FUNCTIONS ===

/**
 * Temperature sensor processing utilities
 */
namespace TemperatureUtils {
  float analogToTemperature(int analogReading) {
    float voltage = analogReading * (5.0 / 1023.0);
    return (voltage - TEMP_SENSOR_OFFSET) * TEMP_SENSOR_SCALE;
  }
  
  bool isValidTemperature(float temp) {
    return (temp >= MIN_VALID_TEMP && temp <= MAX_VALID_TEMP);
  }
  
  SensorReading readTemperatureSensor(int pin) {
    int reading = analogRead(pin);
    float temp = analogToTemperature(reading);
    bool valid = isValidTemperature(temp);
    return SensorReading(temp, valid);
  }
}

/**
 * Ultrasonic sensor utilities
 */
namespace UltrasonicUtils {
  float readSinglePinUltrasonic(int pin) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
    delayMicroseconds(2);
    digitalWrite(pin, HIGH);
    delayMicroseconds(10);
    digitalWrite(pin, LOW);
    
    pinMode(pin, INPUT);
    long duration = pulseIn(pin, HIGH, ULTRASONIC_TIMEOUT_US);
    
    if (duration == 0) {
      return 0.0; // No echo received
    }
    
    return duration * 0.034 / 2.0;
  }
  
  bool isValidDistance(float distance) {
    return (distance >= MIN_VALID_DISTANCE && distance <= MAX_VALID_DISTANCE);
  }
  
  SensorReading readDistanceSensor(int pin) {
    float distance = readSinglePinUltrasonic(pin);
    bool valid = (distance > 0) && isValidDistance(distance);
    return SensorReading(distance, valid);
  }
}

/**
 * Motor control utilities
 */
namespace MotorUtils {
  int mapSpeed(int potValue) {
    return map(potValue, 0, 1023, MIN_MOTOR_SPEED, MAX_MOTOR_SPEED);
  }
  
  int constrainSpeed(int speed) {
    return constrain(speed, 0, MAX_MOTOR_SPEED);
  }
  
  void setMotorDirection(int motorIndex, int in1Pin, int in2Pin, int enPin, bool forward, int speed) {
    digitalWrite(in1Pin, forward ? HIGH : LOW);
    digitalWrite(in2Pin, forward ? LOW : HIGH);
    analogWrite(enPin, speed);
  }
  
  void stopMotor(int in1Pin, int in2Pin, int enPin) {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
    analogWrite(enPin, 0);
  }
}

/**
 * Safety monitoring utilities
 */
namespace SafetyUtils {
  struct SafetyStatus {
    bool temperatureOk;
    bool gasOk;
    bool distanceOk;
    bool overallSafe;
    String alertMessage;
    
    SafetyStatus() : temperatureOk(true), gasOk(true), distanceOk(true), overallSafe(true), alertMessage("") {}
  };
  
  SafetyStatus evaluateTemperature(float temp) {
    SafetyStatus status;
    status.temperatureOk = (temp <= TEMP_EMERGENCY_THRESHOLD);
    if (!status.temperatureOk) {
      status.overallSafe = false;
      status.alertMessage = "Temperature critical: " + String(temp, 1) + "°C";
    }
    return status;
  }
  
  SafetyStatus evaluateGasLevels(int gasValues[], int numSensors) {
    SafetyStatus status;
    for (int i = 0; i < numSensors; i++) {
      if (gasValues[i] > GAS_EMERGENCY_THRESHOLD) {
        status.gasOk = false;
        status.overallSafe = false;
        status.alertMessage = "Gas sensor " + String(i) + " critical: " + String(gasValues[i]);
        break;
      }
    }
    return status;
  }
  
  SafetyStatus evaluateDistances(float distances[], int numSensors) {
    SafetyStatus status;
    for (int i = 0; i < numSensors; i++) {
      if (distances[i] > 0 && distances[i] <= DISTANCE_EMERGENCY_THRESHOLD) {
        status.distanceOk = false;
        status.overallSafe = false;
        status.alertMessage = "Obstacle detected at " + String(distances[i], 1) + "cm";
        break;
      }
    }
    return status;
  }
}

/**
 * Communication protocol utilities
 */
namespace CommUtils {
  String formatSensorData(float temp, int gasValues[], float distances[]) {
    String data = "SENSORS:T:" + String(temp, 1);
    
    for (int i = 0; i < NUM_GAS_SENSORS; i++) {
      data += ":G" + String(i) + ":" + String(gasValues[i]);
    }
    
    for (int i = 0; i < NUM_ULTRA_SENSORS; i++) {
      data += ":D" + String(i) + ":" + String(distances[i], 1);
    }
    
    return data;
  }
  
  String formatCommand(String movement, int speed) {
    return movement + ":" + String(speed);
  }
  
  bool parseCommand(String input, String &command, int &speed) {
    input.trim();
    int separatorIndex = input.lastIndexOf(':');
    if (separatorIndex != -1) {
      command = input.substring(0, separatorIndex);
      speed = input.substring(separatorIndex + 1).toInt();
      return command.startsWith("CMD:");
    }
    return false;
  }
}

/**
 * Data filtering and smoothing utilities
 */
namespace FilterUtils {
  class SimpleMovingAverage {
    private:
      float* values;
      int size;
      int index;
      int count;
      float sum;
      
    public:
      SimpleMovingAverage(int windowSize) {
        size = windowSize;
        values = new float[size];
        reset();
      }
      
      ~SimpleMovingAverage() {
        delete[] values;
      }
      
      void reset() {
        for (int i = 0; i < size; i++) {
          values[i] = 0.0;
        }
        index = 0;
        count = 0;
        sum = 0.0;
      }
      
      float addValue(float value) {
        if (count < size) {
          values[index] = value;
          sum += value;
          count++;
        } else {
          sum -= values[index];
          values[index] = value;
          sum += value;
        }
        
        index = (index + 1) % size;
        return sum / count;
      }
      
      float getAverage() {
        return (count > 0) ? sum / count : 0.0;
      }
      
      bool isFull() {
        return count == size;
      }
  };
  
  // Simple exponential smoothing
  float exponentialSmooth(float newValue, float oldValue, float alpha = 0.1) {
    return alpha * newValue + (1.0 - alpha) * oldValue;
  }
}

/**
 * System diagnostics utilities
 */
namespace DiagnosticsUtils {
  struct SystemHealth {
    unsigned long uptime;
    unsigned long lastSensorUpdate;
    unsigned long lastCommUpdate;
    bool sensorsHealthy;
    bool commHealthy;
    int freeMemory;
    
    SystemHealth() {
      uptime = millis();
      lastSensorUpdate = 0;
      lastCommUpdate = 0;
      sensorsHealthy = true;
      commHealthy = true;
      freeMemory = 0;
    }
  };
  
  // Estimate free memory (Arduino specific)
  int getFreeMemory() {
    extern int __heap_start, *__brkval;
    int v;
    return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
  }
  
  SystemHealth getSystemHealth() {
    SystemHealth health;
    health.uptime = millis();
    health.freeMemory = getFreeMemory();
    return health;
  }
}

#endif // ROBOT_UTILS_H