/*
  Arduino Robot System - Shared Configuration
  
  This header file contains shared constants and configuration parameters
  used by both the Control Unit and Motor/Sensor Unit.
  
  Version: 2.0
*/

#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

// === COMMUNICATION SETTINGS ===
#define SERIAL_BAUD_RATE 9600
#define SERIAL_TIMEOUT_MS 1000

// === TIMING CONSTANTS ===
#define SENSOR_READ_INTERVAL_MS 200
#define DATA_SEND_INTERVAL_MS 300
#define COMMAND_SEND_INTERVAL_MS 200
#define DISPLAY_UPDATE_INTERVAL_MS 500
#define EMERGENCY_BLINK_INTERVAL_MS 250

// === SAFETY THRESHOLDS ===
#define TEMP_EMERGENCY_THRESHOLD 100.0    // °C
#define GAS_EMERGENCY_THRESHOLD 150       // analog reading (0-1023)
#define DISTANCE_WARNING_THRESHOLD 30.0   // cm
#define DISTANCE_EMERGENCY_THRESHOLD 10.0 // cm

// === MOTOR CONFIGURATION ===
#define NUM_MOTORS 3
#define MIN_MOTOR_SPEED 50    // Minimum PWM value for reliable motor operation
#define MAX_MOTOR_SPEED 255   // Maximum PWM value

// === SENSOR CONFIGURATION ===
#define NUM_GAS_SENSORS 3
#define NUM_ULTRA_SENSORS 2
#define TEMP_SENSOR_OFFSET 0.5        // TMP36 offset voltage
#define TEMP_SENSOR_SCALE 100.0       // TMP36 scale factor (10mV/°C)
#define ULTRASONIC_TIMEOUT_US 30000   // 30ms timeout for ultrasonic sensors

// === COMMAND STRINGS ===
#define CMD_FORWARD "CMD:ALL_FORWARD"
#define CMD_REVERSE "CMD:ALL_REVERSE"
#define CMD_RIGHT "CMD:PARTIAL_RIGHT"
#define CMD_LEFT "CMD:PARTIAL_LEFT"
#define CMD_STOP "CMD:OFF"

// === DATA VALIDATION RANGES ===
#define MIN_VALID_TEMP -50.0     // °C
#define MAX_VALID_TEMP 150.0     // °C
#define MIN_VALID_DISTANCE 2.0   // cm
#define MAX_VALID_DISTANCE 400.0 // cm

// === DISPLAY SETTINGS ===
#define LCD_I2C_ADDRESS 0x27
#define LCD_COLUMNS 16
#define LCD_ROWS 2

// === VERSION INFO ===
#define SYSTEM_VERSION "2.0"
#define SYSTEM_NAME "Arduino Robot System"

#endif // ROBOT_CONFIG_H