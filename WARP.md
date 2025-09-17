# WARP.md

This file provides guidance to WARP (warp.dev) when working with code in this repository.

## Common Commands

### Building and Uploading Code
This project uses the Arduino IDE for compilation and upload. No automated build system is present.

**To compile and upload Control Unit:**
```bash
# Using Arduino CLI (if available)
arduino-cli compile --fqbn arduino:avr:uno src/control_unit/control_unit.ino
arduino-cli upload -p COM3 --fqbn arduino:avr:uno src/control_unit/control_unit.ino
```

**To compile and upload Motor/Sensor Unit:**
```bash
arduino-cli compile --fqbn arduino:avr:uno src/motor_sensor_unit/motor_sensor_unit.ino
arduino-cli upload -p COM4 --fqbn arduino:avr:uno src/motor_sensor_unit/motor_sensor_unit.ino
```

### Testing
No automated test suite exists. Testing is done via Serial Monitor:

**Monitor Control Unit (usually COM3):**
```bash
arduino-cli monitor -p COM3 -c baudrate=9600
```

**Monitor Motor/Sensor Unit (usually COM4):**
```bash
arduino-cli monitor -p COM4 -c baudrate=9600
```

### Validation
**Check code syntax without upload:**
```bash
arduino-cli compile --verify-only --fqbn arduino:avr:uno src/control_unit/control_unit.ino
arduino-cli compile --verify-only --fqbn arduino:avr:uno src/motor_sensor_unit/motor_sensor_unit.ino
```

## Architecture Overview

This is a **dual-Arduino robot control system** designed for fire safety applications with environmental monitoring.

### System Components
- **Control Unit** (`src/control_unit/`): User interface Arduino with LCD, buttons, potentiometer
- **Motor/Sensor Unit** (`src/motor_sensor_unit/`): Movement and sensing Arduino with motors, temperature, gas, and distance sensors
- **Shared Libraries** (`lib/`): Common configuration and utility functions
- **Documentation** (`docs/`): Technical specifications and protocols

### Key Architecture Patterns

**Dual-Unit Communication:**
- Serial communication at 9600 baud between units
- Control Unit sends movement commands in format: `CMD:MOVEMENT_TYPE:SPEED_VALUE`
- Motor Unit sends sensor data in format: `SENSORS:T:XX.X:G0:Y1:G1:Y2:G2:Y3:D0:Z1:D1:Z2`

**Safety-First Design:**
- Emergency stop system triggers on temperature > 100°C, gas > 150, or distance ≤ 10cm
- All motors immediately stop during emergency conditions
- Emergency status persists until all conditions are safe

**Modular Configuration:**
- `RobotConfig.h` contains all system constants and thresholds
- `system_calibration.ini` provides runtime configuration parameters
- Utility functions in `RobotUtils.h` handle sensor processing, safety checks, and communication

### Data Structures
The system uses several key data structures defined in `RobotUtils.h`:
- `SensorReading`: Timestamped sensor values with validation
- `SafetyStatus`: Emergency condition evaluation results
- `SystemHealth`: Diagnostic information for system monitoring

### Communication Protocol
Commands flow Control Unit → Motor Unit:
- `CMD:ALL_FORWARD:512` - Forward movement at ~50% speed
- `CMD:PARTIAL_RIGHT:256` - Right turn at ~25% speed
- `CMD:OFF:0` - Stop all motors

Sensor data flows Motor Unit → Control Unit with individual readings for temperature, 3 gas sensors, and 2 distance sensors.

## Development Guidelines

### Adding New Sensors
1. Update `NUM_*_SENSORS` constants in `RobotConfig.h`
2. Add pin definitions to the appropriate unit's `.ino` file
3. Update sensor reading loops and data formatting in `CommUtils::formatSensorData()`
4. Add new thresholds to safety evaluation functions
5. Update pin mappings documentation

### Modifying Safety Thresholds
- Edit values in `lib/RobotConfig.h` for compile-time changes
- Modify `config/system_calibration.ini` for runtime configuration
- Safety thresholds are checked every 200ms in the motor unit

### Adding New Movement Commands
1. Define command string constant in `RobotConfig.h`
2. Add command processing in Control Unit's `determineMovementCommand()`
3. Implement motor control logic in Motor Unit's `executeCurrentCommand()`
4. Update communication protocol documentation

### Hardware Pin Configuration
Pin mappings are documented in `docs/PIN_MAPPINGS.md`. When changing pins:
- Update pin arrays in the relevant `.ino` file
- Update documentation
- Consider power and timing constraints (especially for PWM pins)

## Important Implementation Details

### Serial Communication Timing
- Sensor readings: 200ms intervals
- Data transmission: 300ms intervals  
- Command sending: 200ms intervals
- Display updates: 500ms intervals
- Communication timeout: 1000ms

### Motor Control Patterns
The system uses 3 motors with L293D drivers:
- **Forward/Reverse**: All 3 motors same direction
- **Right Turn**: Motors 1 & 3 active (Motor 2 off)  
- **Left Turn**: Motors 2 & 3 active (Motor 1 off)
- Speed mapping: 0-1023 potentiometer → 50-255 PWM

### Safety System Implementation
Emergency detection occurs in `checkSafetyConditions()` every sensor read cycle:
1. Temperature evaluation using TMP36 formula: `(voltage - 0.5) × 100`
2. Gas sensor threshold checking (analog readings 0-1023)
3. Ultrasonic distance measurement with single-pin sensors
4. Immediate motor shutdown and LED activation on any violation

### Memory and Performance
- Uses basic Arduino Uno/Nano (ATmega328P)
- No dynamic memory allocation in main loops
- Sensor smoothing available via `FilterUtils::SimpleMovingAverage`
- Memory diagnostic functions in `DiagnosticsUtils`

## Required Libraries
Install these Arduino libraries before compiling:
- `LiquidCrystal_I2C` - For 16x2 LCD display (Control Unit)
- `Wire` - For I2C communication (usually pre-installed)

## File Organization Rules
- **Shared code**: Place constants in `RobotConfig.h`, utilities in `RobotUtils.h`
- **Unit-specific code**: Keep in respective `.ino` files
- **Configuration**: Hardware constants in headers, calibration values in `.ini` files
- **Documentation**: Technical details in `docs/`, user instructions in `README.md`

## Debugging and Diagnostics
Each unit provides Serial output at 9600 baud:
- Control Unit logs connection status and command sending
- Motor Unit logs sensor readings, safety alerts, and emergency conditions
- Enable additional debug output by uncommenting debug Serial.print lines
- Use `DiagnosticsUtils::getSystemHealth()` for memory and uptime monitoring