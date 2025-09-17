# Arduino Robot System v2.0

A dual-Arduino robot control system with environmental monitoring and safety features, designed for fire safety applications.

## 🚀 Features

- **Dual-Arduino Architecture**: Separate control and motor/sensor units for modularity
- **Environmental Monitoring**: Temperature, gas, and proximity sensors
- **Safety-First Design**: Automatic emergency stop on hazardous conditions  
- **User Interface**: LCD display with button controls and speed adjustment
- **Real-time Communication**: Serial protocol between units
- **Modular Code Structure**: Shared libraries and configuration files
- **Enhanced Error Handling**: Timeout detection, data validation, sensor smoothing

## 📁 Project Structure

```
arduino-robot-system/
├── src/
│   ├── control_unit/           # Control Unit Arduino sketch
│   │   └── control_unit.ino
│   └── motor_sensor_unit/      # Motor/Sensor Unit Arduino sketch
│       └── motor_sensor_unit.ino
├── lib/                        # Shared libraries  
│   ├── RobotConfig.h          # Configuration constants
│   └── RobotUtils.h           # Utility functions
├── config/                     # Configuration files
│   └── system_calibration.ini # Calibration parameters
├── docs/                       # Documentation
│   ├── SYSTEM_ARCHITECTURE.md
│   ├── COMMUNICATION_PROTOCOL.md
│   └── PIN_MAPPINGS.md
└── README.md
```

## 🔧 Hardware Requirements

### Control Unit (Arduino 1)
- **Arduino Uno/Nano** (or compatible)
- **16x2 LCD with I2C backpack** (address 0x27)
- **4 Push buttons** (momentary)
- **1 Potentiometer** (10kΩ)
- **1 Slide switch** (SPDT)
- **Breadboard and jumper wires**

### Motor/Sensor Unit (Arduino 2)  
- **Arduino Uno/Nano** (or compatible)
- **3 DC Motors** with **L293D motor driver**
- **Temperature sensor** (TMP36 or similar)
- **3 Gas sensors** (analog output)
- **2 Ultrasonic sensors** (single-pin type like HC-SR04)
- **3 LEDs** (status and emergency indicators)
- **Power supply** (12V/3A recommended for motors)

## ⚡ Quick Start

### 1. Hardware Setup

**Control Unit Connections:**
```
Arduino Pin → Component
2, 3, 4, 5  → Push buttons (with pull-up resistors)
A0          → Slide switch
A1          → Potentiometer center pin
A4          → LCD SDA (I2C)
A5          → LCD SCL (I2C)
```

**Motor/Sensor Unit Connections:**
```
Arduino Pin → Component
5, 6        → Motor 3 (L293D IN1, IN2)
7, 8        → Motor 1 (L293D IN1, IN2)  
9, 10, 11   → Motor PWM (L293D Enable pins)
12, 13      → Motor 2 (L293D IN1, IN2)
2, 3, 4     → Status LEDs
A0, A2, A3  → Gas sensors
A1          → Temperature sensor (TMP36)
A4, A5      → Ultrasonic sensors
```

### 2. Software Installation

1. **Clone or download** this repository
2. **Install Arduino IDE** (version 1.8+ or Arduino IDE 2.0+)
3. **Install required libraries:**
   - `LiquidCrystal_I2C` (for LCD display)
   - `Wire` (usually pre-installed)

4. **Copy library files:**
   - Copy `lib/RobotConfig.h` and `lib/RobotUtils.h` to your Arduino libraries folder
   - Or place them in the same directory as your `.ino` files

### 3. Upload Code

1. **Upload `src/control_unit/control_unit.ino`** to Arduino 1
2. **Upload `src/motor_sensor_unit/motor_sensor_unit.ino`** to Arduino 2
3. **Connect the serial communication** between the two Arduinos:
   - Arduino 1 TX → Arduino 2 RX
   - Arduino 1 RX → Arduino 2 TX
   - Common GND

### 4. Power On & Test

1. **Power both units** (USB for control unit, external supply for motor unit)
2. **Check LCD display** - should show startup message then sensor data
3. **Test buttons** - each should trigger different movements
4. **Test override switch** - should enable continuous forward movement
5. **Test safety features** - heat up temperature sensor or block ultrasonic sensors

## 🎮 Operation

### Control Interface

**Buttons:**
- **Button 1**: Forward movement
- **Button 2**: Right turn  
- **Button 3**: Reverse movement
- **Button 4**: Left turn

**Controls:**
- **Potentiometer**: Adjusts movement speed (0-100%)
- **Override Switch**: When ON, forces continuous forward movement
- **LCD Display**: Shows temperature, gas levels, distance, and system status

### Safety Features

The system automatically stops all motors when:
- **Temperature exceeds 100°C**
- **Any gas sensor reading > 150** 
- **Any obstacle detected within 10cm**

**Emergency indicators:**
- Blinking red LEDs on motor unit
- "EMERGENCY" status on LCD
- All motors immediately stopped

## 🔧 Configuration & Calibration

### Adjusting Safety Thresholds

Edit values in `lib/RobotConfig.h`:
```cpp
#define TEMP_EMERGENCY_THRESHOLD 100.0    // °C
#define GAS_EMERGENCY_THRESHOLD 150       // analog reading  
#define DISTANCE_EMERGENCY_THRESHOLD 10.0 // cm
```

### Sensor Calibration

For detailed calibration procedures, see `config/system_calibration.ini` and the calibration guide in `docs/`.

**Common adjustments:**
- **Temperature**: Verify TMP36 offset voltage (typically 0.5V)
- **Gas sensors**: Calibrate baseline readings in clean air
- **Distance sensors**: Check measurement accuracy against known distances

## 📊 System Monitoring

### Serial Monitor Output

Both units provide diagnostic information via Serial (9600 baud):

**Control Unit:**
```
Control Unit initialized
Connecting...
```

**Motor/Sensor Unit:**  
```
Motor/Sensor Unit initialized
SENSORS:T:23.5:G0:120:G1:135:G2:140:D0:45.0:D1:50.3
EMERGENCY: Safety threshold exceeded!
Temperature critical: 105.2
```

## 🚨 Troubleshooting

### Common Issues

**LCD shows "Connecting..."**
- Check serial wiring between Arduinos
- Verify baud rate matches (9600) in both sketches
- Ensure common ground connection

**Motors don't respond**
- Verify L293D wiring and power supply  
- Check emergency status - motors disabled during safety alerts
- Test with Serial monitor to see received commands

**Erratic sensor readings**
- Check sensor power supply (stable 5V)
- Verify analog pin connections
- Consider sensor filtering/averaging for noisy environments

**Emergency mode won't clear**
- Check all safety conditions (temp, gas, distance)
- Verify sensor calibration and thresholds
- Monitor Serial output for specific error messages

### Advanced Diagnostics

Enable debug mode by uncommenting debug lines in the code:
```cpp
// Serial.print(">> Temp_Analog: "); Serial.println(reading);
```

## 🔄 System Improvements in v2.0

### From Original Code
- **Enhanced sensor data format**: Individual sensor readings vs aggregated values
- **Better error handling**: Timeout detection, data validation, recovery mechanisms  
- **Modular architecture**: Shared libraries, configuration files, utility functions
- **Improved safety logic**: Structured emergency detection and logging
- **Advanced motor control**: Speed ramping, startup tests, precise turning patterns
- **Better user interface**: Status indicators, connection monitoring, diagnostic displays

### New Features
- **Data filtering**: Moving averages and exponential smoothing for sensor stability
- **System health monitoring**: Memory usage, uptime, communication status
- **Configurable parameters**: Runtime adjustable via configuration files  
- **Predictive safety**: Optional predictive algorithms for enhanced safety
- **Enhanced documentation**: Comprehensive guides, wiring diagrams, troubleshooting

## 🤝 Contributing

1. **Fork the repository**
2. **Create a feature branch**: `git checkout -b feature/amazing-feature`
3. **Commit changes**: `git commit -m 'Add amazing feature'`  
4. **Push to branch**: `git push origin feature/amazing-feature`
5. **Open a Pull Request**

## 📝 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🆘 Support

- **Documentation**: Check the `docs/` folder for detailed technical information
- **Issues**: Report bugs or request features via GitHub Issues
- **Community**: Join discussions in GitHub Discussions

## 🏗️ Future Enhancements

### Planned Features
- **Wireless communication**: Replace serial with WiFi/Bluetooth
- **Mobile app control**: Smartphone interface for remote operation
- **Advanced sensors**: Camera integration, IMU for navigation
- **Machine learning**: Adaptive safety thresholds and predictive maintenance
- **Data logging**: SD card storage for sensor history and analysis
- **Web dashboard**: Real-time monitoring and configuration via web interface

### Hardware Upgrades
- **More powerful microcontroller**: ESP32 for additional features
- **Better motors**: Encoders for precise positioning and speed control
- **Advanced sensors**: Higher accuracy temperature and gas sensors
- **Power management**: Battery monitoring and charging system

---

**Version**: 2.0  
**Last Updated**: December 2024  
**Authors**: Arduino Robot System Project Team