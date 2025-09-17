# Hardware Pin Mappings

## Control Unit (Arduino 1)

### Digital Pins
| Pin | Component | Description | Notes |
|-----|-----------|-------------|-------|
| 2   | Button 2  | Right Turn  | INPUT_PULLUP |
| 3   | Button 3  | Reverse     | INPUT_PULLUP |
| 4   | Button 1  | Forward     | INPUT_PULLUP |
| 5   | Button 4  | Left Turn   | INPUT_PULLUP |

### Analog Pins  
| Pin | Component        | Description   | Range    |
|-----|------------------|---------------|----------|
| A0  | Override Switch  | Slide Switch  | 0-1023   |
| A1  | Potentiometer    | Speed Control | 0-1023   |

### I2C Bus
| Pin | Signal | Component     | Address |
|-----|--------|---------------|---------|
| A4  | SDA    | LCD Display   | 0x27    |
| A5  | SCL    | LCD Display   | 0x27    |

**LCD Specifications**: 16x2 Character Display with I2C Backpack

---

## Motor/Sensor Unit (Arduino 2)

### Motor Control (L293D Driver)
| Pin | Motor | Signal | Description |
|-----|--------|--------|-------------|
| 5   | M3    | IN2    | Direction Control 2 |
| 6   | M3    | IN1    | Direction Control 1 |
| 7   | M1    | IN1    | Direction Control 1 |
| 8   | M1    | IN2    | Direction Control 2 |
| 9   | M3    | EN     | PWM Speed Control |
| 10  | M1    | EN     | PWM Speed Control |
| 11  | M2    | EN     | PWM Speed Control |
| 12  | M2    | IN1    | Direction Control 1 |
| 13  | M2    | IN2    | Direction Control 2 |

**Motor Movement Patterns**:
- **Forward**: All 3 motors rotate forward
- **Reverse**: All 3 motors rotate backward  
- **Right Turn**: Motors 1 and 3 active (Motor 2 off)
- **Left Turn**: Motors 2 and 3 active (Motor 1 off)

### Status LEDs
| Pin | LED | Function | Behavior |
|-----|-----|----------|----------|
| 2   | Emergency LED 2 | Safety Alert | Blinks during emergency |
| 3   | Status LED      | System Status | On when operational |
| 4   | Emergency LED 1 | Safety Alert | Blinks during emergency |

### Sensors - Analog Pins
| Pin | Sensor Type | Quantity | Description |
|-----|-------------|----------|-------------|
| A0  | Gas Sensor  | 1        | Gas Sensor 0 |
| A1  | Temperature | 1        | TMP36 Sensor |
| A2  | Gas Sensor  | 1        | Gas Sensor 1 |
| A3  | Gas Sensor  | 1        | Gas Sensor 2 |
| A4  | Ultrasonic  | 1        | Distance Sensor 0 (single-pin) |
| A5  | Ultrasonic  | 1        | Distance Sensor 1 (single-pin) |

---

## Safety Thresholds (Configurable)

### Temperature Sensor (TMP36)
- **Operating Range**: -50°C to +150°C
- **Emergency Threshold**: 100°C
- **Resolution**: ±0.1°C
- **Formula**: `Temperature = (Voltage - 0.5) × 100`

### Gas Sensors 
- **Operating Range**: 0-1023 (analog reading)
- **Emergency Threshold**: 150
- **Sensor Type**: Generic analog gas sensors
- **Response Time**: Varies by sensor type

### Ultrasonic Sensors (Single-Pin)
- **Operating Range**: 2-400 cm
- **Warning Threshold**: 30 cm  
- **Emergency Threshold**: 10 cm
- **Resolution**: ±1 cm
- **Update Rate**: 200ms intervals

---

## Power Requirements

### Control Unit
- **Voltage**: 5V (via USB or barrel jack)
- **Current**: ~200mA (LCD backlight + logic)
- **Peak Current**: ~300mA

### Motor/Sensor Unit  
- **Logic Voltage**: 5V
- **Motor Voltage**: 6-12V (via L293D VCC2)
- **Logic Current**: ~150mA
- **Motor Current**: Up to 600mA per motor (1.8A total max)
- **Recommended Power Supply**: 12V/3A

---

## Wiring Notes

1. **Serial Connection**: Use crossed TX/RX or USB-TTL adapters
2. **Power Isolation**: Separate 5V logic and motor power supplies recommended
3. **Sensor Calibration**: May require adjustment for specific sensor models
4. **Emergency Stop**: Hardware emergency stop switch can be added to motor power circuit
5. **Cable Length**: Keep sensor cables < 50cm to minimize noise