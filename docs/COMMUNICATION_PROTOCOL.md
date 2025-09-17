# Communication Protocol Documentation

## Overview
The Arduino Robot System uses Serial communication (UART) at 9600 baud between the Control Unit and Motor/Sensor Unit.

## Message Formats

### Commands (Control Unit → Motor/Sensor Unit)

**Format**: `CMD:MOVEMENT_TYPE:SPEED_VALUE`

**Examples**:
- `CMD:ALL_FORWARD:512` - Move forward at ~50% speed
- `CMD:ALL_REVERSE:1023` - Move backward at full speed
- `CMD:PARTIAL_RIGHT:256` - Turn right at ~25% speed
- `CMD:PARTIAL_LEFT:768` - Turn left at ~75% speed
- `CMD:OFF:0` - Stop all motors

**Speed Mapping**:
- Input range: 0-1023 (potentiometer reading)
- Output range: 50-255 (PWM values, minimum speed 50)

### Sensor Data (Motor/Sensor Unit → Control Unit)

**Enhanced Format**: `SENSORS:T:XX.X:G0:YYY:G1:ZZZ:G2:WWW:D0:AA.A:D1:BB.B`

**Components**:
- `T:XX.X` - Temperature in Celsius (1 decimal place)
- `G0:YYY` - Gas sensor 0 analog reading (0-1023)
- `G1:ZZZ` - Gas sensor 1 analog reading (0-1023)  
- `G2:WWW` - Gas sensor 2 analog reading (0-1023)
- `D0:AA.A` - Distance sensor 0 in cm (1 decimal place)
- `D1:BB.B` - Distance sensor 1 in cm (1 decimal place)

**Example**: `SENSORS:T:23.5:G0:120:G1:135:G2:140:D0:45.0:D1:50.3`

### Status Messages (Motor/Sensor Unit → Control Unit)

**Emergency Messages**:
- `EMERGENCY: Safety threshold exceeded!`
- `RECOVERY: Safety conditions restored`

**Debug Messages** (with specific conditions):
- `Temperature critical: XX.X`
- `Gas levels critical: G0:YYY G1:ZZZ G2:WWW`
- `Obstacle detected: D0:AA.Acm D1:BB.Bcm`

## Communication Timing

### Control Unit Timing
- **Command Send Interval**: 200ms
- **Display Update Interval**: 500ms
- **Serial Timeout**: 1000ms (considers data invalid)

### Motor/Sensor Unit Timing
- **Sensor Read Interval**: 200ms
- **Data Send Interval**: 300ms
- **Emergency LED Blink**: 250ms

## Error Handling

### Control Unit
- Displays "Connecting..." when no valid sensor data received
- Maintains last known sensor values during communication gaps
- Continues sending commands even without sensor feedback

### Motor/Sensor Unit  
- Validates sensor readings (range checks)
- Maintains last valid readings for out-of-range values
- Logs emergency condition changes
- Immediate motor shutdown on safety violations

## Protocol Improvements in v2.0

1. **Enhanced Sensor Data**: Individual sensor readings instead of aggregated values
2. **Better Error Handling**: Timeout detection and recovery
3. **Structured Data**: Clear separation of sensor types with indexed naming
4. **Status Reporting**: Detailed emergency condition logging
5. **Timing Control**: Configurable intervals for different operations