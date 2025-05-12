# Magnetic Sensor Protocol Documentation

## Protocol Overview
The magnetic sensor array uses a TCP-based binary protocol operating on port 2468. All communication follows a command-response pattern.

## Connection Requirements
- IP Address: 192.168.1.254 (fixed)
- Port: 2468
- Connection Type: TCP
- Timeout: 2 seconds recommended

## Command Structure
All commands follow this byte sequence pattern:
1. Header bytes (0x01, 0x01)
2. Command type bytes
3. Data length byte
4. Command data bytes
5. CRC bytes (if applicable)

## Key Commands

### Initialize Sensor Array ("ON" Command)
Used once to activate the sensor array's data reporting mode.
```hex
01 01 02 01 05 10 03 10 01 01 0C BD
```

### Query All Sensors Command
Requests data from all sensors in a single packet.
```hex
01 01 01 01 10 10 03 10 03 10 03 10 04 10 03 10 05 10 03 10 06 70 75
```

### Response Format
#### All Sensors Response (31 bytes)
```
Byte Offsets:
 9-10:  Sensor A value (16-bit BE unsigned)
15-16:  Sensor B value (16-bit BE unsigned)
21-22:  Sensor C value (16-bit BE unsigned)
27-28:  Sensor Z value (16-bit BE unsigned)
```

Shutdown Command ("OFF" Command)
Used to deactivate the sensor array.
```hex
01 01 02 01 05 10 03 10 01 00 CC 7C
```

## Implementation Notes
1. Connection must be established and "ON" command sent before any sensor queries
1. Keep connection persistent - do not reconnect for each query
1. Always send "OFF" command before disconnecting
1. Handle timeouts and reconnections gracefully