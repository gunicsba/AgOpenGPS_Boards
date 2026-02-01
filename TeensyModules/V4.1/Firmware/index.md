# Machinery Guide Steer Motor – Reverse Engineering Notes

Stepper-motor based steering motor by **Machinery Guide**.

This document summarizes CAN traces, wiring, and byte-level decoding of messages observed during calibration and operation.

---

## General Information

- **Bus:** CAN  
- **Bitrate:** 250 kbps  
- **Motor type:** Internal stepper motor  
- **Main connector:** DTM 12P  

---

## Connector Pinout

### Power & Wheel Angle Sensor (WAS)

| Wire color | Function |
|------------|----------|
| Blue | GND (2× for power input, 1× for WAS) |
| Brown | +12 V |
| Brown (separate) | WAS +5 V output |
| Yellow/Green stripe | WAS signal (appears digital on oscilloscope) |

### CAN Bus

| Wire color | Function |
|------------|----------|
| Red | CAN_H |
| Green | CAN_L |
| Black | CAN_GND |

---

## CAN Message Roles

| CAN ID | Direction | Role |
|--------|-----------|------|
| 0x500 | Controller → Motor | Commands & configuration |
| 0x50F | Motor → Controller | Periodic feedback (10 Hz) |
| 0x503 | Motor → Controller | ACK / state response |

---

## Byte-Level Decoding Rules

### 16-bit decoding

```c
uint16_t u16 = (B1 << 8) | B2;
int16_t  i16 = (int16_t)u16;
```

- Big-endian
- Used for angle and status words

---

## 0x50F – Motor Feedback Frame (DLC = 3, ~10 Hz)

Two subtypes exist based on Byte0 (B0).

### Case A: B0 = 0x01 → Wheel Angle Feedback

Payload:
```
50F: 01 AA BB
```

Decode:
```c
int16_t angle = (int16_t)((AA << 8) | BB);
```

Examples:
- F6 90 → 0xF690 → -2416 (full left)
- 03 E0 → 0x03E0 → +992 (full right)

Interpretation:
- Negative = left
- Positive = right
- Zero ≈ center

| Byte | Meaning |
|------|--------|
| B0 | 0x01 = angle feedback |
| B1 | Angle MSB |
| B2 | Angle LSB |

---

### Case B: B0 = 0x00 → Status / Internal Value (unknown)

Payload:
```
50F: 00 AA BB
```

Decode:
```c
uint16_t status = (AA << 8) | BB;
```

| Byte | Meaning |
|------|--------|
| B0 | 0x00 = status frame |
| B1 | Status MSB |
| B2 | Status LSB |

---

## 0x503 – ACK / State Response

| CAN ID | Data | Meaning |
|--------|------|---------|
| 0x503 | 01 | Start / calibration ACK |
| 0x503 | 02 | Stop / calibration complete |

---

## 0x500 – Controller Commands

### Start / Stop

| Data | Meaning |
|------|--------|
| 04 | Start / enable |
| 03 | Stop / disable |

Sequence:
```
500 02 00 00
500 04
503 01
```

Stop:
```
500 03
503 02
```

---

### Steering Command

Format:
```
500: 02 XX YY
```

Decode:
```c
uint16_t target = (XX << 8) | YY;
```

Examples:
- 500 02 FF FF → center
- 500 02 00 00 → center
- 500 02 00 20 → small left
- 500 02 00 50 → medium left
- 500 02 0F 00 → full left
- 500 02 FF 50 → medium right

---

### Calibration Trigger

| CAN ID | Data | Meaning |
|--------|------|--------|
| 0x500 | 22 | Calibration start |
| 0x503 | 02 | Calibration complete |

---

## Calibration Behavior

During calibration, the wheel angle feedback (0x50F B0=01) shows:

- Sinus-like oscillation
- Increasing amplitude over time
- Then settling near a reference point
- Followed by 0x503 02 (calibration complete)

Likely algorithm:
1. Sweep left/right with growing range  
2. Detect mechanical endstops  
3. Compute center offset  
4. Store limits  
5. Report completion  

---

## Signed vs Offset Representation

Raw decoding in one calibration trace produced only positive values (≈10…878).  
Other captures show negative values (example F690 → -2416).

This indicates:
- Protocol supports signed angle
- Firmware may apply an internal center offset
- Tablet UI likely displays centered values

Two representations are useful:
- Raw: direct int16
- Centered: raw – offset

---

## Calibration Chart

Annotated chart file:
```
wheel_angle_calibration_trimmed_annotated_v3.png
```

Embed example:
```markdown
![Wheel angle during calibration](wheel_angle_calibration_trimmed_annotated_v3.png)
```

---

## Summary

- 0x50F B0=01 = wheel angle feedback (signed 16-bit)
- 0x50F B0=00 = status / unknown internal value
- 0x500 = controller command/config
- 0x503 = ACK / state
- Calibration uses oscillating sweep to determine limits and center
- Angle is logically signed even if some traces remain positive due to offset

---

## Open Questions

- Exact physical unit scaling (counts → degrees/radians)
- Meaning of 0x50F B0=00 status word
- Full mapping of configuration parameters (0x500 06, 0x500 1D, etc.)
