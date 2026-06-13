# Drone Transmitter Firmware

## Overview

This project is the transmitter firmware for a custom drone.

Hardware used:

* Arduino Uno
* NRF24L01 with antenna
* Two analog joysticks
* One arm/disarm switch

The transmitter reads joystick positions, filters the values to remove noise, and sends the data wirelessly to the drone using NRF24L01.

---

## Pin Configuration

### Joystick

| Function   | Pin |
| ---------- | --- |
| Throttle   | A0  |
| Yaw        | A1  |
| Pitch      | A2  |
| Roll       | A3  |
| Arm Switch | D3  |

### NRF24L01

| NRF24L01 | Arduino Uno |
| -------- | ----------- |
| CE       | D7          |
| CSN      | D10         |
| MOSI     | D11         |
| MISO     | D12         |
| SCK      | D13         |
| VCC      | 3.3V        |
| GND      | GND         |

---

## Data Packet

The transmitter sends the following structure:

```cpp
struct DataPacket
{
    uint16_t throttle;

    uint16_t yaw;
    uint16_t pitch;
    uint16_t roll;

    bool arm;
};
```

### Description

* `throttle`

  * Joystick throttle value.
  * Range: 0 - 1023

* `yaw`

  * Left/Right rotation command.
  * Range: 0 - 1023

* `pitch`

  * Forward/Backward command.
  * Range: 0 - 1023

* `roll`

  * Left/Right tilt command.
  * Range: 0 - 1023

* `arm`

  * Drone arming state.
  * `0` = Disarmed
  * `1` = Armed

---

## Program Flow

The transmitter performs the following operations repeatedly.

### 1. Read Joysticks

The analog values are read from:

```text
A0 -> Throttle
A1 -> Yaw
A2 -> Pitch
A3 -> Roll
```

The arm switch is read from:

```text
D3 -> Arm/Disarm
```

---

### 2. Deadband

Deadband is applied to:

* Yaw
* Pitch
* Roll

Purpose:

Joystick modules produce small fluctuations around the center position.

Example:

```text
512
510
514
511
```

Without deadband:

The drone may slowly rotate or drift.

With deadband:

Values near the center are forced to:

```text
512
```

This creates a stable center position.

---

### 3. Low Pass Filter

A first-order low-pass filter is applied:

```cpp
filtered =
alpha * current +
(1-alpha) * previous;
```

where:

```cpp
alpha = 0.585
```

Purpose:

* Smooth joystick movements.
* Remove sudden spikes.
* Reduce electrical noise.

---

## Why Throttle Does Not Use Deadband

Throttle does not have a center position.

```text
Bottom -> Minimum thrust
Top    -> Maximum thrust
```

Applying deadband around the center would create unwanted jumps in throttle.

Therefore:

* Throttle:

  * Low-pass filter only

* Roll:

  * Deadband + Low-pass

* Pitch:

  * Deadband + Low-pass

* Yaw:

  * Deadband + Low-pass

---

## NRF24L01 Configuration

The radio is configured as:

```text
Mode         : Transmitter
Power Level  : HIGH
Data Rate    : 250 kbps
Retries      : Enabled
```

Reason:

* Longer communication range.
* Better reliability.
* Less packet loss.

---

## Transmission Frequency

The main loop runs every:

```text
4 ms
```

which gives approximately:

```text
250 Hz
```

The target transmission frequency is:

```text
220 - 250 Hz
```

This provides responsive control while keeping radio traffic low.

---

## Future Improvements

Planned features:

* Exact 220 Hz timing using `micros()`
* Joystick calibration
* Packet counter
* Battery monitor for transmitter
* OLED status display
* Two-way communication with the drone

---

## Current Status

```text
Joystick Reading      ✓
Deadband             ✓
Low Pass Filter      ✓
Arm Switch           ✓
NRF Configuration    ✓
NRF Transmission     ✓

Joystick Calibration □
```

This firmware is designed to work with the custom ESP32-S3 based drone flight controller operating in Angle Mode.
