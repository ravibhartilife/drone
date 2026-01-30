# System Overview

This project implements a custom-built quadcopter drone designed from scratch.

The system is developed in phases, starting with an Arduino-based flight
controller and expanding toward a ROS 2 and Raspberry Pi architecture.

---

## Version 1 – Arduino Only

- Arduino Uno handles real-time motor control
- MPU6050 IMU provides roll and pitch orientation
- NRF24L01 used for remote control communication
- PID control loop stabilizes the drone
- Calibration data stored in EEPROM

This version focuses on stability and manual control.

---

## Version 2 – ROS 2 + Raspberry Pi (Planned)

- Raspberry Pi will handle high-level logic
- ROS 2 used for modular node-based architecture
- Arduino will act as a low-level motor controller
- Serial/UART communication between Pi and Arduino
- Designed for future autonomy and sensor expansion
