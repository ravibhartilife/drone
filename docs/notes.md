# Development Notes

## Design Decisions
- Arduino used for real-time motor and ESC control
- Complementary filter chosen for simplicity and speed
- EEPROM used to store MPU6050 calibration offsets

## Issues Faced
- MPU6050 gyro drift during long operation
- NRF24L01 instability due to power noise
- ESC arming sequence timing issues

## Fixes & Learnings
- Added startup calibration routine
- Improved power supply for NRF24L01
- Tuned PID values experimentally

## Ideas for v2 (ROS + Raspberry Pi)
- Replace complementary filter with Kalman filter
- Move high-level control to ROS 2 nodes
- Use serial/UART for Arduino ↔ Raspberry Pi communication
