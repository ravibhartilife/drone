# Calibration Process

## MPU6050 Calibration
- Drone is kept stationary at startup
- Gyroscope and accelerometer offsets are calculated
- Offsets stored in EEPROM for reuse

## ESC Calibration
- ESCs calibrated using standard arming sequence
- Throttle range verified for all motors

## PID Tuning
- Initial PID values set conservatively
- Gains adjusted experimentally
- Stability verified through test flights

## Notes
- Calibration should be repeated after hardware changes
- Battery voltage affects performance and tuning
