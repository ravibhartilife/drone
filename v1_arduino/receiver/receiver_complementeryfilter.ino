#include <Wire.h>
#include <SPI.h>
#include <RF24.h>
#include <Servo.h>
#include <MPU6050.h>
#include <EEPROM.h>

// === Hardware Setup ===
MPU6050 mpu;
RF24 radio(8, 10); // CE, CSN
Servo escFL, escFR, escBL, escBR;

const byte address[6] = "00001";

// === Pin Mapping ===
const int FL_PIN = 3;
const int FR_PIN = 5;
const int BL_PIN = 6;
const int BR_PIN = 9;

// === Data Structure (from transmitter) ===
struct DataPacket {
  int throttle;
  int yaw;
  int pitch;
  int roll;
};
DataPacket data;

// === PID variables ===
float kp = 1.85, ki = 0.0035, kd = 0.003;
float errorX, errorY, prevErrorX = 0, prevErrorY = 0;
float integralX = 0, integralY = 0;

// === MPU Variables ===
float accX, accY, accZ, gyroX, gyroY, gyroZ;
float angleX = 0, angleY = 0;
unsigned long lastTime;

// === Gyro calibration ===
float gyroX_offset = 0, gyroY_offset = 0, gyroZ_offset = 0;
const int GYRO_X_ADDR = 0;
const int GYRO_Y_ADDR = 4;
const int GYRO_Z_ADDR = 8;
const int MAGIC_ADDR = 12;
const int MAGIC_VALUE = 4242;
bool gyroCalibrated = false;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  // MPU init
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while (1);
  }

  // === EEPROM gyro calibration ===
  int magic;
  EEPROM.get(MAGIC_ADDR, magic);

  if (magic == MAGIC_VALUE) {
    EEPROM.get(GYRO_X_ADDR, gyroX_offset);
    EEPROM.get(GYRO_Y_ADDR, gyroY_offset);
    EEPROM.get(GYRO_Z_ADDR, gyroZ_offset);
    gyroCalibrated = true;
    Serial.println("Gyro calibration loaded from EEPROM.");
  } else {
    Serial.println("Calibrating Gyroscope...");
    long sumX = 0, sumY = 0, sumZ = 0;
    int16_t ax, ay, az, gx, gy, gz;
    for (int i = 0; i < 2000; i++) {
      mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
      sumX += gx; sumY += gy; sumZ += gz;
      delay(1);
    }
    gyroX_offset = (float)sumX / 2000.0;
    gyroY_offset = (float)sumY / 2000.0;
    gyroZ_offset = (float)sumZ / 2000.0;
    EEPROM.put(GYRO_X_ADDR, gyroX_offset);
    EEPROM.put(GYRO_Y_ADDR, gyroY_offset);
    EEPROM.put(GYRO_Z_ADDR, gyroZ_offset);
    EEPROM.put(MAGIC_ADDR, MAGIC_VALUE);
    Serial.println("Gyro calibration done and saved.");
  }

  // ESC init
  escFL.attach(FL_PIN);
  escFR.attach(FR_PIN);
  escBL.attach(BL_PIN);
  escBR.attach(BR_PIN);

  escFL.writeMicroseconds(1000);
  escFR.writeMicroseconds(1000);
  escBL.writeMicroseconds(1000);
  escBR.writeMicroseconds(1000);
  delay(3000);

  // Radio init
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_LOW);
  radio.startListening();

  lastTime = micros();
}

void loop() {
  // === Receive control data ===
  if (radio.available()) {
    radio.read(&data, sizeof(DataPacket));
  }

  // === Read MPU6050 ===
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Apply gyro offsets
  gx -= gyroX_offset;
  gy -= gyroY_offset;
  gz -= gyroZ_offset;

  // Convert to angles (complementary filter)
  float dt = (micros() - lastTime) / 1000000.0;
  lastTime = micros();

  float accAngleX = atan2(ay, az) * 180 / PI;
  float accAngleY = atan2(ax, az) * -180 / PI;

  angleX = 0.98 * (angleX + gx / 131.0 * dt) + 0.02 * accAngleX;
  angleY = 0.98 * (angleY + gy / 131.0 * dt) + 0.02 * accAngleY;

  // === PID Control ===
  errorX = (data.roll - 512) / 50.0 - angleX;
  errorY = (data.pitch - 512) / 50.0 - angleY;

  integralX += errorX * dt;
  integralY += errorY * dt;

  float derivativeX = (errorX - prevErrorX) / dt;
  float derivativeY = (errorY - prevErrorY) / dt;

  float outputX = kp * errorX + ki * integralX + kd * derivativeX;
  float outputY = kp * errorY + ki * integralY + kd * derivativeY;

  prevErrorX = errorX;
  prevErrorY = errorY;

  // === Motor Mixing ===
  int baseThrottle = map(data.throttle, 0, 1023, 1000, 2000);

  int escFL_val = baseThrottle - outputY + outputX;
  int escFR_val = baseThrottle - outputY - outputX;
  int escBL_val = baseThrottle + outputY + outputX;
  int escBR_val = baseThrottle + outputY - outputX;

  // Constrain values
  escFL_val = constrain(escFL_val, 1000, 2000);
  escFR_val = constrain(escFR_val, 1000, 2000);
  escBL_val = constrain(escBL_val, 1000, 2000);
  escBR_val = constrain(escBR_val, 1000, 2000);

  // Write to ESCs
  escFL.writeMicroseconds(escFL_val);
  escFR.writeMicroseconds(escFR_val);
  escBL.writeMicroseconds(escBL_val);
  escBR.writeMicroseconds(escBR_val);

  // Debug
  Serial.print("X:"); Serial.print(angleX);
  Serial.print(" Y:"); Serial.print(angleY);
  Serial.print(" | FL:"); Serial.print(escFL_val);
  Serial.print(" FR:"); Serial.print(escFR_val);
  Serial.print(" BL:"); Serial.print(escBL_val);
  Serial.print(" BR:"); Serial.println(escBR_val);
}
