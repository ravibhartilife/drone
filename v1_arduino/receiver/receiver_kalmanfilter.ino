#include <Wire.h>
#include <SPI.h>
#include <RF24.h>
#include <Servo.h>
#include <MPU6050.h>
#include <EEPROM.h>

// === Hardware Setup ===
MPU6050 mpu;
RF24 radio(8, 10);  // CE, CSN
Servo escFL, escFR, escBL, escBR;

const byte address[6] = "00001";
const int FL_PIN = 3, FR_PIN = 5, BL_PIN = 6, BR_PIN = 9;

// === Joystick Data ===
struct DataPacket {
  int throttle;
  int yaw;
  int pitch;
  int roll;
};
DataPacket data;

// === IMU & Timing Variables ===
float accAngleX, accAngleY, gyroRateX, gyroRateY;
unsigned long lastTime;

// Angle and Motor tunning
float trimX = 0.0, trimY = 0.0;
int trimFL = 0, trimFR = 0, trimBL = 0, trimBR = 0;

// === Kalman Filter Variables ===
float kalAngleX = 0, kalAngleY = 0;
float biasX = 0, biasY = 0;
float P_X[2][2] = {{0, 0}, {0, 0}}, P_Y[2][2] = {{0, 0}, {0, 0}};
float Q_angle = 0.0005, Q_bias = 0.0015, R_measure = 0.03;

// === PID Variables ===
float Kp = 2.7, Ki = 0.004, Kd = 0.025;
float integralX = 0, integralY = 0;
float lastErrorX = 0, lastErrorY = 0;

// for pid value change
String inputString = "";
bool stringComplete = false;

// === Calibration Offsets ===
float angleX_offset = 0, angleY_offset = 0;
float gyroX_offset = 0, gyroY_offset = 0, gyroZ_offset = 0;
bool calibrated = false, gyroCalibrated = false;
float sumX = 0, sumY = 0;
int sampleCount = 0;

// === EEPROM Calibration ===
const int GYRO_X_ADDR = 0, GYRO_Y_ADDR = 4, GYRO_Z_ADDR = 8, GYRO_MAGIC_ADDR = 12;
const int GYRO_MAGIC_VALUE = 4242;

// === Fail-Safe ===
unsigned long lastSignalTime = 0;
const unsigned long signalTimeout = 1000;

void kalmanFilter(float accAngle, float gyroRate, float dt,
                  float &angle, float &bias, float P[2][2]) {
  angle += dt * (gyroRate - bias);
  P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += Q_bias * dt;

  float S = P[0][0] + R_measure;
  float K[2] = { P[0][0] / S, P[1][0] / S };
  float y = accAngle - angle;
  angle += K[0] * y;
  bias += K[1] * y;

  float P00_temp = P[0][0], P01_temp = P[0][1];
  P[0][0] -= K[0] * P00_temp;
  P[0][1] -= K[0] * P01_temp;
  P[1][0] -= K[1] * P00_temp;
  P[1][1] -= K[1] * P01_temp;
}

void setup() {
  Serial.begin(9600);
  inputString.reserve(50); // for pid value change || prevent dynamic memory usage
  Wire.begin();
  mpu.initialize();

    if (!mpu.testConnection()) {
    while (1) {
      Serial.println("MPU6050 not connected!");
      delay(1000);
    }
  }

  int magic;
  EEPROM.get(GYRO_MAGIC_ADDR, magic);
  if (magic == GYRO_MAGIC_VALUE) {
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
    EEPROM.put(GYRO_MAGIC_ADDR, GYRO_MAGIC_VALUE);
    Serial.println("Gyro calibration saved.");
  }

  escFL.attach(FL_PIN); escFR.attach(FR_PIN);
  escBL.attach(BL_PIN); escBR.attach(BR_PIN);
  escFL.writeMicroseconds(1000);
  escFR.writeMicroseconds(1000);
  escBL.writeMicroseconds(1000);
  escBR.writeMicroseconds(1000);
  delay(2000);

  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_LOW);
  radio.startListening();

  lastTime = micros();
  lastSignalTime = millis();
}

void loop() {
  // === 1. Check for Serial input === || for pid value change
if (stringComplete) {
  processCommand(inputString);
  inputString = "";
  stringComplete = false;
}

  if (radio.available()) {
    radio.read(&data, sizeof(DataPacket));
    lastSignalTime = millis();
  }

  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  unsigned long now = micros();
  float dt = (now - lastTime) / 1000000.0;
  lastTime = now;
  if (dt <= 0 || isnan(dt)) return;

  gyroRateX = (gx - gyroX_offset) / 131.0;
  gyroRateY = (gy - gyroY_offset) / 131.0;

  float safeAccZ = (az == 0) ? 0.01 : az;
  accAngleX = atan2(ay, safeAccZ) * 180 / PI;
  accAngleY = atan2(ax, safeAccZ) * 180 / PI;

  kalmanFilter(accAngleX, gyroRateX, dt, kalAngleX, biasX, P_X);
  kalmanFilter(accAngleY, gyroRateY, dt, kalAngleY, biasY, P_Y);

  if (!calibrated && millis() < 3000) {
    sumX += kalAngleX; sumY += kalAngleY;
    sampleCount++;
  } else if (!calibrated) {
    if (sampleCount > 0) {
      angleX_offset = sumX / sampleCount;
      angleY_offset = sumY / sampleCount;
    }
    calibrated = true;
    Serial.print("Offsets set => X: "); Serial.print(angleX_offset);
    Serial.print(" Y: "); Serial.println(angleY_offset);
  }

  float correctedX = kalAngleX - angleX_offset + trimX;    // with triming of angles
  float correctedY = kalAngleY - angleY_offset + trimY;

  float desiredX = map(data.roll, 0, 1023, 30, -30);
  float desiredY = map(data.pitch, 0, 1023, -30, 30);

  float errorX = desiredX - correctedX;
  float errorY = desiredY - correctedY;

  integralX += errorX * dt;
  integralY += errorY * dt;
  integralX = constrain(integralX, -100, 100);
  integralY = constrain(integralY, -100, 100);

  float derivativeX = (errorX - lastErrorX) / dt;
  float derivativeY = (errorY - lastErrorY) / dt;
  lastErrorX = errorX;
  lastErrorY = errorY;

  int correctionX = Kp * errorX + Ki * integralX + Kd * derivativeX;
  int correctionY = Kp * errorY + Ki * integralY + Kd * derivativeY;

  int baseThrottle;
  if (millis() - lastSignalTime > signalTimeout) {
    baseThrottle = 1000;
    correctionX = 0;
    correctionY = 0;
    integralX = 0;
    integralY = 0;
    lastErrorX = 0;
    lastErrorY = 0;
    Serial.println("FAIL-SAFE: NO SIGNAL");
  } else {
    baseThrottle = map(data.throttle, 0, 1023, 1000, 2000);
  }

  int escFL_val = baseThrottle - correctionY + correctionX + trimFL; // with triming motors
  int escFR_val = baseThrottle - correctionY - correctionX + trimFR;
  int escBL_val = baseThrottle + correctionY + correctionX + trimBL;
  int escBR_val = baseThrottle + correctionY - correctionX + trimBR;

  escFL_val = constrain(escFL_val, 1000, 2000);
  escFR_val = constrain(escFR_val, 1000, 2000);
  escBL_val = constrain(escBL_val, 1000, 2000);
  escBR_val = constrain(escBR_val, 1000, 2000);

  escFL.writeMicroseconds(escFL_val);
  escFR.writeMicroseconds(escFR_val);
  escBL.writeMicroseconds(escBL_val);
  escBR.writeMicroseconds(escBR_val);

// === 7. Serial Plotter Output ===
Serial.print(" FL:"); Serial.print(escFL_val); 
Serial.print(" BL:"); Serial.print(escBL_val); 
Serial.print(" FR:"); Serial.print(escFR_val);
Serial.print(" BR:"); Serial.print(escBR_val); 
Serial.print("X:"); Serial.print(correctedX);
//Serial.print(" DX:"); Serial.print(desiredX);
Serial.print(" Y:"); Serial.print(correctedY);
//Serial.print(" DY:"); Serial.println(desiredY);

delay(8); 
}

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      stringComplete = true;
    } else {
      inputString += inChar;
    }
  }
}
void processCommand(String cmd) {
  cmd.trim();
  cmd.toUpperCase();

  // pid and kalman value tunning
  if (cmd.startsWith("SET KP ")) {
    Kp = cmd.substring(7).toFloat();
    Serial.print("Updated Kp: "); Serial.println(Kp, 4);
  }
  else if (cmd.startsWith("SET KI ")) {
    Ki = cmd.substring(7).toFloat();
    Serial.print("Updated Ki: "); Serial.println(Ki, 4);
  }
  else if (cmd.startsWith("SET KD ")) {
    Kd = cmd.substring(7).toFloat();
    Serial.print("Updated Kd: "); Serial.println(Kd, 4);
  }
  else if (cmd.startsWith("SET QA ")) {
    Q_angle = cmd.substring(7).toFloat();
    Serial.print("Updated Q_angle: "); Serial.println(Q_angle, 6);
  }
  else if (cmd.startsWith("SET QB ")) {
    Q_bias = cmd.substring(7).toFloat();
    Serial.print("Updated Q_bias: "); Serial.println(Q_bias, 6);
  }
  else if (cmd.startsWith("SET RM ")) {
    R_measure = cmd.substring(8).toFloat();
    Serial.print("Updated R_measure: "); Serial.println(R_measure, 6);
  }
  else if (cmd == "SHOW PID") {
    Serial.print("Kp: "); Serial.print(Kp, 4);
    Serial.print(" | Ki: "); Serial.print(Ki, 4);
    Serial.print(" | Kd: "); Serial.println(Kd, 4);
  }
  else if (cmd == "SHOW KALMAN") {
    Serial.print("Q_angle: "); Serial.print(Q_angle, 6);
    Serial.print(" | Q_bias: "); Serial.print(Q_bias, 6);
    Serial.print(" | R_measure: "); Serial.println(R_measure, 6);
  }

  // angle and motors tunning
  else if (cmd.startsWith("SET TX ")) {
  trimX = cmd.substring(7).toFloat();
  Serial.print("Updated trimX: "); Serial.println(trimX, 4);
 }
 else if (cmd.startsWith("SET TY ")) {
  trimY = cmd.substring(7).toFloat();
  Serial.print("Updated trimY: "); Serial.println(trimY, 4);
 }
 else if (cmd.startsWith("SET FL ")) {
  trimFL = cmd.substring(7).toInt();
  Serial.print("Updated trimFL: "); Serial.println(trimFL);
 }
 else if (cmd.startsWith("SET FR ")) {
  trimFR = cmd.substring(7).toInt();
  Serial.print("Updated trimFR: "); Serial.println(trimFR);
 }
 else if (cmd.startsWith("SET BL ")) {
  trimBL = cmd.substring(7).toInt();
  Serial.print("Updated trimBL: "); Serial.println(trimBL);
 }
 else if (cmd.startsWith("SET BR ")) {
  trimBR = cmd.substring(7).toInt();
  Serial.print("Updated trimBR: "); Serial.println(trimBR);
 }
 else if (cmd == "SHOW TRIMS") {
  Serial.print("trimX: "); Serial.print(trimX, 4);
  Serial.print(" | trimY: "); Serial.print(trimY, 4);
  Serial.print(" | FL: "); Serial.print(trimFL);
  Serial.print(" | FR: "); Serial.print(trimFR);
  Serial.print(" | BL: "); Serial.print(trimBL);
  Serial.print(" | BR: "); Serial.println(trimBR);
 }

 // === If no command matches ===
else {
  Serial.println("wrong command.");
  Serial.println("Valid Commands:");
  Serial.println("    SET KP <value>");
  Serial.println("    SET KI <value>");
  Serial.println("    SET KD <value>");
  Serial.println("    SET QA <value>");
  Serial.println("    SET QB <value>");
  Serial.println("    SET RM <value>");
  Serial.println("    SET TX <value>");
  Serial.println("    SET TY <value>");
  Serial.println("    SET FL <value>");
  Serial.println("    SET FR <value>");
  Serial.println("    SET BL <value>");
  Serial.println("    SET BR <value>");
  Serial.println("    SHOW PID");
  Serial.println("    SHOW KALMAN");
  Serial.println("    SHOW TRIMS");
}

}