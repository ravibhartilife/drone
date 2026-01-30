#include <SPI.h>
#include <RF24.h>

// === Joystick Pins ===
const int throttlePin = A0;
const int yawPin      = A1;
const int pitchPin    = A2;
const int rollPin     = A3;

// === NRF24L01 Pins ===
RF24 radio(7, 10);
const byte address[6] = "00001";

// === Data Structures ===
struct DataPacket {
  int throttle;
  int yaw;
  int pitch;
  int roll;
};
DataPacket data;

// === Smoothing Variables ===
float smoothThrottle = 0, smoothYaw = 0, smoothPitch = 0, smoothRoll = 0;
float alpha = 0.7;

int applyFilter(float &prev, int current) {
  prev = alpha * current + (1 - alpha) * prev;
  return (int)prev;
}

void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_HIGH);
  radio.stopListening(); // Transmitter mode
}

void loop() {
  // === 1. Read and Filter Joystick Inputs ===
  int rawThrottle = analogRead(throttlePin);
  int rawYaw      = analogRead(yawPin);
  int rawPitch    = analogRead(pitchPin);
  int rawRoll     = analogRead(rollPin);

  data.throttle = applyFilter(smoothThrottle, rawThrottle);
  data.yaw      = applyFilter(smoothYaw, rawYaw);
  data.pitch    = applyFilter(smoothPitch, rawPitch);
  data.roll     = applyFilter(smoothRoll, rawRoll);

  // === 2. Send Joystick Data ===
  bool success = radio.write(&data, sizeof(DataPacket));

  // === 3. Optional Debug Output ===
  Serial.print("Throttle: "); Serial.print(data.throttle);
  Serial.print(" | Yaw: "); Serial.print(data.yaw);
  Serial.print(" | Pitch: "); Serial.print(data.pitch);
  Serial.print(" | Roll: "); Serial.println(data.roll);

  delay(20);
}
