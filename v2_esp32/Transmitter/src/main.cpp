#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>

// =================== Joystick Pins ===================
const uint8_t pins[5] = {
    A0, // throttle
    A1, // yaw
    A2, // pitch
    A3, // roll
    3   // arm switch
};

// =================== NRF ===================
RF24 radio(7, 10);        // CE, CSN

const byte address[6] = "00001";

// =================== Data Packet ===================
struct DataPacket
{
    uint16_t throttle;

    uint16_t yaw;
    uint16_t pitch;
    uint16_t roll;

    bool arm;
};

DataPacket data;

// =================== Filter ===================

const float alpha = 0.585f;

int LowPassFilter(float &prev, int current)
{
    prev = alpha * current + (1.0f - alpha) * prev;

    return (int)prev;
}

// =================== Deadband ===================

const int center = 512;
const int deadband = 15;

int applyDeadband(int value)
{
    if (abs(value - center) < deadband)
    {
        return center;
    }

    return value;
}

// =================== Variables ===================

int raw[4] = {0, 0, 0, 0};

float pthrottle = 0;
float pyaw      = 0;
float ppitch    = 0;
float proll     = 0;

bool arm = false;

// =================== Setup ===================

void setup()
{
    Serial.begin(115200);

    pinMode(pins[4], INPUT_PULLUP);

    // NRF

    if (!radio.begin())
    {
        Serial.println("NRF NOT FOUND!");

        while (1);
    }

    radio.openWritingPipe(address);

    radio.setPALevel(RF24_PA_HIGH);

    radio.setDataRate(RF24_250KBPS);

    radio.setRetries(5, 15);

    radio.stopListening();

    Serial.println("Transmitter Ready");
}

// =================== Loop ===================

void loop()
{
    // Read joystick

    for (int i = 0; i < 4; i++)
    {
        raw[i] = analogRead(pins[i]);
    }

    arm = !digitalRead(pins[4]);

    // Deadband

    raw[1] = applyDeadband(raw[1]); // yaw
    raw[2] = applyDeadband(raw[2]); // pitch
    raw[3] = applyDeadband(raw[3]); // roll

    // Low-pass filter

    pthrottle = LowPassFilter(pthrottle, raw[0]);

    pyaw = LowPassFilter(pyaw, raw[1]);

    ppitch = LowPassFilter(ppitch, raw[2]);

    proll = LowPassFilter(proll, raw[3]);

    // Store in packet

    data.throttle = (uint16_t)pthrottle;

    data.yaw = (uint16_t)pyaw;

    data.pitch = (uint16_t)ppitch;

    data.roll = (uint16_t)proll;

    data.arm = arm;

    // Send packet

    bool success = radio.write(&data, sizeof(data));

    // Debug

    Serial.print("T : ");
    Serial.print(data.throttle);

    Serial.print(" | Y : ");
    Serial.print(data.yaw);

    Serial.print(" | P : ");
    Serial.print(data.pitch);

    Serial.print(" | R : ");
    Serial.print(data.roll);

    Serial.print(" | Arm : ");
    Serial.print(data.arm);

    Serial.print(" | TX : ");
    Serial.println(success);

    delay(4); // ~250 Hz
}