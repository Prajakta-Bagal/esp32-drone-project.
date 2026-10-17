/*
  Arduino UNO code - Amphibian Drone
  - Reads PM1.0, PM2.5, PM10 from the PMS7003 sensor (UART, 9600 baud)
  - Acts as I2C slave (address 0x08) for the ESP32
  - Sends the 3 PM values to the ESP32 as 6 bytes (big-endian)
  - Receives ground motor commands from the ESP32 and drives the L293D

  Wiring (from project report):
    PMS7003 TX -> UNO D2   (RX of PMS7003 not connected)
    L293D: ENA=D5, IN1=D7, IN2=D8, ENB=D6, IN3=D9, IN4=D10
    I2C: UNO SDA (A4) / SCL (A5) to ESP32 through a level shifter
*/

#include <Wire.h>
#include <SoftwareSerial.h>

#define I2C_ADDRESS 0x08

// PMS7003 on software serial: RX = D2, TX = D3
SoftwareSerial pmsSerial(2, 3);

// L293D motor driver pins
const int ENA = 5;
const int IN1 = 7;
const int IN2 = 8;
const int ENB = 6;
const int IN3 = 9;
const int IN4 = 10;

// Latest pollution values (updated in loop, sent in requestEvent)
volatile uint16_t pm1_0 = 0;
volatile uint16_t pm2_5 = 0;
volatile uint16_t pm10  = 0;

// Motor command received from ESP32
volatile char    pendingCommand = 'S';
volatile uint8_t pendingSpeed   = 0;
volatile bool    newCommand     = false;

// PMS7003 frame buffer (32 bytes)
uint8_t buffer[32];
uint8_t bufferIndex = 0;

void setup() {
  Wire.begin(I2C_ADDRESS);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);

  pmsSerial.begin(9600);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  stopMotors();
}

void loop() {
  readPms7003();

  if (newCommand) {
    char cmd;
    uint8_t spd;
    noInterrupts();
    cmd = pendingCommand;
    spd = pendingSpeed;
    newCommand = false;
    interrupts();
    runCommand(cmd, spd);
  }
}

// ---------- PMS7003 reading ----------

void readPms7003() {
  while (pmsSerial.available()) {
    uint8_t b = pmsSerial.read();

    // Wait for the frame header 0x42 0x4D
    if (bufferIndex == 0 && b != 0x42) continue;
    if (bufferIndex == 1 && b != 0x4D) {
      bufferIndex = 0;
      continue;
    }

    buffer[bufferIndex++] = b;

    // A full frame is 32 bytes
    if (bufferIndex == 32) {
      bufferIndex = 0;
      checkFrame();
    }
  }
}

void checkFrame() {
  // Checksum = sum of the first 30 bytes
  uint16_t checksum = 0;
  for (int i = 0; i < 30; i++) {
    checksum += buffer[i];
  }
  uint16_t receivedChecksum = ((uint16_t)buffer[30] << 8) | buffer[31];

  if (checksum == receivedChecksum) {
    uint16_t a = ((uint16_t)buffer[10] << 8) | buffer[11];  // PM1.0
    uint16_t b = ((uint16_t)buffer[12] << 8) | buffer[13];  // PM2.5
    uint16_t c = ((uint16_t)buffer[14] << 8) | buffer[15];  // PM10

    noInterrupts();
    pm1_0 = a;
    pm2_5 = b;
    pm10  = c;
    interrupts();
  }
}

// ---------- I2C ----------

// ESP32 asks for data: send 6 bytes, high byte first
void requestEvent() {
  Wire.write(highByte(pm1_0));
  Wire.write(lowByte(pm1_0));
  Wire.write(highByte(pm2_5));
  Wire.write(lowByte(pm2_5));
  Wire.write(highByte(pm10));
  Wire.write(lowByte(pm10));
}

// ESP32 sends a motor command: 1st byte = letter, 2nd byte = speed (0-255)
void receiveEvent(int howMany) {
  if (howMany < 1) return;

  char cmd = (char)Wire.read();
  uint8_t spd = 255;               // default speed if none is sent
  if (Wire.available()) {
    spd = Wire.read();
  }
  while (Wire.available()) Wire.read();   // clear anything extra

  pendingCommand = cmd;
  pendingSpeed = spd;
  newCommand = true;
}

// ---------- Motor control ----------

void runCommand(char cmd, uint8_t spd) {
  if (cmd == 'F') {
    forward(spd);
  } else if (cmd == 'B') {
    backward(spd);
  } else if (cmd == 'L') {
    turnLeft(spd);
  } else if (cmd == 'R') {
    turnRight(spd);
  } else {
    stopMotors();   // 'S' or anything else
  }
}

void forward(uint8_t spd) {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(ENA, spd);
  analogWrite(ENB, spd);
}

void backward(uint8_t spd) {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  analogWrite(ENA, spd);
  analogWrite(ENB, spd);
}

// Motors turn in opposite directions.
// If left and right are swapped on your robot, swap turnLeft and turnRight.
void turnLeft(uint8_t spd) {
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(ENA, spd);
  analogWrite(ENB, spd);
}

void turnRight(uint8_t spd) {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
  analogWrite(ENA, spd);
  analogWrite(ENB, spd);
}

void stopMotors() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}
