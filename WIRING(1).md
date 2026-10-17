# Wiring

All grounds (UNO, ESP32, level shifter, PMS7003, motor driver, GPS, MPU6050) are connected together.
Pin numbers are taken from the project report and circuit diagram. Check them against your final code before building.

## PMS7003 to Arduino UNO (UART, 9600 baud)

| PMS7003 | UNO |
|---------|-----|
| TX | D2 |
| RX | not connected |
| VCC | 5V |
| GND | GND |

## ESP32 to Arduino UNO (I2C, ESP32 is master, UNO is slave at 0x08)

Connected through a bidirectional I2C level shifter.

| Level shifter pin | Connected to |
|-------------------|--------------|
| HV | UNO 5V |
| HV1 | UNO SDA (A4) |
| HV2 | UNO SCL (A5) |
| LV | ESP32 3V3 |
| LV1 | ESP32 GPIO21 (SDA) |
| LV2 | ESP32 GPIO22 (SCL) |

## Arduino UNO to L293D Motor Driver

| L293D pin | UNO pin |
|-----------|---------|
| ENA | D5 |
| IN1 | D7 |
| IN2 | D8 |
| ENB | D6 |
| IN3 | D9 |
| IN4 | D10 |

Motor 1 and Motor 2 (N20) connect to the L293D outputs.

## MPU6050 to ESP32 (separate I2C bus, address 0x68)

| MPU6050 | ESP32 |
|---------|-------|
| VCC | 3V3 |
| GND | GND |
| SDA | GPIO4 |
| SCL | GPIO13 |
| AD0 | GND |
| INT | not connected |

## NEO-6M GPS to ESP32 (UART2, 9600 baud)

| GPS | ESP32 |
|-----|-------|
| TX | GPIO16 |
| RX | GPIO17 |
| VCC | 5V |
| GND | GND |

## Servos (SG90) to ESP32

| Servo | ESP32 |
|-------|-------|
| Servo 1 signal | GPIO18 |
| Servo 2 signal | GPIO19 |
| Servo 3 signal | GPIO25 |
| Servo 4 signal | GPIO23 |

## ESCs to ESP32

| ESC | Position | ESP32 |
|-----|----------|-------|
| ESC 1 | Front | GPIO32 |
| ESC 2 | Right | GPIO33 |
| ESC 3 | Rear | GPIO26 |
| ESC 4 | Left | GPIO27 |

## Power

The 3S Li-Po battery feeds the power distribution board, which supplies the four ESCs. The 5V and 12V outputs supply the other parts.
