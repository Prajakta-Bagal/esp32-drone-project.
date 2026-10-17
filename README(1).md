# Design & Implementation of Volumetric Air Pollution Mapping Using Amphibian Drone

B.Tech Project-B, Electronics and Telecommunication Engineering
Pillai College of Engineering, New Panvel (University of Mumbai), 2026-27

## About the Project

An amphibian drone that can drive on the ground and fly like a quadcopter. While moving, it measures air pollution (PM1.0, PM2.5, PM10) and shows the data on a web dashboard with 2D and 3D pollution maps.

- **Ground mode:** the body lies horizontal and drives on two wheels (2 x N20 geared motors).
- **Flight mode:** four SG90 servos with four-bar linkages unfold the rotor arms, the body stands vertical, and four A2212 BLDC motors provide thrust.

## Team

- Suhanisingh Chandel
- Gayatri Birajdar
- Prajakta Bagal
- Aaryan Gaurat

Supervisor: Prof. Shubham Thakur

## Main Features

- Ground driving: Forward, Backward, Left, Right, Stop
- Foldable rotor arms (folded 0 degrees, deployed 109 degrees)
- PM1.0, PM2.5, PM10 measurement with PMS7003
- GPS location with NEO-6M
- Roll, pitch and yaw with MPU6050
- Web dashboard hosted on the ESP32 for control and monitoring
- Pollution data saved as CSV on the ESP32 (LittleFS)
- Pollution map: 2D map, 3D map, and a K-Means pollution level model

## Hardware

| Part | Qty | Use |
|------|-----|-----|
| ESP32-WROOM | 1 | Main controller, web server |
| Arduino UNO | 1 | Reads PMS7003, drives ground motors |
| PMS7003 | 1 | Air-quality sensor |
| NEO-6M GPS | 1 | Position |
| MPU6050 | 1 | Motion and orientation |
| L293D motor driver | 1 | Drives N20 motors |
| N20 geared motor | 2 | Ground wheels |
| A2212 1400KV BLDC motor | 4 | Flight |
| SimonK 30A ESC | 4 | Controls BLDC motors |
| SG90 servo | 4 | Fold and deploy arms |
| I2C level shifter | 1 | Connects UNO (5V) and ESP32 (3.3V) |
| Matek PDB with 5V and 12V BEC | 1 | Power distribution |
| 3S 3000mAh Li-Po battery | 1 | Power |

## How It Works

```
PMS7003 --UART--> Arduino UNO --I2C--> ESP32 --Wi-Fi--> Web Dashboard
                      |                  |
                  L293D + N20        GPS, MPU6050,
                  (ground motors)    servos, ESCs
```

1. The UNO reads the PMS7003 and checks each data frame.
2. The ESP32 asks the UNO for the PM values every second over I2C (UNO address 0x08, 6 bytes: PM1.0, PM2.5, PM10).
3. The ESP32 also reads the GPS and MPU6050, and saves valid readings to a CSV file.
4. Commands from the web dashboard go from the ESP32 to the UNO over I2C to drive the motors.
5. The dashboard shows live values and the pollution maps.

## Folder Structure

```
embedded1/
├── README.md
├── WIRING.md
├── esp32/        <- ESP32 code (.ino)
├── uno/          <- Arduino UNO code (.ino)
├── dashboard/    <- Air Quality ML dashboard (.html)
└── report/       <- Project report (.pdf)
```

## How to Run

1. Install the Arduino IDE and add ESP32 board support.
2. Upload the code in `uno/` to the Arduino UNO.
3. Upload the code in `esp32/` to the ESP32.
4. Connect to the ESP32 Wi-Fi and open its IP address in a browser to use the control dashboard.
5. Open the PMS dashboard page to see live air-quality data and maps.

## Status

Subsystem testing is done: ground drive, rotor-arm folding, BLDC motors with ESCs, PMS7003 data transfer, GPS data and dashboard control. Final integrated flight testing and field testing are the next stage.

## Future Work

- More spatial data collection and better interpolation
- Integrated flight testing
- More environmental sensors
- Autonomous navigation and autonomous transformation
- Larger machine-learning dataset
