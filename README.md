# Airbrakes CSUF 2026

A real-time sensor fusion and flight dynamics system for high-altitude rocket airbrake deployment, built for the Cal State University Fullerton aerospace team. This embedded system integrates multiple sensors and advanced filtering algorithms to enable precise altitude tracking and autonomous airbrake control.

## Features

- **Dual Accelerometer IMU**: Has both a low-G and high-G accelerometer for comprehensive acceleration measurement across a wide dynamic range
- **Barometric Altitude Sensing**: Real-time atmospheric pressure measurement and conversion to altitude
- **AHRS (Attitude and Heading Reference System)**: Quaternion-based orientation estimation using sensor fusion
- **Linear Kalman Filter**: Altitude and velocity estimation with adaptive filtering for optimal state prediction
- **Real-Time Scheduling**: Deterministic task scheduling for sensor polling at independent frequencies
  - IMU polling at 240 Hz
  - Barometer polling at 50 Hz
  - Logging at 10 Hz
- **Telemetry Integration**: Serial output for real-time data monitoring and logging

## Hardware

- **Microcontroller**: Raspberry Pi Pico 2
- **Sensors**:
  - ISM6HG256X: 6-DoF IMU with dual accelerometer ranges
  - BMP581: Barometric pressure sensor for altitude estimation
  - LSM6DSOX: Alternative IMU (legacy support)
- **Communication**: SPI for sensor interfaces, Serial for telemetry

## Project Structure

```
├── src/
│   ├── main.cpp                 # Main program and sensor scheduler
│   └── sensors/
│       ├── bmp581.cpp           # Barometer driver
│       ├── ism6hg256x.cpp       # Primary IMU driver
│       ├── kalman_filter.cpp    # Linear Kalman filter implementation
│       ├── lsm6dsox.cpp         # Alternative IMU driver
│       ├── rocket_ahrs.cpp      # AHRS and sensor fusion logic
│       └── virtualbaro.cpp      # Virtual barometer (reserved for future use)
├── include/                     # Header files for sensor and utility modules
├── lib/                         # External libraries
├── test/                        # Unit tests and validation
├── platformio.ini              # PlatformIO configuration
└── .vscode/                    # Visual Studio Code settings

```

## Dependencies

- **Adafruit BMP5xx Library** v1.0.2+
- **Adafruit LSM6DS** v4.7.4+
- **STM32duino ISM6HG256X** v2.0.0+
- Arduino Framework (via PlatformIO/earlephilhower core)
- Fusion Library (for quaternion-based AHRS)

## Building and Deployment

### Prerequisites

- [PlatformIO](https://platformio.org/) installed (VS Code extension or CLI)
- Raspberry Pi Pico 2 connected via USB

### Build

```bash
platformio run -e rpipico2
```

### Upload

```bash
platformio run -e rpipico2 --target upload
```

### Monitor Serial Output

```bash
platformio device monitor -e rpipico2 -b 115200
```

## System Overview

### Initialization Flow

1. **Serial and SPI Setup**: Establishes communication protocols
2. **Sensor Initialization**: Powers up IMU and barometer
3. **Sensor Calibration**: 200-sample calibration with 5-point averaging for offset compensation
4. **AHRS Convergence**: 3-second warm-up to stabilize orientation estimates
5. **Vertical Bias Calibration**: Determines true vertical acceleration reference
6. **Kalman Filter Reset**: Initializes state estimation

### Runtime Operation

- **IMU Task**: Reads accelerometer and gyroscope data, updates AHRS quaternion
- **Barometer Task**: Reads pressure and altitude, updates Kalman filter measurements
- **Logger Task**: Outputs formatted telemetry including:
  - Roll, pitch, yaw (Euler angles from quaternion)
  - Vertical acceleration (Earth-referenced)
  - Kalman-filtered altitude and velocity
  - Per-task timing statistics

## Data Output Format

The system outputs real-time data via serial at 115200 baud:

```
AHRS: roll, pitch, yaw | Vert: vertical_accel (ACCEL_SOURCE) | LKF: Alt: altitude, Vel: velocity | Task timing...
```

**AHRS**: Quaternion-derived Euler angles (degrees)  
**Vert**: Vertical acceleration in m/s² with accelerometer source (HIGH_G/LOW_G)  
**LKF**: Linear Kalman Filter altitude (m) and velocity (m/s)

## Key Components
- BMP581
- ISM6HG256X
### Scheduler
Manages three concurrent sensor tasks with deterministic timing and task-specific delta-time reporting for accurate sensor fusion.

### Sensor Fusion
Uses a quaternion-based AHRS algorithm to combine accelerometer and gyroscope data for robust 3D orientation estimation independent of Earth's magnetic field.

### Kalman Filter
Implements a linear Kalman filter for altitude and velocity state estimation, fusing barometric altitude measurements with acceleration predictions.

## Development Notes

- Dual accelerometer support allows seamless switching between high-G and low-G ranges to maintain accuracy across the full acceleration spectrum
- Task scheduling ensures predictable latency for real-time control applications
- Extensive telemetry output enables post-flight analysis and algorithm validation

## Contact

For questions or contributions, contact **aiaa.csufresno@gmail.com**.
```
