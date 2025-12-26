# Autonomous Emergency Braking with Driver Monitoring

An intelligent emergency braking system that adapts intervention levels based on real-time driver attention state monitoring.

## Overview

This system combines:
- **Driver Monitoring System (DMS)**: Real-time attention state detection using computer vision
- **Autonomous Emergency Braking (AEB)**: Distance-based collision avoidance with adaptive intervention

## Key Features

- Real-time driver state detection (drowsy, distracted, no driver)
- Distance measurement using VL53L0X sensor
- Adaptive motor speed control based on:
  - Driver state (intervention aggressiveness)
  - Object distance (intervention timing)
- Visual and audio alerts (LED, buzzer)

## System States

### Driver States Detected:
1. **Drowsy** (Eyes Closed) - Most aggressive intervention
2. **Distracted** (Looking Away) - Moderate intervention  
3. **No Face Detected** - Moderate intervention

### Intervention Levels:

| Distance | Drowsy | Distracted/No Face |
|----------|--------|-------------------|
| > 70cm   | 80%    | 100%              |
| 40-70cm  | 60%    | 75%               |
| 20-40cm  | 40%    | 50%               |
| 10-20cm  | 25%    | 30%               |
| < 10cm   | STOP   | STOP              |

## Technologies Used

### Software:
- **Python**: OpenCV, MediaPipe (face detection & attention monitoring)
- **C**: STM32 HAL (embedded control)

### Hardware:
- STM32 microcontroller
- VL53L0X distance sensor
- DC motor + driver
- Buzzer & LED indicators
- Camera (for driver monitoring)

## Demo Videos

### 1. Driver State Detection
[Watch Video](videos/driver_state_based_warning.mp4)

### 2. Drowsy Driver Intervention  
[Watch Video](videos/motor_control_closed_eyes.mp4)

### 3. Distracted Driver Intervention
[Watch Video](Videos/motor_control_no_look_or_face_not_detected.mp4) 

## Project Structure
```
├── python/          # Driver monitoring code
├── stm32/           # STM32 embedded code
├── videos/          # Demo videos
└── README.md        # This file
```

## How It Works

1. **Camera captures driver's face** → Python processes with MediaPipe
2. **Python detects driver state** → Sends status to STM32 via serial
3. **VL53L0X measures distance** → STM32 reads sensor data
4. **STM32 makes decision** → Controls motor, LED, buzzer based on both inputs

## Future Improvements

### Short-term (Feasible with current hardware)
- **Ultrasonic sensor integration**: Add redundant distance measurement for improved accuracy
- **OLED display**: Visualize system status (driver state, distance, motor speed) in real-time
- **Data logging**: Record driver states and interventions for analysis and tuning

### Medium-term (Next project evolution)
- **Lane departure warning**: Use camera to detect lane markings and alert driver
- **Traffic sign recognition**: Detect and classify road signs for context-aware warnings
- **Multi-object tracking**: Handle multiple obstacles simultaneously

### Long-term (Production-oriented)
- **CAN bus integration**: Connect to actual vehicle systems
- **ISO 26262 compliance**: Implement automotive safety standards
- **Speed measurement**: Add encoder for closed-loop motor control

## Challenges & Solutions

### Challenge 1: Roll Angle Instability (Gimbal Lock)

**Problem:**  
When using Euler angle decomposition from `cv2.solvePnP()`, the roll angle became extremely unstable when the driver tilted their head slightly down. The roll value would jump between -180° and +180°, causing false alarms.

**Root Cause:**  
Gimbal lock occurs when pitch approaches ±90°, making roll and yaw mathematically indistinguishable. Even small head movements caused massive roll angle fluctuations.

**Solution:**  
Implemented geometric roll calculation by measuring the angle between the eye centers, instead of relying on Euler angle decomposition. This approach:
- Eliminates gimbal lock completely
- Provides stable measurements regardless of pitch/yaw
- Uses simple geometry: `roll = atan2(Δy, Δx)` between left and right eye positions

**Result:**  
Roll angle stability improved dramatically, with smooth values (±5°) during normal head movements and accurate detection of actual head tilting (±25-30°).

---

### Challenge 2: Serial Port Conflict with Debugging Tools

**Problem:**  
During development, the Python script couldn't send driver status data to STM32 when CoolTerm (serial monitor) was running simultaneously. Only one application could access the COM port at a time, preventing real-time debugging of serial communication.

**Root Cause:**  
Windows serial ports allow exclusive access - when CoolTerm occupied COM3, Python's `serial.Serial()` connection would fail with a "port already in use" error.

**Solution:**  
Switched to STM32's built-in debugging capabilities:
- Closed CoolTerm during system operation
- Used ST-Link debugger in STM32CubeIDE to monitor variables in real-time
- Observed `driver_status` and `distance` variables directly in the debug perspective
- This allowed verification of data reception without occupying the COM port

**Result:**  
Successfully debugged serial communication by leveraging ST-Link debugging instead of serial monitoring, allowing simultaneous Python-to-STM32 communication and variable observation.

---

### Challenge 3: Insufficient Power Supply for Motor

**Problem:**  
The DC motor couldn't be powered directly from the STM32's 5V pin, as it drew too much current and caused the microcontroller to reset or behave erratically. The STM32's power supply couldn't handle the motor's current demands.

**Root Cause:**  
DC motors require significantly higher current (typically 500mA-2A) than what STM32's voltage regulator can safely provide (~200mA), especially under load.

**Solution:**  
Implemented an external power supply solution:
1. Used a USB power bank as external 5V power source
2. Cut a USB cable to access VCC and GND wires
3. Connected power bank VCC to motor driver's VIN
4. **Established common ground** between STM32 GND and power bank GND (critical for proper operation)
5. STM32 controls motor via PWM signals to driver, while motor driver is powered separately

**Result:**  
Motor operates reliably with sufficient current, while STM32 remains stable and only handles low-current control signals (PWM). This is standard practice in embedded systems where actuators require external power supplies.