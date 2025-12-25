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
[Watch Video](videos/motor_control_no_look_or_face _not_detected.mp4)

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

- Add ultrasonic sensor for redundancy
- Implement OLED display for status visualization
- Add lane departure warning
- Integrate GPS for location-based alerts