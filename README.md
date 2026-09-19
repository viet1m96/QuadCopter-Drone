# STM32F446RE Quadcopter Drone

## Flight Demo

[▶ Watch the flight demo](./video_2026-09-19_22-12-27.mp4)

## Overview

This project implements a custom flight controller for an **X-configuration quadcopter**. It runs on a **NUCLEO-F446RE** board and uses **FreeRTOS** to organize sensor acquisition, radio-command reception, attitude stabilization, motor control, and flight-data logging.

The main objectives are:

- Receive pilot commands through the iBUS protocol.
- Measure angular velocity and acceleration.
- Estimate the roll and pitch angles.
- Stabilize the aircraft with PID control.
- Mix the control outputs and generate PWM signals for four ESCs.
- Record telemetry to a microSD card for analysis and PID tuning.

## Hardware

| Component | Purpose |
| --- | --- |
| NUCLEO-F446RE (STM32F446RET6) | Main controller running the flight algorithms and FreeRTOS |
| MPU6050 | Six-axis IMU containing a three-axis gyroscope and accelerometer |
| iBUS-compatible RC receiver | Receives throttle, roll, pitch, yaw, flight-mode, and arm/disarm commands |
| Four ESCs using SimonK-compatible PWM signaling | Control the four motors with 1,000–2,000 µs pulses |
| Four brushless motors | Generate lift and control torque |
| SDHC/SDXC microSD card | Stores flight telemetry through SPI |

### Sensors

The current version uses only the **MPU6050**:

- Three-axis gyroscope configured for a ±500 °/s measurement range.
- Three-axis accelerometer configured for a ±4 g measurement range.
- MPU6050 digital low-pass filter enabled to reduce measurement noise.
- Data acquired over 400 kHz I2C whenever the Data Ready interrupt is asserted.
- Raw samples converted to physical units and corrected using calibrated offsets.
- Automatic gyroscope and accelerometer offset calibration during startup. The drone must remain still on a level surface with its Z-axis pointing upward.

The aircraft does **not** currently include a barometer, GPS/GNSS receiver, magnetometer, optical-flow sensor, or rangefinder. It therefore has no reliable absolute reference for altitude, position, or heading.

## Main Connections

| Peripheral | STM32 pins | Purpose |
| --- | --- | --- |
| I2C1 | PB8 (SCL), PB9 (SDA) | MPU6050 communication |
| MPU6050 INT | PB12 | Rising-edge Data Ready interrupt |
| USART1 RX | PB7 | DMA-based iBUS reception |
| TIM3 CH4 | PC9 | Front-left motor PWM |
| TIM3 CH1 | PC6 | Front-right motor PWM |
| TIM3 CH2 | PC7 | Rear-right motor PWM |
| TIM3 CH3 | PB0 | Rear-left motor PWM |
| SPI2 | PB13, PB14, PB15 | microSD SCK, MISO, and MOSI |
| SD CS | PB6 | microSD Chip Select |
| USART2 | PA2, PA3 | 115,200-baud debug serial port |

## Control Algorithms

### 1. RC command processing

The firmware validates each 14-channel iBUS frame, including its header and checksum, before using it. Input channels are converted as follows:

- Throttle is normalized to the range 0–1.
- Roll, pitch, and yaw are normalized to the range -1 to 1, with a deadband around the center position.
- Separate channels select the flight mode, arm or disarm the controller, and report receiver failsafe status.
- If no valid command is received for 100 ms, a UART error occurs, or receiver failsafe is asserted, the controller enters failsafe and stops all motors.

### 2. IMU calibration

Gyroscope and accelerometer samples are checked for stillness during calibration. The resulting offsets are subtracted from subsequent measurements before the data reaches the attitude estimator and PID controllers. This reduces the MPU6050's static measurement bias.

### 3. Complementary-filter attitude estimation

Roll and pitch are calculated from the accelerometer and combined with the integrated gyroscope measurements using a **complementary filter**:

```text
angle = α × (previous_angle + gyro × dt) + (1 - α) × accelerometer_angle
α = τ / (τ + dt), where τ = 0.1 s
```

The gyroscope provides a fast response but drifts over time. The accelerometer provides a gravity reference but becomes noisy while the aircraft is moving. Combining both measurements produces a more stable attitude estimate.

### 4. Cascaded PID control

Two flight modes are implemented:

- **Rate mode:** stick input directly commands roll, pitch, and yaw angular rates. The limits are ±200 °/s for roll and pitch and ±80 °/s for yaw.
- **Angle mode:** roll and pitch sticks command angles within ±30°. An outer angle PID loop converts angle error into a target angular rate, while the inner rate PID loop tracks that target. Yaw remains rate-controlled.

The inner controller uses proportional, integral, and derivative terms. The derivative is calculated from the measurement to reduce derivative kick and is passed through a 20 Hz low-pass filter. Integral and output values are limited to prevent saturation, and the integral state is reset at low throttle.

### 5. Motor mixing and desaturation

Roll, pitch, and yaw corrections are distributed across the X-frame motors as follows:

```text
Front Left  = throttle + roll - pitch - yaw
Front Right = throttle - roll - pitch + yaw
Rear Right  = throttle - roll + pitch - yaw
Rear Left   = throttle + roll + pitch + yaw
```

The mixer scales the corrections and shifts the available throttle range when necessary, keeping every motor command between 0 and 1 before converting it to an ESC PWM pulse.

## Software Architecture

The firmware is split into independent FreeRTOS tasks:

| Task | Responsibility |
| --- | --- |
| Sensor Task | Handles Data Ready events, reads the MPU6050 through interrupt-driven I2C, and publishes the latest sample |
| Receiver Task | Receives iBUS frames through UART DMA, decodes commands, and manages timeouts and failsafe behavior |
| Control Task | Estimates attitude, runs the PID controllers and motor mixer, and updates the ESC outputs |
| Telemetry Task | Writes RC commands, IMU data, attitude, PID outputs, and individual motor throttle values to the SD card |

Single-element overwrite queues ensure that consumers always process the newest available command or sensor sample. The Control Task has the highest priority among the application tasks.

## Results

### Attitude stability and command response

The drone **maintains roll and pitch stability well** using the cascaded PID controller and complementary filter. It also **responds reliably to pilot commands** for throttle, roll, pitch, and yaw received over iBUS. Arm/disarm state handling, input validation, and failsafe logic make flight testing safer.

### Position and altitude hold

The drone's **position and altitude hold remain poor**. The primary reason is that the current system relies only on the MPU6050. This sensor cannot measure absolute position or altitude, while acceleration data is noisy and accumulates significant drift when integrated. The MPU6050 is also a low-cost IMU whose measurement quality is not sufficient for accurate position holding by itself.

Throttle is still controlled manually in the current firmware, and the altitude-hold mode has not yet been implemented in the control loop.

## Future Improvements

- Add a BMP280 or MS5611 barometer for altitude estimation and hold.
- Add optical flow and a ToF/LiDAR rangefinder for indoor position hold.
- Add GPS/GNSS for outdoor position hold.
- Add a magnetometer for absolute yaw stabilization.
- Upgrade to a lower-noise IMU and improve mechanical vibration isolation.
- Use a Kalman filter or EKF to fuse IMU, barometer, GPS, and optical-flow data.
- Implement altitude and position control loops.
- Continue analyzing telemetry and tuning the PID gains under different payloads and flight conditions.

## Telemetry and Data Analysis

Each telemetry sample contains a timestamp, pilot commands, four motor outputs, roll and pitch estimates, gyroscope and accelerometer measurements, and PID corrections. The repository includes the following Scilab scripts:

- `plot_telemetry.sce` — overall command, attitude, IMU, PID, and motor analysis.
- `plot_roll_pitch.sce` — focused roll and pitch response analysis.
- `plot_sensor_telemetry.sce` — sensor-data visualization.

These plots help evaluate the real flight response, identify vibration and noise, and tune the PID gains.

## Building and Flashing

1. Open the project in **STM32CubeIDE**.
2. Select the `Debug` or `Release` configuration and build the project.
3. Connect the NUCLEO-F446RE through ST-LINK and flash the firmware.
4. Keep the drone level and completely still during startup while the MPU6050 warms up and calibrates.
5. Verify motor order, rotation direction, propeller orientation, arm/disarm operation, and failsafe behavior before flight.

> **Safety warning:** Remove all propellers when calibrating ESCs, checking motor direction, or testing new firmware on a bench. Install the propellers only after the system has been secured and fully verified in a safe test area.
