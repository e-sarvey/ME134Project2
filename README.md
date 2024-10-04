# ME134Project2: 5-Bar Drawing Arm

## Table of Contents

1. [Project Overview](#project-overview)
2. [Repository Contents](#repository-contents)
    1. [main_pc_program.py](#1-main_pc_programpy)
    2. [trackpad_tracing.py](#2-trackpad_tracingpy)
    3. [stepper_testing.ino](#3-stepper_testingino)
    4. [MQTT_testing.ino](#4-mqtt_testingino)
    5. [main_esp32_code.ino](#5-main_esp32_codeino)
3. [How to Run the Project](#how-to-run-the-project)
4. [Future Work](#future-work)
## Project Overview

The assignment for this project was to build a robot arm that can draw our initials. The robot arm had to have coupled degrees of freedom, meaning no 3d-printer style linear gantrys were allowed.

<img width="497" alt="Screenshot 2024-10-03 at 8 53 23 PM" src="https://github.com/user-attachments/assets/482dc666-0159-4b6e-83b4-d2996179d737">

Our goal with this project was to make a drawing robot that was versitile and unique. We chose a 5-bar design because it offers both stability with both actuators at the base, and a "coolness" factor since it is unlike the common two link drawing arms. Our stretch goals were to implement a retractable pen holder for drawing discontinuous curves and include an easy method for the user to input any trajectory using the computer trackpad and have the robot draw it out.

The largest challenges with this project were getting the steppers to move reliably and avoiding singularity points returned from the inverse kinematics since certain configurations of the arm would remove a degree of freedom and lock up the arm. Late in the project we realized we were not delivering enough current to the steppers and switched to a higher current power supply which moved the steppers more smoothly but not long after switching power supplies we accidentally shorted one of the motor drivers by accident... This is likely why we saw very jittery and inconsistant motion in much of our testing. With the underpowered motors it was hard to tell what inaccuracies were a result fo the hardware and what might be a result of the software. Additional work on the IK solver could address this by using forward kinematics to verify solutions. We had this implemented at one point before realizing our IK was wrong and having to re-calculate it.

<img width="725" alt="Screenshot 2024-10-03 at 3 40 18 PM" src="https://github.com/user-attachments/assets/a4e6fc4c-802e-4c4c-acd0-f669093b4a13">
<img width="414" alt="Screenshot 2024-09-26 at 2 41 44 PM" src="https://github.com/user-attachments/assets/64d0eb51-2ec1-4ae1-8a40-0da38484db22">

## Repository Contents

### 1. `main_pc_program.py`

This is the primary Python program for running the trajectory planning and inverse kinematics on the computer. It connects to the MQTT broker, calculates the desired motor angles using inverse kinematics, and then publishes the results to the ESP32.

Key functions:
- `invKin(x, y)`: Calculates the motor angles for a given (x, y) coordinate.
- `workspaceMap(...)`: Visualizes the workspace of the robot, showing valid and invalid areas for the arm to reach.
- `armSim(...)`: Simulates the arm's motion based on the calculated motor angles.
- `main()`: The main function, which runs the trajectory planning, inverse kinematics, and sends the calculated motor angles to the ESP32 via MQTT.

### 2. `main_esp32_code.ino`

This is the main Arduino sketch running on the ESP32. It controls the stepper motors based on trajectory data received wirelessly from the computer via the MQTT protocol. The received data includes motor angles that are converted to stepper motor movements.

Key features:
- Handles MQTT message reception and parses angle data.
- Controls two stepper motors using the `AccelStepper` library.
- Manages Wi-Fi connection and MQTT subscription.
- Debugging LEDs provide visual feedback for Wi-Fi connection, MQTT messages, and motor trajectory completion.


### 3. `trackpad_tracing.py`

This file contains the `TrackpadDraw` class, which provides functionality for users to draw a trajectory on a virtual trackpad using Matplotlib. The drawn trajectory is captured and scaled for use in the inverse kinematics calculations in `main_pc_program.py`.

Key functions:
- `on_click(event)`: Captures user clicks on the trackpad and stores them as coordinates.
- `clickDraw(...)`: Allows the user to draw a trajectory, scale it, and apply an offset to the coordinates.

### 4. `stepper_testing.ino`

This Arduino sketch is used to test the stepper motors connected to the ESP32. It supports basic motor control, including homing the motors using limit switches and moving the motors to a specific angle.

Key features:
- Stepper motor control using the `AccelStepper` library.
- Homing functionality for both motors using limit switches.
- Serial interface for manually entering commands to move the motors.

### 5. `MQTT_testing.ino`

This Arduino sketch tests the ESP32's connection to the MQTT broker. It connects the ESP32 to Wi-Fi, subscribes to an MQTT topic, and publishes a test message every 5 seconds. It also handles incoming MQTT messages and displays them over the serial monitor.

Key features:
- Wi-Fi connection setup.
- MQTT client setup and message handling.
- Continuous publishing of test messages to verify MQTT communication.

## How to Run the Project

1. **Install Required Libraries**:
    - For Python, ensure you have the following installed:
        ```bash
        pip install numpy matplotlib paho-mqtt
        ```
    - For the ESP32, use the Arduino IDE to install the following libraries:
        - [AccelStepper](https://www.airspayce.com/mikem/arduino/AccelStepper/)
        - [PubSubClient](https://pubsubclient.knolleary.net/)

2. **Set Up MQTT**:
    - In `mysecrets.py` (on the PC) and in the ESP32 code (`MQTT_testing.ino` and `main_pc_program.py`), update the Wi-Fi and MQTT broker credentials.

3. **PC-side (Python)**:
    - Run `main_pc_program.py` on your computer to start the trajectory planning and inverse kinematics calculations. The results will be sent to the ESP32 via MQTT.

4. **ESP32-side (C++/Arduino)**:
    - Flash the ESP32 with `main_esp32_code.ino` to control the stepper motors.
    - Alternatively, run `stepper_testing.ino` to test motor control and `MQTT_testing.ino` to verify MQTT connectivity.

## Future Work

- Integration of limit switches for homing and emergency stops in the main motor control program.
- Improvements in trajectory planning for more complex paths and continuous motion.
- MQTT buffer handling on the ESP-32 to handle large trajectories
