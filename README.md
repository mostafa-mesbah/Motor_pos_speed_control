# README: Speed and Position Control for DC Motor with Arduino and Encoder

This project demonstrates precise **speed** and **position control** of a DC motor using an Arduino microcontroller, an encoder, and a PID control algorithm. It enables the motor to operate at desired speeds or move to specific positions, providing a robust solution for automation and robotics applications.

---

## Features

* **Speed Control:**

  * Adjust motor speed using a potentiometer.
  * Real-time RPM calculation and display.

* **Position Control:**

  * Set target positions using a potentiometer.
  * PID control ensures accurate positioning.

* **Debounced Button Control:**

  * Toggle between speed and position control modes.
  * Change motor direction with a button press.

* **Encoder Feedback:**

  * Tracks motor position and speed for accurate control.

---

## Components Required

* Arduino Board (e.g., Uno, Mega, or Nano)
* DC Motor with Encoder
* Motor Driver (e.g., L298N, BTS7960)
* Rotary Encoder
* Potentiometers (2x for speed and position control)
* Push Buttons (2x for mode selection and motor direction)
* Power Supply
* Connecting Wires and Breadboard

---

## Pin Configuration

| Pin            | Function                   |
| -------------- | -------------------------- |
| `ENCA` (Pin 8) | Encoder Channel A          |
| `ENCB` (Pin 3) | Encoder Channel B          |
| `PWM` (Pin 5)  | Motor PWM Signal           |
| `IN1` (Pin 7)  | Motor Driver Input 1       |
| `IN2` (Pin 6)  | Motor Driver Input 2       |
| `BUTTON_PIN1`  | Toggle Speed/Position Mode |
| `BUTTON_PIN2`  | Change Motor Direction     |
| `A0`           | Potentiometer for Speed    |
| `A5`           | Potentiometer for Position |

---

## How It Works

1. **Speed Control Mode:**

   * Adjust the potentiometer connected to `A0` to control motor speed.
   * The motor's current speed (RPM) is calculated and displayed via the serial monitor.

2. **Position Control Mode:**

   * Adjust the potentiometer connected to `A5` to set the target position.
   * The PID algorithm calculates error and generates a PWM signal to move the motor to the desired position.

3. **Button Functionality:**

   * Press `BUTTON_PIN1` to toggle between speed and position control modes.
   * Press `BUTTON_PIN2` to reverse motor direction during speed control.

---

## Code Overview

### Initialization (`setup`)

* Configures pins for the encoder, motor driver, buttons, and potentiometers.
* Sets up interrupts for encoder position tracking and button press detection.

### Main Loop (`loop`)

* Executes speed or position control based on the selected mode.
* Continuously calculates and updates the motor's behavior using encoder feedback.

### Key Functions

* **`project_speed_control()`**: Handles speed control using encoder feedback.
* **`project_pos_control()`**: Implements position control with PID.
* **`move_motor(dir, pwmVal)`**: Moves the motor in the specified direction with the given PWM value.
* **`updateMotorPosition()`**: Updates the motor position using encoder pulses.
* **`calculate_rpm()`**: Calculates motor RPM based on encoder data.
* **`buttonISR()`**: Toggles between speed and position control modes.

---

## Setup Instructions

1. **Hardware Connections:**

   * Follow the pin configuration table for wiring.
   * Connect the encoder to the Arduino for feedback.
   * Attach the motor driver to control the DC motor.

2. **Software Upload:**

   * Open the code in Arduino IDE.
   * Set the PID parameters (`kp`, `ki`, `kd`) in the code to match your system.
   * Upload the code to your Arduino board.

3. **Operation:**

   * Use the potentiometer to set speed or position based on the mode.
   * Observe the output on the serial monitor for debugging.

---

## Troubleshooting

* **Motor Not Moving:**

  * Check motor driver connections and power supply.
  * Ensure encoder pins are correctly connected.

* **Incorrect Positioning:**

  * Fine-tune the PID constants (`kp`, `ki`, `kd`).

* **Button Not Working:**

  * Verify debounce delay and button wiring.

---


