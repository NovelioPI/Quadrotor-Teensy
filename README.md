# Quadrotor-Teensy

A lightweight, open-source quadcopter (drone) flight controller for the Teensy microcontroller. Designed for hobbyists who want to build and experiment with their own quadrotor using simple, easy-to-understand code.

<div align=center>
  <img src=https://github.com/user-attachments/assets/12a0205a-8a06-4331-9e00-24e171f475cc />
</div>

## Overview
Quadrotor-Teensy provides the basic software needed to control a four-rotor drone using a Teensy board and a standard IMU sensor (like the MPU-6050). The firmware includes essential features for stable flight—sensor reading, stabilization (PID), and motor control—so you can quickly get your own quadcopter flying and start tinkering with flight code.


## 1. Installation
### Requirements
- Teensy board (recommended: Teensy 4.0 or 4.1)
- Visual Studio Code with PlatformIO extension (recommended)
- USB cable for Teensy

### Step
**1. Install Visual Studio Code and PlatformIO**
   - Download [VS Code](https://code.visualstudio.com/)
   - Install the [PlatformIO extension](https://platformio.org/install)
     
**2. Get the Code**
   - Open VS Code
   - Open a terminal and run:
   - 
     ```bash
     git clone https://github.com/NovelioPI/Quadrotor-Teensy.git
     ```
**3. Open the Project in VS Code**

**4. Select Your Board**
   - Check `platformio.ini` (edit if not using Teensy 4.0).
     
**5. Build the Firmware**
   - Click the ✓ (checkmark) in the PlatformIO sidebar to build.
     
**6. Connect the Teensy & Upload**
   - Plug in your Teensy with USB.
   - Click the → (arrow) to upload the firmware.


## 2. Usage
**1. Connect your hardware**
   - Teensy (powered by USB or a 5V supply)
   - IMU (e.g., MPU-6050) to I2C pins (usually SDA: 18, SCL: 19 on Teensy 4.0)
   - ESC signal wires to digital pins
   - Radio receiver to input pins (PPM/SBUS/PWM—see your hardware)
   - Motors and power system as per your frame

**2. Initial Tests**
   - Open the Serial Monitor (plug icon in PlatformIO) to view startup info.
   - When you power on, keep the quad still for sensor calibration.

**3. Test Motor Outputs**
   - Remove propellers for safety!
   - Use your transmitter to check if motors spin on command.

**4. First Flight**
   - Move to a safe area.
   - Arm the quad (see code or your transmitter settings).
   - Gradually raise throttle and test hover.

**5. PID Tuning**
   - Adjust PID constants in the code for best flight (details in comments).


## 3. Hardware Setup

![Rancangan Perangkat Keras drawio](https://github.com/user-attachments/assets/f9df5c25-206b-4f2f-adfa-d77e82a55376)

**Typical components:**
- Teensy 4.0/4.1 board
- IMU (e.g., MPU-6050)
- 4 × Brushless motors and ESCs
- LiPo battery (3S/4S)
- Quadcopter frame + propellers (2×CW, 2×CCW)
- Radio receiver (PPM, SBUS, or PWM)
- Wires, connectors, and basic tools

**Basic wiring:**
- IMU → I2C pins (SDA/SCL) + power & ground
- ESC signal wires → 4 Teensy digital pins
- Receiver → input pin(s), power & ground
- Motors → ESCs → battery
- Teensy power: via USB (for development) or BEC/5V regulator (for flight)

**Tip:**
Double-check all wiring before connecting the battery. Keep IMU orientation consistent (Z up, X forward).


## Safety Notice
- Always remove propellers when testing indoors!
- Check code for arming/disarming logic before your first flight.


## Enjoy!
Feel free to modify, extend, and experiment with the code. This project is made for learning and DIY fun—happy flying!
