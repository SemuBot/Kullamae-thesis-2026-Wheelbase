# Kullamae-thesis-2026-Wheelbase

Repository for the master’s thesis project **“Controller for the Mobility Subsystem of the SemuBot Social Humanoid Robot”**.

This repository contains the hardware design files, STM32 firmware and ROS 2 workspace developed for the SemuBot mobility subsystem.

The project focuses on replacing the previous bulky SemuBot wheelbase electronics, which used three separate commercial BLDC driver modules and Raspberry Pi PWM control, with a compact STM32-based controller board supporting:

- 3 BLDC motors
- 3 DRV8353 gate drivers
- 3 absolute SSI encoders
- STM32F303RET6 microcontroller
- USB CDC communication
- micro-ROS communication
- ROS 2 integration
- `ros2_control` support

The main goal is to provide a compact, maintainable and ROS 2-compatible controller for the SemuBot wheelbase.

---

## Repository structure

```text
Kullamae-thesis-2026-Wheelbase/
├── semubot-electronics/
├── semubot-firmware/
├── wheelbase_ws/
├── .gitignore
├── .gitmodules
├── LICENSE
└── README.md
