# DroneForce
> Interface to write your custom control logic, for ArduPilot and PX4 (coming soon) autopilot systems.

[![GitHub license](https://img.shields.io/github/license/Embedded-Fault-Tolerant-Control/DroneForce-internal)](https://github.com/Embedded-Fault-Tolerant-Control/DroneForce-internal/blob/master/LICENSE)
[![GitHub issues](https://img.shields.io/github/issues/Embedded-Fault-Tolerant-Control/DroneForce-internal)](https://github.com/Embedded-Fault-Tolerant-Control/DroneForce-internal/issues)
[![GitHub forks](https://img.shields.io/github/forks/Embedded-Fault-Tolerant-Control/DroneForce-internal)](https://github.com/Embedded-Fault-Tolerant-Control/DroneForce-internal/network)
[![GitHub stars](https://img.shields.io/github/stars/Embedded-Fault-Tolerant-Control/DroneForce-internal)](https://github.com/Embedded-Fault-Tolerant-Control/DroneForce-internal/stargazers)

DroneForce lets you:
- Control individual motor PWM outputs
- Give body torque as input and automatically resolve to PWM

The framework is designed with ArduPilot in mind, and is currently being extended to PX4.


## Requirements

To build and run DroneForce-internal, you'll need:

- A compatible drone platform
- A supported embedded system (e.g., Raspberry Pi, Arduino, STM32)
- C++ compiler with C++17 support
- [CMake](https://cmake.org/) version 3.10 or higher
- Basic knowledge of drone control systems and embedded programming


## Usage

Refer to the [documentation](documentation.md) for detailed information on how to configure and use DroneForce-internal with your drone