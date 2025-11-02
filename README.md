# EDU_Robotics_Club

This repository contains the code and resources for Mark's Robotics Club project, focusing on developing an autonomous robot car with advanced navigation capabilities.

## Project Overview

The project consists of two main components:

1. **Arduino Firmware** - Controls the robot car hardware including motors, sensors, and Bluetooth communication
2. **Python Control Client** - A desktop application for controlling and monitoring the robot car via Bluetooth Low Energy (BLE)

## Features

- **Bluetooth Communication**: Real-time control and telemetry over BLE
- **IMU Integration**: Inertial Measurement Unit for heading and orientation tracking
- **PID Control**: Proportional-Integral-Derivative control for precise heading maintenance
- **Sensor Ranging**: Front sensor ranging capabilities for obstacle detection
- **Route Following**: Implementation of route following functionality
- **Telemetry Visualization**: Real-time data visualization in the Python client

## Hardware Components

- Arduino-based microcontroller platform
- IMU sensor for orientation tracking
- Motor drivers for wheel control
- Bluetooth Low Energy module
- Front-facing sensor array

## Software Architecture

### Arduino Code (`Codes/Arduino/Car_software`)

The Arduino firmware is organized into several modules:

- `Car_software.ino` - Main application entry point
- `car.hpp/.cpp` - Car control logic and motor management
- `IMU.hpp/.cpp` - Inertial Measurement Unit handling
- `BLE_Comm.hpp/.cpp` - Bluetooth Low Energy communication
- `tracker.hpp/.cpp` - Data tracking and logging
- `helpers.hpp/.cpp` - Utility functions
- `definitions.hpp` - Global definitions and data structures
- `personal_config.hpp` - Platform-specific configuration

### Python Control Client (`Codes/Python/Remte_Control_Client`)

The Python client provides a graphical interface for controlling the robot car:

- `main.py` - Main application logic and UI
- GUI built with NiceGUI framework
- Real-time telemetry visualization using ECharts
- BLE communication layer using bleak library

## Getting Started

### Prerequisites

- Arduino IDE for firmware flashing
- Python 3.8+ for the control client
- Required Python packages (numpy, nicegui, pywebview, bleak)

### Installation

1. Clone this repository
2. Install Python dependencies: `pip install numpy nicegui pywebview bleak`
3. Flash the Arduino firmware to your robot car
4. Run the Python client: `python main.py` in the `Codes/Python/Remte_Control_Client` directory

## Usage

### Controlling the Car

The Python client provides several control modes:

- **Manual Control**: Direct motor speed control
- **Heading Control**: Maintain a specific heading direction
- **Auto Mode**: Follow predefined headings and speeds
- **Sensor Ranging**: Scan the front arc for obstacles

### Monitoring Telemetry

Real-time telemetry data is displayed including:

- IMU sensor readings (heading, acceleration)
- Motor speeds and status
- Sensor ranging data
- Tracking data visualization

## Examples

Several example sketches are included:

- `Car_Route_Follow` - Route following implementation with waypoints
- `Car_Heading_Keeping` - Heading keeping functionality
- Various sensor examples (Accelerometer, Magnetometer, etc.)

## Contributing

Contributions are welcome! Please fork the repository and submit a pull request with your improvements.

## License

This project is licensed under the MIT License - see the LICENSE file for details.
