# Electric Toy Car Firmware (ESP32-C6)

This repository contains the firmware for a dual-ESP32-C6 system controlling a custom electric toy car.

## System Architecture

1.  **Controller Board (Remote)**
    - Interfaces with an I2C mini-joystick to gather user input.
    - Transmits control state to the Main Drive Board.
2.  **Main Drive Board (Vehicle)**
    - Receives instructions from the Controller Board.
    - Processes safety overrides and onboard controls.
    - Drives the motors via an H-bridge.
    - Manages other vehicle peripherals (e.g., lights, battery monitoring).

## AI Assistant Guidelines

When generating code or answering questions for this project, adhere to the following rules:

### Embedded Systems Context
- **Target Architecture**: Assume the target is **ESP32-C6 (RISC-V)**. Ensure any assembly or architecture-specific features recommended are RISC-V compatible (do not suggest Xtensa-specific features).
- **Performance & Constraints**: 
  - Code must be non-blocking. Avoid `delay()` or busy-wait loops. Use hardware timers, interrupts, or async executors.
  - Avoid dynamic memory allocation (`malloc`, `String`, or `Vec` resizing) in hot execution paths or control loops to prevent fragmentation.
- **Hardware Interfacing**: Provide robust error handling for hardware buses. I2C reads and network packet receptions can fail; always handle timeouts and missing data gracefully to keep the vehicle safe.

### Focus on Safety
- Provide fail-safes: If the Main Drive Board loses connection with the Controller Board, the motors must immediately default to a stopped/safe state.
- Ensure PWM configurations for the H-bridge include dead-time insertion if applicable to prevent shoot-through short circuits.

### Communication
- Note that the two boards must remain in sync. When writing communication protocols (e.g., ESP-NOW, BLE, or UDP), ensure data structures are tightly packed, endian-aware, and include sequence numbers or checksums to drop stale/corrupted packets.