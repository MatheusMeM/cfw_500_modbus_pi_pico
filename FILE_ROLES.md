# Project File Roles (Post-Modbus Refactoring)

This document describes the purpose of each key Python file in the `modbus_micropython` directory after refactoring the Relay-Main communication to use Modbus RTU.

## Relay Pico (`modbus_micropython/relay_pico/`)

*   **`main.py`**:
    *   Runs on the Relay Pico.
    *   Acts as a **Modbus RTU Master** communicating with the Main Pico over RS485 (Network 1 - UART0).
    *   Handles communication with the Host PC via USB Serial.
    *   Reads text commands from the PC.
    *   Translates PC commands into Modbus write operations (e.g., writing command codes, target RPM) to specific Holding Registers on the Main Pico Slave.
    *   Periodically reads various Input Registers from the Main Pico Slave (e.g., current RPM, VFD status, encoder position, fault flags) via Modbus read operations.
    *   Formats and prints the received status information to the PC via USB Serial.
    *   Uses the `umodbus` library for Modbus communication.

*   **`lib/umodbus/`**: (Directory to be created/copied)
    *   Contains the `micropython-modbus` library files required for the Relay Pico's Modbus Master functionality.

## Main Pico (`modbus_micropython/main_pico/`)

*   **`main.py`**:
    *   Runs on the Main Pico.
    *   The main application entry point, orchestrating operations using `uasyncio`.
    *   Initializes all hardware components (Pins, UARTs for Modbus).
    *   Initializes both Modbus roles:
        *   **Modbus RTU Slave** on UART1 (Network 1) to listen for requests from the Relay Pico Master.
        *   **Modbus RTU Master** on UART0 (Network 2) to communicate with the VFD Slave.
    *   Manages background asynchronous tasks:
        *   Polling VFD status (`vfd_status_request_task`).
        *   Controlling status relays (`relay_control_task`).
        *   Processing incoming Modbus Slave requests (`modbus_slave_poll_task`).
    *   Handles the motor homing sequence.
    *   Runs the main asynchronous loop, which periodically calls `command_processor.process_modbus_commands`.

*   **`utils.py`**:
    *   Contains shared utility functions, constants, and data structures for the Main Pico code.
    *   Defines the **Modbus Slave Register Map** (`slave_registers`) which dictates how the Relay Master interacts with the Main Pico's data.
    *   Manages the internal application state (`internal_state`), distinct from the Modbus registers but often mirrored to/from them.
    *   Handles loading and saving persistent configuration to `config.json`.
    *   Provides the local verbose printing function (`print_verbose`).
    *   Includes helper functions like `update_input_registers` to safely update the Modbus input registers exposed to the Relay Master.

*   **`command_processor.py`**: (New file)
    *   Encapsulates the logic for acting upon commands received via Modbus from the Relay Master.
    *   Reads the relevant Modbus Holding Registers (e.g., `REG_CMD`, `REG_TARGET_RPM`, `REG_VERBOSE`, `REG_ENC_MODE`).
    *   Detects changes or specific command codes written by the Relay Master.
    *   Executes the corresponding actions, such as:
        *   Calling methods on the VFD Master object (`vfd_master`) to start/stop/reverse the motor or reset faults.
        *   Triggering the encoder calibration process.
        *   Updating internal settings (`internal_state`) like verbosity or encoder mode.
    *   Resets the command register after processing.

*   **`cfw500_modbus.py`**:
    *   Defines the `CFW500Modbus` class.
    *   Encapsulates the Modbus RTU **Master** communication logic specifically for interacting with the WEG CFW500 VFD (Network 2 - UART0).
    *   Provides methods for reading VFD parameters (speed, status, max RPM, etc.) and writing control words or speed references to the VFD.

*   **`encoder_module.py`**:
    *   Initializes the low-level encoder driver (`encoder.py`).
    *   Defines the `encoder_callback` function which is executed upon encoder movement.
    *   Processes the raw encoder steps provided by the driver.
    *   Applies homing and calibration offsets (`internal_state['encoder_zero_offset']`, `internal_state['encoder_offset_steps']`).
    *   Calculates the final position in steps and degrees.
    *   Updates the corresponding Modbus Input Registers (`slave_registers['input'][REG_ENC_POS_STEPS]`, `slave_registers['input'][REG_ENC_POS_DEG]`) via `update_input_registers`.
    *   Updates `internal_state` variables related to the encoder.

*   **`encoder.py`**:
    *   The low-level asynchronous quadrature encoder driver (based on Peter Hinch's library).
    *   Uses hardware pin interrupts (`Pin.IRQ_RISING | Pin.IRQ_FALLING`) to detect encoder rotation.
    *   Counts raw steps and triggers the `encoder_callback` in `encoder_module.py` via `uasyncio`.

*   **`lib/umodbus/`**:
    *   Contains the `micropython-modbus` library files. Used by both the `ModbusRTUSlave` instance (Network 1) and the `CFW500Modbus` class (which uses `ModbusRTUMaster` internally, Network 2).

*   **`config.json`**:
    *   Stores persistent configuration values (currently `encoder_offset_steps`, `encoder_output_mode`, `VERBOSE_LEVEL`) for the Main Pico, loaded/saved by `utils.py`.

*   **~~`motor_control.py`~~**: **(Redundant)**
    *   This file previously contained only an initialization function for the `CFW500Modbus` class.
    *   This initialization is now done directly within `main.py`.
    *   **This file should be deleted.**

*   **~~`commands.py`~~**: **(Redundant)**
    *   This file previously handled parsing text commands received via direct UART.
    *   Command handling is now done by `command_processor.py` by reading Modbus registers.
    *   **This file should be deleted.** (Assuming it was successfully deleted earlier).