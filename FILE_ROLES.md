# Project File Roles (Post-Modbus Refactoring)

This document describes the purpose of each key Python file in the `modbus_micropython` directory after refactoring the Relay-Main communication to use Modbus RTU.

## Relay Pico (`modbus_micropython/relay_pico/`)

*   **`main.py`**:
    *   Runs on the Relay Pico.
    *   Acts as a **Modbus RTU Master** communicating with the Main Pico over RS485 (Network 1 - UART0, 115200 baud).
    *   Handles communication with the Host PC via USB Serial.
    *   Reads text commands from the PC.
    *   Translates PC commands into Modbus write operations (Function Code 06 or 16) to specific Holding Registers (100-103) on the Main Pico Slave.
    *   Periodically reads multiple Input Registers (0-7) from the Main Pico Slave via Modbus read operations (Function Code 04).
    *   Formats and prints the received status information to the PC via USB Serial.
    *   Uses the `umodbus` library for Modbus communication. Includes short delays around Modbus calls to aid stability.
*   **`lib/umodbus/`**:
    *   Contains the `micropython-modbus` library files required for the Relay Pico's Modbus Master functionality.

## Main Pico (`modbus_micropython/main_pico/`)

*   **`main.py`**:
    *   Runs on the Main Pico.
    *   Main application entry point using `uasyncio`.
    *   Initializes hardware (Pins, UARTs).
    *   Initializes **Modbus RTU Slave** on UART1 (Network 1) via `umodbus.modbus.Modbus` and `umodbus.serial.Serial`. Listens for requests from Relay Master.
    *   Initializes **Modbus RTU Master** on UART0 (Network 2) via `cfw500_modbus.CFW500Modbus` class for VFD communication.
    *   Assigns `handle_command_register_write` callback to Holding Register 100 (`command`) *before* calling `setup_registers`. This callback handles action commands (Start, Stop, Reverse, Reset, Calibrate) immediately upon receiving the Modbus write.
    *   Manages background asynchronous tasks:
        *   `vfd_status_request_task`: Polls VFD status/speed via Modbus Master (Network 2). Updates Input Registers via `utils.update_input_registers`.
        *   `relay_control_task`: Controls status relays based on fault state.
        *   `modbus_slave_poll_task`: Periodically calls `modbus_slave_handler.process()` to handle incoming Modbus Slave requests (Network 1). Executes callbacks (like `handle_command_register_write`) within this context.
    *   Performs initial safety stop and homing sequence.
    *   Runs the main async loop, periodically calling `command_processor.process_modbus_commands` to handle settings changes.
*   **`utils.py`**:
    *   Contains shared utility functions, constants, and data structures.
    *   Defines the **Modbus Slave Register Map** (`slave_registers`) dictionary (Holding Regs 100-103, Input Regs 0-7).
    *   Defines the `internal_state` dictionary for non-Modbus state variables.
    *   Handles loading/saving persistent configuration (`config.json`).
    *   Provides `print_verbose` for local conditional printing.
    *   Provides `update_input_registers` helper to safely update values in the `slave_registers['IREGS']` dictionary, which are then read by the Relay Master.
*   **`command_processor.py`**:
    *   Contains `process_modbus_commands` async function.
    *   Called periodically by `main.py`'s main loop.
    *   Reads **settings** Holding Registers (`verbosity_level`, `encoder_mode`) from the `slave_registers` dictionary.
    *   Compares read values to `internal_state` and updates `internal_state` and `config.json` if changes are detected.
    *   **Does NOT handle action commands (Start/Stop etc.)** - these are handled by the callback in `main.py`.
*   **`cfw500_modbus.py`**:
    *   Defines the `CFW500Modbus` class.
    *   Encapsulates Modbus RTU **Master** logic for VFD communication (Network 2).
    *   Provides methods like `start_motor`, `stop_motor`, `read_current_speed`, `check_fault`, etc.
*   **`encoder_module.py`**:
    *   Initializes the low-level encoder driver (`encoder.py`).
    *   Defines the `encoder_callback` function triggered by `encoder.py`.
    *   Processes raw steps, applies offsets (`encoder_zero_offset`, `encoder_offset_steps`).
    *   Calculates final steps/degrees.
    *   Calls `update_input_registers` in `utils.py` to update the corresponding Modbus Input Registers.
*   **`encoder.py`**:
    *   Low-level asynchronous quadrature encoder driver using pin interrupts.
*   **`lib/umodbus/`**:
    *   Contains `micropython-modbus` library files. Used by both Slave (Network 1) and Master (Network 2 via `cfw500_modbus.py`).
*   **`config.json`**:
    *   Stores persistent configuration (`encoder_offset_steps`, `encoder_output_mode`, `VERBOSE_LEVEL`).

*   **~~`motor_control.py`~~**: **(Redundant/Deleted)**
*   **~~`commands.py`~~**: **(Redundant/Deleted)**