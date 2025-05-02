# Raspberry Pi Pico VFD Motor Control with Modbus RTU

This project implements a system for controlling a Variable Frequency Drive (VFD) and motor using two Raspberry Pi Pico boards, communicating with a host PC, and reading a quadrature encoder for position feedback. One Pico acts as the main controller (`main_pico`), directly interfacing with the VFD (WEG CFW500) via Modbus RTU and the encoder. The second Pico (`relay_pico`) acts as a communication bridge between the host PC (USB Serial) and the main controller using Modbus RTU over RS485.

---

## System Architecture

```
+------+   USB Serial   +-----------------+   Modbus RTU (RS485, 19200 baud)    +----------------+   Modbus RTU (RS485, 19200 baud)      +---------+       +-------+
|  PC  | <------------> | Relay Pico      | <---------------------------------> | Main Pico      | <-----------------------------------> | CFW500  | ----> | Motor |
|      |                | (Modbus Master) |          (Network 1)                | (Modbus Slave) |        (Network 2 - VFD Master)       | (Slave) |       +-------+
+------+                +-----------------+                                     +----------------+                                       +---------+
                                                                                        |
                                                                                        | Encoder Signals (A, B)
                                                                                        v
                                                                                  +-----------+
                                                                                  |  Encoder  |
                                                                                  +-----------+
                                                                                        | Z Signal (Endstop)
                                                                                        v
                                                                                     GPIO Pin
```

**Network 1 (Relay <-> Main):**
-   **Protocol:** Modbus RTU over RS485
-   **Baud Rate:** 115200 (Configurable)
-   **Roles:** Relay Pico (Master), Main Pico (Slave Address 1)
-   **Purpose:** Relay sends commands (writes to Holding Registers) and requests status (reads Input Registers) from Main Pico.

**Network 2 (Main <-> VFD):**
-   **Protocol:** Modbus RTU over RS485
-   **Baud Rate:** 19200 (Configurable)
-   **Roles:** Main Pico (Master), CFW500 VFD (Slave Address 1)
-   **Purpose:** Main Pico sends control commands (writes) and reads status/parameters (reads) from the VFD.

---

## Features

-   **Two-Pico Design:** Separates PC communication (Relay Pico) from real-time control (Main Pico).
-   **Modbus Communication:** Uses Modbus RTU for both Inter-Pico (Network 1) and VFD (Network 2) communication via the `micropython-umodbus` library.
-   **Motor Control via VFD (CFW500):**
    -   Start, stop, and reverse the motor with specified RPM via Modbus commands from Relay to Main.
    -   Read motor speed (RPM) and VFD status/faults via Main Pico polling VFD.
    -   Reset VFD faults via Modbus command.
-   **Quadrature Encoder Integration:**
    -   Read encoder positions in steps or degrees using hardware interrupts and `uasyncio`.
    -   Support for zero-calibration (`calibrate` command) storing offset in `config.json`.
    -   Homing Routine: Uses the encoder's Z signal connected as an endstop.
-   **Command Handling (Main Pico):**
    -   **Action Commands (Start, Stop, Reverse, Reset, Calibrate):** Handled immediately via a Modbus register write callback (`handle_command_register_write` in `main.py`) for responsiveness. Target RPM is read from the Modbus write payload if available (multi-register write).
    -   **Settings Commands (Verbose Level, Encoder Mode):** Handled by polling Holding Registers in the main loop (`process_modbus_commands` in `command_processor.py`).
-   **Status Reporting:** Main Pico updates Input Registers periodically (e.g., RPM, VFD Status, Encoder Position, Flags). Relay Pico periodically reads these Input Registers and displays them on the PC.
-   **Configuration Persistence:** Saves `encoder_offset_steps`, `encoder_output_mode`, and `VERBOSE_LEVEL` to `config.json` on the Main Pico.
-   **Asynchronous Operation:** Uses `uasyncio` for managing concurrent tasks (Modbus polling, VFD status checks, relay control, encoder handling).
-   **Verbose Levels (0-3):** Controls local debug output on the Main Pico.

---

## Requirements

### Hardware

-   **Host PC:** To send commands and receive status.
-   **Raspberry Pi Pico (x2):** One for Relay, one for Main Controller.
-   **Quadrature Encoder (e.g., Omron E6B2-CWZ6C):** Connected to Main Pico.
-   **WEG CFW500 VFD (or compatible):** With Modbus RTU interface.
-   **RS485 to UART Converters (x2):** With DE/RE control pins.
-   **Motor:** Compatible with the VFD.
-   **Power Supplies:** For Picos, VFD, Encoder.
-   **Wiring:** Appropriate cables for USB, RS485, Encoder, Motor.

### Software

-   **MicroPython Firmware:** Flashed onto both Picos (v1.17+ recommended).
-   **Required Libraries:**
    -   `uasyncio` (built-in)
    -   `umodbus` (included in `lib/` directories for both Picos)
-   **Host PC Software:** Serial terminal emulator (e.g., Thonny REPL, PuTTY).

---

## Connections

*(Refer to the ASCII diagram above)*

### Relay Pico (Network 1 Master)

| Function         | Relay Pico Pin | Connection                             |
|------------------|----------------|----------------------------------------|
| USB Serial       | Micro USB Port | Host PC                                |
| RS485 TX (UART0) | GPIO0 (Pin 1)  | RS485 Converter (Net 1) - TXD / DI     |
| RS485 RX (UART0) | GPIO1 (Pin 2)  | RS485 Converter (Net 1) - RXD / RO     |
| RS485 DE/RE      | GPIO2 (Pin 4)  | RS485 Converter (Net 1) - DE & RE      |
| Power (+5V)      | VBUS / VSYS    | 5V Power Supply                        |
| Ground           | GND            | Common Ground                          |

### Main Pico (Network 1 Slave / Network 2 Master)

| Function            | Main Pico Pin  | Connection                          |
|---------------------|----------------|-------------------------------------|
| RS485 TX (UART1)    | GPIO4 (Pin 6)  | RS485 Converter (Net 1) - TXD / DI  |
| RS485 RX (UART1)    | GPIO5 (Pin 7)  | RS485 Converter (Net 1) - RXD / RO  |
| RS485 DE/RE (UART1) | GPIO6 (Pin 9)  | RS485 Converter (Net 1) - DE & RE   |
| Modbus TX (UART0)   | GPIO0 (Pin 1)  | RS485 Converter (Net 2) - TXD / DI  |
| Modbus RX (UART0)   | GPIO1 (Pin 2)  | RS485 Converter (Net 2) - RXD / RO  |
| Modbus DE/RE (UART0)| GPIO2 (Pin 4)  | RS485 Converter (Net 2) - DE & RE   |
| Encoder A           | GPIO16 (Pin 21)| Encoder A Signal                    |
| Encoder B           | GPIO17 (Pin 22)| Encoder B Signal                    |
| Encoder Z / Endstop | GPIO18 (Pin 24)| Encoder Z Signal (Used for Homing)  |
| Relay 1 Output      | GPIO20 (Pin 26)| External Relay Coil (+)             |
| Relay 2 Output      | GPIO21 (Pin 27)| External Relay Coil (+)             |
| Onboard LED         | GPIO25         | (Internally connected)              |
| Power (+5V)         | VBUS / VSYS    | 5V Power Supply                     |
| Ground              | GND            | Common Ground                       |

*Note: Ensure all devices share a common ground.*
*Note: Encoder power (+5V/Brown, GND/Blue) should be connected appropriately.*

---

## Modbus Register Map (Network 1: Relay <-> Main)

*(Defined in `main_pico/utils.py`)*

**Holding Registers (Writeable by Relay Master)**

| Address | Name              | Description                                                              | Default | Handled By        |
|---------|-------------------|--------------------------------------------------------------------------|---------|-------------------|
| 100     | `command`         | 1=Start, 2=Stop, 3=Reverse, 4=Reset Fault, 5=Calibrate, 0=No Action    | 0       | Callback (`main.py`) |
| 101     | `target_rpm`      | Target RPM for Start/Reverse commands                                    | 0       | Callback (`main.py`) |
| 102     | `verbosity_level` | Verbosity level for Main Pico local prints (0-3)                         | 1       | Polling (`command_processor.py`) |
| 103     | `encoder_mode`    | Encoder output mode (0=steps, 1=degrees)                                 | 1       | Polling (`command_processor.py`) |

**Input Registers (Readable by Relay Master)**

| Address | Name              | Description                                           | Scaled | Updated By                  |
|---------|-------------------|-------------------------------------------------------|--------|-----------------------------|
| 0       | `current_rpm`     | Current Motor RPM                                     | x10    | `vfd_status_request_task` |
| 1       | `vfd_status`      | VFD Status Word (P0680)                               | No     | `vfd_status_request_task` |
| 2       | `encoder_steps`   | Encoder Position (Steps, relative to offset)          | No     | `encoder_module.py`       |
| 3       | `encoder_degrees` | Encoder Position (Degrees, relative to offset)        | x100   | `encoder_module.py`       |
| 4       | `fault_flag`      | VFD Fault Detected Flag (0=No, 1=Fault)               | No     | `vfd_status_request_task` |
| 5       | `homing_flag`     | Homing Completed Flag (0=No, 1=Yes)                   | No     | `homing()` in `main.py`   |
| 6       | `offset_steps`    | Calibrated Encoder Offset (Steps)                     | No     | `load_configuration`, Callback |
| 7       | `max_rpm`         | Configured Max RPM from VFD (P0208)                   | No     | `main()` startup          |

---

## Getting Started

1.  **Clone:** `git clone <repository-url>`
2.  **Firmware:** Flash MicroPython (v1.17+) to both Picos.
3.  **Upload Files:**
    *   **Relay Pico:** Upload `modbus_micropython/relay_pico/main.py` and the `modbus_micropython/relay_pico/lib/` directory.
    *   **Main Pico:** Upload all files and directories from `modbus_micropython/main_pico/` (including `main.py`, `*.py`, `config.json`, `lib/`).
4.  **Connect Hardware:** Wire according to **Connections**.
5.  **Power Up.**

---

## Usage

1.  Connect **Relay Pico** to PC via USB.
2.  Open a serial terminal to Relay Pico COM port.
3.  Main Pico boots, performs safety stop, loads config, (attempts homing), starts tasks.
4.  Enter commands in Relay Pico terminal.

### Commands (Sent to Relay Pico)

*(See Modbus Register Map for underlying mechanism)*

-   `start [rpm]`: Start motor forward (Default 1000 RPM). Writes HReg 100=1, HReg 101=rpm.
-   `stop`: Stop motor. Writes HReg 100=2.
-   `reverse [rpm]`: Start motor reverse (Default 1000 RPM). Writes HReg 100=3, HReg 101=rpm.
-   `set_speed [rpm]`: Update target RPM. Writes HReg 100=0, HReg 101=rpm.
-   `reset_fault`: Attempt VFD fault reset. Writes HReg 100=4.
-   `calibrate`: Set current encoder position as zero offset. Writes HReg 100=5.
-   `set_verbose [0-3]`: Set Main Pico local print level. Writes HReg 102=level.
-   `set_encoder_output [step|deg]`: Set encoder reporting mode. Writes HReg 103=(0 or 1).
-   `status`: (Local Relay command) Prints last read status from Main Pico Input Registers.
-   `help`: (Local Relay command) Show command list.

*(Note: `read_speed`, `read_offset`, `read_max_rpm` commands are effectively replaced by observing the periodic `status` output from the Relay Pico).*

---

## Current Status & Debugging Plan (As of 2025-04-29 ~16:55)

**Working:**
-   Relay Pico sends Modbus write commands (START, STOP, REVERSE, settings) to Main Pico.
-   Main Pico receives writes via Modbus Slave interface.
-   Main Pico's command callback (`handle_command_register_write`) correctly triggers for action commands (1, 2, 3, 4, 5).
-   Main Pico correctly extracts target RPM from multi-register writes (e.g., `start 1000`).
-   Main Pico successfully controls the VFD (start/stop/reverse) based on received commands.
-   Main Pico's settings polling (`process_modbus_commands`) appears functional.
-   Main Pico's internal tasks (VFD status polling, encoder reading) update internal state correctly.

**Problem:**
-   **Relay Pico (Master) fails to reliably read Input Registers (status) from Main Pico (Slave) via Modbus Function Code 04.**
-   Symptoms: Relay Pico logs show `[ERROR] Modbus RX Error: no data received from slave` or read stale data (often all zeros), even when Main Pico logs confirm the motor is running and internal state is updated.

**Debugging Roadmap:**

*   **Phase 1: Isolate Main Pico Slave Response (In Progress)**
    *   **Step 1.1:** Drastically Simplify Main Pico Load & Verify Slave Response
        *   **Goal:** Determine if Main Pico can respond when minimally loaded.
        *   **Action:** Disable VFD polling, relay control, encoder task, and homing on Main Pico. Set static initial values for Input Registers in `utils.py`. Add debug prints to `modbus_slave_poll_task`.
        *   **Test:** Deploy modified Main Pico. Run both. Observe if Relay Pico can reliably read the static values.
        *   **Current Sub-step:** Preparing to commit and test these changes.
    *   **Step 1.2:** Explicitly Check Slave Response Construction (If 1.1 Fails)
        *   **Goal:** Verify `umodbus` slave library attempts to build/send FC04 response.
        *   **Action:** Add debug prints inside `umodbus/modbus.py` on Main Pico before UART write for FC04.
*   **Phase 2: Investigate RS485 Link & Timing (If Phase 1 Fails)**
    *   **Step 2.1:** Reduce Baud Rate (e.g., to 19200) on both Picos for Network 1.
    *   **Step 2.2:** Investigate/Increase Modbus Timeouts (if possible in library/UART).
    *   **Step 2.3:** Verify DE/RE Timing (Logs / Oscilloscope).
*   **Phase 3: Check Wiring & Relay Reception (If Phase 2 Fails)**
    *   **Step 3.1:** Physical Check (Wiring, GND).
    *   **Step 3.2:** Explicitly Check Relay UART Reception (Log raw bytes).

---

## Troubleshooting

*(Existing troubleshooting steps remain relevant but add:)*
-   **Relay Pico Status Reads Failing:**
    -   Follow the current debugging roadmap above.
    -   Check for noise on the RS485 lines (Network 1).
    -   Consider potential `uasyncio` scheduling conflicts or blocking code on the Main Pico.
    -   Verify `umodbus` library versions are consistent if manually copied.

---

## Contributing

1.  Fork this repository.
2.  Create a new branch (`git checkout -b feature/YourFeature`).
3.  Commit your changes (`git commit -am 'Add some feature'`).
4.  Push to the branch (`git push origin feature/YourFeature`).
5.  Create a new Pull Request.

---

## License

This project is licensed under the [MIT License](LICENSE). The included `encoder.py` and `umodbus` library may have their own specific licenses (MIT).

---

## Acknowledgments

-   MicroPython development team
-   Peter Hinch for `micropython-async` and encoder library contributions.
-   `umodbus` library developers.
-   WEG for CFW500 documentation.
-   Omron for encoder specifications.