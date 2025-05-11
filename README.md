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
-   **Baud Rate:** 19200 (Configurable, current default in code)
-   **Roles:** Relay Pico (Master), Main Pico (Slave Address 1)
-   **Purpose:** Relay sends commands (by writing to Main Pico's Holding Registers) and requests status (by reading Main Pico's Input Registers).

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
    -   Start, stop, and reverse the motor. RPM can be specified with start/reverse, or the last set target RPM will be used.
    -   Dynamic speed changes for a running motor via the `set_speed` command.
    -   Reading motor speed (RPM) and VFD status/faults.
    -   Resetting VFD faults.
-   **Quadrature Encoder Integration:**
    -   Reads encoder for position in steps/degrees. Supports zero-calibration (`calibrate` command storing offset in `config.json`). Homing routine using Z-signal endstop (to be fully tested).
-   **Command Handling (Main Pico):**
    -   **Action Commands (Start, Stop, Reverse, Reset, Calibrate, Set Speed):** Handled via Modbus register write callback (`handle_command_register_write` in `main_pico/main.py`). Target RPM is taken from the command payload if provided, or from the current `REG_TARGET_RPM` for parameter-less start/reverse.
    -   **Settings Commands (Verbose Level, Encoder Mode):** Handled by polling Holding Registers in the main loop (`process_modbus_commands` in `command_processor.py`).
-   **Status Reporting:** Main Pico periodically updates its Modbus Input Registers with live data (e.g., RPM, VFD Status). Relay Pico periodically reads and displays these values.
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
    -   `uasyncio` (built-in MicroPython library)
    -   `umodbus` (Modbus library, included in `lib/` directories for both Picos)
    -   `encoder.py` (Quadrature encoder driver by Peter Hinch, included in `main_pico/`)
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
*Critical Verification: All GPIO pin numbers listed in the tables above for both Relay Pico and Main Pico (including UARTs, DE/RE control, Encoder signals, Relays, and Endstop) must be meticulously verified against the current constants defined in `relay_pico/main.py` and `main_pico/main.py` to ensure 100% accuracy.*

---

## Modbus Register Map (Network 1: Relay <-> Main)

*(Defined in `main_pico/utils.py`)*

**Holding Registers (Writeable by Relay Master)**

| Address | Name              | Description                                                              | Default | Handled By        |
|---------|-------------------|--------------------------------------------------------------------------|---------|-------------------|
| 100     | `command`         | 1=Start, 2=Stop, 3=Reverse, 4=Reset Fault, 5=Calibrate. Command 0 is used with REG_TARGET_RPM for set_speed functionality. | 0       | Callback (`main.py`) |
| 101     | `target_rpm`      | Target RPM for Start/Reverse/Set_Speed commands. Used by set_speed (cmd 0), or by start/reverse (cmd 1 or 3) if no RPM is explicitly sent with those commands. | 0       | Callback (`main.py`) |
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
| 6       | `offset_steps`    | Calibrated Encoder Offset (Steps)                     | No     | `load_configuration()` from `config.json`, `calibrate` command callback. |
| 7       | `max_rpm`         | Configured Max RPM from VFD (P0208)                   | No     | `main()` startup VFD read, `update_input_registers`. |

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

-   `start [rpm]`: Start motor forward.
    - If `[rpm]` is provided, it sets that as the target. If `[rpm]` is omitted, the Main Pico uses its current `REG_TARGET_RPM` value (see `set_speed`). Writes HReg 100=1. If `[rpm]` is provided, also writes rpm to HReg 101.
-   `stop`: Stop motor. Writes HReg 100=2.
-   `reverse [rpm]`: Start motor reverse.
    - If `[rpm]` is provided, it sets that as the target. If `[rpm]` is omitted, Main Pico uses current `REG_TARGET_RPM`. Writes HReg 100=3. If `[rpm]` is provided, also writes rpm to HReg 101.
-   `set_speed [rpm]`: Updates the motor's target speed to `[rpm]`. If the motor is running, its speed will change. If stopped, this sets the speed for the next parameter-less start/reverse command. Writes HReg 100=0 and rpm to HReg 101.
-   `reset_fault`: Attempt VFD fault reset. Writes HReg 100=4.
-   `calibrate`: Set current encoder position as zero offset. Writes HReg 100=5.
-   `set_verbose [0-3]`: Set Main Pico local print level. Writes HReg 102=level.
-   `set_encoder_output [step|deg]`: Set encoder reporting mode. Writes HReg 103=(0 or 1).
-   `status`: (Local Relay command) Prints last read status from Main Pico Input Registers.
-   `help`: (Local Relay command) Show command list.

## Troubleshooting (General)

-   **Verify Wiring:** Double-check all RS485 (A/B lines, DE/RE control signals), encoder (if enabled), and power connections. Ensure all devices share a common ground.
-   **Check Baud Rates & Serial Parameters:**
    -   Network 1 (Relay Pico <-> Main Pico): Ensure both Picos are configured for the same RS485 baud rate (e.g., 19200, 8-N-1) in their respective code (`relay_pico/main.py` and `main_pico/main.py`).
    -   Network 2 (Main Pico <-> VFD): Ensure the Main Pico's VFD communication settings match the VFD's Modbus parameters (baud rate, slave ID, data bits, parity, stop bits).
-   **Modbus Slave Addresses:** Confirm the Main Pico slave address (Network 1) and VFD slave address (Network 2) in the code match hardware/VFD configurations.
-   **Host PC Serial Terminal:** Ensure your serial terminal application (e.g., Thonny REPL, PuTTY) is connected to the Relay Pico's USB serial COM port and configured with the correct baud rate for USB serial communication (typically 115200).
-   **Main Pico Logs:** Observe the Main Pico's USB serial output (via Thonny REPL or another terminal connected to Main Pico's USB). If `VERBOSE_LEVEL` (HReg 102) is > 0, it will print status and error messages. This is invaluable for diagnosing issues on the Main Pico.
-   **Relay Pico Logs:** Check the Relay Pico's USB serial output for Modbus communication errors (e.g., "no data received from slave," "invalid CRC") or status messages.
-   **RS485 Transceiver DE/RE Logic:** Ensure the DE/RE pins on the RS485 converters are correctly wired to the Pico GPIOs specified in the "Connections" section and that the `ctrl_pin` is correctly assigned in the `ModbusRTUMaster` / `ModbusRTUSerial` initializations.
-   **Power Supplies:** Verify adequate and stable power for both Picos, the VFD, and the encoder.

---
*(Note: `read_speed`, `read_offset`, `read_max_rpm` commands are effectively replaced by observing the periodic `status` output from the Relay Pico).*

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