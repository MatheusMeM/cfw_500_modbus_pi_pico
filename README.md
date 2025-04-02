# Raspberry Pi Pico VFD Motor Control with Relay and Encoder

This project implements a system for controlling a Variable Frequency Drive (VFD) and motor using two Raspberry Pi Pico boards, communicating with a host PC, and reading a quadrature encoder for position feedback. One Pico acts as the main controller (`main_pico`), directly interfacing with the VFD (WEG CFW500) via Modbus RTU and the encoder. The second Pico (`relay_pico`) acts as a communication bridge between the host PC (USB Serial) and the main controller (RS485).

---

## System Architecture

```
+------+   USB Serial   +------------+   RS485 (115200 baud)   +-----------+   Modbus RTU (RS485, 19200 baud)      +---------+       +-------+
|  PC  | <------------> | Relay Pico | <---------------------> | Main Pico | <-----------------------------------> | CFW500  | ----> | Motor |
+------+                +------------+                         +-----------+                                       +---------+       +-------+
                                                                    |
                                                                    | Encoder Signals (A, B, Z)
                                                                    v
                                                              +-----------+
                                                              |  Encoder  |
                                                              +-----------+
                                                                    | Z Signal (Endstop)
                                                                    v
                                                                 GPIO Pin
```

---

## Features

-   **Two-Pico Design:** Separates PC communication (Relay Pico) from real-time control (Main Pico).
-   **Motor Control via VFD (CFW500):**
    -   Start, stop, and reverse the motor with precise RPM control via Modbus RTU.
    -   Read motor speed (RPM) and VFD status/faults.
    -   Reset VFD faults.
-   **Quadrature Encoder Integration:**
    -   Read encoder positions in steps or degrees using hardware interrupts and `uasyncio`. Encoder direction is configured so positive values indicate forward rotation.
    -   Support for zero-calibration (`calibrate` command) which stores the offset in steps (`encoder_offset_steps`).
    -   Simplified Homing Routine: Uses the encoder's Z signal connected as an endstop. Moves towards the endstop, attempts to stop reliably upon trigger using an `asyncio.Event` and a stop command loop, and sets the zero reference point (`encoder_zero_offset`).
-   **Communication:**
    -   PC <-> Relay Pico: Standard USB Serial.
    -   Relay Pico <-> Main Pico: Robust RS485 communication.
    -   Main Pico <-> VFD: Modbus RTU over RS485.
-   **Refined Verbose Levels (0-3):**
    -   Controls detail of console/serial output.
    -   Critical messages (prefixed with `[SAFETY]`, `[WARNING]`, `[ERROR]`, `[ALERT]`) are always displayed, regardless of level.
    -   Direct command confirmations (e.g., `[ACTION] Starting...`, `[ACTION] Stopping...`) are always displayed.
    -   Level 0: Only critical messages and command confirmations.
    -   Level 1: Adds rate-limited encoder position updates.
    -   Level 2: Adds non-critical motor/VFD action info, status reads, etc.
    -   Level 3: Adds detailed debug messages.
-   **Configuration Persistence:** Saves `encoder_offset_steps`, `encoder_output_mode`, and `VERBOSE_LEVEL` to `config.json` on the Main Pico.
-   **Async Event Handling & Performance:**
    -   Uses `uasyncio` for non-blocking operations.
    -   Homing uses `asyncio.Event` for improved responsiveness to the endstop trigger.
    -   Background tasks (status/relay checks) are temporarily disabled during homing to improve reliability and reduce scheduler load.
    -   LED blink task removed. Onboard LED is now permanently on.
    -   Command reading loop checks more frequently.
    -   `print_verbose` function optimized slightly to reduce potential blocking.
-   **Modular Structure:** Code is organized into logical modules.

---

## Requirements

### Hardware

-   **Host PC:** To send commands and receive status.
-   **Raspberry Pi Pico (x2):** One for the Relay role, one for the Main Controller role.
-   **Omron E6B2-CWZ6C Quadrature Encoder (or similar):** Connected to Main Pico.
-   **WEG CFW500 VFD (or compatible):** With Modbus RTU interface.
-   **RS485 to UART Converters (x2):**
    -   One for Relay Pico <-> Main Pico link.
    -   One for Main Pico <-> VFD link.
    *Note: Ensure converters have DE/RE control pins if using the provided code.*
-   **Motor:** Compatible with the VFD.
-   **Power Supplies:** For Picos, VFD, Encoder.
-   **Wiring:** Appropriate cables for USB, RS485, Encoder, Motor.

### Software

-   **MicroPython Firmware:** Flashed onto both Picos (v1.17 or later recommended for `uasyncio` and `umodbus`).
-   **Required Libraries (on Main Pico):**
    -   `uasyncio` (usually built-in)
    -   `umodbus` (included in `main_pico/lib/`)
-   **Host PC Software:** Serial terminal emulator (e.g., PuTTY, Tera Term, Thonny's REPL) to communicate with the Relay Pico.
-   **Thonny IDE (Recommended):** For flashing firmware and uploading code.

---

## Connections

### Relay Pico

| Function         | Relay Pico Pin | Connection                             |
|------------------|----------------|----------------------------------------|
| USB Serial       | Micro USB Port | Host PC                                |
| RS485 TX (UART0) | GPIO0 (Pin 1)  | RS485 Converter (Main Pico Link) - TXD |
| RS485 RX (UART0) | GPIO1 (Pin 2)  | RS485 Converter (Main Pico Link) - RXD |
| RS485 DE/RE      | GPIO2 (Pin 4)  | RS485 Converter (Main Pico Link) - DE/RE |
| Power (+5V)      | VBUS / VSYS    | 5V Power Supply                        |
| Ground           | GND            | Common Ground                          |

### Main Pico

| Function            | Main Pico Pin  | Connection                          |
|---------------------|----------------|-------------------------------------|
| RS485 TX (UART1)    | GPIO4 (Pin 6)  | RS485 Converter (Relay Link) - TXD  |
| RS485 RX (UART1)    | GPIO5 (Pin 7)  | RS485 Converter (Relay Link) - RXD  |
| RS485 DE/RE (UART1) | GPIO6 (Pin 9)  | RS485 Converter (Relay Link) - DE/RE|
| Modbus TX (UART0)   | GPIO0 (Pin 1)  | RS485 Converter (VFD Link) - TXD    |
| Modbus RX (UART0)   | GPIO1 (Pin 2)  | RS485 Converter (VFD Link) - RXD    |
| Modbus DE/RE (UART0)| GPIO2 (Pin 4)  | RS485 Converter (VFD Link) - DE/RE  |
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

## Getting Started

### Installation

1.  **Clone the Repository:**
    ```bash
    git clone <repository-url>
    cd <repository-directory>
    ```
2.  **Prepare Picos:**
    -   Flash MicroPython firmware onto both Picos using Thonny or `rp2-boot`.
3.  **Upload Files:**
    -   **Relay Pico:** Upload `modbus_micropython/relay_pico/main.py`.
    -   **Main Pico:** Upload all files and directories from `modbus_micropython/main_pico/` (including `main.py`, `*.py` files, `config.json`, and the `lib/` directory) to the root directory of the Pico. Ensure `config.json` exists and contains desired initial settings (or defaults will be used).
4.  **Connect Hardware:** Wire the components according to the **Connections** section.
5.  **Power Up:** Apply power to the Picos, VFD, and Encoder.

---

## Usage

1.  Connect the **Relay Pico** to the Host PC via USB.
2.  Open a serial terminal connection to the Relay Pico's COM port (Baud rate doesn't strictly matter for USB CDC).
3.  The **Main Pico** should automatically start, display the manual, perform the safety stop (with delay), load settings, perform the homing routine (move until endstop, stop, set zero), restart background tasks, and then wait for commands.
4.  Enter commands in the serial terminal connected to the Relay Pico.

### Commands (Sent to Relay Pico)

#### Motor Control
-   `start [rpm]`: Start motor forward at specified RPM (default: 1000).
-   `stop`: Stop the motor.
-   `reverse [rpm]`: Start motor reverse at specified RPM (default: 1000).
-   `set_speed [rpm]`: Update motor speed reference while running.
-   `reset_fault`: Attempt to reset VFD faults.

#### VFD/System Status
-   `read_speed`: Read and display the current motor speed from VFD.
-   `status`: Read and display the VFD status word (P0680). Shows detailed bits if verbose=3.
-   `read_max_rpm`: Read the configured maximum RPM from the VFD (P0208).

#### Encoder
-   `set_encoder_output [step|deg]`: Set encoder output format (steps or degrees). Saved to config.
-   `calibrate`: Set current encoder position as the zero reference point (updates `encoder_offset_steps`). Saves offset to config.
-   `read_offset`: Display the currently saved encoder offset in steps (`encoder_offset_steps`).

#### Configuration & Debugging
-   `set_verbose [0-3]`: Adjust verbosity level (0: Critical/Cmd Confirm Only, 1: +Encoder, 2: +Motor/VFD Info, 3: +Debug). Saved to config.
-   `help`: Show command list.
-   `exit`: Stop the program on the Main Pico.
-   `test`: Run a simple test sequence (start, wait, stop).

### Examples

1.  **Start motor at 1200 RPM:** `start 1200`
2.  **Set encoder output to degrees:** `set_encoder_output deg`
3.  **Calibrate the encoder:** `calibrate`
4.  **Check VFD status:** `status`
5.  **Stop the motor:** `stop`

---

## Code Overview

-   **`relay_pico/main.py`**:
    -   Runs on the Relay Pico.
    -   Acts as a simple USB Serial <-> RS485 bridge.
-   **`main_pico/`**: (Runs on the Main Pico)
    -   **`main.py`**: Main application logic using `uasyncio`. Initializes modules, displays manual, performs safety stop, loads config, performs simplified homing (Phase 1 only), manages background tasks (cancels/restarts around homing), starts command reading loop. Onboard LED is turned on.
    -   **`utils.py`**: Defines shared `state` dictionary (including `encoder_offset_steps`, `encoder_print_counter`), initializes UART1 (RS485 to Relay Pico), provides `print_verbose` (handles verbosity levels, critical messages, rate-limiting), `show_manual`, `save_configuration`, `load_configuration`.
    -   **`commands.py`**: Parses and executes commands received from UART1. Interacts with `cfw500_modbus` object and `state`. Ensures command confirmations are always printed (`override=True`). Uses `encoder_offset_steps`.
    -   **`cfw500_modbus.py`**: Class implementing Modbus RTU communication with the CFW500 VFD via UART0.
    -   **`motor_control.py`**: Simple helper to initialize the `CFW500Modbus` instance.
    -   **`encoder_module.py`**: Initializes the encoder driver (`encoder.py`) and defines the `encoder_callback` function to process position updates (inverting value, applying `encoder_offset_steps`), handle degree wrapping, format output, rate-limit printing based on counter and verbosity level, and update `state`.
    -   **`encoder.py`**: Low-level asynchronous quadrature encoder driver using pin interrupts.
    -   **`config.json`**: Stores persistent configuration (`encoder_offset_steps`, `encoder_output_mode`, `VERBOSE_LEVEL`).
    -   **`lib/umodbus/`**: MicroPython Modbus library.

---

## Troubleshooting

1.  **No Communication with Main Pico:**
    -   Check RS485 wiring between Relay and Main Picos.
    -   Verify both Picos are running their respective `main.py`.
    -   Ensure correct UART pins and baud rate (115200) for the RS485 link.
    -   Check DE/RE pin connections.
2.  **Motor does not start / No VFD Communication:**
    -   Check RS485 wiring between Main Pico (UART0) and VFD.
    -   Verify VFD Modbus parameters (Slave Address=1, Baud=19200, 8N1).
    -   Ensure VFD is powered and not in fault. Use `status` command.
    -   Check Main Pico's UART0 pins (GPIO0, 1) and DE/RE pin (GPIO2).
3.  **Encoder position not updating / Incorrect:**
    -   Verify wiring for A, B signals (GPIO16, 17 on Main Pico).
    -   Ensure encoder has power and common ground.
    -   Check if Z signal/Endstop (GPIO18) is correctly wired and triggering during homing.
    -   Use `calibrate` command after homing or manual positioning. Use `read_offset` to check the value (in steps).
4.  **Configuration Not Saving/Loading:**
    -   Ensure `main_pico/config.json` exists and is valid JSON. Check MicroPython filesystem access.
5.  **Homing Fails to Stop Motor:**
    -   Verify endstop wiring (GPIO18) and mechanical trigger.
    -   Check VFD deceleration ramp time (P0101 and P0100). Shorter ramps (e.g., 1s) improve homing stop accuracy.
    -   Ensure `asyncio` loop isn't excessively blocked. The temporary disabling of background tasks during homing aims to mitigate this. Check if reducing encoder print rate further helps.
6.  **Commands Unresponsive:**
    -   Check RS485 link between Picos.
    -   Suspect potential blocking in `asyncio` tasks on Main Pico (e.g., long Modbus timeouts, excessive printing). Reducing `print_verbose` frequency or temporarily disabling tasks like `status_request_task` can help diagnose.

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