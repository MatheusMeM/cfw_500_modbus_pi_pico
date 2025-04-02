# Raspberry Pi Pico VFD Motor Control with Relay and Encoder

This project implements a system for controlling a Variable Frequency Drive (VFD) and motor using two Raspberry Pi Pico boards, communicating with a host PC, and reading a quadrature encoder for position feedback. One Pico acts as the main controller (`main_pico`), directly interfacing with the VFD (WEG CFW500) via Modbus RTU and the encoder. The second Pico (`relay_pico`) acts as a communication bridge between the host PC (USB Serial) and the main controller (RS485).

---

## System Architecture

```
+------+   USB Serial   +------------+   RS485 (115200 baud)   +-----------+   Modbus RTU (RS485, 19200 baud)   +---------+       +-------+
|  PC  | <------------> | Relay Pico | <---------------------> | Main Pico | <-----------------------------------> | CFW500  | ----> | Motor |
+------+                +------------+                         +-----------+                                     +---------+       +-------+
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
    -   Read encoder positions in steps or degrees using hardware interrupts and `uasyncio`.
    -   Support for zero-calibration (`calibrate` command).
    -   Homing routine using the encoder's Z signal connected as an endstop.
-   **Communication:**
    -   PC <-> Relay Pico: Standard USB Serial.
    -   Relay Pico <-> Main Pico: Robust RS485 communication.
    -   Main Pico <-> VFD: Modbus RTU over RS485.
-   **Configurable Verbose Levels:** Control console/serial output detail (0-3).
-   **Configuration Persistence:** Saves encoder offset, output mode, and verbosity to `config.json` on the Main Pico.
-   **Async Event Handling:** Non-blocking operations using `uasyncio` on the Main Pico.
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
    -   **Main Pico:** Upload all files and directories from `modbus_micropython/main_pico/` (including `main.py`, `*.py` files, `config.json`, and the `lib/` directory) to the root directory of the Pico.
4.  **Connect Hardware:** Wire the components according to the **Connections** section.
5.  **Power Up:** Apply power to the Picos, VFD, and Encoder.

---

## Usage

1.  Connect the **Relay Pico** to the Host PC via USB.
2.  Open a serial terminal connection to the Relay Pico's COM port (Baud rate doesn't strictly matter for USB CDC, but the Pico code uses 115200 for RS485).
3.  The **Main Pico** should automatically start, perform homing (if configured), load settings, and wait for commands. The instruction manual should be displayed via the Relay Pico.
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
-   `calibrate`: Set current encoder position as the zero reference point. Saves offset to config.
-   `read_offset`: Display the currently saved encoder offset.

#### Configuration & Debugging
-   `set_verbose [0-3]`: Adjust verbosity level (0: None, 1: Encoder, 2: Motor/VFD, 3: All). Saved to config.
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
    -   Listens on USB (`sys.stdin`), forwards to RS485 (UART0).
    -   Listens on RS485 (UART0), forwards to USB (`print`).
    -   Uses GPIO2 for RS485 DE/RE control.

-   **`main_pico/`**: (Runs on the Main Pico)
    -   **`main.py`**: Main application logic using `uasyncio`. Initializes modules, sets up async tasks (command reading, VFD status polling, encoder handling, LED blink, relay control), performs homing routine.
    -   **`utils.py`**: Defines shared `state` dictionary, initializes UART1 (RS485 to Relay Pico), provides `print_verbose` (sends output to USB and UART1), `show_manual`, `save_configuration`, `load_configuration`.
    -   **`commands.py`**: Parses and executes commands received from UART1 (via `utils.py`). Interacts with `cfw500_modbus` object and `state`.
    -   **`cfw500_modbus.py`**: Class implementing Modbus RTU communication with the CFW500 VFD via UART0. Handles reading/writing registers for control and status. Uses `umodbus` library.
    -   **`motor_control.py`**: Simple helper to initialize the `CFW500Modbus` instance.
    -   **`encoder_module.py`**: Initializes the encoder driver (`encoder.py`) and defines the `encoder_callback` function to process position updates, apply offsets, handle wrapping, format output, and update `state`.
    -   **`encoder.py`**: Low-level asynchronous quadrature encoder driver (based on Peter Hinch's library) using pin interrupts.
    -   **`config.json`**: Stores persistent configuration (encoder offset, output mode, verbosity). Created/updated by `utils.py`.
    -   **`lib/umodbus/`**: MicroPython Modbus library used for VFD communication.

---

## Troubleshooting

1.  **No Communication with Main Pico:**
    -   Check RS485 wiring between Relay and Main Picos (TX->RX, RX->TX, A->A, B->B, GND->GND).
    -   Verify Relay Pico's `relay_pico/main.py` is running.
    -   Verify Main Pico's `main_pico/main.py` is running.
    -   Ensure correct UART pins and baud rate (115200) are used on both Picos for the RS485 link.
    -   Check DE/RE pin connections and logic.
2.  **Motor does not start / No VFD Communication:**
    -   Check RS485 wiring between Main Pico (UART0) and VFD.
    -   Verify VFD Modbus parameters (Slave Address=1, Baud=19200, 8N1).
    -   Ensure VFD is powered and not in fault (check VFD display). Use `status` command.
    -   Check Main Pico's UART0 pins (GPIO0, 1) and DE/RE pin (GPIO2).
3.  **Encoder position not updating / Incorrect:**
    -   Verify wiring for A, B signals (GPIO16, 17 on Main Pico).
    -   Ensure encoder has power and common ground.
    -   Check if Z signal/Endstop (GPIO18) is correctly wired and triggering during homing.
    -   Use `calibrate` command after homing or manual positioning.
4.  **Configuration Not Saving/Loading:**
    -   Ensure `main_pico/config.json` can be written/read by the MicroPython filesystem on the Main Pico.

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