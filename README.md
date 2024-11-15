# Raspberry Pi Pico VFD Motor Control with Quadrature Encoder

This repository provides a robust and modular solution to control a Variable Frequency Drive (VFD) using Modbus RTU communication and read encoder positions with a quadrature encoder. The project is designed with best practices and features asynchronous operations for seamless performance.

---

## Features

- **Motor Control via VFD (CFW500):**
  - Start, stop, and reverse the motor with precise RPM control.
  - Read motor speed (RPM) and status.
  - Reset VFD faults.
  
- **Quadrature Encoder Integration:**
  - Read encoder positions in steps or degrees.
  - Support for zero-calibration to set custom reference positions.
  - Handle "zero position" using the encoder’s Z signal.

- **Configurable Verbose Levels:**
  - Control console output:
    - **0**: No output.
    - **1**: Encoder position only.
    - **2**: Motor actions and VFD status.
    - **3**: Full details (encoder and motor/VFD statuses).

- **Async Event Handling:**
  - Non-blocking encoder reading and motor control using `uasyncio`.

- **Modular Structure:**
  - `cfw500_modbus.py`: VFD-related functions.
  - `encoder.py`: Encoder handling.
  - `main.py`: Main application logic.

---

## Requirements

### Hardware

- Raspberry Pi Pico
- Omron E6B2-CWZ6C Quadrature Encoder (2000 PPR)
- WEG CFW500 VFD with Modbus RTU
- RS485 to UART converter
- Motor compatible with VFD

### Connections

| Encoder Pin       | Pico Pin     |
|-------------------|--------------|
| A (Black)         | GPIO16       |
| B (White)         | GPIO17       |
| Z (Orange)        | GPIO18       |
| +5V (Brown)       | VBUS (5V)    |
| GND (Blue)        | GND          |

### Software

- Thonny IDE
- MicroPython firmware on Pico
- `uasyncio` library for async operations

---

## Getting Started

### Installation

1. **Clone the Repository:**
   ```bash
   git clone https://github.com/<your-username>/<your-repo-name>.git
   cd <your-repo-name>
   ```

2. **Prepare the Raspberry Pi Pico:**
   - Flash the MicroPython firmware onto the Pico.
   - Ensure `uasyncio` is included (standard in MicroPython).

3. **Upload Files:**
   - Use Thonny IDE to upload `main.py`, `cfw500_modbus.py`, and `encoder.py` to the Pico.

---

## Usage

### Commands

#### Motor Control
- `start [rpm]`: Start motor at the specified RPM (default: 1000 RPM).
- `stop`: Stop the motor.
- `reverse [rpm]`: Reverse motor at the specified RPM.
- `set_speed [rpm]`: Update motor speed reference.
- `reset_fault`: Reset VFD faults.

#### Encoder
- `set_encoder_output [step|deg]`: Set encoder output to steps or degrees.
- `calibrate`: Set current encoder position as zero.

#### Verbose Control
- `set_verbose [0-3]`: Adjust verbosity level.

#### Miscellaneous
- `help`: Show command list.
- `exit`: Exit the program.
- `test`: Run a test sequence.

### Examples

1. **Start motor at 1200 RPM:**
   ```bash
   start 1200
   ```

2. **Set encoder output to degrees:**
   ```bash
   set_encoder_output deg
   ```

3. **Calibrate the encoder:**
   ```bash
   calibrate
   ```

4. **Stop the motor:**
   ```bash
   stop
   ```

---

## Code Overview

- **`main.py`**:
  - Main logic with asynchronous loops for encoder and motor control.
  - Command processing and zero-endstop integration.

- **`cfw500_modbus.py`**:
  - Modbus communication for VFD operations.
  - Functions for reading/writing parameters and fault handling.

- **`encoder.py`**:
  - Encoder handling with real-time position updates.
  - A/B/Z signal processing and calibration support.

---

## Troubleshooting

1. **Motor does not start:**
   - Check VFD connections.
   - Ensure Modbus parameters match `cfw500_modbus.py`.
   - Verify VFD power and fault status.

2. **Encoder position not updating:**
   - Verify wiring for A, B, Z signals (GPIO16, GPIO17, GPIO18).
   - Ensure encoder is powered and functional.

3. **Verbose output issues:**
   - Adjust verbosity using `set_verbose [0-3]`.

---

## Contributing

1. Fork this repository.
2. Create a new branch for your changes.
3. Submit a pull request with a description of your updates.

---

## License

This project is licensed under the [MIT License](LICENSE).

---

## Acknowledgments

- MicroPython development team
- Peter Hinch for encoder library contributions
- WEG for CFW500 documentation
- Omron for encoder specifications
- All contributors and testers