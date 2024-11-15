Here is the finalized and corrected **`README.md`** file, thoroughly reviewed for accuracy and completeness:

```markdown
# Raspberry Pi Pico VFD Motor Control with Quadrature Encoder

Welcome to the **Raspberry Pi Pico VFD Motor Control with Quadrature Encoder** project! This repository provides a robust and modular solution to control a Variable Frequency Drive (VFD) using Modbus RTU communication and read encoder positions using a quadrature encoder. The code is structured with best practices and features asynchronous operations for seamless performance.

---

## Features

1. **Motor Control via VFD (CFW500):**
   - Start, stop, and reverse the motor with precise RPM control.
   - Read current motor speed (RPM) and status.
   - Reset VFD faults.

2. **Quadrature Encoder Integration:**
   - Read encoder positions in steps or degrees.
   - Support for zero-calibration to set custom reference positions.
   - Handle "zero position" using the encoder's Z (endstop) signal.

3. **Configurable Verbose Levels:**
   - Control the amount of information displayed in the console:
     - **0**: No output.
     - **1**: Encoder position only.
     - **2**: Motor actions and VFD status/information.
     - **3**: Full details of both encoder and motor/VFD.

4. **Async Event Handling:**
   - Non-blocking encoder reading and motor control via `uasyncio`.

5. **Modular Structure:**
   - Separation of concerns:
     - `cfw500_modbus.py` for VFD-related functions.
     - `encoder.py` for encoder handling.
     - `main.py` for main application logic.

---

## Requirements

### Hardware

- [Raspberry Pi Pico](https://www.raspberrypi.com/products/raspberry-pi-pico/)
- Omron E6B2-CWZ6C Quadrature Encoder (2000 PPR)
- WEG CFW500 VFD with Modbus RTU communication
- RS485 to UART converter module
- Motor compatible with the VFD

### Connections

| Encoder Pin       | Raspberry Pi Pico Pin |
|-------------------|-----------------------|
| **A (Black)**     | GPIO16                |
| **B (White)**     | GPIO17                |
| **Z (Orange)**    | GPIO18                |
| **+5V (Brown)**   | VBUS (5V)             |
| **GND (Blue)**    | GND                   |

### Software

- [Thonny IDE](https://thonny.org/)
- MicroPython firmware installed on the Raspberry Pi Pico
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
   - Ensure `uasyncio` is available (it's included in standard MicroPython firmware).

3. **Upload Files:**
   Upload `main.py`, `cfw500_modbus.py`, and `encoder.py` to the Pico using Thonny IDE or your preferred method.

---

## Usage

### Connect the Hardware

Wire the encoder and VFD to the Pico according to the connections table above.

### Run the Program

Execute `main.py` on the Pico via Thonny IDE.

### Available Commands

#### Motor Commands:
- `start [rpm]` - Start the motor at the specified RPM (default: 1000 RPM).
- `stop` - Stop the motor.
- `reverse [rpm]` - Reverse the motor at the specified RPM.
- `set_speed [rpm]` - Update the motor's speed reference.
- `reset_fault` - Reset any VFD faults.

#### Encoder Commands:
- `set_encoder_output [step|deg]` - Set encoder output to steps or degrees.
- `calibrate` - Set the current encoder position as the zero reference.

#### Verbose Control:
- `set_verbose [0-3]` - Adjust verbosity levels.

#### Miscellaneous:
- `help` - Display the instruction manual.
- `exit` - Exit the program.
- `test` - Execute the default test sequence.

---

## Examples

### Start the motor at 1200 RPM:
```bash
start 1200
```

### Set encoder output to degrees:
```bash
set_encoder_output deg
```

### Calibrate the encoder to zero:
```bash
calibrate
```

### Stop the motor:
```bash
stop
```

---

## Code Structure

1. **`main.py`**  
   Handles the main logic of the application:
   - Asynchronous loops for encoder reading and motor control.
   - Command processing from the user.
   - Integration of the zero endstop detection.

2. **`cfw500_modbus.py`**  
   Contains all the VFD-related functions:
   - Modbus communication setup and handling.
   - Reading and writing VFD parameters.
   - Error handling and fault resetting.

3. **`encoder.py`**  
   Driver for the quadrature encoder:
   - Handles A, B, and Z signals.
   - Provides real-time position updates and callbacks.
   - Supports calibration and output formatting.

---

## Verbose Levels

| Level | Description                                                  |
|-------|--------------------------------------------------------------|
| 0     | No output.                                                   |
| 1     | Encoder position updates only.                               |
| 2     | Motor actions and all Modbus/VFD-related statuses.           |
| 3     | Full details: both encoder and motor/VFD statuses are shown. |

---

## Troubleshooting

### Motor does not start:
- Check VFD connections and ensure Modbus parameters match those in the `cfw500_modbus.py` file.
- Verify that the VFD is powered on and not in a fault state.

### Encoder position not updating:
- Verify encoder wiring and connections to GPIO16, GPIO17, and GPIO18.
- Ensure that the encoder is receiving power and functioning correctly.

### Verbose output too noisy or silent:
- Use `set_verbose [level]` to adjust verbosity levels as needed.

---

## Contributing

Contributions are welcome! Please follow these steps:

1. Fork this repository.
2. Create a new branch for your feature or bug fix.
3. Submit a pull request describing your changes.

---

## License

This project is licensed under the MIT License. See the `LICENSE` file for more details.

---

## Acknowledgments

- **MicroPython development team** for the MicroPython firmware.
- **Peter Hinch** for the encoder library.
- **WEG** for providing detailed documentation for the CFW500 VFD.
- **Omron** for the encoder specifications.
- Community contributors and testers.
```
