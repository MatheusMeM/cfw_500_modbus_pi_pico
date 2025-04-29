# Modbus Refactoring Testing Plan

This plan outlines the steps to test the refactored system where communication between the Relay Pico and Main Pico now uses Modbus RTU.

**Phase A: Preparation**

1.  **Backup:** Ensure you have a working backup of the *previous* version before deploying the new code.
2.  **Deploy Code:**
    *   **Relay Pico:** Upload `modbus_micropython/relay_pico/main.py`. Upload the `modbus_micropython/relay_pico/lib/` directory (containing `umodbus`).
    *   **Main Pico:** Upload `main.py`, `utils.py`, `command_processor.py`, `encoder_module.py`, `cfw500_modbus.py`, `encoder.py`. Ensure the `lib/` directory and `config.json` are present. Ensure old `commands.py` and `motor_control.py` are deleted.
3.  **Hardware Connections:** Double-check all wiring according to the README:
    *   PC <-> Relay Pico (USB)
    *   Relay Pico UART0 (GPIO 0,1,2) <-> RS485 Converter <-> Main Pico UART1 (GPIO 4,5,6) - **Network 1**
    *   Main Pico UART0 (GPIO 0,1,2) <-> RS485 Converter <-> VFD Modbus Port - **Network 2**
    *   Main Pico Encoder (GPIO 16, 17, 18)
    *   Main Pico Relays (GPIO 20, 21)
    *   Power supplies and **common ground** for all components.
4.  **Serial Terminals:**
    *   Open a serial terminal connected to the **Relay Pico** (e.g., Thonny, PuTTY). This is your command/status interface.
    *   Open a *separate* serial terminal connected to the **Main Pico** (e.g., another Thonny instance, PuTTY). This is for observing local debug/verbose messages from the Main Pico.

**Phase B: Initial Checks**

5.  **Power Up:** Power on both Picos and the VFD.
6.  **Initialization Logs:**
    *   **Main Pico Terminal:** Observe the boot sequence. Look for "[INFO] Modbus Slave ... initialized", "[INFO] Modbus Master ... initialized", "[INFO] Encoder initialized", safety stop messages, configuration loading messages, and homing sequence logs. Check for any immediate errors.
    *   **Relay Pico Terminal:** Observe its boot message: "[INFO] Relay Pico initialized as Modbus Master...".
7.  **Homing:**
    *   **Main Pico Terminal:** Observe the homing sequence logs. Verify it attempts to start the motor, waits for the endstop event (or times out), attempts to stop the motor, and reports completion status.
    *   **Relay Pico Terminal:** After homing should complete on the Main Pico, check the periodic status messages. Verify the `Homing:` flag becomes `True`.

**Phase C: Network 1 Testing (Relay <-> Main)**

8.  **Settings Commands (via Relay Terminal):**
    *   `set_verbose 3`: Send this command.
        *   *Expected (Relay):* `[PC CMD RX] set_verbose 3`, `[MODBUS TX] Writing Verbosity Level=3`.
        *   *Expected (Main):* `[INFO] Verbosity level updated to 3 via Modbus.` (and subsequent logs should be more detailed).
    *   `set_encoder_output step`: Send this command.
        *   *Expected (Relay):* `[PC CMD RX] set_encoder_output step`, `[MODBUS TX] Writing Encoder Mode=0 (step)`.
        *   *Expected (Main):* `[INFO] Encoder output mode updated to 'step' via Modbus.`
    *   `set_encoder_output deg`: Send this command.
        *   *Expected (Relay):* `[PC CMD RX] set_encoder_output deg`, `[MODBUS TX] Writing Encoder Mode=1 (deg)`.
        *   *Expected (Main):* `[INFO] Encoder output mode updated to 'deg' via Modbus.`
    *   `set_verbose 1`: Set verbosity back if desired.
9.  **Status Reading (via Relay Terminal):**
    *   Observe the periodic status messages (`[STATUS RX] ...`).
    *   Manually rotate the motor/encoder. Verify `EncSteps` and `EncDeg` values change appropriately based on the selected mode.
    *   Verify `Fault:` flag is `False` (assuming no VFD fault).
    *   Verify `Homing:` flag is `True`.
    *   Verify `Offset:` shows the calibrated offset (should be 0 initially after homing).
10. **Calibration Command (via Relay Terminal):**
    *   Manually move the encoder to a desired zero position.
    *   Send `calibrate`.
        *   *Expected (Relay):* `[PC CMD RX] calibrate`, `[MODBUS TX] Writing CALIBRATE command`.
        *   *Expected (Main):* `[ACTION] Encoder CALIBRATE command processed. New offset: <value>`.
    *   Observe subsequent `[STATUS RX]` messages on the Relay terminal. Verify the `Offset:` value matches the value logged on the Main Pico, and that `EncSteps`/`EncDeg` now reflect the position relative to this new zero. Check `config.json` on the Main Pico to see if the offset was saved.

**Phase D: Network 2 Testing (Main <-> VFD) & Motor Control**

11. **Motor Commands (via Relay Terminal):**
    *   `start 500`: Send command.
        *   *Expected (Relay):* `[PC CMD RX] start 500`, `[MODBUS TX] Writing START command and RPM=500`.
        *   *Expected (Main):* `[ACTION] Motor START command processed (RPM: 500).`
        *   *Expected (Physical):* Motor starts and runs at approx 500 RPM.
        *   *Expected (Relay - Status):* `RPM:` value should approach 500.0. `VFD:` status word should reflect running state.
    *   `set_speed 1000`: Send command while running.
        *   *Expected (Relay):* `[PC CMD RX] set_speed 1000`, `[MODBUS TX] Writing Target RPM=1000`.
        *   *Expected (Main):* `[ACTION] Speed reference updated to 1000 RPM via Modbus.`
        *   *Expected (Physical):* Motor speed increases to approx 1000 RPM.
        *   *Expected (Relay - Status):* `RPM:` value should approach 1000.0.
    *   `reverse 300`: Send command.
        *   *Expected (Relay):* `[PC CMD RX] reverse 300`, `[MODBUS TX] Writing REVERSE command and RPM=300`.
        *   *Expected (Main):* `[ACTION] Motor REVERSE command processed (RPM: 300).`
        *   *Expected (Physical):* Motor stops, then runs in reverse at approx 300 RPM.
        *   *Expected (Relay - Status):* `RPM:` value should approach -300.0 (or positive 300 depending on how `read_current_speed` handles direction). `VFD:` status should reflect reverse run.
    *   `stop`: Send command.
        *   *Expected (Relay):* `[PC CMD RX] stop`, `[MODBUS TX] Writing STOP command`.
        *   *Expected (Main):* `[ACTION] Motor STOP command processed.`
        *   *Expected (Physical):* Motor stops.
        *   *Expected (Relay - Status):* `RPM:` value should approach 0.0.
12. **Fault Handling:**
    *   If possible, safely induce a VFD fault (e.g., trigger an overcurrent if safe, or simulate by disconnecting motor briefly if appropriate - **CAUTION!**).
    *   *Expected (Relay - Status):* `Fault:` flag should become `True`.
    *   *Expected (Main):* `[ALERT] VFD Fault Detected!` message. Relays (GPIO 20, 21) should start blinking.
    *   Send `reset_fault`.
        *   *Expected (Relay):* `[PC CMD RX] reset_fault`, `[MODBUS TX] Writing RESET_FAULT command`.
        *   *Expected (Main):* `[ACTION] VFD Fault RESET command processed.`
        *   *Expected (Relay - Status):* `Fault:` flag should become `False` (if reset was successful).
        *   *Expected (Main):* Relays should stop blinking and stay solid ON.

**Phase E: Stress & Error Testing**

13. **Rapid Commands:** Send multiple commands quickly from the PC to test responsiveness.
14. **Communication Interruptions:**
    *   Temporarily disconnect the RS485 line between Relay and Main (Network 1).
        *   *Expected (Relay):* Should log `[ERROR] Modbus RX Error: No response...` or similar timeout errors periodically. Command sends might also log errors. PC interface should remain responsive.
        *   *Expected (Main):* Should continue operating (controlling VFD based on last command, polling VFD status). Local logs should continue.
    *   Reconnect Network 1. Communication should resume.
    *   Temporarily disconnect the RS485 line between Main and VFD (Network 2).
        *   *Expected (Main):* Should log errors/warnings related to failed VFD reads/writes (`[WARNING] Failed to read...`, `[ERROR] VFD Status Task Error...`).
        *   *Expected (Relay - Status):* Status values related to VFD (RPM, VFD Status, Fault) might become stale or show error indicators if implemented.
    *   Reconnect Network 2. Communication should resume.
15. **Long Run:** Leave the system running for an extended period, monitoring for unexpected behavior, crashes, or memory issues.