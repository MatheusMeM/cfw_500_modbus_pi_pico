Project Roadmap
Phase 0: Initial Setup & Foundation (✔️ COMPLETE)
System architecture defined (Relay Pico, Main Pico, VFD, Encoder).
Modbus communication established (Relay <-> Main, Main <-> VFD).
Basic VFD control commands implemented (start, stop, reverse, speed set, fault handling).
Initial Modbus register map defined.
Configuration persistence (config.json) for Main Pico.
Asynchronous task structure on Main Pico.

Phase 1: Encoder Integration & Basic Functionality (Main Pico) (✔️ COMPLETE)
Task 1.1: Enable Encoder & Basic Absolute Tracking (✔️ COMPLETE)
    Encoder driver initialized and active.
    encoder_callback updates internal_state['internal_absolute_degrees'] (continuous, non-wrapping).
    encoder_callback updates Modbus Input Registers (REG_ENC_POS_STEPS, REG_ENC_POS_DEG) with wrapped (0-360) values, relative to calibrated zero.
    Relay Pico successfully reads and displays wrapped encoder values.
Add-on Task: Comment out LED Relay Control (✔️ COMPLETE)
    Main Pico's active relay control based on fault state is disabled. Pin definitions remain.

Phase 2: Homing Implementation (Main Pico) (✔️ TESTING & REFINEMENT)
Task 2.1: Define Homing Constants & Refine State (✔️ COMPLETE)
    [✔️] Define RPMs for homing search/creep, backup distance. (HOMING_SEARCH_RPM, HOMING_CREEP_RPM, HOMING_BACKUP_DEGREES, SETTLE_TIME_MS defined and used).
Task 2.2: Implement the Homing Async Function (`homing`) (✔️ IMPLEMENTED - Z-Pin Polling)
    [✔️] Implement robust multi-phase homing sequence using **Z-pin polling**:
        [✔️] Coarse forward search for Z-pulse (falling edge).
        [✔️] Backup past Z-pulse using encoder counts for distance.
            *   **Note:** Verify consistency between `HOMING_BACKUP_DEGREES` constant, `STEPS_PER_DEGREE` calculation, and observed `TargetSteps` in logs if discrepancies persist in future tests. Current log shows 444 steps for backup, while code with 20 degrees implies ~222 steps.
        [✔️] Slow forward approach to Z-pulse (falling edge).
    [✔️] Capture `internal_state['encoder_zero_offset']` at Z-pulse detection.
    [✔️] Set `internal_state['homing_completed'] = True` and update `REG_HOMING_FLAG`.
    [✔️] Ensure proper handling of timeouts and motor control during homing (timeouts implemented and motor control is functional).
    [✔️] Manage background task interaction: Current implementation uses `await asyncio.sleep_ms()` which yields control, allowing background tasks to run. Explicit cancellation/resumption is currently commented out and appears not strictly necessary for basic operation. Further evaluation if specific race conditions or performance issues arise.
Task 2.3: Update `calibrate` Command Logic (➡️ NEXT - PARTIALLY COMPLETE)
    [ ] Ensure `calibrate` (Modbus command 5) only functions if `internal_state['homing_completed']` is `True`. (This check is NOT YET IMPLEMENTED in `handle_command_register_write`).
    [✔️] Calculate `internal_state['encoder_offset_steps']` based on `encoder_raw_position` relative to the Z-pulse (`encoder_zero_offset`).
    [✔️] Update `REG_OFFSET_STEPS` and save to `config.json`.
Task 2.4: Homing Testing & Refinement (IN PROGRESS)
    [✔️] Initial successful homing sequence observed via logs.
    [ ] Conduct further testing for reliability across multiple runs.
    [ ] Test edge cases (e.g., starting on/near Z-pulse for each phase).
    [ ] Verify accuracy of `encoder_zero_offset` and subsequent calibrated positions.
    [ ] Optimize `HOMING_POLL_INTERVAL_MS` if necessary (current: 2ms).
    [ ] Confirm behavior of `HOMING_BACKUP_DEGREES` and step calculation.

Phase 3: "Rotate Forward X Degrees" MVP (Main Pico & Relay Pico) (UPCOMING)
Task 3.1: Define New Modbus Communication (Main Pico & Relay Pico)
    [ ] Main Pico: Define `REG_TARGET_ANGLE_DEG` (e.g., HReg 104) and new command code (e.g., 6 for "GOTO_RELATIVE_POS_FORWARD").
    [ ] Relay Pico: Update to send new command and target angle to Main Pico.
Task 3.2: Implement Core Positioning Logic (Main Pico)
    [ ] Define constants: `POSITIONING_RPM`, `MIN_ROTATION_ANGLE_DEG`.
    [ ] Calibrate and define `DECELERATION_DEGREES_AT_POSITIONING_RPM`.
    [ ] Implement command handling in `handle_command_register_write` to:
        Validate (homed, not already positioning, angle valid).
        Calculate absolute target and early stop target using `internal_absolute_degrees`.
        Initiate motor movement at `POSITIONING_RPM`.
Task 3.3: Create Positioning Monitor Task (Main Pico)
    [ ] Implement `positioning_monitor_task` to:
        Monitor `internal_absolute_degrees`.
        Issue VFD stop command when early stop target is reached.
        Manage `positioning_active` state.
        Clear `REG_CMD` upon completion/error.
        Handle VFD faults during the move.

Phase 4: System Testing & Refinement (FUTURE)
Task 4.1: Full Workflow Testing
    [ ] Test the complete sequence: Power-on -> Homing -> Calibration -> "Rotate Forward X Degrees" commands.
Task 4.2: Accuracy and Robustness Evaluation
    [ ] Measure positioning accuracy for various angles.
    [ ] Test behavior with small angle commands.
    [ ] Observe system stability and error handling.
Task 4.3: Iteration & Bug Fixing
    [ ] Address any issues found during testing.
    [ ] Potentially refine `DECELERATION_DEGREES_AT_POSITIONING_RPM` or other parameters.