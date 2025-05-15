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
Task 2.3: Update `calibrate` Command Logic (✔️ COMPLETE)
    [✔️] Ensure `calibrate` (Modbus command 5) only functions if `internal_state['homing_completed']` is `True`. (Implemented in `handle_command_register_write`).
    [✔️] Calculate `internal_state['encoder_offset_steps']` based on `encoder_raw_position` relative to the Z-pulse (`encoder_zero_offset`).
    [✔️] Update `REG_OFFSET_STEPS` and save to `config.json`.
Task 2.4: Homing Testing & Refinement (IN PROGRESS)
    [✔️] Initial successful homing sequence observed via logs.
    [✔️] Conduct further testing for reliability across multiple runs.
    [✔️] Test edge cases (e.g., starting on/near Z-pulse for each phase).
    [✔️] Verify accuracy of `encoder_zero_offset` and subsequent calibrated positions.
    [✔️] Optimize `HOMING_POLL_INTERVAL_MS` if necessary (current: 2ms).
    [✔️] Modified homing Phase 2 to use `internal_absolute_degrees` instead of raw encoder steps for consistency.

Phase 3: Rotation and Positioning Implementation (✔️ COMPLETE)
Task 3.1: Define Rotation Command and Modbus Communication (✔️ COMPLETE)
    [✔️] Main Pico: Implemented command code 7 (ROTATE) that takes an angle parameter directly in the Modbus write.
    [✔️] Relay Pico: Implemented "rotate [angle]" command that sends command 7 and angle to Main Pico.
Task 3.2: Implement Core Positioning Logic (✔️ COMPLETE)
    [✔️] Defined constants: `POSITIONING_RPM` (600), `MIN_ROTATION_ANGLE_DEG` (1.0), `POSITIONING_STOP_MARGIN_DEG` (5.0).
    [✔️] Implemented proper early stopping with margin to account for inertia.
    [✔️] Implemented command handling in `handle_command_register_write` to:
        [✔️] Validate (homing completed, not already positioning, angle valid).
        [✔️] Calculate absolute target and early stop target using `internal_absolute_degrees`.
        [✔️] Initiate motor movement at `POSITIONING_RPM`.
Task 3.3: Implement Position Monitoring (✔️ COMPLETE)
    [✔️] Implemented motion monitoring within `position_motor` function to:
        [✔️] Monitor `internal_absolute_degrees`.
        [✔️] Issue VFD stop command when target is reached or on timeout.
        [✔️] Manage `positioning_in_progress_event` state.
        [✔️] Clear `REG_CMD` upon completion/error.
        [✔️] Provide error handling and reporting.
Task 3.4: Implement "Go to Calibrated Position" Functionality (✔️ COMPLETE)
    [✔️] Added `go_to_calibrated_position` function that moves to the position where `internal_absolute_degrees` = 0.
    [✔️] Added automatic movement to calibrated position after successful homing.
    [✔️] Added command code 8 (GO_TO_CALIBRATED_POSITION) to manually trigger the movement.
    [✔️] Relay Pico: Added "go_to_calib" command to send command 8 to Main Pico.
Task 3.5: Consistent Unidirectional Movement (✔️ COMPLETE)
    [✔️] Modified `position_motor` to always use VFD physical FORWARD direction.
    [✔️] Uses magnitude of requested angle for distance calculation.
    [✔️] Properly handles positioning with decreasing absolute degree values.

Phase 4: System Testing & Refinement (FUTURE)
Task 4.1: Full Workflow Testing
    [✔️] Test the complete sequence: Power-on -> Homing -> Auto-move to Calibrated Position.
    [✔️] Test manual calibration and go_to_calib commands.
    [✔️] Test rotation with various angles (positive and negative).
Task 4.2: Accuracy and Robustness Evaluation
    [✔️] Measure positioning accuracy for various angles.
    [✔️] Test behavior with small angle commands (implemented MIN_ROTATION_ANGLE_DEG check).
    [✔️] Observe system stability and error handling (improved error handling throughout).
Task 4.3: User Interface Improvements
    [✔️] Improved help display in relay_pico with structured, categorized commands.
    [✔️] Added clear logging for all operations.
    [✔️] Updated documentation to reflect new features and operation.

Phase 5: Additional Features and Refinements (FUTURE)
Task 5.1: Enhanced Error Recovery
    [ ] Implement automatic recovery paths for common error conditions.
    [ ] Add more detailed diagnostic information for troubleshooting.
Task 5.2: Performance Optimization
    [ ] Fine-tune movement parameters for optimal speed and accuracy.
    [ ] Evaluate and optimize communication timing and task scheduling.
Task 5.3: Additional User Features
    [ ] Add more convenience commands and feedback.
    [ ] Consider implementing absolute position movement command.