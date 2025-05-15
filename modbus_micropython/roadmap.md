Phase 1: Encoder Enhancements & Basic Setup (Main Pico)
Task 1.1: Enable Encoder Initialization & Basic Absolute Tracking
[ ] In main_pico/main.py:
Uncomment the line initialize_encoder(16, 17).
Ensure the STATUS_REQUEST_INTERVAL_MS is reasonably short for testing (e.g., 500-1000ms) to see encoder updates frequently via Relay Pico.
[ ] In main_pico/utils.py:
In internal_state dictionary, add a new key: 'internal_absolute_degrees': 0.0.
[ ] In main_pico/encoder_module.py (encoder_callback function):
Define STEPS_PER_DEGREE = MAX_STEPS / 360.0.
After calculating adjusted_position_steps, calculate current_absolute_steps = adjusted_position_steps. Initially, internal_absolute_degrees will just track the calibrated position without continuous accumulation beyond one revolution, until homing defines encoder_zero_offset properly.
internal_state['internal_absolute_degrees'] = current_absolute_steps / STEPS_PER_DEGREE
(Self-correction: adjusted_position_steps already includes the encoder_offset_steps. The raw value relative to Z-home is position_relative_to_home = inverted_value - internal_state['encoder_zero_offset']. So, for absolute tracking from Z-home + calibrated offset, it should be:
absolute_steps_from_true_zero = internal_state['encoder_raw_position'] - internal_state['encoder_zero_offset'] - internal_state['encoder_offset_steps']
internal_state['internal_absolute_degrees'] = absolute_steps_from_true_zero / STEPS_PER_DEGREE
Ensure internal_state['encoder_raw_position'] is correctly updated from the inverted_value.
The existing logic for wrapped_position_steps and degrees (for Modbus reporting 0-360) should remain.
Comment: Verify that REG_ENC_POS_STEPS and REG_ENC_POS_DEG (0-360 range) are updating correctly on the Relay Pico. Manually turn the encoder and observe. The internal_absolute_degrees won't be fully meaningful until after homing and calibration are implemented and tested, but the structure should be in place.
Phase 2: Homing Implementation (Main Pico)
Task 2.1: Define Homing Constants & Refine State
[ ] In main_pico/main.py (or utils.py if preferred for constants):
Define HOMING_SEARCH_RPM = 150 (or a suitable slow speed).
Define HOMING_CREEP_RPM = 75 (or a suitable very slow speed).
Define HOMING_BACKUP_DEGREES = 20.0 (degrees to back up past Z-pulse).
Task 2.2: Implement the Homing Async Function (homing)
[ ] In main_pico/main.py (homing async function):
Uncomment the homing call in async def main(). Ensure background tasks (status_task, slave_poll_task) are temporarily cancelled before homing and restarted after. (Self-correction: The slave_poll_task might be needed for the VFD control commands sent by homing itself. Consider if it truly needs to be cancelled, or if its interactions are safe. For now, let's assume it can stay running or only status_task needs cancelling).
Implement Phase 1: Coarse Search (Optional but Recommended):
Start motor forward at HOMING_SEARCH_RPM.
Enable Z-signal IRQ (zero_pin.irq(...) pointing to endstop_triggered_callback_irq).
await endstop_event.wait() (with a suitable timeout, e.g., 60 seconds).
On event or timeout: vfd_master.stop_motor(), zero_pin.irq(handler=None), endstop_event.clear().
await asyncio.sleep_ms(3000) (for motor to stop).
Handle timeout: print error, set homing_completed = False, return.
Implement Phase 2: Backup Past Z-Pulse:
Calculate backup_steps = HOMING_BACKUP_DEGREES * STEPS_PER_DEGREE.
Record start_backup_pos_raw = internal_state['encoder_raw_position'].
Start motor reverse at HOMING_CREEP_RPM.
Loop, monitoring internal_state['encoder_raw_position'], until abs(internal_state['encoder_raw_position'] - start_backup_pos_raw) >= backup_steps.
vfd_master.stop_motor().
await asyncio.sleep_ms(3000).
Implement Phase 3: Slow Forward Approach to Z-Pulse:
Start motor forward at HOMING_CREEP_RPM.
Clear and enable Z-signal IRQ and endstop_event.
await endstop_event.wait() (with a shorter timeout, e.g., 30 seconds).
Immediately upon event detection (or right after await):
vfd_master.stop_motor().
internal_state['encoder_zero_offset'] = internal_state['encoder_raw_position'] (Capture raw value AT Z-pulse).
Set internal_state['homing_completed'] = True.
Call update_input_registers(modbus_slave_handler, homing=True).
Print success message with encoder_zero_offset.
Handle timeout: print error, set homing_completed = False, update_input_registers(modbus_slave_handler, homing=False), call vfd_master.stop_motor(), return.
Finally block: zero_pin.irq(handler=None), endstop_event.clear().
Comment: Test homing thoroughly. Verify encoder_zero_offset is captured. Verify REG_HOMING_FLAG updates. Check behavior on timeouts.
Task 2.3: Update calibrate Command Logic
[ ] In main_pico/main.py (handle_command_register_write for command_to_process == 5 (CALIBRATE)):
Ensure it only proceeds if internal_state['homing_completed'] == True. If not, print error and do not clear REG_CMD.
The logic:
current_raw_pos = internal_state['encoder_raw_position']
offset_val = current_raw_pos - internal_state['encoder_zero_offset']
internal_state['encoder_offset_steps'] = offset_val
update_input_registers(modbus_slave_handler, offset=internal_state['encoder_offset_steps'])
save_configuration()
Print success message with new offset.
Comment: After homing, move the motor manually to the desired application zero and send the calibrate command. Verify REG_OFFSET_STEPS updates and config.json saves the correct value.
Phase 3: "Rotate Forward X Degrees" MVP (Main Pico)
Task 3.1: Define New Modbus Registers & Constants
[ ] In main_pico/utils.py:
Add REG_TARGET_ANGLE_DEG = 104 to Holding Register definitions.
In slave_registers['HREGS'], add: 'target_angle_deg': {'register': REG_TARGET_ANGLE_DEG, 'val': 0.0}.
(Ensure REG_CMD = 6 is available or choose another for GOTO_RELATIVE_POS).
[ ] In main_pico/main.py (or utils.py):
Define POSITIONING_RPM = 500 (or suitable fixed RPM).
Define DECELERATION_DEGREES_AT_POSITIONING_RPM = 15.0 (placeholder, to be calibrated).
Define MIN_ROTATION_ANGLE_DEG = 30.0 (placeholder, e.g., 2 * DECELERATION_DEGREES_AT_POSITIONING_RPM).
Task 3.2: Add State for Positioning Move
[ ] In main_pico/utils.py (internal_state):
Add 'positioning_active': False.
Add 'positioning_final_target_abs_deg': 0.0.
Add 'positioning_early_stop_abs_deg': 0.0.
Task 3.3: Implement "Rotate Forward" Command Handling
[ ] In main_pico/main.py (handle_command_register_write):
Add a new elif command_to_process == 6: # GOTO_RELATIVE_POS_FORWARD
Check internal_state['homing_completed']. If false, print error, do not clear REG_CMD, return.
Check internal_state['positioning_active']. If true, print "already positioning", do not clear REG_CMD, return.
Read target_relative_angle = modbus_slave_handler.get_hreg(REG_TARGET_ANGLE_DEG).
If target_relative_angle <= 0: print error "Angle must be positive", do not clear REG_CMD, return.
If target_relative_angle < MIN_ROTATION_ANGLE_DEG: print warning "Angle too small, best effort", proceed.
internal_state['positioning_final_target_abs_deg'] = internal_state['internal_absolute_degrees'] + target_relative_angle
internal_state['positioning_early_stop_abs_deg'] = internal_state['positioning_final_target_abs_deg'] - DECELERATION_DEGREES_AT_POSITIONING_RPM
Set internal_state['positioning_active'] = True.
vfd_master.start_motor(POSITIONING_RPM).
Print info: "Starting GOTO_RELATIVE_POS_FORWARD to {final_target}, early stop at {early_stop_target}".
Do not clear REG_CMD here; it will be cleared by the monitoring task upon completion or error.
Task 3.4: Create Positioning Monitoring Async Task
[ ] In main_pico/main.py, create a new async def positioning_monitor_task():
This task will run continuously but only act when internal_state['positioning_active'] == True.
while True:
if internal_state['positioning_active']:
current_abs_deg = internal_state['internal_absolute_degrees']
early_stop_target = internal_state['positioning_early_stop_abs_deg']
If vfd_master.check_fault():
vfd_master.stop_motor() (attempt)
internal_state['positioning_active'] = False
modbus_slave_handler.set_hreg(REG_CMD, 0) (clear command to signal error/completion)
Print "VFD Fault during positioning".
Continue to next await asyncio.sleep_ms().
If current_abs_deg >= early_stop_target:
vfd_master.stop_motor()
Print "Early stop triggered. Decelerating..."
await asyncio.sleep_ms(3000) (Wait for deceleration + a margin).
final_pos = internal_state['internal_absolute_degrees'] (Read final position)
Print "Positioning complete. Target: {final_target}, Actual: {final_pos}, Error: {error}".
internal_state['positioning_active'] = False
modbus_slave_handler.set_hreg(REG_CMD, 0) (Clear command to signal completion).
await asyncio.sleep_ms(10) (Poll frequently during active positioning).
[ ] In main_pico/main.py (async def main()):
Create and start this task: asyncio.create_task(positioning_monitor_task()).
Comment: This task handles the actual stopping and completion signaling. The command callback just initiates the move.
Phase 4: Relay Pico Integration
Task 4.1: Add New Command to Relay Pico
[ ] In relay_pico/main.py:
Define REG_TARGET_ANGLE_DEG = 104 (matching Main Pico).
Add a new PC command, e.g., rotate_fwd <degrees>:
Parse degrees.
modbus_master.write_multiple_registers(MAIN_PICO_SLAVE_ADDR, REG_CMD, [6, int(degrees * 10)]) (Send command 6, and angle scaled if necessary, or send as float if library/Pico supports it directly in registers. umodbus typically expects integers for registers. If sending float, it would need to be packed into two registers, which adds complexity. Let's assume for now we send degrees * 10 as an integer, and Main Pico divides by 10). Self-correction: HRegs are 16-bit integers. Sending float degrees directly is not standard. Let's send degrees as a whole number (e.g., 90 for 90.0) or degrees10 (900 for 90.0) if more precision is needed and fits in a single HReg. For now, int(degrees). Main Pico expects float target, so it might need to read this int and convert.*
A better approach for REG_TARGET_ANGLE_DEG: send int(degrees_value * SCALING_FACTOR) e.g. degrees_value * 10 or 100 and then Main Pico divides by the same factor. Let's use scaling factor 10. So, int(degrees * 10). Max angle ~3200 degrees.
So, modbus_master.write_multiple_registers(MAIN_PICO_SLAVE_ADDR, REG_CMD, [6, angle_scaled_int]). The second register written will be REG_CMD+1 which is REG_TARGET_RPM. This is an issue. We need to write to REG_CMD and REG_TARGET_ANGLE_DEG (104).
Corrected Relay Pico write:
Write angle to REG_TARGET_ANGLE_DEG: modbus_master.write_single_register(MAIN_PICO_SLAVE_ADDR, REG_TARGET_ANGLE_DEG, int(degrees * 10))
Write command to REG_CMD: modbus_master.write_single_register(MAIN_PICO_SLAVE_ADDR, REG_CMD, 6)
Comment: Test sending the command from PC through Relay Pico. Monitor Main Pico logs to see if it receives and attempts the action.
Phase 5: Calibration and Testing
Task 5.1: Calibrate DECELERATION_DEGREES_AT_POSITIONING_RPM
[ ] Manually run the motor at POSITIONING_RPM, issue stop, measure coasting degrees. Update this constant in main_pico/main.py. Repeat several times for consistency.
Task 5.2: Full System Test
[ ] Test homing.
[ ] Test calibrate command after homing.
[ ] Test rotate_fwd <degrees> for various angles (large and small).
[ ] Observe accuracy and behavior.
[ ] Check VFD fault handling during positioning.
This checklist provides a structured approach for RooCode. Each task builds upon the previous one. Remember to commit frequently after completing and testing each logical sub-task.

Phase 1: Encoder Enhancements & Basic Setup (Main Pico)
Task 1.1: Enable Encoder Initialization & Basic Absolute Tracking (CURRENT FOCUS)
[ ] In main_pico/main.py:
Uncomment initialize_encoder(16, 17).
Update print message for encoder status.
Adjust STATUS_REQUEST_INTERVAL_MS to 1000 for testing.
[ ] In main_pico/utils.py:
Add 'internal_absolute_degrees': 0.0 to internal_state.
[ ] In main_pico/encoder_module.py (encoder_callback):
Define STEPS_PER_DEGREE = MAX_STEPS / 360.0.
Ensure internal_state['encoder_raw_position'] is updated.
Calculate and store internal_state['internal_absolute_degrees'] using encoder_raw_position, encoder_zero_offset, and encoder_offset_steps.
Retain existing logic for wrapped_position_steps and degrees (0-360) for Modbus reporting.
(Optional) Add debug print for internal_absolute_degrees.
Verification: Confirm wrapped EncSteps/EncDeg update on Relay Pico. Confirm (via debug) internal_absolute_degrees accumulates correctly on Main Pico.
Phase 2: Homing Implementation (Main Pico)
Task 2.1: Define Homing Constants & Refine State
[ ] Define HOMING_SEARCH_RPM, HOMING_CREEP_RPM, HOMING_BACKUP_DEGREES.
Task 2.2: Implement the Homing Async Function (homing)
[ ] Uncomment homing call in main(), manage background task cancellation/restarting around it.
[ ] Implement Phase 1: Coarse Search (optional but recommended).
[ ] Implement Phase 2: Backup Past Z-Pulse.
[ ] Implement Phase 3: Slow Forward Approach to Z-Pulse, capture encoder_zero_offset, set homing_completed.
[ ] Handle timeouts and errors robustly.
Task 2.3: Update calibrate Command Logic
[ ] Ensure calibrate only runs if homed.
[ ] Correctly calculate encoder_offset_steps based on encoder_raw_position and encoder_zero_offset.
[ ] Update REG_OFFSET_STEPS and save config.
Phase 3: "Rotate Forward X Degrees" MVP (Main Pico)
Task 3.1: Define New Modbus Registers & Constants
[ ] Add REG_TARGET_ANGLE_DEG (HReg 104) to utils.py and slave_registers.
[ ] Define POSITIONING_RPM, DECELERATION_DEGREES_AT_POSITIONING_RPM (calibrate this value later), MIN_ROTATION_ANGLE_DEG.
[ ] Define GOTO_RELATIVE_POS_FORWARD command code (e.g., 6).
Task 3.2: Add State for Positioning Move
[ ] Add positioning_active, positioning_final_target_abs_deg, positioning_early_stop_abs_deg to internal_state.
Task 3.3: Implement "Rotate Forward" Command Handling in handle_command_register_write
[ ] Handle new command code.
[ ] Validate (homed, not already positioning, angle positive).
[ ] Calculate targets using internal_absolute_degrees.
[ ] Set positioning_active = True, start motor. Do not clear REG_CMD here.
Task 3.4: Create positioning_monitor_task()
[ ] Runs continuously, acts when positioning_active == True.
[ ] Monitors internal_absolute_degrees against positioning_early_stop_abs_deg.
[ ] Issues vfd_master.stop_motor() when early stop target is reached.
[ ] Waits for deceleration, logs final position/error.
[ ] Sets positioning_active = False, clears REG_CMD in slave_registers (HReg 100).
[ ] Handles VFD faults during positioning.
[ ] Start this task in main().
Phase 4: Relay Pico Integration
Task 4.1: Add New Command to Relay Pico
[ ] In relay_pico/main.py:
Define REG_TARGET_ANGLE_DEG = 104.
Add PC command rotate_fwd <degrees>.
Relay Pico writes int(degrees * 10) to Main Pico's REG_TARGET_ANGLE_DEG (HReg 104) using write_single_register.
Relay Pico writes command code (e.g., 6) to Main Pico's REG_CMD (HReg 100) using write_single_register.
Phase 5: Calibration and Testing
Task 5.1: Calibrate DECELERATION_DEGREES_AT_POSITIONING_RPM
[ ] Manually determine and update this constant.
Task 5.2: Full System Test
[ ] Test complete workflow: Homing -> Calibrate -> Rotate Forward.
[ ] Evaluate accuracy and robustness.