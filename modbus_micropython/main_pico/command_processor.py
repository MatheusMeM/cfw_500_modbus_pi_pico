# main_pico/command_processor.py

import uasyncio as asyncio
from utils import (
    print_verbose, internal_state, slave_registers, update_input_registers,
    save_configuration,
    REG_CMD, REG_TARGET_RPM, REG_VERBOSE, REG_ENC_MODE, REG_OFFSET_STEPS
)
# No direct VFD master needed if actions are simple state changes or call other modules

async def process_modbus_commands(vfd_master):
    """
    Checks Modbus holding registers for commands written by the master (Relay Pico)
    and executes corresponding actions.
    Should be called periodically from the main async loop.
    """
    # Read current values from the register dictionary
    cmd_reg_val = slave_registers['HREGS']['command']['val']
    target_rpm_val = slave_registers['HREGS']['target_rpm']['val']
    verbose_reg_val = slave_registers['HREGS']['verbosity_level']['val']
    enc_mode_reg_val = slave_registers['HREGS']['encoder_mode']['val']

    # --- Process Action Commands (REG_CMD) ---
    # Check if command register has a new non-zero value
    print_verbose(f"[DEBUG CMD Check] Current CMD Reg: {cmd_reg_val}, Last Written: {internal_state['last_written_cmd']}", 3) # ADDED DEBUG
    if cmd_reg_val != 0 and cmd_reg_val != internal_state['last_written_cmd']:
        command_to_process = cmd_reg_val
        rpm_to_process = target_rpm_val # Use the RPM value present when command is detected
        internal_state['last_written_cmd'] = command_to_process
        internal_state['last_written_rpm'] = rpm_to_process # Store RPM associated with command

        # Reset command register value to 0 immediately after reading to acknowledge
        slave_registers['HREGS']['command']['val'] = 0

        print_verbose(f"[DEBUG] Processing Modbus Command: {command_to_process}, RPM: {rpm_to_process}", 3)

        try:
            if command_to_process == 1: # START
                print_verbose(f"[DEBUG CMD] Calling vfd_master.start_motor({rpm_to_process})", 3) # ADDED DEBUG
                vfd_master.start_motor(rpm_to_process)
                print_verbose(f"[ACTION] Motor START command processed (RPM: {rpm_to_process}).", 0)
            elif command_to_process == 2: # STOP
                print_verbose(f"[DEBUG CMD] Calling vfd_master.stop_motor()", 3) # ADDED DEBUG
                vfd_master.stop_motor()
                print_verbose(f"[ACTION] Motor STOP command processed.", 0)
            elif command_to_process == 3: # REVERSE
                print_verbose(f"[DEBUG CMD] Calling vfd_master.reverse_motor({rpm_to_process})", 3) # ADDED DEBUG
                vfd_master.reverse_motor(rpm_to_process)
                print_verbose(f"[ACTION] Motor REVERSE command processed (RPM: {rpm_to_process}).", 0)
            elif command_to_process == 4: # RESET_FAULT
                print_verbose(f"[DEBUG CMD] Calling vfd_master.reset_fault()", 3) # ADDED DEBUG
                vfd_master.reset_fault()
                print_verbose(f"[ACTION] VFD Fault RESET command processed.", 0)
            elif command_to_process == 5: # CALIBRATE
                print_verbose(f"[DEBUG CMD] Processing CALIBRATE command", 3) # ADDED DEBUG
                # Set current position (relative to home) as the new offset
                current_pos_rel_home = internal_state['encoder_raw_position'] - internal_state['encoder_zero_offset']
                internal_state['encoder_offset_steps'] = current_pos_rel_home
                update_input_registers(offset=internal_state['encoder_offset_steps']) # Update readable offset
                save_configuration() # Save new offset
                print_verbose(f"[ACTION] Encoder CALIBRATE command processed. New offset: {internal_state['encoder_offset_steps']}", 0)
            # Command 0 is No Action / Idle - ignored here

        except Exception as e:
            print_verbose(f"[ERROR] Error processing Modbus command {command_to_process}: {e}", 0)

    # --- Process Setting Changes ---
    # Check Verbosity Level
    if verbose_reg_val != internal_state['VERBOSE_LEVEL']:
        if 0 <= verbose_reg_val <= 3:
            internal_state['VERBOSE_LEVEL'] = verbose_reg_val
            save_configuration()
            print_verbose(f"[INFO] Verbosity level updated to {verbose_reg_val} via Modbus.", 0)
        else:
            # Invalid value written by master, log and optionally write back valid value
            print_verbose(f"[WARNING] Invalid verbose level {verbose_reg_val} received via Modbus. Ignoring.", 1)
            slave_registers['HREGS']['verbosity_level']['val'] = internal_state['VERBOSE_LEVEL'] # Write back

    # Check Encoder Mode
    current_mode_val = 0 if internal_state['encoder_output_mode'] == "step" else 1
    if enc_mode_reg_val != current_mode_val:
        if enc_mode_reg_val in [0, 1]:
            internal_state['encoder_output_mode'] = "step" if enc_mode_reg_val == 0 else "deg"
            save_configuration()
            print_verbose(f"[INFO] Encoder output mode updated to '{internal_state['encoder_output_mode']}' via Modbus.", 0)
        else:
            # Invalid value written by master, log and optionally write back valid value
            print_verbose(f"[WARNING] Invalid encoder mode {enc_mode_reg_val} received via Modbus. Ignoring.", 1)
            slave_registers['HREGS']['encoder_mode']['val'] = current_mode_val # Write back

    # Check for direct RPM change without START/REVERSE command (like set_speed)
    # Only process if command is currently 0 (idle) to avoid conflict
    elif cmd_reg_val == 0 and target_rpm_val != internal_state['last_written_rpm']:
         internal_state['last_written_rpm'] = target_rpm_val # Update last known RPM
         try:
             # Assuming set_speed_reference should be called if motor is already running
             # We need VFD status to know if it's running. This check should ideally
             # happen within the vfd_master object or require reading status first.
             # For simplicity here, we just call set_speed_reference.
             # A more robust implementation might check P0680 bit 1 (Command Rotate).
             vfd_master.set_speed_reference(target_rpm_val)
             print_verbose(f"[ACTION] Speed reference updated to {target_rpm_val} RPM via Modbus.", 0)
         except Exception as e:
             print_verbose(f"[ERROR] Error updating speed reference to {target_rpm_val}: {e}", 0)

    # Reset last_written_cmd if command is 0 and processed
    if cmd_reg_val == 0:
        internal_state['last_written_cmd'] = 0