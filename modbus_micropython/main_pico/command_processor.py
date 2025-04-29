# main_pico/command_processor.py

import uasyncio as asyncio
from utils import (
    print_verbose, internal_state, slave_registers, update_input_registers, # update_input_registers might not be needed here but keep for consistency
    save_configuration,
    REG_CMD, REG_TARGET_RPM, REG_VERBOSE, REG_ENC_MODE, REG_OFFSET_STEPS
)
# No direct VFD master needed if actions are simple state changes or call other modules

async def process_modbus_commands(vfd_master): # vfd_master might not be needed if only settings are processed
    """
    Checks Modbus holding registers for SETTING changes written by the master (Relay Pico)
    and updates internal state accordingly.
    Action commands (START, STOP, etc.) are now handled by the callback
    `handle_command_register_write` in main.py.
    Should be called periodically from the main async loop.
    """
    print_verbose("[DEBUG SETTINGS] process_modbus_commands called", 3) # Add entry debug print

    # Read current values from the register dictionary
    # Note: Accessing '.val' directly assumes the slave task keeps the dict updated.
    try:
        # cmd_reg_val = slave_registers['HREGS']['command']['val'] # Action commands handled by callback
        # target_rpm_val = slave_registers['HREGS']['target_rpm']['val'] # Action commands handled by callback
        verbose_reg_val = slave_registers['HREGS']['verbosity_level']['val']
        enc_mode_reg_val = slave_registers['HREGS']['encoder_mode']['val']
        print_verbose(f"[DEBUG SETTINGS] Read HREGS: Verbose={verbose_reg_val}, EncMode={enc_mode_reg_val}", 3)
    except KeyError as e:
        print_verbose(f"[ERROR SETTINGS] Missing key in slave_registers: {e}", 0)
        return # Exit if we can't read registers

    # --- Process Action Commands (REG_CMD) ---
    # This section is removed as it's handled by the callback handle_command_register_write

    # --- Process Setting Changes ---
    # Check Verbosity Level
    # NOTE: We read the register value directly. The callback mechanism isn't strictly
    # necessary for settings if periodic checking is acceptable.
    # If immediate reaction to setting changes is needed, callbacks could be added
    # for REG_VERBOSE and REG_ENC_MODE as well.
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

    # Check for direct RPM change (set_speed) - This logic is removed for now.
    # It was complex to handle correctly without a dedicated command/callback.
    # Consider adding a specific SET_SPEED command (e.g., 6) handled via callback if needed.