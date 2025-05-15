# utils.py - Refactored for Modbus Slave Role

import time
import json
from machine import Pin # UART removed, handled by Modbus instances

# --- Constants ---
CONFIG_FILE = "config.json"

# Modbus Register Definitions (Slave Map for Network 1 - Relay Master)
# Holding Registers (Writeable by Relay Master)
REG_CMD = 100         # Command Register (1=START, 2=STOP, 3=REVERSE, 4=RESET_FAULT, 5=CALIBRATE, 0=No Action)
REG_TARGET_RPM = 101  # Target RPM for START/REVERSE/SET_SPEED
REG_VERBOSE = 102     # Verbosity Level (0-3) for local printing
REG_ENC_MODE = 103    # Encoder Output Mode (0=steps, 1=degrees)
# Input Registers (Readable by Relay Master)
REG_CURRENT_RPM = 0   # Current Motor RPM (Reported as integer, scaled x10)
REG_VFD_STATUS = 1    # VFD Status Word (P0680)
REG_ENC_POS_STEPS = 2 # Encoder Position (Steps, relative to calibrated offset)
REG_ENC_POS_DEG = 3   # Encoder Position (Degrees x100, relative to calibrated offset)
REG_FAULT_FLAG = 4    # VFD Fault Detected Flag (0/1)
REG_HOMING_FLAG = 5   # Homing Completed Flag (0/1)
REG_OFFSET_STEPS = 6  # Calibrated Encoder Offset (Steps)
REG_MAX_RPM = 7     # Configured Max RPM (P0208)

# Define the structure for the Modbus slave registers
# Format matches the structure expected by umodbus library's setup_registers method
slave_registers = {
    # register_name: {'register': address, 'val': initial_value, 'on_set_cb': ..., 'on_get_cb': ...}
    'HREGS': { # Holding Registers (Writeable by Relay Master)
        'command':          {'register': REG_CMD,          'val': 0}, # on_set_cb will be assigned in main.py before setup
        'target_rpm':       {'register': REG_TARGET_RPM,   'val': 0},
        'verbosity_level':  {'register': REG_VERBOSE,      'val': 3}, # Default verbosity
        'encoder_mode':     {'register': REG_ENC_MODE,     'val': 1}, # Default 'deg'
    },
    'IREGS': { # Input Registers (Readable by Relay Master)
        # --- !!! START OF CHANGE: Add non-zero initial test values !!! ---
        'current_rpm':      {'register': REG_CURRENT_RPM,  'val': 123},    # Test value 12.3 RPM (123 / 10.0)
        'vfd_status':       {'register': REG_VFD_STATUS,   'val': 0xABCD}, # Test status
        'encoder_steps':    {'register': REG_ENC_POS_STEPS,'val': 4567},    # Test steps
        'encoder_degrees':  {'register': REG_ENC_POS_DEG,  'val': 8912},    # Test degrees 89.12 (8912 / 100.0)
        'fault_flag':       {'register': REG_FAULT_FLAG,   'val': 0},       # Test fault = True
        'homing_flag':      {'register': REG_HOMING_FLAG,  'val': 0},       # Test homing = True
        'offset_steps':     {'register': REG_OFFSET_STEPS, 'val': 99},      # Test offset
        'max_rpm':          {'register': REG_MAX_RPM,      'val': 1750},    # Test max RPM
        # --- !!! END OF CHANGE !!! ---
    },
    'COILS': {}, # Not used currently
    'ISTS': {} # Discrete Inputs (ISTS expected by library, was 'discrete_inputs')
}

# Global Variables for shared internal state (distinct from Modbus registers)
# Some state might now be directly read/written via Modbus registers
internal_state = {
    'VERBOSE_LEVEL': 3, # Still needed for local print control
    'encoder_raw_position': 0, # Raw steps from encoder driver before offsets
    'encoder_offset_steps': 0, # Calibration offset
    'encoder_output_mode': "deg", # Internal tracking ('step' or 'deg')
    'homing_completed': False,
    'encoder_zero_offset': 0, # Offset determined during homing
    'fault_detected': False, # Internal flag, mirrored to Modbus register
    'last_written_cmd': 0, # Track last command written by master
    'last_written_rpm': 0, # Track last RPM written by master
    'internal_absolute_degrees': 0.0, # <<< ADD THIS LINE
}

# Define critical message prefixes
CRITICAL_PREFIXES = ("[SAFETY]", "[WARNING]", "[ALERT]", "[ERROR]")

def print_verbose(message, level, override=False):
    """Prints messages locally according to the verbosity level."""
    is_critical = any(message.startswith(prefix) for prefix in CRITICAL_PREFIXES)
    
    # Use internal_state for local verbosity control
    current_level = internal_state['VERBOSE_LEVEL']

    # Determine if the message should be printed locally
    should_print = override or is_critical or (current_level > 0 and level <= current_level)

    if should_print:
        # Only print to local console (USB)
        print(message)

def show_manual():
    """Displays the instruction manual locally."""
    # Manual might need updating to reflect Modbus register usage
    manual = """
========== INSTRUCTION MANUAL (Main Pico - Local Console) ==========
System controlled via Modbus RTU from Relay Pico.
Local Commands Removed. Monitor status via Relay Pico / PC.
Key Modbus Registers (Relay -> Main):
  Holding 100: Command (1=Start, 2=Stop, 3=Reverse, 4=Reset, 5=Calibrate)
  Holding 101: Target RPM
  Holding 102: Verbose Level (Local Print)
  Holding 103: Encoder Mode (0=step, 1=deg)
Key Modbus Registers (Main -> Relay):
  Input 0: Current RPM (x10)
  Input 1: VFD Status (P0680)
  Input 2: Encoder Steps
  Input 3: Encoder Degrees (x100)
  Input 4: Fault Flag (0/1)
  Input 5: Homing Flag (0/1)
  Input 6: Offset Steps
====================================================================
"""
    print_verbose(manual, 0, override=True) # Print locally only

def save_configuration():
    """Saves relevant settings to a configuration file."""
    # Save internal state values that need persistence
    config = {
        "encoder_offset_steps": internal_state['encoder_offset_steps'],
        "encoder_output_mode": internal_state['encoder_output_mode'],
        "VERBOSE_LEVEL": internal_state['VERBOSE_LEVEL']
    }
    try:
        with open(CONFIG_FILE, 'w') as f:
            json.dump(config, f)
        print_verbose("[INFO] Configuration saved.", 0) # Local print
    except Exception as e:
        print_verbose(f"[ERROR] Failed to save configuration: {e}", 0) # Local print

def load_configuration(modbus_handler): # ADD modbus_handler argument
    """
    Loads settings from config.json into internal_state,
    AND updates corresponding Modbus registers VIA THE MODBUS HANDLER.
    """
    # Default values for internal_state if config load fails,
    # derived from initial slave_registers which umodbus has already snapshotted.
    # These defaults are used if config.get fails or file not found.
    default_offset = modbus_handler.get_ireg(REG_OFFSET_STEPS) if modbus_handler else slave_registers['IREGS']['offset_steps']['val']
    default_enc_mode_val = modbus_handler.get_hreg(REG_ENC_MODE) if modbus_handler else slave_registers['HREGS']['encoder_mode']['val']
    default_enc_mode_str = "deg" if default_enc_mode_val == 1 else "step"
    default_verbose = modbus_handler.get_hreg(REG_VERBOSE) if modbus_handler else slave_registers['HREGS']['verbosity_level']['val']

    try:
        with open(CONFIG_FILE, 'r') as f:
            config = json.load(f)

        # Load into internal_state first
        internal_state['encoder_offset_steps'] = config.get("encoder_offset_steps", default_offset)
        internal_state['encoder_output_mode'] = config.get("encoder_output_mode", default_enc_mode_str)
        internal_state['VERBOSE_LEVEL'] = config.get("VERBOSE_LEVEL", default_verbose)

        # Update Modbus registers VIA THE HANDLER
        if modbus_handler:
            modbus_handler.set_hreg(REG_VERBOSE, internal_state['VERBOSE_LEVEL'])
            modbus_handler.set_hreg(REG_ENC_MODE, 0 if internal_state['encoder_output_mode'] == "step" else 1)
            modbus_handler.set_ireg(REG_OFFSET_STEPS, internal_state['encoder_offset_steps'])
        else: # Fallback if handler not passed (e.g. during very early init, though unlikely for this function)
            slave_registers['HREGS']['verbosity_level']['val'] = internal_state['VERBOSE_LEVEL']
            slave_registers['HREGS']['encoder_mode']['val'] = 0 if internal_state['encoder_output_mode'] == "step" else 1
            slave_registers['IREGS']['offset_steps']['val'] = internal_state['encoder_offset_steps']


        print_verbose(f"[INFO] Loaded config: Offset={internal_state['encoder_offset_steps']}, Mode='{internal_state['encoder_output_mode']}', Verbose={internal_state['VERBOSE_LEVEL']}", 0)

    except Exception as e:
        print_verbose(f"[WARNING] Failed to load config ({e}). Using defaults derived from initial register state.", 0)
        # Set internal_state to defaults derived from initial Modbus state
        internal_state['encoder_offset_steps'] = default_offset
        internal_state['encoder_output_mode'] = default_enc_mode_str
        internal_state['VERBOSE_LEVEL'] = default_verbose
        # No need to set modbus_handler registers here, as they retain their initial values from setup_registers


def update_input_registers(modbus_handler, rpm=None, vfd_status=None, enc_steps=None, enc_deg=None, fault=None, homing=None, offset=None, max_rpm=None): # ADD modbus_handler
    """ Safely updates the Modbus slave input registers VIA THE MODBUS HANDLER. """
    if not modbus_handler:
        print_verbose("[ERROR] update_input_registers: modbus_handler is None!", 0)
        return

    try:
        if rpm is not None:
            modbus_handler.set_ireg(REG_CURRENT_RPM, int(rpm * 10))
        if vfd_status is not None:
            modbus_handler.set_ireg(REG_VFD_STATUS, vfd_status)
        if enc_steps is not None:
            modbus_handler.set_ireg(REG_ENC_POS_STEPS, enc_steps)
        if enc_deg is not None:
            modbus_handler.set_ireg(REG_ENC_POS_DEG, int(enc_deg * 100))
        if fault is not None:
            modbus_handler.set_ireg(REG_FAULT_FLAG, 1 if fault else 0)
        if homing is not None:
            modbus_handler.set_ireg(REG_HOMING_FLAG, 1 if homing else 0)
        if offset is not None: # This is typically set by load_configuration or calibrate callback
            modbus_handler.set_ireg(REG_OFFSET_STEPS, offset)
        if max_rpm is not None:
            modbus_handler.set_ireg(REG_MAX_RPM, max_rpm) # Should be int
    except Exception as e:
        print_verbose(f"[ERROR] Failed to update IREG via modbus_handler: {e}", 0)
