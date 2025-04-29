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

# Define the structure for the Modbus slave registers
# Initialize with default/placeholder values
slave_registers = {
    # addr: value
    'holding': {
        REG_CMD: 0,
        REG_TARGET_RPM: 0,
        REG_VERBOSE: 1, # Default verbosity
        REG_ENC_MODE: 1, # Default 'deg'
    },
    'input': {
        REG_CURRENT_RPM: 0,
        REG_VFD_STATUS: 0,
        REG_ENC_POS_STEPS: 0,
        REG_ENC_POS_DEG: 0,
        REG_FAULT_FLAG: 0,
        REG_HOMING_FLAG: 0,
        REG_OFFSET_STEPS: 0,
    },
    'coils': {}, # Not used currently
    'discrete_inputs': {} # Not used currently
}

# Global Variables for shared internal state (distinct from Modbus registers)
# Some state might now be directly read/written via Modbus registers
internal_state = {
    'VERBOSE_LEVEL': 1, # Still needed for local print control
    'encoder_raw_position': 0, # Raw steps from encoder driver before offsets
    'encoder_offset_steps': 0, # Calibration offset
    'encoder_output_mode': "deg", # Internal tracking ('step' or 'deg')
    'homing_completed': False,
    'encoder_zero_offset': 0, # Offset determined during homing
    'fault_detected': False, # Internal flag, mirrored to Modbus register
    'last_written_cmd': 0, # Track last command written by master
    'last_written_rpm': 0, # Track last RPM written by master
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
        # Removed UART1 sending logic

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

def load_configuration():
    """Loads settings from the configuration file into internal_state."""
    try:
        with open(CONFIG_FILE, 'r') as f:
            config = json.load(f)
        # Load into internal_state
        internal_state['encoder_offset_steps'] = config.get("encoder_offset_steps", 0)
        internal_state['encoder_output_mode'] = config.get("encoder_output_mode", "deg")
        internal_state['VERBOSE_LEVEL'] = config.get("VERBOSE_LEVEL", 1)

        # Update corresponding slave registers after loading internal state
        slave_registers['holding'][REG_VERBOSE] = internal_state['VERBOSE_LEVEL']
        slave_registers['holding'][REG_ENC_MODE] = 0 if internal_state['encoder_output_mode'] == "step" else 1
        slave_registers['input'][REG_OFFSET_STEPS] = internal_state['encoder_offset_steps'] # Make offset readable

        print_verbose(f"[INFO] Loaded config: Offset={internal_state['encoder_offset_steps']}, Mode='{internal_state['encoder_output_mode']}', Verbose={internal_state['VERBOSE_LEVEL']}", 0) # Local print
    except FileNotFoundError:
        print_verbose("[WARNING] Configuration file not found. Using default settings.", 0) # Local print
        # Ensure defaults match internal_state definition
        internal_state['encoder_offset_steps'] = 0
        internal_state['encoder_output_mode'] = "deg"
        internal_state['VERBOSE_LEVEL'] = 1
        # Update registers with defaults
        slave_registers['holding'][REG_VERBOSE] = 1
        slave_registers['holding'][REG_ENC_MODE] = 1
        slave_registers['input'][REG_OFFSET_STEPS] = 0
    except Exception as e:
        print_verbose(f"[ERROR] Failed to load configuration: {e}", 0) # Local print
        # Ensure defaults match internal_state definition on error
        internal_state['encoder_offset_steps'] = 0
        internal_state['encoder_output_mode'] = "deg"
        internal_state['VERBOSE_LEVEL'] = 1
        # Update registers with defaults
        slave_registers['holding'][REG_VERBOSE] = 1
        slave_registers['holding'][REG_ENC_MODE] = 1
        slave_registers['input'][REG_OFFSET_STEPS] = 0

# Function to update Modbus input registers from internal state or direct values
def update_input_registers(rpm=None, vfd_status=None, enc_steps=None, enc_deg=None, fault=None, homing=None, offset=None):
    """ Safely updates the Modbus slave input registers. """
    global slave_registers
    if rpm is not None:
        slave_registers['input'][REG_CURRENT_RPM] = int(rpm * 10) # Scale RPM
    if vfd_status is not None:
        slave_registers['input'][REG_VFD_STATUS] = vfd_status
    if enc_steps is not None:
        # Handle potential signed value if necessary, assuming unsigned for now
        slave_registers['input'][REG_ENC_POS_STEPS] = enc_steps
    if enc_deg is not None:
        slave_registers['input'][REG_ENC_POS_DEG] = int(enc_deg * 100) # Scale Degrees
    if fault is not None:
        slave_registers['input'][REG_FAULT_FLAG] = 1 if fault else 0
    if homing is not None:
        slave_registers['input'][REG_HOMING_FLAG] = 1 if homing else 0
    if offset is not None:
        slave_registers['input'][REG_OFFSET_STEPS] = offset
