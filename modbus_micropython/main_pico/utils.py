# utils.py

import time
import json
from machine import UART, Pin

# Global Variables for shared state
state = {
    'VERBOSE_LEVEL': 1,
    'encoder_position': 0,
    'encoder_offset': 0,
    'encoder_output_mode': "deg",
    'homing_completed': False,
    'encoder_zero_offset': 0,
}

CONFIG_FILE = "config.json"  # Configuration file to save encoder offset

# UART1 for RS485 communication
UART1_ID = 1  # UART1 for user commands
USER_CMD_TX_PIN = 4  # GPIO4 (TX) for UART1
USER_CMD_RX_PIN = 5  # GPIO5 (RX) for UART1
DE_RE_UART1_PIN = Pin(3, Pin.OUT)  # GPIO3 for DE/RE control
DE_RE_UART1_PIN.value(0)  # Initially enable receiver

user_cmd_uart = UART(UART1_ID, baudrate=115200, tx=USER_CMD_TX_PIN, rx=USER_CMD_RX_PIN)
user_cmd_uart.init(bits=8, parity=None, stop=1)  # Use 1 stop bit for higher baud rates

def print_verbose(message, level, override=False):
    """Prints messages according to the verbosity level."""
    if override or (state['VERBOSE_LEVEL'] >= level and state['VERBOSE_LEVEL'] > 0):
        # Also print to USB serial for debugging
        print(message)

        # Send message over UART1 (RS485) line by line
        lines = message.split('\n')
        for line in lines:
            line_to_send = line + '\n'
            DE_RE_UART1_PIN.value(1)  # Enable transmitter
            user_cmd_uart.write(line_to_send.encode('utf-8'))
            user_cmd_uart.flush()  # Wait for transmission to complete
            time.sleep(0.02)  # Small delay to ensure data is sent
            DE_RE_UART1_PIN.value(0)  # Enable receiver
            time.sleep(0.01)  # Small delay before sending next line

def show_manual():
    """Displays the instruction manual."""
    manual = """
========== INSTRUCTION MANUAL ==========
Available Commands:
- start [rpm]: Start the motor with the specified RPM.
- stop: Stop the motor.
- reverse [rpm]: Reverse the motor direction with the specified RPM.
- set_speed [rpm]: Change the motor speed reference to the specified RPM.
- read_speed: Read and display the current motor speed.
- status: Read and display the current inverter status.
- reset_fault: Reset inverter faults.
- set_verbose [0-3]: Set verbosity level (0 = no output, 1 = encoder only, 2 = motor info, 3 = all info).
- set_encoder_output [step|deg]: Set encoder output format.
- calibrate: Set current encoder position as zero and save offset.
- read_offset: Read and display the current encoder offset.
- read_max_rpm: Read the maximum RPM from the inverter.
- help: Display this instruction manual.
- exit: Exit the program.
- test: Execute the default test sequence.
===========================================
"""
    print_verbose(manual, 0, override=True)

def save_configuration():
    """Saves the encoder offset to a configuration file."""
    config = {
        "encoder_offset": state['encoder_offset']
    }
    try:
        with open(CONFIG_FILE, 'w') as f:
            json.dump(config, f)
        print_verbose("[INFO] Configuration saved.", 0)
    except Exception as e:
        print_verbose(f"[ERROR] Failed to save configuration: {e}", 0)

def load_configuration():
    """Loads the encoder offset from the configuration file."""
    try:
        with open(CONFIG_FILE, 'r') as f:
            config = json.load(f)
        state['encoder_offset'] = config.get("encoder_offset", 0)
        print_verbose(f"[INFO] Loaded encoder offset from configuration: {state['encoder_offset']}", 0)
    except FileNotFoundError:
        print_verbose("[WARNING] Configuration file not found. Using default settings.", 0)
        state['encoder_offset'] = 0
    except Exception as e:
        print_verbose(f"[ERROR] Failed to load configuration: {e}", 0)
        state['encoder_offset'] = 0