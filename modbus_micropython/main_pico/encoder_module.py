# encoder_module.py

from machine import Pin
from encoder import Encoder
from utils import print_verbose

# Constants for position limits
MAX_STEPS = 8000  # Maximum steps before reset
MAX_DEGREES = 360  # Maximum degrees before reset

def initialize_encoder(pin_a_num, pin_b_num, callback):
    pin_a = Pin(pin_a_num, Pin.IN, Pin.PULL_UP)
    pin_b = Pin(pin_b_num, Pin.IN, Pin.PULL_UP)

    encoder = Encoder(
        pin_x=pin_a,
        pin_y=pin_b,
        v=0,           # Initial value
        div=1,         # Division factor (adjust if needed)
        vmin=None,     # Optional minimum value
        vmax=None,     # Optional maximum value
        mod=None,      # Optional modulus
        callback=callback,  # Function to call on value change
        args=(),       # Arguments for callback
        delay=10       # Delay in ms between callbacks
    )
    return encoder

def encoder_callback(value, delta, state):
    # Invert the raw value to correct the direction
    inverted_value = -value
    encoder_position = inverted_value - state['encoder_zero_offset']

    if not state['homing_completed']:
        adjusted_position = encoder_position
    else:
        adjusted_position = encoder_position - state['encoder_offset']

    # Check for overflow and adjust encoder_offset if necessary
    if adjusted_position >= MAX_STEPS:
        state['encoder_offset'] += MAX_STEPS
        adjusted_position -= MAX_STEPS
    elif adjusted_position <= -MAX_STEPS:
        state['encoder_offset'] -= MAX_STEPS
        adjusted_position += MAX_STEPS

    if state['encoder_output_mode'] == "deg":
        # Calculate degrees and wrap using modulo
        raw_degrees = adjusted_position * (360 / MAX_STEPS)
        wrapped_degrees = raw_degrees % 360
        # Ensure the result is always positive (Python's % can return negative)
        degrees = (wrapped_degrees + 360) % 360
        output = f"Encoder Position: {degrees:.2f} degrees"
    else:
        output = f"Encoder Position: {adjusted_position} steps"

    # Rate limit encoder position printing
    state['encoder_print_counter'] += 1
    if state['encoder_print_counter'] >= 10: # Print roughly every 10 updates
        if state['VERBOSE_LEVEL'] == 1 or state['VERBOSE_LEVEL'] == 3:
            print_verbose(output, 1)
        state['encoder_print_counter'] = 0 # Reset counter

    # Update the shared state
    state['encoder_position'] = encoder_position
