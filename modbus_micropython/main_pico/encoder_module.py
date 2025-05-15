# encoder_module.py - Refactored for Modbus Slave

from machine import Pin
from encoder import Encoder
from utils import print_verbose, internal_state, update_input_registers, REG_ENC_POS_STEPS, REG_ENC_POS_DEG

# Constants for position limits
# Assuming 1000 PPR encoder with 4x decoding = 4000 steps/rev
# Adjust MAX_STEPS based on your actual encoder resolution and desired wrapping behavior
PULSES_PER_REVOLUTION = 1000 # Example: Pulses per revolution of the encoder
ENCODER_RESOLUTION = PULSES_PER_REVOLUTION * 4 # Steps per revolution after 4x decoding
MAX_STEPS = ENCODER_RESOLUTION # Wrap after one full revolution
MAX_DEGREES = 360  # Maximum degrees before reset

# Store encoder instance globally if needed elsewhere, otherwise keep local
_encoder_instance = None

def initialize_encoder(pin_a_num, pin_b_num):
    """Initializes the encoder driver and sets the callback."""
    global _encoder_instance
    pin_a = Pin(pin_a_num, Pin.IN, Pin.PULL_UP)
    pin_b = Pin(pin_b_num, Pin.IN, Pin.PULL_UP)

    _encoder_instance = Encoder(
        pin_x=pin_a,
        pin_y=pin_b,
        v=0,           # Initial hardware value
        div=1,         # Use raw steps (division factor = 1)
        vmin=None,     # No min limit
        vmax=None,     # No max limit
        mod=None,      # No modulus wrapping in the driver itself
        callback=encoder_callback, # Function to call on value change
        args=(),       # No extra args for callback
        delay=10       # Delay in ms between callbacks (rate limit)
    )
    print_verbose("[INFO] Encoder initialized.", 2)
    return _encoder_instance

def encoder_callback(value, delta):
    """
    Callback function executed by the Encoder driver when the value changes.
    Updates internal state and Modbus input registers.
    'value' is the raw hardware step count from the driver.
    'delta' is the change since the last callback.
    """
    STEPS_PER_DEGREE = MAX_STEPS / 360.0 # <<< ADD THIS LINE
    # Invert the raw value to correct the direction if necessary
    # Adjust this based on your wiring and desired positive direction
    inverted_value = -value
    internal_state['encoder_raw_position'] = inverted_value

    # Position relative to the homing switch trigger point
    position_relative_to_home = inverted_value - internal_state['encoder_zero_offset']

    # Apply the calibrated offset (in steps) only after homing is complete
    if not internal_state['homing_completed']:
        adjusted_position_steps = position_relative_to_home
    else:
        # Position relative to the calibrated zero point
        adjusted_position_steps = position_relative_to_home - internal_state['encoder_offset_steps']

    # --- Calculate and store continuous absolute degrees ---
    # This uses the raw position and subtracts both offsets to get steps from true zero
    absolute_steps_from_true_zero = internal_state['encoder_raw_position'] - \
                                    internal_state['encoder_zero_offset'] - \
                                    internal_state['encoder_offset_steps']
    internal_state['internal_absolute_degrees'] = absolute_steps_from_true_zero / STEPS_PER_DEGREE
    
    if internal_state['VERBOSE_LEVEL'] >= 3:
        print_verbose(f"[DEBUG ENC_CB] AbsDeg: {internal_state['internal_absolute_degrees']:.2f}", 3)

    # --- Optional: Handle Wrapping/Overflow ---
    # This simple modulo keeps the step count within one revolution range.
    # The previous dynamic offset adjustment on overflow was removed for clarity.
    # If multi-turn tracking without wrapping is needed, this logic needs changing.
    wrapped_position_steps = adjusted_position_steps % MAX_STEPS
    # Ensure positive result if needed (Python % can be negative)
    # wrapped_position_steps = (wrapped_position_steps + MAX_STEPS) % MAX_STEPS

    # Calculate degrees from the wrapped step position
    degrees = (wrapped_position_steps / MAX_STEPS) * 360.0
    # Ensure degrees are always positive [0, 360)
    degrees = (degrees + 360) % 360

    # --- Update Modbus Input Registers ---
    # Update registers with the final calculated values (relative to calibrated zero, potentially wrapped)
    update_input_registers(enc_steps=wrapped_position_steps, enc_deg=degrees)

    # --- Local Printing based on Verbosity and Mode ---
    # Read mode from internal state (which should reflect Modbus holding register)
    output_mode = internal_state['encoder_output_mode']
    verbose_level = internal_state['VERBOSE_LEVEL']

    # Print locally if verbosity allows (no rate limit needed for local print)
    if verbose_level >= 1:
        if output_mode == "deg":
            output = f"Encoder: {degrees:.2f} deg"
        else: # mode == "step"
            output = f"Encoder: {wrapped_position_steps} steps"
        # Print level 1 messages only at level 1 or 3 (to avoid clutter at level 2)
        if verbose_level == 1 or verbose_level == 3:
             print_verbose(output, 1) # Print locally

    # Note: The raw 'encoder_position' in internal_state is updated if needed elsewhere,
    # but Modbus registers now hold the final processed values.
    # internal_state['encoder_position'] = wrapped_position_steps # Or adjusted_position_steps if no wrapping desired
