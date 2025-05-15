# encoder_module.py - Refactored for Modbus Slave

from machine import Pin
from encoder import Encoder
from utils import (
    print_verbose, internal_state, update_input_registers,
    REG_ENC_POS_STEPS, REG_ENC_POS_DEG,
    PULSES_PER_REVOLUTION, ENCODER_RESOLUTION, MAX_STEPS, MAX_DEGREES, STEPS_PER_DEGREE
)

# Constants for position limits are now imported from utils.py

# Store encoder instance globally if needed elsewhere, otherwise keep local
_encoder_instance = None

def initialize_encoder(pin_a_num, pin_b_num, modbus_handler_instance):
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
        args=(modbus_handler_instance,), # Pass it as a tuple
        delay=10       # Delay in ms between callbacks (rate limit)
    )
    print_verbose("[INFO] Encoder initialized.", 2)
    return _encoder_instance

def encoder_callback(value, delta, modbus_handler):
    """
    Callback function executed by the Encoder driver when the value changes.
    Updates internal state and Modbus input registers.
    'value' is the raw hardware step count from the driver.
    'delta' is the change since the last callback.
    """
    # STEPS_PER_DEGREE is now imported from utils.py
    inverted_value = -value # Or 'value' depending on desired raw positive direction
    internal_state['encoder_raw_position'] = inverted_value

    # Absolute position relative to Z-pulse (Machine Zero)
    # After homing, if at Z-pulse, encoder_raw_position ~= encoder_zero_offset, so this is ~0.
    absolute_steps_from_machine_zero = internal_state['encoder_raw_position'] - internal_state['encoder_zero_offset']
    internal_state['internal_absolute_degrees'] = absolute_steps_from_machine_zero / STEPS_PER_DEGREE if STEPS_PER_DEGREE != 0 else 0

    # Position relative to User Zero (Calibrated Offset) for Modbus reporting
    # encoder_offset_steps is the delta from Z-pulse to the User Zero point.
    steps_from_user_zero = absolute_steps_from_machine_zero - internal_state['encoder_offset_steps']

    # MAX_STEPS is ENCODER_RESOLUTION (e.g. 8000 for a 2000PPR encoder)
    wrapped_report_steps = steps_from_user_zero
    if MAX_STEPS != 0: # Avoid division by zero if MAX_STEPS is somehow 0
      wrapped_report_steps = steps_from_user_zero % MAX_STEPS
      # Ensure positive result for modulo if MAX_STEPS is defined (Python % can be negative)
      # wrapped_report_steps = (wrapped_report_steps + MAX_STEPS) % MAX_STEPS # Alternative for always positive

    report_degrees = 0.0
    if MAX_STEPS != 0: # Avoid division by zero
        report_degrees = (wrapped_report_steps / MAX_STEPS) * 360.0
        # Ensure degrees are always positive [0, 360)
        report_degrees = (report_degrees + 360.0) % 360.0


    if internal_state['VERBOSE_LEVEL'] >= 3:
        print_verbose(f"[DEBUG ENC_CB] Raw: {internal_state['encoder_raw_position']}, Z_Off: {internal_state['encoder_zero_offset']}, Cal_Off: {internal_state['encoder_offset_steps']}", 3)
        print_verbose(f"[DEBUG ENC_CB] AbsStepsMachine0: {absolute_steps_from_machine_zero}, AbsDegMachine0: {internal_state['internal_absolute_degrees']:.2f}", 3)
        print_verbose(f"[DEBUG ENC_CB] StepsUser0: {steps_from_user_zero}, WrappedReportSteps: {wrapped_report_steps}, ReportDeg: {report_degrees:.2f}", 3)

    update_input_registers(modbus_handler, enc_steps=wrapped_report_steps, enc_deg=report_degrees)

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
