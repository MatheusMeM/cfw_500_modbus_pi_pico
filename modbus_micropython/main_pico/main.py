# main.py

import time
import sys
import uasyncio as asyncio
from machine import Pin, UART
from motor_control import initialize_cfw500
from encoder_module import initialize_encoder, encoder_callback
from utils import print_verbose, show_manual, load_configuration, state, user_cmd_uart, DE_RE_UART1_PIN
from commands import process_command

# Configuration for UART0 (Modbus RTU communication with VFD)
UART0_ID = 0  # UART0 for Modbus RTU
TX_PIN_NUM = 0  # GPIO0 (TX) for UART0
RX_PIN_NUM = 1  # GPIO1 (RX) for UART0
DE_RE_PIN = Pin(2, Pin.OUT)
DE_RE_PIN.value(0)  # Initially enable receiver
SLAVE_ADDRESS = 1  # Modbus slave address of the inverter

# LED Configuration
LED_PIN = 25  # On-board LED pin on Raspberry Pi Pico
led = Pin(LED_PIN, Pin.OUT)

# Zero Endstop Detection on GPIO18
zero_pin = Pin(18, Pin.IN, Pin.PULL_UP)

# Initialize Relay Control Pins
relay_pin1 = Pin(20, Pin.OUT)
relay_pin2 = Pin(21, Pin.OUT)
relay_pin1.value(0)
relay_pin2.value(0)

async def led_blink_task():
    """Task to blink the on-board LED."""
    while True:
        led.toggle()
        await asyncio.sleep(0.5)

# Zero Endstop Position Detection
def zero_position_callback(pin):
    """Callback function when the endstop (zero position) is detected."""
    print_verbose("[INFO] Zero position detected.", 1)
    # Do not reset the encoder position; only output a message

async def homing(cfw500):
    """Performs the homing routine to establish the zero position."""
    print_verbose("[INFO] Starting homing routine...", 0)

    # Move motor towards the endstop
    homing_speed_rpm = 1000 # Use 1000 RPM as confirmed
    try:
        cfw500.start_motor(homing_speed_rpm)
        print_verbose(f"[ACTION] Motor started at {homing_speed_rpm} RPM for homing.", 0)
    except Exception as e:
        print_verbose(f"[ERROR] Failed to start motor: {e}", 0)
        return

    # Wait until the endstop is triggered
    endstop_triggered = False

    def endstop_triggered_callback(pin):
        nonlocal endstop_triggered
        # Stop motor immediately upon trigger
        try:
            cfw500.stop_motor()
            print_verbose("[ACTION] Motor stop command sent from endstop IRQ.", 0)
        except Exception as e:
            print_verbose(f"[ERROR] Failed to stop motor from IRQ: {e}", 0)

        endstop_triggered = True # Set flag after attempting stop
        print_verbose("[INFO] Endstop triggered during homing.", 0)

        # Capture the encoder's hardware count as the zero offset
        state['encoder_zero_offset'] = state['encoder_position'] # Assign directly, don't accumulate
        print_verbose(f"[DEBUG] Encoder zero offset set to: {state['encoder_zero_offset']}", 0)

    # Wait in a loop until endstop is triggered
    while not endstop_triggered:
        await asyncio.sleep(0.1)

    # Stop the motor
    try:
        cfw500.stop_motor()
        print_verbose("[ACTION] Motor stopped after homing.", 0)
    except Exception as e:
        print_verbose(f"[ERROR] Failed to stop motor: {e}", 0)
        return

    # Remove the interrupt handler for the endstop to prevent re-triggering during normal operation
    zero_pin.irq(handler=None)

    # Calculate steps to move to align with saved encoder_offset
    steps_to_target = state['encoder_offset']  # Corrected calculation
    print_verbose(f"[DEBUG] Calculated steps to target: {steps_to_target}", 0)

    if steps_to_target == 0:
        print_verbose("[INFO] Already at target position.", 0)
    else:
        # Determine the direction and speed to move
        move_speed_rpm = 1000 # Use 1000 RPM for moving to offset
        if steps_to_target > 0:
            direction = 'forward'
            print_verbose(f"[DEBUG] Homing Phase 2: Moving FORWARD to target {steps_to_target} at {move_speed_rpm} RPM", 0)
            try:
                cfw500.start_motor(move_speed_rpm) # Explicit call
            except Exception as e:
                print_verbose(f"[ERROR] Failed to start motor forward: {e}", 0)
                return
        else: # steps_to_target <= 0
            direction = 'reverse'
            print_verbose(f"[DEBUG] Homing Phase 2: Moving REVERSE to target {steps_to_target} at {move_speed_rpm} RPM", 0)
            try:
                cfw500.reverse_motor(move_speed_rpm) # Explicit call
            except Exception as e:
                print_verbose(f"[ERROR] Failed to start motor reverse: {e}", 0)
                return

        # Log the action
        print_verbose(f"[ACTION] Moving motor {direction} to target position.", 0)

        # Move until we reach the target encoder position
        target_position = state['encoder_offset']  # Since encoder_position is adjusted
        print_verbose(f"[DEBUG] Target encoder position: {target_position}", 0)
        while True:
            current_position = state['encoder_position']
            print_verbose(f"[DEBUG] Current encoder position: {current_position}", 0)
            if (direction == 'forward' and current_position >= target_position) or \
               (direction == 'reverse' and current_position <= target_position):
                break
        await asyncio.sleep(0.02) # Check more frequently

        # Stop the motor
        try:
            cfw500.stop_motor()
            print_verbose("[ACTION] Motor stopped at target position.", 0)
        except Exception as e:
            print_verbose(f"[ERROR] Failed to stop motor: {e}", 0)
            return

    # Do not change encoder_offset during homing
    print_verbose("[INFO] Homing complete. Encoder offset remains unchanged.", 0)

    # Set homing_completed to True
    state['homing_completed'] = True
    print_verbose("[DEBUG] Homing completed. Encoder offset will now be applied.", 0)

    # Re-setup the endstop to only output a message when triggered, but not reset position
    zero_pin.irq(trigger=Pin.IRQ_FALLING, handler=zero_position_callback)

async def read_user_input(cfw500):
    rx_buffer = b''
    while True:
        if user_cmd_uart.any():
            DE_RE_UART1_PIN.value(0)  # Ensure receiver is enabled
            data = user_cmd_uart.read()
            if data:
                rx_buffer += data
                while b'\n' in rx_buffer:
                    line, rx_buffer = rx_buffer.split(b'\n', 1)
                    try:
                        user_input = line.decode('utf-8').strip()
                        if user_input:
                            print(f"[DEBUG] Received command: {user_input}")
                            should_continue = await process_command(user_input, cfw500)
                            if not should_continue:
                                return
                    except Exception as e:
                        print_verbose(f"[ERROR] Exception during decoding: {e}", 2)
        await asyncio.sleep(0.1)

async def status_request_task(cfw500):
    STATUS_REQUEST_INTERVAL = 5  # Interval in seconds
    while True:
        fault = cfw500.check_fault()
        state['fault_detected'] = fault  # Update the fault status in the shared state
        if state['VERBOSE_LEVEL'] >= 2:
            if fault:
                print_verbose("[ALERT] The inverter is in FAULT.", 2)
            else:
                print_verbose("[INFO] The inverter is NOT in fault.", 2)
        await asyncio.sleep(STATUS_REQUEST_INTERVAL)

async def relay_control_task():
    while True:
        if state['fault_detected']:
            # Blink the relay-controlled LEDs
            relay_pin1.value(1)
            relay_pin2.value(1)
            await asyncio.sleep(0.5)
            relay_pin1.value(0)
            relay_pin2.value(0)
            await asyncio.sleep(0.5)
        else:
            # No fault, keep the relay-controlled LEDs on
            relay_pin1.value(1)
            relay_pin2.value(1)
            await asyncio.sleep(1)  # Check the fault status every second

async def await_serial_and_show_manual():
    """Waits for serial communication to be established, then shows the manual."""
    # Wait for serial communication to be established
    print("[DEBUG] Waiting for serial communication to be established...")
    await asyncio.sleep(1)
    # Display the instruction manual
    show_manual()

async def main():
    # Initialize cfw500
    cfw500 = initialize_cfw500(UART0_ID, TX_PIN_NUM, RX_PIN_NUM, DE_RE_PIN, SLAVE_ADDRESS)

    # Start the task to wait for serial communication and show the manual
    asyncio.create_task(await_serial_and_show_manual())

    # Read the maximum RPM from the inverter
    max_rpm = cfw500.read_max_rpm()
    if max_rpm is not None and state['VERBOSE_LEVEL'] >= 2:
        print_verbose(f"[INFO] Maximum RPM Read: {max_rpm} RPM", 2)
    elif max_rpm is None and state['VERBOSE_LEVEL'] >= 2:
        print_verbose("[ERROR] Failed to read maximum RPM.", 2)

    # Read the serial interface status
    serial_state = cfw500.read_p0316()
    if state['VERBOSE_LEVEL'] >= 2:
        if serial_state == 0:
            print_verbose("[SERIAL] Serial Interface Inactive", 2)
        elif serial_state == 1:
            print_verbose("[SERIAL] Serial Interface Active", 2)
        elif serial_state == 2:
            print_verbose("[SERIAL] Watchdog Error on Serial Interface", 2)
        else:
            print_verbose(f"[SERIAL] Unknown State ({serial_state})", 2)

    # Check for faults
    fault = cfw500.check_fault()
    state['fault_detected'] = fault  # Initialize the fault status
    if state['VERBOSE_LEVEL'] >= 2:
        if fault:
            print_verbose("[ALERT] The inverter is in FAULT.", 2)
        else:
            print_verbose("[INFO] The inverter is NOT in fault.", 2)

    # Initialize the Encoder
    initialize_encoder(16, 17, lambda v, d: encoder_callback(v, d, state))

    # Zero Endstop Detection
    zero_pin.irq(trigger=Pin.IRQ_FALLING, handler=zero_position_callback)

    # Load configuration
    load_configuration()

    # Perform homing routine
    await homing(cfw500)

    # Start tasks
    asyncio.create_task(led_blink_task())
    asyncio.create_task(read_user_input(cfw500))
    asyncio.create_task(status_request_task(cfw500))
    asyncio.create_task(relay_control_task())  # Start the relay control task

    # Keep the main loop running
    while True:
        await asyncio.sleep(1)

try:
    asyncio.run(main())
except KeyboardInterrupt:
    if state['VERBOSE_LEVEL'] >= 2:
        print_verbose("\n[INFO] Program interrupted by user.", 2)
finally:
    asyncio.new_event_loop()
