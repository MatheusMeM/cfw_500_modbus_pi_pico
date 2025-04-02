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
led.on() # Turn LED on permanently

# Zero Endstop Detection on GPIO18
zero_pin = Pin(18, Pin.IN, Pin.PULL_UP)

# Event for signaling endstop trigger from IRQ
endstop_event = asyncio.Event()

# Initialize Relay Control Pins
relay_pin1 = Pin(20, Pin.OUT)
relay_pin2 = Pin(21, Pin.OUT)
relay_pin1.value(0)
relay_pin2.value(0)

# Removed led_blink_task to reduce scheduler load

# Zero Endstop Position Detection
def zero_position_callback(pin):
    """Callback function when the endstop (zero position) is detected."""
    print_verbose("[INFO] Zero position detected.", 1)
    # Do not reset the encoder position; only output a message

async def homing(cfw500):
    """Performs the homing routine to establish the zero position."""
    # Clear the event before starting
    endstop_event.clear()
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
    # endstop_triggered = False # Replaced by event

    def endstop_triggered_callback(pin):
        # --- IRQ Handler ---
        # Keep this extremely simple: just set the event.
        # Avoid prints, state updates, or Modbus calls here.
        endstop_event.set()
        # --- End IRQ Handler ---

    # Attach the IRQ handler
    zero_pin.irq(trigger=Pin.IRQ_FALLING, handler=endstop_triggered_callback)

    # Wait efficiently for the IRQ to signal via the event
    print_verbose("[DEBUG] Waiting for endstop event...", 0)
    await endstop_event.wait()
    print_verbose("[DEBUG] Endstop event received.", 0)

    # Stop the motor (This happens *after* the event is set and wait() returns)
    # Loop sending stop command for a short duration to ensure it gets through
    stop_success = False
    stop_duration = 0.5  # seconds
    start_time = time.ticks_ms()
    print_verbose(f"[DEBUG] Attempting to stop motor for {stop_duration}s...", 0)
    while time.ticks_diff(time.ticks_ms(), start_time) < int(stop_duration * 1000):
        try:
            cfw500.stop_motor()
            stop_success = True # Mark success if command sent at least once
            # Don't print repeatedly in loop, print status after
        except Exception as e:
            print_verbose(f"[ERROR] Failed to send stop motor command during loop: {e}", 0)
            stop_success = False
            break # Exit loop if error occurs
        await asyncio.sleep(0.05) # Yield briefly within the stop loop

    if stop_success:
        print_verbose("[ACTION] Motor stop command sent repeatedly after endstop event.", 0)
    else:
        print_verbose("[WARNING] Homing failed to send stop command.", 0)
        # Decide if we should exit homing or try to continue without offset update
        # return # Exit homing if stop fails - maybe allow continuing?

    # Detach the IRQ handler *after* attempting to stop
    zero_pin.irq(handler=None)
    print_verbose("[DEBUG] Endstop IRQ detached.", 0)

    # Update zero offset *after* stopping and detaching IRQ
    state['encoder_zero_offset'] = state['encoder_position'] # Assign directly, don't accumulate
    print_verbose(f"[DEBUG] Encoder zero offset set to: {state['encoder_zero_offset']}", 0)

    # Phase 2 (moving to offset) removed as per user request.
    # Homing now simply finds the endstop, stops, and sets the zero offset.

    print_verbose("[INFO] Homing complete (Phase 1 only).", 0)

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
        await asyncio.sleep(0.02) # Check for commands more frequently

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

    # Start the task to wait for serial communication and show the manual FIRST
    asyncio.create_task(await_serial_and_show_manual())
    await asyncio.sleep(0.1) # Give the manual task a chance to start printing

    # --- Safety Stop ---
    # Ensure motor is stopped at the beginning before any operation
    try:
        print_verbose("[SAFETY] Ensuring motor is stopped on startup...", 2)
        cfw500.stop_motor()
        print_verbose("[SAFETY] Motor stop command sent. Waiting 5s for deceleration...", 2)
        await asyncio.sleep(5.0) # Wait 5 seconds for VFD deceleration ramp
        print_verbose("[SAFETY] Initial 5s delay complete.", 2)
    except Exception as e:
        print_verbose(f"[ERROR] Failed to send initial stop command or wait: {e}", 2)
    # --- End Safety Stop ---

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

    # Zero Endstop Detection (IRQ handler defined in homing)
    # zero_pin.irq(trigger=Pin.IRQ_FALLING, handler=zero_position_callback) # Initial setup moved to homing

    # Load configuration
    load_configuration()

    # --- Temporarily disable background tasks for homing ---
    print_verbose("[DEBUG] Starting background tasks initially...", 3)
    status_task = asyncio.create_task(status_request_task(cfw500))
    relay_task = asyncio.create_task(relay_control_task())
    await asyncio.sleep(0.1) # Allow tasks to start

    print_verbose("[DEBUG] Cancelling background tasks before homing...", 3)
    status_task.cancel()
    relay_task.cancel()
    try: # Allow cancellations to process
        await asyncio.sleep(0.1)
    except asyncio.CancelledError:
        pass
    print_verbose("[DEBUG] Background tasks cancelled.", 3)
    # --- End temporary disable ---

    # Perform homing routine
    await homing(cfw500)

    # --- Restart background tasks after homing ---
    print_verbose("[DEBUG] Restarting background tasks after homing...", 3)
    status_task = asyncio.create_task(status_request_task(cfw500))
    relay_task = asyncio.create_task(relay_control_task())
    # --- End restart ---

    # Start main command reading task
    asyncio.create_task(read_user_input(cfw500))

    # Keep the main loop running (tasks run in background)
    print_verbose("[INFO] Main loop started. System operational.", 2)
    while True:
        await asyncio.sleep(1)

try:
    asyncio.run(main())
except KeyboardInterrupt:
    if state['VERBOSE_LEVEL'] >= 2:
        print_verbose("\n[INFO] Program interrupted by user.", 2)
finally:
    asyncio.new_event_loop()
