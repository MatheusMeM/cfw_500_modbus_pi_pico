# main_pico/main.py - Refactored for Modularity & Modbus

import time
import sys
import uasyncio as asyncio
from machine import Pin, UART
from cfw500_modbus import CFW500Modbus # Import class directly
from encoder_module import initialize_encoder
from command_processor import process_modbus_commands # Import the new processor
from utils import (
    print_verbose, show_manual, load_configuration, save_configuration,
    internal_state, slave_registers, update_input_registers,
    REG_CMD, REG_TARGET_RPM, REG_VERBOSE, REG_ENC_MODE, REG_OFFSET_STEPS,
    REG_FAULT_FLAG, REG_HOMING_FLAG, REG_MAX_RPM # Added REG_MAX_RPM
)
from umodbus.serial import Serial as ModbusRTUSerial # Renamed for clarity, handles serial comms
from umodbus.modbus import Modbus # Base class for register handling

# --- Constants ---
# Network 1: Relay (Master) <-> Main (Slave) - UART1
SLAVE_UART_ID = 1
SLAVE_BAUDRATE = 19200 # Match Relay Pico - REDUCED for testing
SLAVE_TX_PIN_NUM = 4  # GPIO4
SLAVE_RX_PIN_NUM = 5  # GPIO5
SLAVE_DE_RE_PIN_NUM = 6 # GPIO6
MAIN_PICO_SLAVE_ADDR = 1

# Network 2: Main (Master) <-> VFD (Slave) - UART0
VFD_UART_ID = 0
VFD_BAUDRATE = 19200 # Keep original VFD baudrate
VFD_TX_PIN_NUM = 0  # GPIO0
VFD_RX_PIN_NUM = 1  # GPIO1
VFD_DE_RE_PIN_NUM = 2 # GPIO2
VFD_SLAVE_ADDRESS = 1 # Original VFD slave address

# Hardware Pins
LED_PIN = 25
ZERO_ENDSTOP_PIN_NUM = 18
RELAY_PIN1_NUM = 20
RELAY_PIN2_NUM = 21

# Timing
STATUS_REQUEST_INTERVAL_MS = 5000 # VFD status check interval
RELAY_CONTROL_INTERVAL_MS = 1000 # Relay update interval
MODBUS_SLAVE_POLL_INTERVAL_MS = 200 # How often to check for slave requests (Increased from 10ms)
MAIN_LOOP_SLEEP_MS = 20 # Main loop sleep

# --- Hardware Initialization ---
led = Pin(LED_PIN, Pin.OUT)
led.on() # Turn LED on permanently

zero_pin = Pin(ZERO_ENDSTOP_PIN_NUM, Pin.IN, Pin.PULL_UP)
relay_pin1 = Pin(RELAY_PIN1_NUM, Pin.OUT, value=0)
relay_pin2 = Pin(RELAY_PIN2_NUM, Pin.OUT, value=0)

# --- Modbus Initialization ---
# Network 1: Modbus Slave (for Relay Master)
slave_de_re_pin = Pin(SLAVE_DE_RE_PIN_NUM, Pin.OUT)
# 1. Create the Serial interface instance for the slave UART
slave_serial_itf = ModbusRTUSerial(
    uart_id=SLAVE_UART_ID,
    baudrate=SLAVE_BAUDRATE,
    pins=(SLAVE_TX_PIN_NUM, SLAVE_RX_PIN_NUM),
    ctrl_pin=SLAVE_DE_RE_PIN_NUM
)
# 2. Create the Modbus instance, passing the Serial interface
modbus_slave_handler = Modbus(
    itf=slave_serial_itf,
    addr_list=[MAIN_PICO_SLAVE_ADDR], # List of slave addresses this instance responds to
)

# Network 2: Modbus Master (for VFD) - Define vfd_master EARLY
vfd_de_re_pin = Pin(VFD_DE_RE_PIN_NUM, Pin.OUT) # DE/RE pin object for VFD master
vfd_master = CFW500Modbus( # Instantiate the VFD control class
    uart_id=VFD_UART_ID,
    tx_pin=VFD_TX_PIN_NUM,
    rx_pin=VFD_RX_PIN_NUM,
    de_re_pin=VFD_DE_RE_PIN_NUM, # Pass pin number
    slave_address=VFD_SLAVE_ADDRESS
)
print_verbose("[INFO] Modbus Master (UART0 for VFD) initialized.", 2)


# --- Modbus Register Callbacks ---
# Define the callback function BEFORE it's assigned
def handle_command_register_write(reg_type, address, val):
    """
    Callback executed by umodbus when the command register (HREG 100) is written.
    Triggers command processing based on the written value.
    NOTE: This runs in the context of the modbus_slave_poll_task.
          Keep it relatively quick or schedule longer actions.
    """
    # global vfd_master # vfd_master is now defined globally before this function

    # The library might pass a list even for single register writes
    # If multiple registers are written (e.g., CMD and RPM), val might be a list
    print_verbose(f"[DEBUG CB] Callback received: reg_type={reg_type}, address={address}, val={val}", 3)
    command_to_process = 0
    target_rpm_val = 0 # Default RPM

    if isinstance(val, list):
        command_to_process = val[0] # First value is the command
        if len(val) > 1:
            # ASSUMPTION: If val is a list > 1 element, the second element is the RPM
            # This depends on the Relay Pico writing CMD and RPM together using
            # function code 16 (Write Multiple Registers).
            target_rpm_val = val[1]
            print_verbose(f"[DEBUG CB] Extracted RPM from multi-register write: {target_rpm_val}", 3)
        else:
            # Only command was written, try reading RPM from our dictionary (might be stale)
            try:
                target_rpm_val = slave_registers['HREGS']['target_rpm']['val']
                print_verbose(f"[DEBUG CB] Read RPM from dictionary (single write): {target_rpm_val}", 3)
            except Exception as e:
                print_verbose(f"[ERROR CB] Failed to read target RPM from dict: {e}", 0)
                target_rpm_val = 0
    else:
        # Single value write (likely just the command)
        command_to_process = val
        # Try reading RPM from our dictionary (might be stale)
        try:
            target_rpm_val = slave_registers['HREGS']['target_rpm']['val']
            print_verbose(f"[DEBUG CB] Read RPM from dictionary (single write): {target_rpm_val}", 3)
        except Exception as e:
            print_verbose(f"[ERROR CB] Failed to read target RPM from dict: {e}", 0)
            target_rpm_val = 0

    print_verbose(f"[DEBUG CB] Command Register Write Detected: Value={command_to_process}", 3)

    if command_to_process != 0:
        # RPM value determined above

        print_verbose(f"[DEBUG CB] Processing Command: {command_to_process}, RPM: {target_rpm_val}", 3)

        # --- Execute Command ---
        try:
            if command_to_process == 1: # START
                vfd_master.start_motor(target_rpm_val)
                print_verbose(f"[ACTION CB] Motor START processed (RPM: {target_rpm_val}).", 0)
            elif command_to_process == 2: # STOP
                vfd_master.stop_motor()
                print_verbose(f"[ACTION CB] Motor STOP processed.", 0)
            elif command_to_process == 3: # REVERSE
                vfd_master.reverse_motor(target_rpm_val)
                print_verbose(f"[ACTION CB] Motor REVERSE processed (RPM: {target_rpm_val}).", 0)
            elif command_to_process == 4: # RESET_FAULT
                vfd_master.reset_fault()
                print_verbose(f"[ACTION CB] VFD Fault RESET processed.", 0)
            elif command_to_process == 5: # CALIBRATE
                current_pos_rel_home = internal_state['encoder_raw_position'] - internal_state['encoder_zero_offset']
                internal_state['encoder_offset_steps'] = current_pos_rel_home
                update_input_registers(offset=internal_state['encoder_offset_steps'])
                save_configuration()
                print_verbose(f"[ACTION CB] Encoder CALIBRATE processed. New offset: {internal_state['encoder_offset_steps']}", 0)

            # Acknowledge by resetting the command register *in our dictionary*
            slave_registers['HREGS']['command']['val'] = 0
            internal_state['last_written_cmd'] = 0

        except Exception as e:
            print_verbose(f"[ERROR CB] Error processing command {command_to_process}: {e}", 0)
    else:
        # Command 0 written, potentially an acknowledgement clear by master.
        print_verbose(f"[DEBUG CB] Command Register cleared (Value=0 received).", 3)
        slave_registers['HREGS']['command']['val'] = 0
        internal_state['last_written_cmd'] = 0

# --- Assign Callbacks BEFORE Setup ---
slave_registers['HREGS']['command']['on_set_cb'] = handle_command_register_write
print_verbose("[INFO] Command register callback assigned to slave_registers dict.", 2)

# 3. Setup the registers using the map from utils (NOW includes the callback)
modbus_slave_handler.setup_registers(registers=slave_registers)
print_verbose("[INFO] Modbus Slave Handler (UART1 for Relay) initialized and registers set up.", 2)


# --- Async Events ---
endstop_event = asyncio.Event()

# --- Homing ---
def endstop_triggered_callback_irq(pin):
    # --- IRQ Handler --- Keep simple! ---
    endstop_event.set()
    # --- End IRQ Handler ---

async def homing(cfw500_master_obj): # Pass the VFD master object
    """Performs the homing routine."""
    endstop_event.clear()
    print_verbose("[INFO] Starting homing routine...", 0)
    internal_state['homing_completed'] = False
    update_input_registers(homing=False) # Update Modbus register

    homing_speed_rpm = 1000 # Use a reasonable default
    try:
        cfw500_master_obj.start_motor(homing_speed_rpm)
        print_verbose(f"[ACTION] Motor started at {homing_speed_rpm} RPM for homing.", 0)
    except Exception as e:
        print_verbose(f"[ERROR] Homing: Failed to start motor: {e}", 0)
        return # Abort homing

    # Attach IRQ
    zero_pin.irq(trigger=Pin.IRQ_FALLING, handler=endstop_triggered_callback_irq)
    print_verbose("[DEBUG] Homing: Waiting for endstop event...", 3)

    try:
        # Wait for the event with a timeout
        await asyncio.wait_for_ms(endstop_event.wait(), 30000) # 30 second timeout
        print_verbose("[DEBUG] Homing: Endstop event received.", 3)

        # Stop the motor repeatedly
        stop_success = False
        stop_duration_ms = 500
        start_time = time.ticks_ms()
        print_verbose(f"[DEBUG] Homing: Attempting motor stop for {stop_duration_ms}ms...", 3)
        while time.ticks_diff(time.ticks_ms(), start_time) < stop_duration_ms:
            try:
                cfw500_master_obj.stop_motor()
                stop_success = True
            except Exception as e:
                print_verbose(f"[ERROR] Homing: Failed to send stop command: {e}", 0)
                stop_success = False
                break # Exit loop on error
            await asyncio.sleep_ms(50) # Yield briefly

        if stop_success:
            print_verbose("[ACTION] Homing: Motor stop command sent.", 0)
            # Set zero offset based on raw encoder position at trigger
            internal_state['encoder_zero_offset'] = internal_state['encoder_raw_position']
            print_verbose(f"[DEBUG] Homing: Encoder zero offset set to {internal_state['encoder_zero_offset']}", 3)
            internal_state['homing_completed'] = True
            update_input_registers(homing=True) # Update Modbus register
            print_verbose("[INFO] Homing complete.", 0)
        else:
            print_verbose("[WARNING] Homing: Failed to send stop command after endstop.", 0)
            internal_state['homing_completed'] = False # Mark as not completed
            update_input_registers(homing=False)

    except asyncio.TimeoutError:
        print_verbose("[ERROR] Homing: Timed out waiting for endstop.", 0)
        # Ensure motor is stopped if timeout occurs
        try:
            cfw500_master_obj.stop_motor()
            print_verbose("[ACTION] Homing Timeout: Motor stop command sent.", 0)
        except Exception as e:
            print_verbose(f"[ERROR] Homing Timeout: Failed to send stop command: {e}", 0)
        internal_state['homing_completed'] = False
        update_input_registers(homing=False)

    finally:
        # Detach IRQ regardless of outcome
        zero_pin.irq(handler=None)
        print_verbose("[DEBUG] Homing: Endstop IRQ detached.", 3)


# --- Background Tasks ---
async def vfd_status_request_task(cfw500_master_obj): # Pass VFD master object
    """Periodically reads VFD status and updates internal state and Modbus registers."""
    while True:
        try:
            fault = cfw500_master_obj.check_fault()
            status_p0680 = cfw500_master_obj.read_p0680()
            current_rpm = cfw500_master_obj.read_current_speed()

            if fault is not None:
                internal_state['fault_detected'] = fault
                update_input_registers(fault=fault)
                if fault and internal_state['VERBOSE_LEVEL'] >= 1:
                     print_verbose("[ALERT] VFD Fault Detected!", 1)
            else:
                 print_verbose("[WARNING] Failed to read VFD fault status.", 1)

            if status_p0680 is not None:
                 update_input_registers(vfd_status=status_p0680)
                 if internal_state['VERBOSE_LEVEL'] >= 3:
                      print_verbose(f"[DEBUG] VFD Status P0680: 0x{status_p0680:04X}", 3)
            else:
                 print_verbose("[WARNING] Failed to read VFD status P0680.", 1)

            if current_rpm is not None:
                 update_input_registers(rpm=current_rpm)
                 if internal_state['VERBOSE_LEVEL'] >= 2:
                      print_verbose(f"[INFO] VFD Speed: {current_rpm:.1f} RPM", 2)
            else:
                 print_verbose("[WARNING] Failed to read VFD current speed.", 1)

        except Exception as e:
            print_verbose(f"[ERROR] VFD Status Task Error: {e}", 0)

        await asyncio.sleep_ms(STATUS_REQUEST_INTERVAL_MS)

async def relay_control_task():
    """Controls relays based on internal fault state."""
    while True:
        if internal_state['fault_detected']:
            # Blink on fault
            relay_pin1.on()
            relay_pin2.on()
            await asyncio.sleep_ms(500)
            relay_pin1.off()
            relay_pin2.off()
            await asyncio.sleep_ms(500)
        else:
            # Solid on if no fault
            relay_pin1.on()
            relay_pin2.on()
            await asyncio.sleep_ms(RELAY_CONTROL_INTERVAL_MS) # Check state less frequently if ok

async def modbus_slave_poll_task(modbus_handler_obj): # Pass Modbus handler object
    """ Periodically processes incoming Modbus slave requests. """
    print_verbose("[DEBUG SLAVE POLL TASK] Starting...", 3) # Added start message
    debug_print_counter = 0
    # Print approx every 20 seconds (200ms interval * 100 polls)
    debug_print_interval = 100
    while True:
        try:
            print_verbose("[DEBUG SLAVE POLL TASK] Polling...", 3) # Added polling message
            # process() checks for incoming requests via the serial interface
            # and handles them based on the configured register map.
            # Callbacks (like handle_command_register_write) are executed within process()
            result = modbus_handler_obj.process()
            # ADDED DEBUG: Check register value immediately after process() - Reduced Frequency
            if internal_state['VERBOSE_LEVEL'] >= 3:
                debug_print_counter += 1
                if debug_print_counter >= debug_print_interval:
                    debug_print_counter = 0
                    cmd_val_after_process = slave_registers['HREGS']['command']['val']
                    # This print confirms the poll task is running, even if no commands received
                    print_verbose(f"[DEBUG SLAVE POLL] Task running (CMD Reg: {cmd_val_after_process})", 3)
        except Exception as e:
            print_verbose(f"[ERROR] Modbus Slave processing error: {e}", 0)

        # Sleep briefly to yield control and define polling rate
        await asyncio.sleep_ms(MODBUS_SLAVE_POLL_INTERVAL_MS)


# --- Main Application Logic ---
async def main():
    # Show manual locally
    show_manual()

    # Load configuration (loads into internal_state and updates slave_registers)
    load_configuration()
    # Ensure homing flag is initially false in register
    update_input_registers(homing=False)

    # Initialize Encoder (uses internal_state, updates slave_registers via callback)
    # initialize_encoder(16, 17) # Pins for encoder A, B - DISABLED FOR STEP 1.1 TEST
    print_verbose("[INFO TEST] Encoder Disabled.", 0)

    # --- Safety Stop ---
    try:
        print_verbose("[SAFETY] Ensuring VFD motor is stopped on startup...", 0)
        vfd_master.stop_motor() # Use the global vfd_master instance
        print_verbose("[SAFETY] Motor stop command sent. Waiting 5s...", 0)
        await asyncio.sleep(5.0)
        print_verbose("[SAFETY] Initial delay complete.", 1)
    except Exception as e:
        print_verbose(f"[ERROR] Failed initial VFD stop: {e}", 0)
    # --- End Safety Stop ---

    # Read initial VFD parameters if needed (e.g., max RPM)
    try:
        max_rpm = vfd_master.read_max_rpm()
        if max_rpm:
            print_verbose(f"[INFO] VFD Max RPM: {max_rpm}", 1)
            # Update the Modbus register with the read value
            update_input_registers(max_rpm=int(max_rpm)) # Use correct key 'max_rpm' and cast to int
        else:
            print_verbose("[WARNING] Failed to read VFD Max RPM.", 1)
    except Exception as e:
        print_verbose(f"[ERROR] Failed reading VFD Max RPM: {e}", 0)

    # --- Homing Sequence ---
    # Start background tasks first
    # status_task = asyncio.create_task(vfd_status_request_task(vfd_master)) # DISABLED FOR STEP 1.1 TEST
    # relay_task = asyncio.create_task(relay_control_task()) # DISABLED FOR STEP 1.1 TEST
    slave_poll_task = asyncio.create_task(modbus_slave_poll_task(modbus_slave_handler)) # Start ONLY the slave poll task
    print_verbose("[INFO TEST] VFD Status Task Disabled.", 0)
    print_verbose("[INFO TEST] Relay Control Task Disabled.", 0)
    await asyncio.sleep_ms(100) # Let slave poll task start

    # --- Homing Sequence DISABLED FOR STEP 1.1 TEST ---
    # Temporarily cancel background tasks during homing
    # print_verbose("[DEBUG] Cancelling background tasks for homing...", 3)
    # status_task.cancel()
    # relay_task.cancel()
    # slave_poll_task.cancel() # Also cancel slave polling during homing
    # try:
    #     await asyncio.sleep_ms(100) # Allow cancellations
    # except asyncio.CancelledError: pass
    # print_verbose("[DEBUG] Background tasks cancelled.", 3)
    #
    # await homing(vfd_master) # Perform homing, passing the vfd_master instance
    #
    # # Restart background tasks
    # print_verbose("[DEBUG] Restarting background tasks after homing...", 3)
    # status_task = asyncio.create_task(vfd_status_request_task(vfd_master))
    # relay_task = asyncio.create_task(relay_control_task())
    # slave_poll_task = asyncio.create_task(modbus_slave_poll_task(modbus_slave_handler)) # Pass the handler instance
    print_verbose("[INFO TEST] Homing Sequence Disabled.", 0)
    # --- End Homing Sequence ---

    print_verbose("[INFO] Main loop started. System operational (Minimal Tasks for Step 1.1).", 0)

    # --- Main Execution Loop ---
    while True:
        # Command processing is handled by the callback
        # Process settings changes from Modbus:
        await process_modbus_commands(vfd_master) # Pass the vfd_master instance

        # Main loop sleep (still needed to allow other tasks to run)
        await asyncio.sleep_ms(MAIN_LOOP_SLEEP_MS)

# --- Run ---
try:
    asyncio.run(main())
except KeyboardInterrupt:
    print_verbose("\n[INFO] Main Pico program interrupted by user.", 0)
finally:
    # Optional cleanup (e.g., ensure motor is stopped)
    try:
         vfd_master.stop_motor()
         print_verbose("[SAFETY] Motor stop attempted on exit.", 0)
    except Exception as e:
         print_verbose(f"[ERROR] Failed to stop motor on exit: {e}", 0)
    asyncio.new_event_loop() # Reset uasyncio state
