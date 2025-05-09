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
                # ADD DEBUG PRINT HERE
                if internal_state['VERBOSE_LEVEL'] >= 3:
                     print_verbose(f"[DEBUG VFD TASK] After fault update: FAULT_REG={slave_registers['IREGS']['fault_flag']['val']}", 3)
                if fault and internal_state['VERBOSE_LEVEL'] >= 1:
                     print_verbose("[ALERT] VFD Fault Detected!", 1)
            else:
                 print_verbose("[WARNING] Failed to read VFD fault status.", 1)

            if status_p0680 is not None:
                 update_input_registers(vfd_status=status_p0680)
                 # ADD DEBUG PRINT HERE
                 if internal_state['VERBOSE_LEVEL'] >= 3:
                      print_verbose(f"[DEBUG VFD TASK] After VFD status update: VFD_STATUS_REG={slave_registers['IREGS']['vfd_status']['val']}", 3)
                 if internal_state['VERBOSE_LEVEL'] >= 3: # Keep existing P0680 print
                      print_verbose(f"[DEBUG] VFD Status P0680: 0x{status_p0680:04X}", 3)
            else:
                 print_verbose("[WARNING] Failed to read VFD status P0680.", 1)

            if current_rpm is not None:
                 update_input_registers(rpm=current_rpm)
                 # ADD DEBUG PRINT HERE
                 if internal_state['VERBOSE_LEVEL'] >= 3:
                      print_verbose(f"[DEBUG VFD TASK] After RPM update: RPM_REG={slave_registers['IREGS']['current_rpm']['val']}", 3)
                 if internal_state['VERBOSE_LEVEL'] >= 2: # Keep existing speed print
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
    # debug_print_interval = 100 # Original general interval
    poll_debug_print_interval = 10 # Print specific debug every 10 * 200ms = 2 seconds

    while True:
        try:
            # ADD DEBUG PRINT HERE (before modbus_handler_obj.process())
            if internal_state['VERBOSE_LEVEL'] >= 3:
                # This specific print will occur more frequently due to poll_debug_print_interval
                if debug_print_counter % poll_debug_print_interval == 0:
                    rpm_val_before = slave_registers['IREGS']['current_rpm']['val']
                    offset_val_before = slave_registers['IREGS']['offset_steps']['val']
                    max_rpm_val_before = slave_registers['IREGS']['max_rpm']['val'] # Also check max_rpm
                    vfd_status_val_before = slave_registers['IREGS']['vfd_status']['val'] # And VFD status
                    print_verbose(f"[DEBUG SLAVE POLL] BEFORE process(): RPM={rpm_val_before}, OFFSET={offset_val_before}, MAX_RPM={max_rpm_val_before}, VFD_STAT={vfd_status_val_before:#06x}", 3)

            result = modbus_handler_obj.process()

            # General "Task running" print can remain less frequent
            if internal_state['VERBOSE_LEVEL'] >= 3:
                if debug_print_counter % 100 == 0: # e.g., every 100 * 200ms = 20 seconds
                    cmd_val_after = slave_registers['HREGS']['command']['val']
                    print_verbose(f"[DEBUG SLAVE POLL] Task running (CMD Reg: {cmd_val_after})", 3)
            debug_print_counter += 1
        except Exception as e:
            print_verbose(f"[ERROR] Modbus Slave processing error: {e}", 0)

        # Sleep briefly to yield control and define polling rate
        await asyncio.sleep_ms(MODBUS_SLAVE_POLL_INTERVAL_MS)


# --- Main Application Logic ---
async def main():
    # Show manual locally
    show_manual()

    # --- !!! START OF CHANGE: Re-enable config load !!! ---
    load_configuration() # Re-enable this
    print_verbose("[INFO] load_configuration() is ENABLED.", 1) # Keep this log
    # ADD DEBUG PRINT HERE
    print_verbose(f"[DEBUG MAIN] After load_config: OFFSET_REG={slave_registers['IREGS']['offset_steps']['val']}, RPM_REG={slave_registers['IREGS']['current_rpm']['val']}", 3)

    # The load_configuration in utils.py should now correctly load
    # VERBOSE_LEVEL, encoder_offset_steps, and encoder_output_mode,
    # and update the HREGS for verbosity/mode and IREG for offset.
    # It should NOT overwrite the other initial test IREGS (rpm, vfd_status, etc.)
    # if the config file doesn't have entries for them.
    # --- !!! END OF CHANGE !!! ---

    # Ensure other Input Registers start with a known state (or let utils.py initial values be)
    # If load_configuration might reset them, re-initialize them here for clarity before VFD task starts.
    # For now, we assume utils.py sets initial non-zero values that persist unless explicitly overwritten.
    update_input_registers(homing=False) # Explicitly set homing to false initially # Keep this

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

    # --- Read initial VFD parameters (e.g., max RPM) ---
    try:
        max_rpm_val = vfd_master.read_max_rpm() # This reads from VFD
        if max_rpm_val is not None:
            print_verbose(f"[INFO] VFD Max RPM read: {max_rpm_val}", 1) # Keep this log
            # --- !!! START OF CHANGE: Update Max RPM register with live data !!! ---
            update_input_registers(max_rpm=int(max_rpm_val)) # Update IREG_MAX_RPM
            # ADD DEBUG PRINT HERE
            print_verbose(f"[DEBUG MAIN] After max_rpm update: MAX_RPM_REG={slave_registers['IREGS']['max_rpm']['val']}", 3)
            # --- !!! END OF CHANGE !!! ---
        else:
            print_verbose("[WARNING] Failed to read VFD Max RPM.", 1)
            # Optionally, set a default or keep the test value if read fails
            # update_input_registers(max_rpm=slave_registers['IREGS']['max_rpm']['val']) # Keep test value
    except Exception as e:
        print_verbose(f"[ERROR] Failed reading VFD Max RPM: {e}", 0)


    # --- Start Background Tasks ---
    # --- !!! START OF CHANGE: Re-enable VFD Status Task !!! ---
    status_task = asyncio.create_task(vfd_status_request_task(vfd_master)) # RE-ENABLE THIS
    # relay_task = asyncio.create_task(relay_control_task()) # Keep relay task disabled
    slave_poll_task = asyncio.create_task(modbus_slave_poll_task(modbus_slave_handler))
    print_verbose("[INFO] VFD Status Task ENABLED.", 0)
    print_verbose("[INFO TEST] Relay Control Task Disabled.", 0)
    await asyncio.sleep_ms(100) # Let tasks start
    # --- !!! END OF CHANGE !!! ---

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
    print_verbose("[INFO TEST] Homing Sequence Disabled.", 0) # Keep homing disabled

    print_verbose("[INFO] Main loop started. Monitoring VFD status.", 0)

    loop_counter = 0
    while True:
        # process_modbus_commands is for HREGS, can be kept or commented if logs are too noisy
        # await process_modbus_commands(vfd_master)

        loop_counter += 1
        if loop_counter % 500 == 0: # Print roughly every 10 seconds (500 * 20ms)
            if internal_state['VERBOSE_LEVEL'] >= 3:
                print_verbose(f"[DEBUG MAIN LOOP] Still alive...", 3)
        
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
