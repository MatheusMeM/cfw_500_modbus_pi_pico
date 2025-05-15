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
    # Add REG_CURRENT_RPM and REG_VFD_STATUS to the import list
    REG_CMD, REG_TARGET_RPM, REG_VERBOSE, REG_ENC_MODE, REG_OFFSET_STEPS,
    REG_FAULT_FLAG, REG_HOMING_FLAG, REG_MAX_RPM,
    REG_CURRENT_RPM, REG_VFD_STATUS, # These were missing for direct use in main.py
    STEPS_PER_DEGREE, MAX_STEPS # Added for homing function
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
STATUS_REQUEST_INTERVAL_MS = 1000 # VFD status check interval (Reduced for testing encoder updates)
RELAY_CONTROL_INTERVAL_MS = 1000 # Relay update interval
MODBUS_SLAVE_POLL_INTERVAL_MS = 200 # How often to check for slave requests (Increased from 10ms)
MAIN_LOOP_SLEEP_MS = 20 # Main loop sleep

# --- Homing Parameters ---
HOMING_SEARCH_RPM = 800      # RPM for the initial search for the Z-pulse
HOMING_CREEP_RPM = 600       # RPM for the slow final approach to the Z-pulse
HOMING_BACKUP_DEGREES = 20.0 # Degrees to back up past Z-pulse after initial find

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

    # Determine command_to_process and target_rpm_val
    if isinstance(val, list): # Typically from write_multiple_registers (e.g., CMD + RPM)
        command_to_process = val[0]
        if len(val) > 1:
            target_rpm_val = val[1]
            print_verbose(f"[DEBUG CB] Extracted RPM ({target_rpm_val}) from multi-register write.", 3)
        # else: # Command only in list, RPM should be fetched if needed for START/REVERSE
              # This case is less likely if Relay sends single for CMD_ONLY or multi for CMD+RPM
              # For START/REVERSE without explicit RPM, 'val' will be a single int.
    else: # Typically from write_single_register (e.g., just the command)
        command_to_process = val
        # target_rpm_val remains 0 initially if only command is sent.
        # We will fetch it from HREG_TARGET_RPM if needed for START/REVERSE.
        print_verbose(f"[DEBUG CB] Single value write for command: {command_to_process}", 3)

    print_verbose(f"[DEBUG CB] Command Register Write: Value={command_to_process}, Explicit Target RPM={target_rpm_val}", 3)

    if command_to_process == 0: # SET_SPEED
        # This logic assumes target_rpm_val was explicitly sent if 'val' was a list [0, rpm].
        # If 'val' was just '0' (unlikely for set_speed from Relay), target_rpm_val would be 0 here.
        # The Relay Pico's set_speed always sends [0, rpm].
        if not isinstance(val, list) or len(val) < 2 : # Should not happen with current Relay set_speed
             print_verbose(f"[WARNING CB] SET_SPEED (Cmd 0) received without explicit RPM in payload. Using stored Target RPM.", 1)
             target_rpm_val = modbus_slave_handler.get_hreg(REG_TARGET_RPM) # Get stored RPM

        try:
            if target_rpm_val > 0:
                vfd_master.set_speed_reference(target_rpm_val)
                print_verbose(f"[ACTION CB] SET_SPEED: VFD speed ref updated to {target_rpm_val} RPM.", 0)
            else:
                print_verbose(f"[INFO CB] SET_SPEED with RPM <= 0. Speed ref not sent.", 1)
            modbus_slave_handler.set_hreg(REG_TARGET_RPM, target_rpm_val) # Ensure HREG is updated
        except Exception as e:
            print_verbose(f"[ERROR CB] Error processing SET_SPEED (Cmd 0): {e}", 0)
        # Clear command register
        try:
            modbus_slave_handler.set_hreg(REG_CMD, 0)
            internal_state['last_written_cmd'] = 0
            print_verbose(f"[DEBUG CB] REG_CMD (for SET_SPEED) cleared.", 3)
        except Exception as e:
            print_verbose(f"[ERROR CB] Failed to clear REG_CMD for SET_SPEED: {e}", 0)

    elif command_to_process in [1, 3]: # START (1) or REVERSE (3)
        # --- !!! START OF MODIFICATION FOR MAIN PICO 'start/reverse' COMMAND !!! ---
        rpm_to_use = 0
        if isinstance(val, list) and len(val) > 1: # RPM explicitly provided with START/REVERSE
            rpm_to_use = val[1] # This was target_rpm_val
            modbus_slave_handler.set_hreg(REG_TARGET_RPM, rpm_to_use) # Store this explicit RPM
            print_verbose(f"[DEBUG CB] START/REVERSE with explicit RPM: {rpm_to_use}", 3)
        else: # No explicit RPM with START/REVERSE (val was just the command code)
            rpm_to_use = modbus_slave_handler.get_hreg(REG_TARGET_RPM) # Use stored Target RPM
            print_verbose(f"[DEBUG CB] START/REVERSE using stored Target RPM: {rpm_to_use}", 3)

        if rpm_to_use <= 0: # If stored RPM is also 0 or invalid, use a safe default
            rpm_to_use = 1000 # Safe default if no valid RPM found
            print_verbose(f"[WARNING CB] No valid RPM for START/REVERSE, using default: {rpm_to_use}", 1)
            modbus_slave_handler.set_hreg(REG_TARGET_RPM, rpm_to_use) # Also update HREG with this default

        print_verbose(f"[DEBUG CB] Processing Action Command: {command_to_process}, Effective RPM: {rpm_to_use}", 3)
        # --- !!! END OF MODIFICATION FOR MAIN PICO 'start/reverse' COMMAND !!! ---
        try:
            if command_to_process == 1: # START
                vfd_master.start_motor(rpm_to_use)
                print_verbose(f"[ACTION CB] Motor START processed (RPM: {rpm_to_use}).", 0)
            elif command_to_process == 3: # REVERSE
                vfd_master.reverse_motor(rpm_to_use)
                print_verbose(f"[ACTION CB] Motor REVERSE processed (RPM: {rpm_to_use}).", 0)
        except Exception as e:
            print_verbose(f"[ERROR CB] Error processing START/REVERSE: {e}", 0)
        # Clear command register
        try:
            modbus_slave_handler.set_hreg(REG_CMD, 0)
            internal_state['last_written_cmd'] = 0
            print_verbose(f"[DEBUG CB] Action Cmd {command_to_process}: REG_CMD cleared.", 3)
        except Exception as e:
            print_verbose(f"[ERROR CB] Failed to clear REG_CMD for action {command_to_process}: {e}", 0)

    elif command_to_process in [2, 4, 5]: # STOP, RESET_FAULT, CALIBRATE (no RPM directly used for action)
        rpm_from_payload = 0 # Default if not multi-register
        if isinstance(val, list) and len(val) > 1:
             rpm_from_payload = val[1] # This would be target_rpm_val if sent
             modbus_slave_handler.set_hreg(REG_TARGET_RPM, rpm_from_payload) # Update HREG_TARGET_RPM if sent
             print_verbose(f"[DEBUG CB] HREG_TARGET_RPM updated to {rpm_from_payload} from command {command_to_process} payload.", 3)

        print_verbose(f"[DEBUG CB] Processing Action Command: {command_to_process} (RPM in payload if any: {rpm_from_payload})", 3)
        try:
            if command_to_process == 2: # STOP
                vfd_master.stop_motor()
                print_verbose(f"[ACTION CB] Motor STOP processed.", 0)
            elif command_to_process == 4: # RESET_FAULT
                vfd_master.reset_fault()
                print_verbose(f"[ACTION CB] VFD Fault RESET processed.", 0)
            elif command_to_process == 5: # CALIBRATE
                current_pos_rel_home = internal_state['encoder_raw_position'] - internal_state['encoder_zero_offset']
                internal_state['encoder_offset_steps'] = current_pos_rel_home
                update_input_registers(modbus_slave_handler, offset=internal_state['encoder_offset_steps'])
                save_configuration()
                print_verbose(f"[ACTION CB] Encoder CALIBRATE processed. New offset: {internal_state['encoder_offset_steps']}", 0)
        except Exception as e:
            print_verbose(f"[ERROR CB] Error processing command {command_to_process}: {e}", 0)
        # Clear command register
        try:
            modbus_slave_handler.set_hreg(REG_CMD, 0)
            internal_state['last_written_cmd'] = 0
            print_verbose(f"[DEBUG CB] Action Cmd {command_to_process}: REG_CMD cleared.", 3)
        except Exception as e:
            print_verbose(f"[ERROR CB] Failed to clear REG_CMD for action {command_to_process}: {e}", 0)

# --- Assign Callbacks BEFORE Setup ---
slave_registers['HREGS']['command']['on_set_cb'] = handle_command_register_write
print_verbose("[INFO] Command register callback assigned to slave_registers dict.", 2)

# 3. Setup the registers using the map from utils (NOW includes the callback)
# IMPORTANT: slave_registers global dict from utils.py is used here for initial setup.
modbus_slave_handler.setup_registers(registers=slave_registers)
print_verbose("[INFO] Modbus Slave Handler (UART1 for Relay) initialized and registers set up.", 2)


# --- Async Events ---
endstop_event = asyncio.Event()

# --- Homing ---
def endstop_triggered_callback_irq(pin):
    # --- IRQ Handler --- Keep simple! ---
    endstop_event.set()
    # --- End IRQ Handler ---

async def homing(cfw500_master_obj):
    """
    Performs the multi-phase homing routine to find the encoder's Z-pulse.
    Controls the VFD to move the motor through different stages for reliable detection.
    """
    # Ensure globals are accessible. These should be defined at the module level.
    # global modbus_slave_handler, internal_state, endstop_event, zero_pin
    # (No 'global' needed if they are module-level and accessed directly)

    print_verbose("[INFO] Starting full multi-phase homing routine...", 0)
    internal_state['homing_completed'] = False
    update_input_registers(modbus_slave_handler, homing=False)
    endstop_event.clear() # Ensure event is clear before starting

    # STEPS_PER_DEGREE is now imported from utils.py
    # Ensure HOMING_BACKUP_DEGREES is defined (e.g., 20.0)
    # Ensure STEPS_PER_DEGREE is correctly calculated/imported based on MAX_STEPS from utils.py
    # If STEPS_PER_DEGREE or MAX_STEPS are not yet in utils.py, this will cause an error.
    # For now, assuming they will be added to utils.py or this part needs adjustment.
    # If MAX_STEPS is not available from utils, we might need a local _MAX_STEPS_TEMP as before.
    # Let's assume STEPS_PER_DEGREE is correctly available via import for now.
    backup_steps_to_move = int(HOMING_BACKUP_DEGREES * STEPS_PER_DEGREE)

    try:
        # --- Phase 1: Coarse Search for Z-pulse (Forward) ---
        print_verbose(f"[HOMING] Phase 1: Coarse search forward at {HOMING_SEARCH_RPM} RPM...", 1)
        cfw500_master_obj.start_motor(HOMING_SEARCH_RPM)
        zero_pin.irq(trigger=Pin.IRQ_FALLING, handler=endstop_triggered_callback_irq)
        
        try:
            await asyncio.wait_for_ms(endstop_event.wait(), 60000) # 60-second timeout
            print_verbose("[HOMING] Phase 1: Z-pulse detected during coarse search.", 2)
            # Motor is still running here; stop it.
        except asyncio.TimeoutError:
            print_verbose("[ERROR HOMING] Phase 1: Timeout waiting for Z-pulse during coarse search.", 0)
            # Attempt to stop motor and exit homing if Z-pulse not found in reasonable time/distance
            cfw500_master_obj.stop_motor()
            internal_state['homing_completed'] = False
            update_input_registers(modbus_slave_handler, homing=False)
            # Detach IRQ in the main finally block
            return # Exit homing
        finally:
            # This inner finally ensures motor stop and IRQ detach specifically for Phase 1
            # regardless of whether Z-pulse was found or timed out, before proceeding or returning.
            cfw500_master_obj.stop_motor()
            zero_pin.irq(handler=None) # Detach IRQ after this phase
            endstop_event.clear()      # Clear event for next phase
            print_verbose("[HOMING] Phase 1: Motor stopped, IRQ detached. Waiting for motor to settle...", 2)
            await asyncio.sleep_ms(3500) # Wait for motor to physically stop (VFD ramp + margin)

        # --- Phase 2: Backup Past Z-Pulse (Reverse) ---
        # Motor has overshot the Z-pulse from Phase 1. Now back up.
        print_verbose(f"[HOMING] Phase 2: Backing up {HOMING_BACKUP_DEGREES} degrees at {HOMING_CREEP_RPM} RPM...", 1)
        start_backup_pos_raw = internal_state['encoder_raw_position']
        cfw500_master_obj.reverse_motor(HOMING_CREEP_RPM)

        backup_loop_start_time = time.ticks_ms()
        moved_backup_steps = 0
        while moved_backup_steps < backup_steps_to_move:
            current_raw_pos = internal_state['encoder_raw_position']
            # Calculate movement in reverse direction
            if HOMING_CREEP_RPM > 0 : # Assuming reverse makes raw position decrease
                 moved_backup_steps = start_backup_pos_raw - current_raw_pos
            else: # Should not happen, but as a fallback
                 moved_backup_steps = abs(current_raw_pos - start_backup_pos_raw)


            if time.ticks_diff(time.ticks_ms(), backup_loop_start_time) > 45000: # 45s timeout for backup
                print_verbose("[ERROR HOMING] Phase 2: Timeout during backup movement.", 0)
                cfw500_master_obj.stop_motor()
                internal_state['homing_completed'] = False
                update_input_registers(modbus_slave_handler, homing=False)
                return # Exit homing
            await asyncio.sleep_ms(20) # Yield briefly, allow encoder updates

        cfw500_master_obj.stop_motor()
        print_verbose("[HOMING] Phase 2: Backup complete. Waiting for motor to settle...", 2)
        await asyncio.sleep_ms(3500) # Wait for motor to stop

        # --- Phase 3: Slow Forward Approach to Z-Pulse ---
        print_verbose(f"[HOMING] Phase 3: Slow forward approach at {HOMING_CREEP_RPM} RPM...", 1)
        cfw500_master_obj.start_motor(HOMING_CREEP_RPM)
        endstop_event.clear() # Crucial: ensure event is clear before re-attaching IRQ
        zero_pin.irq(trigger=Pin.IRQ_FALLING, handler=endstop_triggered_callback_irq)
        
        try:
            await asyncio.wait_for_ms(endstop_event.wait(), 45000) # 45-second timeout for final approach
            # Z-PULSE DETECTED!
            # Stop motor IMMEDIATELY upon event detection.
            # The IRQ sets the event, this task wakes up and stops the motor.
            cfw500_master_obj.stop_motor()
            
            # Capture raw encoder position. It's assumed encoder_raw_position is updated
            # frequently by the encoder_callback in its own async context.
            # There might be a slight latency between IRQ and this capture.
            internal_state['encoder_zero_offset'] = internal_state['encoder_raw_position']
            internal_state['homing_completed'] = True
            update_input_registers(modbus_slave_handler, homing=True)
            print_verbose(f"[INFO] Homing complete. Encoder zero offset captured at raw steps: {internal_state['encoder_zero_offset']}", 0)
            
            # Wait for the motor to physically stop after the command.
            print_verbose("[HOMING] Phase 3: Z-pulse detected. Waiting for motor to settle...", 2)
            await asyncio.sleep_ms(3500)

        except asyncio.TimeoutError:
            print_verbose("[ERROR HOMING] Phase 3: Timeout waiting for Z-pulse during final approach.", 0)
            cfw500_master_obj.stop_motor()
            internal_state['homing_completed'] = False
            update_input_registers(modbus_slave_handler, homing=False)
            # IRQ will be detached in the main finally block
            return # Exit homing
        # No specific finally for IRQ here, handled by outer finally

    except Exception as e:
        print_verbose(f"[ERROR HOMING] An unexpected error occurred during homing: {e}", 0)
        cfw500_master_obj.stop_motor() # Attempt to stop motor
        internal_state['homing_completed'] = False
        update_input_registers(modbus_slave_handler, homing=False)

    finally:
        # This main finally block ensures cleanup regardless of how homing exits.
        zero_pin.irq(handler=None) # Always detach IRQ
        endstop_event.clear()      # Always clear the event
        
        # Final check to ensure motor is stopped if homing didn't complete successfully.
        if not internal_state['homing_completed']:
            print_verbose("[HOMING] Ensuring motor is stopped after incomplete or failed homing attempt.", 1)
            cfw500_master_obj.stop_motor() # Send stop one last time if needed
            # Give it a moment if stop was just issued
            await asyncio.sleep_ms(3000)
        
        print_verbose(f"[DEBUG] Homing routine finished. Homing completed status: {internal_state['homing_completed']}", 3)


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
                update_input_registers(modbus_slave_handler, fault=fault) # Pass handler
                # ADD DEBUG PRINT HERE
                if internal_state['VERBOSE_LEVEL'] >= 3:
                     print_verbose(f"[DEBUG VFD TASK] After fault update: FAULT_REG={modbus_slave_handler.get_ireg(REG_FAULT_FLAG)}", 3) # Use get_ireg
                if fault and internal_state['VERBOSE_LEVEL'] >= 1:
                     print_verbose("[ALERT] VFD Fault Detected!", 1)
            else:
                 print_verbose("[WARNING] Failed to read VFD fault status.", 1)

            if status_p0680 is not None:
                 update_input_registers(modbus_slave_handler, vfd_status=status_p0680) # Pass handler
                 # ADD DEBUG PRINT HERE
                 if internal_state['VERBOSE_LEVEL'] >= 3:
                      print_verbose(f"[DEBUG VFD TASK] After VFD status update: VFD_STATUS_REG={modbus_slave_handler.get_ireg(REG_VFD_STATUS)}", 3) # Use get_ireg
                 if internal_state['VERBOSE_LEVEL'] >= 3: # Keep existing P0680 print
                      print_verbose(f"[DEBUG] VFD Status P0680: 0x{status_p0680:04X}", 3)
            else:
                 print_verbose("[WARNING] Failed to read VFD status P0680.", 1)

            if current_rpm is not None:
                 update_input_registers(modbus_slave_handler, rpm=current_rpm) # Pass handler
                 # ADD DEBUG PRINT HERE
                 if internal_state['VERBOSE_LEVEL'] >= 3:
                      print_verbose(f"[DEBUG VFD TASK] After RPM update: RPM_REG={modbus_slave_handler.get_ireg(REG_CURRENT_RPM)}", 3) # Use get_ireg
                 if internal_state['VERBOSE_LEVEL'] >= 2: # Keep existing speed print
                      print_verbose(f"[INFO] VFD Speed: {current_rpm:.1f} RPM", 2)
            else:
                 print_verbose("[WARNING] Failed to read VFD current speed.", 1)

        except Exception as e:
            print_verbose(f"[ERROR] VFD Status Task Error: {e}", 0)

        await asyncio.sleep_ms(STATUS_REQUEST_INTERVAL_MS)

async def relay_control_task():
    """Controls relays based on internal fault state."""
    # while True:
    #     if internal_state['fault_detected']:
    #         # Blink on fault
    #         relay_pin1.on()
    #         relay_pin2.on()
    #         await asyncio.sleep_ms(500)
    #         relay_pin1.off()
    #         relay_pin2.off()
    #         await asyncio.sleep_ms(500)
    #     else:
    #         # Solid on if no fault
    #         relay_pin1.on()
    #         relay_pin2.on()
    #         await asyncio.sleep_ms(RELAY_CONTROL_INTERVAL_MS) # Check state less frequently if ok
    pass

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
                    # Use get_ireg to read current values from the handler's perspective
                    rpm_val_before = modbus_handler_obj.get_ireg(REG_CURRENT_RPM)
                    offset_val_before = modbus_handler_obj.get_ireg(REG_OFFSET_STEPS)
                    max_rpm_val_before = modbus_handler_obj.get_ireg(REG_MAX_RPM)
                    vfd_status_val_before = modbus_handler_obj.get_ireg(REG_VFD_STATUS)
                    print_verbose(f"[DEBUG SLAVE POLL] BEFORE process(): RPM={rpm_val_before}, OFFSET={offset_val_before}, MAX_RPM={max_rpm_val_before}, VFD_STAT={vfd_status_val_before:#06x}", 3)

            result = modbus_handler_obj.process()

            # General "Task running" print can remain less frequent
            if internal_state['VERBOSE_LEVEL'] >= 3:
                if debug_print_counter % 100 == 0: # e.g., every 100 * 200ms = 20 seconds
                    cmd_val_after = modbus_handler_obj.get_hreg(REG_CMD) # Use get_hreg
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
    # Pass modbus_slave_handler to load_configuration
    load_configuration(modbus_slave_handler)
    print_verbose("[INFO] load_configuration() is ENABLED.", 1) # Keep this log
    # ADD DEBUG PRINT HERE
    # Debug print after load_configuration can be removed or kept for one last check
    # print_verbose(f"[DEBUG MAIN] After load_config: OFFSET_REG={modbus_slave_handler.get_ireg(REG_OFFSET_STEPS)}, RPM_REG={modbus_slave_handler.get_ireg(REG_CURRENT_RPM)}", 3)

    # The load_configuration in utils.py should now correctly load
    # VERBOSE_LEVEL, encoder_offset_steps, and encoder_output_mode,
    # and update the HREGS for verbosity/mode and IREG for offset.
    # It should NOT overwrite the other initial test IREGS (rpm, vfd_status, etc.)
    # if the config file doesn't have entries for them.
    # --- !!! END OF CHANGE !!! ---

    # Ensure other Input Registers start with a known state (or let utils.py initial values be)
    # If load_configuration might reset them, re-initialize them here for clarity before VFD task starts.
    # For now, we assume utils.py sets initial non-zero values that persist unless explicitly overwritten.
    # Pass modbus_slave_handler to update_input_registers
    update_input_registers(modbus_slave_handler, homing=False) # Explicitly set homing to false initially # Keep this

    # Initialize Encoder (uses internal_state, updates slave_registers via callback)
    # modbus_slave_handler is initialized earlier in main.py
    initialize_encoder(16, 17, modbus_slave_handler)
    print_verbose("[INFO] Encoder ENABLED.", 0)

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
            # Pass modbus_slave_handler
            update_input_registers(modbus_slave_handler, max_rpm=int(max_rpm_val)) # Update IREG_MAX_RPM
            # ADD DEBUG PRINT HERE
            # print_verbose(f"[DEBUG MAIN] After max_rpm update: MAX_RPM_REG={modbus_slave_handler.get_ireg(REG_MAX_RPM)}", 3)
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

    # --- Homing Sequence ---
    # Temporarily cancel background tasks during homing - Kept disabled for now as per instructions
    # print_verbose("[DEBUG] Cancelling background tasks for homing...", 3)
    # status_task.cancel()
    # relay_task.cancel()
    # slave_poll_task.cancel() # Also cancel slave polling during homing
    # try:
    #     await asyncio.sleep_ms(100) # Allow cancellations
    # except asyncio.CancelledError: pass
    # print_verbose("[DEBUG] Background tasks cancelled.", 3)
    
    await homing(vfd_master) # Perform homing, passing the vfd_master instance
    
    # # Restart background tasks - Kept disabled for now
    # print_verbose("[DEBUG] Restarting background tasks after homing...", 3)
    # status_task = asyncio.create_task(vfd_status_request_task(vfd_master))
    # relay_task = asyncio.create_task(relay_control_task())
    # slave_poll_task = asyncio.create_task(modbus_slave_poll_task(modbus_slave_handler)) # Pass the handler instance
    print_verbose("[INFO] Homing Sequence ENABLED.", 0) # This will run before the modified homing test

    # Step 0: Isolate the Homing Function by adding an infinite loop after its call
    print_verbose("[INFO] Main Pico execution paused after homing test for observation.", 0)
    while True:
        await asyncio.sleep_ms(1000) # Keep alive for USB serial, etc.

    # The original main loop content below will not be reached due to the infinite loop above.
    # print_verbose("[INFO] Main loop started. Monitoring VFD status.", 0)

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
