# main_pico/main.py - Refactored for Modularity & Modbus

import time
import sys
import uasyncio as asyncio
from machine import Pin, UART
from cfw500_modbus import CFW500Modbus # Import class directly
from encoder_module import initialize_encoder # Removed get_raw_position
from command_processor import process_modbus_commands # Import the new processor
from utils import (
    print_verbose, show_manual, load_configuration, save_configuration,
    internal_state, slave_registers, update_input_registers,
    # Add REG_CURRENT_RPM and REG_VFD_STATUS to the import list
    REG_CMD, REG_TARGET_RPM, REG_VERBOSE, REG_ENC_MODE, REG_OFFSET_STEPS,
    REG_FAULT_FLAG, REG_HOMING_FLAG, REG_MAX_RPM, REG_ENC_POS_STEPS,
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
STATUS_REQUEST_INTERVAL_MS = 300 # VFD status check interval (Reduced for testing encoder updates)
RELAY_CONTROL_INTERVAL_MS = 1000 # Relay update interval
MODBUS_SLAVE_POLL_INTERVAL_MS = 200 # How often to check for slave requests (Increased from 10ms)
MAIN_LOOP_SLEEP_MS = 20 # Main loop sleep
SETTLE_TIME_MS = 3500        # Time for motor to settle after VFD stop command

# --- Homing Parameters ---
HOMING_SEARCH_RPM = 800      # RPM for the initial search for the Z-pulse
HOMING_CREEP_RPM = 400       # RPM for the slow final approach to the Z-pulse
HOMING_BACKUP_DEGREES = 20.0 # Degrees to back up past Z-pulse after initial find
HOMING_TIMEOUT_MS = 180000    # Timeout for major homing phases (e.g., P1, P3 search)
HOMING_BACKUP_DURATION_MS = 30000 # Max duration for backup phase (P2)

# --- Positioning Parameters ---
POSITIONING_RPM = 1600        # RPM to use during rotate commands
MIN_ROTATION_ANGLE_DEG = 1.0 # Minimum angle to rotate (smaller values rejected)
POSITIONING_STOP_MARGIN_DEG = 5.0 # Stop motor this many degrees before target to account for inertia
POSITIONING_TIMEOUT_MS = 50000 # Timeout for positioning movements

FAST_ZONE_DEG   = 60.0   # quando faltar <30° já troca p/ médio
SLOW_ZONE_DEG   = 30.0    # quando faltar <5° entra em creep
APPROACH_RPM    = 1000
CREEP_RPM       = 400
FINAL_TOLERANCE_DEG = 0.5  # erro máximo aceitável

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

# Global for encoder instance
main_encoder_instance = None

def _clear_cmd():
    try:
        modbus_slave_handler.set_hreg(REG_CMD, 0)
        internal_state['last_written_cmd'] = 0
    except: pass

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

    elif command_to_process == 6: # HOME Command
        print_verbose(f"[ACTION CB] HOME command (6) received.", 0)
        if homing_in_progress_event.is_set():
            print_verbose("[WARNING CB] Homing command (6) received, but homing already in progress. Ignoring.", 1)
        else:
            # It's important that vfd_master is available. It should be, as it's initialized globally.
            if vfd_master:
                print_verbose("[INFO CB] Scheduling homing routine via command...", 1)
                asyncio.create_task(homing(vfd_master)) # Schedule the homing task
            else:
                print_verbose("[ERROR CB] HOME command: vfd_master is None. Cannot schedule homing.", 0)
        
        # Clear command register
        try:
            modbus_slave_handler.set_hreg(REG_CMD, 0)
            internal_state['last_written_cmd'] = 0 # Update internal tracking
            print_verbose(f"[DEBUG CB] REG_CMD (for HOME) cleared.", 3)
        except Exception as e:
            print_verbose(f"[ERROR CB] Failed to clear REG_CMD for HOME: {e}", 0)
            
    elif command_to_process == 7: # ROTATE Command
        print_verbose(f"[ACTION CB] ROTATE command (7) received.", 0)
        
        # Extract angle from payload
        angle_to_rotate = 0
        if isinstance(val, list) and len(val) > 1:
            angle_to_rotate = val[1]
            print_verbose(f"[DEBUG CB] ROTATE with target angle: {angle_to_rotate} degrees", 2)
        else:
            print_verbose("[ERROR CB] ROTATE command received without angle parameter.", 0)
            # Clear command register and exit
            try:
                modbus_slave_handler.set_hreg(REG_CMD, 0)
                internal_state['last_written_cmd'] = 0
            except Exception as e:
                print_verbose(f"[ERROR CB] Failed to clear REG_CMD for ROTATE: {e}", 0)
            return
        
        # Check if angle is valid
        if abs(angle_to_rotate) < MIN_ROTATION_ANGLE_DEG:
            print_verbose(f"[ERROR CB] ROTATE angle too small (min: {MIN_ROTATION_ANGLE_DEG}°)", 0)
            try:
                modbus_slave_handler.set_hreg(REG_CMD, 0)
                internal_state['last_written_cmd'] = 0
            except Exception as e:
                print_verbose(f"[ERROR CB] Failed to clear REG_CMD for ROTATE: {e}", 0)
            return
            
        # Check if homing has been completed
        if not internal_state['homing_completed']:
            print_verbose("[ERROR CB] ROTATE command failed: Homing must be completed first.", 0)
            try:
                modbus_slave_handler.set_hreg(REG_CMD, 0)
                internal_state['last_written_cmd'] = 0
            except Exception as e:
                print_verbose(f"[ERROR CB] Failed to clear REG_CMD for ROTATE: {e}", 0)
            return
            
        # Check if positioning is already in progress
        if positioning_in_progress_event.is_set():
            print_verbose("[WARNING CB] ROTATE command received, but positioning already in progress. Ignoring.", 1)
            try:
                modbus_slave_handler.set_hreg(REG_CMD, 0)
                internal_state['last_written_cmd'] = 0
            except Exception as e:
                print_verbose(f"[ERROR CB] Failed to clear REG_CMD for ROTATE: {e}", 0)
            return
            
        # All checks passed, schedule the positioning task
        if vfd_master:
            print_verbose(f"[INFO CB] Starting rotation of {angle_to_rotate} degrees...", 0)
            
            # Create and start the positioning task
            asyncio.create_task(position_motor(vfd_master, angle_to_rotate))
        else:
            print_verbose("[ERROR CB] ROTATE command: vfd_master is None. Cannot schedule positioning.", 0)
            
        # Note: We don't clear the command register here - the positioning task will clear it when complete

    elif command_to_process == 8: # GO_TO_CALIBRATED_POSITION
        print_verbose(f"[ACTION CB] GO_TO_CALIBRATED_POSITION command (8) received.", 0)
        
        # Check if positioning is already in progress
        if positioning_in_progress_event.is_set() or homing_in_progress_event.is_set():
            print_verbose("[WARNING CB] GO_TO_CALIBRATED_POSITION command received, but positioning or homing already in progress. Ignoring.", 1)
            try:
                modbus_slave_handler.set_hreg(REG_CMD, 0)
                internal_state['last_written_cmd'] = 0
            except Exception as e:
                print_verbose(f"[ERROR CB] Failed to clear REG_CMD for GO_TO_CALIBRATED_POSITION: {e}", 0)
            return
            
        # Check if homing has been completed
        if not internal_state['homing_completed']:
            print_verbose("[ERROR CB] GO_TO_CALIBRATED_POSITION command failed: Homing must be completed first.", 0)
            try:
                modbus_slave_handler.set_hreg(REG_CMD, 0)
                internal_state['last_written_cmd'] = 0
            except Exception as e:
                print_verbose(f"[ERROR CB] Failed to clear REG_CMD for GO_TO_CALIBRATED_POSITION: {e}", 0)
            return
            
        # Schedule the go_to_calibrated_position task
        if vfd_master:
            print_verbose("[INFO CB] Starting movement to calibrated position...", 0)
            
            # Create and start the task to go to calibrated position
            asyncio.create_task(go_to_calibrated_position(vfd_master))
            
            # Clear the command register immediately since the task has its own completion handling
            try:
                modbus_slave_handler.set_hreg(REG_CMD, 0)
                internal_state['last_written_cmd'] = 0
                print_verbose("[DEBUG CB] REG_CMD (for GO_TO_CALIBRATED_POSITION) cleared after scheduling task.", 3)
            except Exception as e:
                print_verbose(f"[ERROR CB] Failed to clear REG_CMD for GO_TO_CALIBRATED_POSITION: {e}", 0)
        else:
            print_verbose("[ERROR CB] GO_TO_CALIBRATED_POSITION command: vfd_master is None. Cannot schedule movement.", 0)
            try:
                modbus_slave_handler.set_hreg(REG_CMD, 0)
                internal_state['last_written_cmd'] = 0
            except Exception as e:
                print_verbose(f"[ERROR CB] Failed to clear REG_CMD for GO_TO_CALIBRATED_POSITION: {e}", 0)

    elif command_to_process in [2, 4, 5]: # STOP, RESET_FAULT, CALIBRATE
        rpm_from_payload = 0
        if isinstance(val, list) and len(val) > 1:
             rpm_from_payload = val[1]
             modbus_slave_handler.set_hreg(REG_TARGET_RPM, rpm_from_payload)
             print_verbose(f"[DEBUG CB] HREG_TARGET_RPM updated to {rpm_from_payload} from command {command_to_process} payload.", 3)

        print_verbose(f"[DEBUG CB] Processing Action Command: {command_to_process} (RPM in payload if any: {rpm_from_payload})", 3)
        
        if command_to_process == 5: # CALIBRATE - Special handling
            print_verbose(f"[ACTION CB] CALIBRATE command (5) received.", 0)
            if not internal_state['homing_completed']:
                print_verbose("[WARNING CB] CALIBRATE command ignored: Homing not completed.", 1)
            else:
                try:
                    # 1. Get current raw position (e.g., -7500)
                    current_raw_pos = internal_state['encoder_raw_position']
                    
                    # 2. Get the machine zero reference (e.g., -7984, the raw reading at Z-pulse)
                    machine_zero_raw_reading = internal_state['encoder_zero_offset']
                    
                    # 3. Calculate how many steps the current position is from the Z-pulse (machine zero)
                    #    steps_from_machine_zero = -7500 - (-7984) = 484
                    steps_from_machine_zero = current_raw_pos - machine_zero_raw_reading
                    
                    # 4. This delta IS the new user calibration offset.
                    internal_state['encoder_offset_steps'] = steps_from_machine_zero
                    
                    update_input_registers(modbus_slave_handler, offset=internal_state['encoder_offset_steps'])
                    save_configuration()
                    print_verbose(f"[ACTION CB] CALIBRATE: Current raw pos: {current_raw_pos}, Machine Zero (Z-raw): {machine_zero_raw_reading}", 1)
                    print_verbose(f"[ACTION CB] New User Zero Offset (delta from Z): {internal_state['encoder_offset_steps']} steps.", 0)
                    print_verbose(f"[ACTION CB] Config saved. Modbus reports will now be zeroed at this current physical position.", 0)
                except Exception as e:
                    print_verbose(f"[ERROR CB] Error processing CALIBRATE (Cmd 5): {e}", 0)
        else: # STOP (2), RESET_FAULT (4)
            try:
                if command_to_process == 2: # STOP
                    if vfd_master: vfd_master.stop_motor()
                    print_verbose(f"[ACTION CB] Motor STOP processed.", 0)
                elif command_to_process == 4: # RESET_FAULT
                    if vfd_master: vfd_master.reset_fault()
                    print_verbose(f"[ACTION CB] VFD Fault RESET processed.", 0)
            except Exception as e:
                print_verbose(f"[ERROR CB] Error processing command {command_to_process}: {e}", 0)

        # Clear command register (for CALIBRATE, STOP, RESET_FAULT)
        try:
            modbus_slave_handler.set_hreg(REG_CMD, 0)
            internal_state['last_written_cmd'] = 0
            print_verbose(f"[DEBUG CB] Action Cmd {command_to_process}: REG_CMD cleared.", 3)
        except Exception as e:
            print_verbose(f"[ERROR CB] Failed to clear REG_CMD for action {command_to_process}: {e}", 0)
    elif (command_to_process == 9): # goto
        if isinstance(val, list) and len(val) > 1:
            target_steps = val[1] & 0xFFFF
            print(f"[CB] GOTO target_steps={target_steps}",2)
        else:
            print_verbose("[ERROR CB] GOTO sem parâmetro",0)
            _clear_cmd()
            return

        # Homing precisa ter sido feito
        if not internal_state['homing_completed']:
            print_verbose("[ERROR CB] GOTO falhou: faça homing antes",0)
            _clear_cmd()
            return

        # Posição atual em steps relativa ao ZERO calibrado
        current_steps = modbus_slave_handler.get_ireg(REG_ENC_POS_STEPS)  # já compensado por offset
        delta_steps   = (target_steps - current_steps) % MAX_STEPS
        delta_deg     = delta_steps / (STEPS_PER_DEGREE)

        # Ignora movimentos muito pequenos
        if delta_deg < MIN_ROTATION_ANGLE_DEG:
            print_verbose("[CB] GOTO: já estamos no alvo (≤1°)",1)
            _clear_cmd()
            return

        print(f"[CB] GOTO vai avançar {delta_steps} steps  ({delta_deg:.2f}°)")
        asyncio.create_task(position_motor(vfd_master, delta_deg))
        # IMPORTANTE – não limpa REG_CMD aqui; position_motor cuidará disso

# --- Assign Callbacks BEFORE Setup ---
slave_registers['HREGS']['command']['on_set_cb'] = handle_command_register_write
print_verbose("[INFO] Command register callback assigned to slave_registers dict.", 2)

# 3. Setup the registers using the map from utils (NOW includes the callback)
# IMPORTANT: slave_registers global dict from utils.py is used here for initial setup.
modbus_slave_handler.setup_registers(registers=slave_registers)
print_verbose("[INFO] Modbus Slave Handler (UART1 for Relay) initialized and registers set up.", 2)


# --- Async Events ---
endstop_event = asyncio.Event()
homing_in_progress_event = asyncio.Event() # For signalling commanded homing
positioning_in_progress_event = asyncio.Event() # For signalling positioning movement

# --- Homing ---
def endstop_triggered_callback_irq(pin):
    # --- IRQ Handler --- Keep simple! ---
    endstop_event.set()
    # --- End IRQ Handler ---

async def homing(cfw500_master_obj):
    if homing_in_progress_event.is_set():
        print_verbose("[WARNING HOMING] Homing command received, but homing is already in progress. Ignoring.", 1)
        return

    homing_in_progress_event.set()
    print_verbose("[INFO HOMING] Starting Z-pulse homing routine...", 0)
    internal_state['homing_completed'] = False
    update_input_registers(modbus_slave_handler, homing=False) # Signal homing started

    HOMING_POLL_INTERVAL_MS = 2
    # We no longer need backup_steps_to_move since we're using absolute degrees for Phase 2 control
    # backup_steps_to_move = int(HOMING_BACKUP_DEGREES * STEPS_PER_DEGREE)
    z_pulse_detected_this_phase = False # Re-initialize for each phase inside loop if needed, or manage scope
    initial_motor_start_failed = False

    try:
        # --- Phase 1: Coarse Search for Z-pulse (Forward) ---
        print_verbose(f"[HOMING P1] Coarse search forward at {HOMING_SEARCH_RPM} RPM...", 1)
        try:
            cfw500_master_obj.start_motor(HOMING_SEARCH_RPM)
        except Exception as e:
            print_verbose(f"[ERROR HOMING P1] Failed to start motor: {e}", 0)
            initial_motor_start_failed = True
            # No return here, finally block will handle cleanup

        if not initial_motor_start_failed:
            phase1_start_time_ms = time.ticks_ms()
            last_z_pin_state = zero_pin.value()
            z_pulse_detected_this_phase = False # Reset for this phase
            print_verbose(f"[HOMING P1] Initial Z-pin: {last_z_pin_state}. Listening for FALLING edge...", 2)

            while time.ticks_diff(time.ticks_ms(), phase1_start_time_ms) < HOMING_TIMEOUT_MS:
                current_z_pin_state = zero_pin.value()
                if last_z_pin_state == 1 and current_z_pin_state == 0: # Falling edge
                    print_verbose("[HOMING P1] Z-pulse FALLING edge detected.", 2)
                    z_pulse_detected_this_phase = True
                    break
                last_z_pin_state = current_z_pin_state
                await asyncio.sleep_ms(HOMING_POLL_INTERVAL_MS)
            
            cfw500_master_obj.stop_motor()
            print_verbose("[HOMING P1] Motor stop. Settling...", 2)
            await asyncio.sleep_ms(SETTLE_TIME_MS)

            if not z_pulse_detected_this_phase:
                print_verbose("[ERROR HOMING P1] Timeout: Z-pulse not found in coarse search.", 0)
                raise Exception("Homing P1 Timeout")

        if initial_motor_start_failed:
            raise Exception("Homing P1 Motor Start Failed")

        # --- Phase 2: Backup Past Z-Pulse (Reverse) ---
        print_verbose(f"[HOMING P2] Backing up {HOMING_BACKUP_DEGREES} deg at {HOMING_CREEP_RPM} RPM using absolute degrees...", 1)
        
        # Use internal_absolute_degrees for Phase 2 backup control
        start_backup_abs_deg = internal_state['internal_absolute_degrees']
        target_backup_abs_deg = start_backup_abs_deg + HOMING_BACKUP_DEGREES # VFD Reverse INCREASES absolute degrees
        
        print_verbose(f"[DEBUG HOMING P2] Start abs_deg for backup: {start_backup_abs_deg:.2f}°, Target abs_deg: {target_backup_abs_deg:.2f}°", 3)
        
        try:
            cfw500_master_obj.reverse_motor(HOMING_CREEP_RPM) # This causes internal_absolute_degrees to INCREASE
        except Exception as e:
            print_verbose(f"[ERROR HOMING P2] Failed to start motor for backup: {e}", 0)
            raise Exception("Homing P2 Motor Start Failed")

        backup_loop_start_time = time.ticks_ms()
        
        # Change loop condition for Phase 2 to use absolute degrees
        while internal_state['internal_absolute_degrees'] < target_backup_abs_deg:
            current_abs_deg_p2 = internal_state['internal_absolute_degrees'] # For logging
            
            if internal_state['VERBOSE_LEVEL'] >= 3:
                print_verbose(f"[DEBUG HOMING P2] Current abs_deg: {current_abs_deg_p2:.2f}°, Target abs_deg: {target_backup_abs_deg:.2f}°", 3)

            if time.ticks_diff(time.ticks_ms(), backup_loop_start_time) > HOMING_BACKUP_DURATION_MS:
                print_verbose("[ERROR HOMING P2] Timeout during backup.", 0)
                cfw500_master_obj.stop_motor()
                raise Exception("Homing P2 Backup Timeout")
            await asyncio.sleep_ms(20)

        cfw500_master_obj.stop_motor()
        print_verbose("[HOMING P2] Backup complete. Settling...", 2)
        await asyncio.sleep_ms(SETTLE_TIME_MS)

        # --- Phase 3: Slow Forward Approach to Z-Pulse (Polling) ---
        print_verbose(f"[HOMING P3] Slow forward approach at {HOMING_CREEP_RPM} RPM...", 1)
        try:
            cfw500_master_obj.start_motor(HOMING_CREEP_RPM)
        except Exception as e:
            print_verbose(f"[ERROR HOMING P3] Failed to start motor for final approach: {e}", 0)
            raise Exception("Homing P3 Motor Start Failed")

        phase3_start_time_ms = time.ticks_ms()
        last_z_pin_state = zero_pin.value()
        z_pulse_detected_this_phase = False # Reset for this phase
        print_verbose(f"[HOMING P3] Initial Z-pin: {last_z_pin_state}. Listening for FALLING edge...", 2)

        raw_reading_at_z = 0

        while time.ticks_diff(time.ticks_ms(), phase3_start_time_ms) < HOMING_TIMEOUT_MS:
            current_z_pin_state = zero_pin.value()
            if last_z_pin_state == 1 and current_z_pin_state == 0: # Falling edge
                raw_reading_at_z = internal_state['encoder_raw_position']
                cfw500_master_obj.stop_motor()
                
                internal_state['encoder_zero_offset'] = raw_reading_at_z
                internal_state['homing_completed'] = True
                update_input_registers(modbus_slave_handler, homing=True)
                print_verbose(f"[INFO HOMING] Z-pulse found! Machine Zero established.", 0)
                print_verbose(f"[INFO HOMING] Raw encoder reading at Z-pulse was: {raw_reading_at_z}", 1)
                print_verbose(f"[INFO HOMING] 'encoder_zero_offset' set to: {internal_state['encoder_zero_offset']}", 1)
                z_pulse_detected_this_phase = True
                break
            last_z_pin_state = current_z_pin_state
            await asyncio.sleep_ms(HOMING_POLL_INTERVAL_MS)

        if not z_pulse_detected_this_phase:
            cfw500_master_obj.stop_motor()
            print_verbose("[ERROR HOMING P3] Timeout: Z-pulse not detected in final approach.", 0)
            raise Exception("Homing P3 Timeout")

        print_verbose("[HOMING P3] Z-pulse detected & motor stopped. Settling...", 2)
        await asyncio.sleep_ms(SETTLE_TIME_MS)
        print_verbose("[INFO HOMING] Homing routine successful.", 0)
        
        # After successful homing, move to the calibrated position (where internal_absolute_degrees = 0)
        print_verbose("[INFO HOMING] Now moving to user's calibrated position...", 0)
        calib_pos_result = await go_to_calibrated_position(cfw500_master_obj)
        if calib_pos_result:
            print_verbose("[INFO HOMING] Successfully moved to calibrated position.", 0)
        else:
            print_verbose("[WARNING HOMING] Homing was successful, but movement to calibrated position failed.", 1)

    except Exception as e:
        print_verbose(f"[ERROR HOMING] Homing routine failed: {e}", 0)
        internal_state['homing_completed'] = False
        update_input_registers(modbus_slave_handler, homing=False)
        try:
            if cfw500_master_obj: # Ensure cfw500_master_obj is not None
                cfw500_master_obj.stop_motor()
                print_verbose("[SAFETY HOMING] Ensured motor is stopped after homing failure.", 1)
                await asyncio.sleep_ms(SETTLE_TIME_MS)
        except Exception as stop_e:
            print_verbose(f"[ERROR HOMING] Failed to stop motor during error cleanup: {stop_e}", 0)
            
    finally:
        homing_in_progress_event.clear()
        print_verbose(f"[DEBUG HOMING] Homing routine finished. Homing completed status: {internal_state['homing_completed']}", 3)


# --- Positioning Task ---
async def position_motor(cfw500, angle_deg):
    """
    Gira SEMPRE para frente (VFD FORWARD) a 'angle_deg' positivos,
    reduzindo RPM conforme se aproxima do alvo.
    """
    if positioning_in_progress_event.is_set():
        print_verbose("[WARN] positioning busy", 1)
        return
    positioning_in_progress_event.set()

    try:
        start_abs = internal_state["internal_absolute_degrees"]
        target_abs = start_abs - angle_deg     # FORWARD => diminui
        print_verbose(f"[POS] start={start_abs:.2f}°  target={target_abs:.2f}°", 1)
        print(f"[POS] start={start_abs:.2f}°  target={target_abs:.2f}°")

        # --- arranca na velocidade alta -------------
        cfw500.start_motor(POSITIONING_RPM)
        await asyncio.sleep_ms(100)            # deixa ganhar torque

        stage = 1
        start_ms = time.ticks_ms()

        while time.ticks_diff(time.ticks_ms(), start_ms) < POSITIONING_TIMEOUT_MS:

            current = internal_state["internal_absolute_degrees"]
            remaining = current - target_abs   # valor positivo enquanto não atingiu

            # ----- mudança de faixa -----
            if stage == 1 and remaining <= FAST_ZONE_DEG:
                cfw500.set_speed_reference(APPROACH_RPM)
                stage = 2
                print_verbose("[POS] → APPROACH_RPM", 2)
                print("[POS] → APPROACH_RPM")
                val = cfw500.read_p0316()   # ou read_p0680()
                print("P0316:", hex(val) if val else val)

            elif stage == 2 and remaining <= SLOW_ZONE_DEG:
                cfw500.set_speed_reference(CREEP_RPM)
                stage = 3
                print_verbose("[POS] → CREEP_RPM", 2)
                print("[POS] → CREEP_RPM")
                val = cfw500.read_p0316()   # ou read_p0680()
                print("P0316:", hex(val) if val else val)


            # ----- alvo atingido? -----
            if remaining <= FINAL_TOLERANCE_DEG:
                print_verbose("[POS] reached target window", 2)
                break

            await asyncio.sleep_ms(20)

        # --- parada & settling -------------
        cfw500.stop_motor()
        await asyncio.sleep_ms(SETTLE_TIME_MS)

        final_pos = internal_state["internal_absolute_degrees"]
        err = final_pos - target_abs
        print_verbose(f"[POS] final={final_pos:.2f}°  err={err:.2f}°", 0)
        print(f"[POS] final={final_pos:.2f}°  err={err:.2f}°")

    except Exception as e:
        print_verbose(f"[ERR POS] {e}", 0)
        try: cfw500.stop_motor()
        except: pass

    finally:
        positioning_in_progress_event.clear()
        try:
            modbus_slave_handler.set_hreg(REG_CMD, 0)
            internal_state["last_written_cmd"] = 0
        except: pass


# --- Go to Calibrated Position Task ---
async def go_to_calibrated_position(cfw500_master_obj):
    """
    Rotates the motor to the user-calibrated position (where internal_absolute_degrees = 0)
    after successful homing.
    """
    # Check if homing was completed successfully
    if not internal_state['homing_completed']:
        print_verbose("[ERROR CALIB_POS] Cannot move to calibrated position: Homing not completed.", 0)
        return False
        
    print_verbose("[INFO CALIB_POS] Moving to user calibrated position...", 0)
    
    try:
        # Get current position relative to calibrated zero
        current_absolute_degrees = internal_state['internal_absolute_degrees']
        
        print_verbose(f"[DEBUG CALIB_POS] Current absolute_degrees: {current_absolute_degrees:.2f}°, Target: 0.00°", 1)
        
        # Check if we're already close enough to the calibrated position
        if abs(current_absolute_degrees) <= MIN_ROTATION_ANGLE_DEG:
            print_verbose("[INFO CALIB_POS] Already at calibrated position (within tolerance).", 0)
            return True
            
        # Determine the direction and method of movement
        if current_absolute_degrees > 0:
            # Need to decrease absolute_degrees - can use position_motor
            print_verbose(f"[INFO CALIB_POS] Using position_motor to decrease absolute_degrees by {current_absolute_degrees:.2f}°", 1)
            await position_motor(cfw500_master_obj, current_absolute_degrees)
            print_verbose("[INFO CALIB_POS] Movement to calibrated position completed.", 0)
            return True
        else:  # current_absolute_degrees < 0
            # Need to increase absolute_degrees - must use direct VFD control
            target_degrees = 0.0
            angle_to_move = abs(current_absolute_degrees)
            early_stop_degrees = target_degrees - MIN_ROTATION_ANGLE_DEG  # Stop slightly before target
            
            print_verbose(f"[INFO CALIB_POS] Using VFD REVERSE to increase absolute_degrees by {angle_to_move:.2f}°", 1)
            
            try:
                # Start VFD in REVERSE direction (which increases absolute_degrees)
                cfw500_master_obj.reverse_motor(POSITIONING_RPM)
                print_verbose(f"[INFO CALIB_POS] Motor started in REVERSE at {POSITIONING_RPM} RPM", 1)
                await asyncio.sleep_ms(100)  # Allow motor to start
                
                # Monitor position until we reach target
                start_time_ms = time.ticks_ms()
                position_reached = False
                
                while time.ticks_diff(time.ticks_ms(), start_time_ms) < POSITIONING_TIMEOUT_MS:
                    current_position = internal_state['internal_absolute_degrees']
                    
                    # For REVERSE movement (increasing values), stop when we reach or exceed early_stop_degrees
                    if current_position >= early_stop_degrees:
                        position_reached = True
                        break
                    
                    print_verbose(f"[DEBUG CALIB_POS] Current: {current_position:.2f}°, Target: {target_degrees:.2f}°, Early stop: {early_stop_degrees:.2f}°", 3)
                    await asyncio.sleep_ms(20)
                
                # Stop the motor
                cfw500_master_obj.stop_motor()
                print_verbose(f"[INFO CALIB_POS] Motor stopped. Waiting {SETTLE_TIME_MS}ms to settle.", 1)
                await asyncio.sleep_ms(SETTLE_TIME_MS)
                
                # Check final position and report results
                final_position = internal_state['internal_absolute_degrees']
                position_error = final_position - target_degrees
                
                if position_reached:
                    print_verbose(f"[INFO CALIB_POS] Reached calibrated position. Final: {final_position:.2f}°, Error: {position_error:.2f}°", 0)
                    return True
                else:
                    print_verbose(f"[ERROR CALIB_POS] Timeout reached. Final: {final_position:.2f}°, Error: {position_error:.2f}°", 0)
                    return False
                
            except Exception as e:
                print_verbose(f"[ERROR CALIB_POS] Error during reverse movement: {e}", 0)
                try:
                    cfw500_master_obj.stop_motor()
                    await asyncio.sleep_ms(SETTLE_TIME_MS)
                except Exception as stop_e:
                    print_verbose(f"[ERROR CALIB_POS] Failed to stop motor during error cleanup: {stop_e}", 0)
                return False
    
    except Exception as e:
        print_verbose(f"[ERROR CALIB_POS] Failed to move to calibrated position: {e}", 0)
        try:
            cfw500_master_obj.stop_motor()
            await asyncio.sleep_ms(SETTLE_TIME_MS)
        except Exception as stop_e:
            print_verbose(f"[ERROR CALIB_POS] Failed to stop motor during error cleanup: {stop_e}", 0)
        return False


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
                     # print_verbose(f"[DEBUG VFD TASK] After fault update: FAULT_REG={modbus_slave_handler.get_ireg(REG_FAULT_FLAG)}", 3) # Use get_ireg
                     pass # Added pass to avoid indentation error
                if fault and internal_state['VERBOSE_LEVEL'] >= 1:
                     print_verbose("[ALERT] VFD Fault Detected!", 1)
            else:
                 print_verbose("[WARNING] Failed to read VFD fault status.", 1)

            if status_p0680 is not None:
                 update_input_registers(modbus_slave_handler, vfd_status=status_p0680) # Pass handler
                 # ADD DEBUG PRINT HERE
                 if internal_state['VERBOSE_LEVEL'] >= 3:
                      # print_verbose(f"[DEBUG VFD TASK] After VFD status update: VFD_STATUS_REG={modbus_slave_handler.get_ireg(REG_VFD_STATUS)}", 3) # Use get_ireg
                      pass # Added pass to avoid indentation error
                 if internal_state['VERBOSE_LEVEL'] >= 3: # Keep existing P0680 print
                      # print_verbose(f"[DEBUG] VFD Status P0680: 0x{status_p0680:04X}", 3)
                      pass # Added pass to avoid indentation error
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
                    # print_verbose(f"[DEBUG SLAVE POLL] BEFORE process(): RPM={rpm_val_before}, OFFSET={offset_val_before}, MAX_RPM={max_rpm_val_before}, VFD_STAT={vfd_status_val_before:#06x}", 3)
                    pass # Added pass to avoid indentation error

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
    global main_encoder_instance
    main_encoder_instance = initialize_encoder(16, 17, modbus_slave_handler)
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

    # --- Attempt Homing on Startup ---
    print_verbose("[INFO] Attempting homing on startup...", 0)
    await homing(vfd_master) # This will set internal_state['encoder_zero_offset']
    if internal_state['homing_completed']:
        print_verbose("[INFO] Homing on startup successful.", 1)
    else:
        print_verbose("[WARNING] Homing on startup failed or was skipped.", 1)
    
    # # Restart background tasks - Kept disabled for now
    # print_verbose("[DEBUG] Restarting background tasks after homing...", 3)
    # status_task = asyncio.create_task(vfd_status_request_task(vfd_master))
    # relay_task = asyncio.create_task(relay_control_task())
    # slave_poll_task = asyncio.create_task(modbus_slave_poll_task(modbus_slave_handler)) # Pass the handler instance
    # print_verbose("[INFO] Homing Sequence ENABLED.", 0) # This was for the startup homing test - now part of above message

    # The infinite loop for observation is removed. The main loop below is now active.
    print_verbose("[INFO] Background tasks started. System operational.", 0)
    # Adjusted message as homing is now on startup:
    print_verbose("[INFO] Main Pico execution running. Homing attempted on startup.", 0)
    print_verbose("[INFO] Send 'home' command to re-initiate homing if needed, or other commands.", 0)
    
    print_verbose("[INFO] Main loop started. Monitoring for commands.", 0)
    loop_counter = 0
    while True:
        # process_modbus_commands is for HREGS (verbosity, enc_mode)
        # This is still useful to poll for settings changes.
        await process_modbus_commands(vfd_master) # Pass vfd_master if it might be used by it

        loop_counter += 1
        if loop_counter % 250 == 0: # Print roughly every 5 seconds (250 * 20ms)
            if internal_state['VERBOSE_LEVEL'] >= 3:
                # More informative alive message
                homing_status_str = "Completed" if internal_state['homing_completed'] else "Not Done"
                cal_offset_val = internal_state['encoder_offset_steps']
                raw_zero_offset_val = internal_state['encoder_zero_offset']
                print_verbose(f"[DEBUG MAIN LOOP] Alive. Homing: {homing_status_str}, Cal Offset: {cal_offset_val}, Raw Z Offset: {raw_zero_offset_val}", 3)
        
        await asyncio.sleep_ms(MAIN_LOOP_SLEEP_MS) # Existing main loop sleep

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
