# main.py

import time
import sys
import uasyncio as asyncio
from machine import Pin, UART
from cfw500_modbus import CFW500Modbus
from encoder import Encoder
import json  # Added for configuration file handling

# Configuration for UART0 (Modbus RTU communication with VFD)
UART0_ID = 0  # UART0 for Modbus RTU
TX_PIN_NUM = 0  # GPIO0 (TX) for UART0
RX_PIN_NUM = 1  # GPIO1 (RX) for UART0
DE_RE_PIN = Pin(2, Pin.OUT)
DE_RE_PIN.value(0)  # Initially enable receiver
SLAVE_ADDRESS = 1  # Modbus slave address of the inverter

# Configuration for UART1 (RS485 communication with PC via Relay Pico)
UART1_ID = 1  # UART1 for user commands
USER_CMD_TX_PIN = 4  # GPIO4 (TX) for UART1
USER_CMD_RX_PIN = 5  # GPIO5 (RX) for UART1
DE_RE_UART1_PIN = Pin(3, Pin.OUT)  # GPIO3 for DE/RE control
DE_RE_UART1_PIN.value(0)  # Initially enable receiver

# Initialize Modbus and CFW500
cfw500 = CFW500Modbus(
    uart_id=UART0_ID,
    tx_pin=TX_PIN_NUM,
    rx_pin=RX_PIN_NUM,
    de_re_pin=DE_RE_PIN,
    slave_address=SLAVE_ADDRESS
)

# Initialize UART1 for RS485 communication
user_cmd_uart = UART(UART1_ID, baudrate=115200, tx=USER_CMD_TX_PIN, rx=USER_CMD_RX_PIN)
user_cmd_uart.init(bits=8, parity=None, stop=1)  # Use 1 stop bit for higher baud rates

# LED Configuration
LED_PIN = 25  # On-board LED pin on Raspberry Pi Pico
led = Pin(LED_PIN, Pin.OUT)

# Global Variables
VERBOSE_LEVEL = 1
encoder_position = 0        # Current encoder position
encoder_offset = 0          # Offset for calibration
encoder_output_mode = "deg"  # "step" or "deg"
CONFIG_FILE = "config.json"  # Configuration file to save encoder offset

# P0680 Bit Descriptions
P0680_BITS = {
    0: "STO - Safe Torque Off",
    1: "Command Rotate",
    2: "Reserved",
    3: "Reserved",
    4: "Rapid Stop Active",
    5: "Second Ramp Active",
    6: "Configuration Mode",
    7: "Alarm",
    8: "Ramp Enabled - RUN",
    9: "General Enabled",
    10: "Rotation Direction",
    11: "JOG Active",
    12: "LOC/REM Mode",
    13: "Undervoltage",
    14: "Automatic - PID",
    15: "Fault"
}

def print_verbose(message, level, override=False):
    """Prints messages according to the verbosity level."""
    if override or (VERBOSE_LEVEL >= level and VERBOSE_LEVEL > 0):
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

async def led_blink_task():
    """Task to blink the LED."""
    while True:
        led.toggle()
        await asyncio.sleep(0.5)

def save_configuration():
    """Saves the encoder offset to a configuration file."""
    config = {
        "encoder_offset": encoder_offset
    }
    try:
        with open(CONFIG_FILE, 'w') as f:
            json.dump(config, f)
        print_verbose("[INFO] Configuration saved.", 0)
    except Exception as e:
        print_verbose(f"[ERROR] Failed to save configuration: {e}", 0)

def load_configuration():
    """Loads the encoder offset from the configuration file."""
    global encoder_offset
    try:
        with open(CONFIG_FILE, 'r') as f:
            config = json.load(f)
        encoder_offset = config.get("encoder_offset", 0)
        print_verbose(f"[INFO] Loaded encoder offset from configuration: {encoder_offset}", 0)
    except FileNotFoundError:
        print_verbose("[WARNING] Configuration file not found. Using default settings.", 0)
        encoder_offset = 0
    except Exception as e:
        print_verbose(f"[ERROR] Failed to load configuration: {e}", 0)
        encoder_offset = 0

# Callback function to handle encoder changes
def encoder_callback(value, delta):
    global encoder_position
    encoder_position = value
    adjusted_position = encoder_position - encoder_offset

    if encoder_output_mode == "deg":
        # Convert steps to degrees
        degrees = adjusted_position * (360 / 8000)  # Adjust as per encoder resolution
        output = f"Encoder Position: {degrees:.2f} degrees"
    else:
        # Output in steps
        steps = adjusted_position
        output = f"Encoder Position: {steps} steps"

    if VERBOSE_LEVEL == 1 or VERBOSE_LEVEL == 3:
        print_verbose(output, 1)

# Zero Endstop Position Detection
def zero_position_callback(pin):
    """Callback function when the endstop (zero position) is detected."""
    print_verbose("[INFO] Zero position detected.", 1)
    # Do not reset the encoder position; only output a message

async def homing():
    """Performs the homing routine to establish the zero position."""
    global encoder_offset, encoder_position, zero_pin
    print_verbose("[INFO] Starting homing routine...", 0)

    # Move motor towards the endstop slowly
    try:
        cfw500.start_motor(100)  # Start motor at 100 RPM for homing
        print_verbose("[ACTION] Motor started at 100 RPM for homing.", 0)
    except Exception as e:
        print_verbose(f"[ERROR] Failed to start motor: {e}", 0)
        return

    # Wait until the endstop is triggered
    endstop_triggered = False

    def endstop_triggered_callback(pin):
        nonlocal endstop_triggered
        endstop_triggered = True
        print_verbose("[INFO] Endstop triggered during homing.", 0)

    zero_pin.irq(trigger=Pin.IRQ_FALLING, handler=endstop_triggered_callback)

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

    # Record the encoder position as the offset
    encoder_offset = encoder_position
    print_verbose(f"[INFO] Homing complete. Encoder offset set to {encoder_offset}.", 0)

    # Save the offset to configuration
    save_configuration()

    # Remove the interrupt handler for the endstop to prevent re-triggering during normal operation
    zero_pin.irq(handler=None)

    # Re-setup the endstop to only output a message when triggered, but not reset position
    zero_pin.irq(trigger=Pin.IRQ_FALLING, handler=zero_position_callback)

async def read_user_input():
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
                            should_continue = await process_command(user_input)
                            if not should_continue:
                                return
                    except Exception as e:
                        print_verbose(f"[ERROR] Exception during decoding: {e}", 2)
        await asyncio.sleep(0.1)

async def status_request_task():
    STATUS_REQUEST_INTERVAL = 5  # Interval in seconds
    while True:
        fault = cfw500.check_fault()
        if VERBOSE_LEVEL >= 2:
            if fault:
                print_verbose("[ALERT] The inverter is in FAULT.", 2)
            else:
                print_verbose("[INFO] The inverter is NOT in fault.", 2)
        await asyncio.sleep(STATUS_REQUEST_INTERVAL)

async def process_command(command):
    """Processes a single command."""
    global VERBOSE_LEVEL, encoder_output_mode, encoder_offset
    print(f"[DEBUG] Processing command: {command}")
    parts = command.strip().split()
    if not parts:
        return True  # Continue running
    cmd = parts[0].lower()

    try:
        if cmd == "start":
            rpm = float(parts[1]) if len(parts) >= 2 else 1000  # Default value
            cfw500.start_motor(rpm)
            if VERBOSE_LEVEL >= 2:
                print_verbose(f"[ACTION] Starting the motor at {rpm} RPM.", 2)
        elif cmd == "stop":
            cfw500.stop_motor()
            if VERBOSE_LEVEL >= 2:
                print_verbose("[ACTION] Stopping the motor.", 2)
        elif cmd == "reverse":
            rpm = float(parts[1]) if len(parts) >= 2 else 1000  # Default value
            cfw500.reverse_motor(rpm)
            if VERBOSE_LEVEL >= 2:
                print_verbose(f"[ACTION] Reversing the motor at {rpm} RPM.", 2)
        elif cmd == "set_speed":
            if len(parts) >= 2:
                rpm = float(parts[1])
                cfw500.set_speed_reference(rpm)
                if VERBOSE_LEVEL >= 2:
                    print_verbose(f"[ACTION] Setting speed reference to {rpm} RPM.", 2)
            else:
                print_verbose("[ERROR] Specify the speed in RPM.", 2, override=True)
        elif cmd == "read_speed":
            rpm = cfw500.read_current_speed()
            if rpm is not None:
                print_verbose(f"[INFO] Current Speed: {rpm:.2f} RPM", 2, override=True)
            else:
                print_verbose("[ERROR] Failed to read current speed.", 2, override=True)
        elif cmd == "status":
            state = cfw500.read_p0680()
            if state is not None:
                print_verbose(f"[STATUS] P0680 = 0x{state:04X}", 2, override=True)
                if VERBOSE_LEVEL == 3:
                    for bit in range(16):
                        if state & (1 << bit):
                            description = P0680_BITS.get(bit, f"Bit {bit} Unknown")
                            print_verbose(f"    Bit {bit}: {description} ACTIVE", 3)
            else:
                print_verbose("[ERROR] Failed to read inverter status.", 2, override=True)
        elif cmd == "reset_fault":
            cfw500.reset_fault()
            print_verbose("[ACTION] Fault reset command sent.", 2, override=True)
        elif cmd == "set_verbose":
            if len(parts) >= 2:
                level = int(parts[1])
                if level in [0, 1, 2, 3]:
                    VERBOSE_LEVEL = level
                    print_verbose(f"[INFO] Verbosity level set to {VERBOSE_LEVEL}.", 2, override=True)
                else:
                    print_verbose("[ERROR] Invalid verbosity level. Use a value between 0 and 3.", 2, override=True)
            else:
                print_verbose("[ERROR] Specify the verbosity level (0 to 3).", 2, override=True)
        elif cmd == "set_encoder_output":
            if len(parts) >= 2:
                mode = parts[1].lower()
                if mode in ["step", "deg"]:
                    encoder_output_mode = mode
                    print_verbose(f"[INFO] Encoder output mode set to '{encoder_output_mode}'.", 2, override=True)
                else:
                    print_verbose("[ERROR] Invalid encoder output mode. Use 'step' or 'deg'.", 2, override=True)
            else:
                print_verbose("[ERROR] Specify the encoder output mode ('step' or 'deg').", 2, override=True)
        elif cmd == "calibrate":
            encoder_offset = encoder_position
            save_configuration()
            print_verbose("[ACTION] Encoder calibrated. Current position set as zero.", 2, override=True)
        elif cmd == "read_offset":
            print_verbose(f"[INFO] Encoder offset: {encoder_offset}", 2, override=True)
        elif cmd == "read_max_rpm":
            max_rpm = cfw500.read_max_rpm()
            if max_rpm is not None:
                print_verbose(f"[INFO] Maximum RPM Read: {max_rpm} RPM", 2, override=True)
            else:
                print_verbose("[ERROR] Failed to read maximum RPM.", 2, override=True)
        elif cmd == "help":
            show_manual()
        elif cmd == "exit":
            print_verbose("[INFO] Exiting the program.", 2, override=True)
            return False  # Signal to exit
        elif cmd == "test":
            print_verbose("[INFO] Executing the default test sequence.", 2, override=True)
            cfw500.start_motor(1000)
            await asyncio.sleep(5)
            cfw500.stop_motor()
        else:
            print_verbose("[ERROR] Unrecognized command. Type 'help' to see the list of commands.", 2, override=True)
    except Exception as e:
        print_verbose(f"[ERROR] Exception occurred: {e}", 2, override=True)
    return True  # Continue running

async def await_serial_and_show_manual():
    """Waits for serial communication to be established, then shows the manual."""
    # Wait for serial communication to be established
    print("[DEBUG] Waiting for serial communication to be established...")
    await asyncio.sleep(1)
    # Display the instruction manual
    show_manual()

async def main():
    global VERBOSE_LEVEL, encoder_position, cfw500, zero_pin

    # Start the task to wait for serial communication and show the manual
    asyncio.create_task(await_serial_and_show_manual())

    # Read the maximum RPM from the inverter
    max_rpm = cfw500.read_max_rpm()
    if max_rpm is not None and VERBOSE_LEVEL >= 2:
        print_verbose(f"[INFO] Maximum RPM Read: {max_rpm} RPM", 2)
    elif max_rpm is None and VERBOSE_LEVEL >= 2:
        print_verbose("[ERROR] Failed to read maximum RPM.", 2)

    # Read the serial interface status
    serial_state = cfw500.read_p0316()
    if VERBOSE_LEVEL >= 2:
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
    if VERBOSE_LEVEL >= 2:
        if fault:
            print_verbose("[ALERT] The inverter is in FAULT.", 2)
        else:
            print_verbose("[INFO] The inverter is NOT in fault.", 2)

    # Configure the encoder pins with internal pull-ups
    pin_a = Pin(16, Pin.IN, Pin.PULL_UP)
    pin_b = Pin(17, Pin.IN, Pin.PULL_UP)

    # Initialize the Encoder
    encoder = Encoder(
        pin_x=pin_a,
        pin_y=pin_b,
        v=0,           # Initial value
        div=1,         # Division factor (adjust if needed)
        vmin=None,     # Optional minimum value
        vmax=None,     # Optional maximum value
        mod=None,      # Optional modulus
        callback=encoder_callback,  # Function to call on value change
        args=(),       # Arguments for callback
        delay=10       # Delay in ms between callbacks
    )

    # Zero Endstop Detection on GPIO18
    zero_pin = Pin(18, Pin.IN, Pin.PULL_UP)
    zero_pin.irq(trigger=Pin.IRQ_FALLING, handler=zero_position_callback)

    # Load configuration
    load_configuration()

    # Perform homing routine
    await homing()

    # Start tasks
    asyncio.create_task(led_blink_task())
    asyncio.create_task(read_user_input())
    asyncio.create_task(status_request_task())

    # Keep the main loop running
    while True:
        await asyncio.sleep(1)

try:
    asyncio.run(main())
except KeyboardInterrupt:
    if VERBOSE_LEVEL >= 2:
        print_verbose("\n[INFO] Program interrupted by user.", 2)
finally:
    asyncio.new_event_loop()
