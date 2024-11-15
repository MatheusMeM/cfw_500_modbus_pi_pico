# main.py

import time
import sys
import uasyncio as asyncio
from machine import Pin
from cfw500_modbus import CFW500Modbus
from encoder import Encoder
import select

# Configuration
UART_ID = 0  # UART0 for Modbus RTU
TX_PIN_NUM = 0  # GPIO0 (TX) for UART0
RX_PIN_NUM = 1  # GPIO1 (RX) for UART0
DE_RE_PIN = Pin(2, Pin.OUT)
DE_RE_PIN.value(0)  # Initially enable receiver
SLAVE_ADDRESS = 1  # Modbus slave address of the inverter

# Initialize Modbus and CFW500
cfw500 = CFW500Modbus(
    uart_id=UART_ID,
    tx_pin=TX_PIN_NUM,
    rx_pin=RX_PIN_NUM,
    de_re_pin=DE_RE_PIN,
    slave_address=SLAVE_ADDRESS
)

# LED Configuration
LED_PIN = 25  # On-board LED pin on Raspberry Pi Pico
led = Pin(LED_PIN, Pin.OUT)

# Global Variables
VERBOSE_LEVEL = 1
encoder_position = 0        # Current encoder position
encoder_offset = 0          # Offset for calibration
encoder_output_mode = "step"  # "step" or "deg"

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

def print_verbose(message, level):
    """Prints messages according to the verbosity level."""
    if VERBOSE_LEVEL >= level and VERBOSE_LEVEL > 0:
        print(message)

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
    - calibrate: Set current encoder position as zero.
    - read_max_rpm: Read the maximum RPM from the inverter.
    - help: Display this instruction manual.
    - exit: Exit the program.
    - test: Execute the default test sequence.
    ===========================================
    """
    print_verbose(manual, 3)

async def led_blink_task():
    """Task to blink the LED."""
    while True:
        led.toggle()
        await asyncio.sleep(0.5)

# Callback function to handle encoder changes
def encoder_callback(value, delta):
    global encoder_position
    encoder_position = value
    adjusted_position = encoder_position - encoder_offset

    if encoder_output_mode == "deg":
        # Convert steps to degrees
        degrees = (adjusted_position / 2000) * 360  # 2000 pulses per revolution
        output = f"Encoder Position: {degrees:.2f} degrees"
    else:
        # Output in steps
        output = f"Encoder Position: {adjusted_position} steps"

    if VERBOSE_LEVEL == 1 or VERBOSE_LEVEL == 3:
        print(output)

# Zero Endstop Position Detection
def zero_position_callback(pin):
    global encoder_position, encoder_offset
    encoder_offset = encoder_position
    print_verbose("[INFO] Zero endstop position hit! Encoder position reset.", 1)

async def read_user_input():
    loop = asyncio.get_event_loop()
    while True:
        if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
            user_input = sys.stdin.readline().strip()
            if user_input:
                should_continue = await process_command(user_input)
                if not should_continue:
                    loop.stop()
                    break
        await asyncio.sleep(0.1)

async def status_request_task():
    STATUS_REQUEST_INTERVAL = 5  # Interval in seconds
    while True:
        fault = cfw500.check_fault()
        if VERBOSE_LEVEL >= 2:
            if fault:
                print("[ALERT] The inverter is in FAULT.")
            else:
                print("[INFO] The inverter is NOT in fault.")
        await asyncio.sleep(STATUS_REQUEST_INTERVAL)

async def process_command(command):
    """Processes a single command."""
    global VERBOSE_LEVEL, encoder_output_mode, encoder_offset
    parts = command.strip().split()
    if not parts:
        return True  # Continue running
    cmd = parts[0].lower()

    try:
        if cmd == "start":
            rpm = float(parts[1]) if len(parts) >= 2 else 1000  # Default value
            cfw500.start_motor(rpm)
            if VERBOSE_LEVEL >= 2:
                print(f"[ACTION] Starting the motor at {rpm} RPM.")
        elif cmd == "stop":
            cfw500.stop_motor()
            if VERBOSE_LEVEL >= 2:
                print("[ACTION] Stopping the motor.")
        elif cmd == "reverse":
            rpm = float(parts[1]) if len(parts) >= 2 else 1000  # Default value
            cfw500.reverse_motor(rpm)
            if VERBOSE_LEVEL >= 2:
                print(f"[ACTION] Reversing the motor at {rpm} RPM.")
        elif cmd == "set_speed":
            if len(parts) >= 2:
                rpm = float(parts[1])
                cfw500.set_speed_reference(rpm)
                if VERBOSE_LEVEL >= 2:
                    print(f"[ACTION] Setting speed reference to {rpm} RPM.")
            else:
                if VERBOSE_LEVEL >= 2:
                    print("[ERROR] Specify the speed in RPM.")
        elif cmd == "read_speed":
            rpm = cfw500.read_current_speed()
            if rpm is not None and VERBOSE_LEVEL >= 2:
                print(f"[INFO] Current Speed: {rpm:.2f} RPM")
            elif rpm is None and VERBOSE_LEVEL >= 2:
                print("[ERROR] Failed to read current speed.")
        elif cmd == "status":
            state = cfw500.read_p0680()
            if state is not None and VERBOSE_LEVEL >= 2:
                print(f"[STATUS] P0680 = 0x{state:04X}")
                if VERBOSE_LEVEL == 3:
                    for bit in range(16):
                        if state & (1 << bit):
                            description = P0680_BITS.get(bit, f"Bit {bit} Unknown")
                            print(f"    Bit {bit}: {description} ACTIVE")
            elif state is None and VERBOSE_LEVEL >= 2:
                print("[ERROR] Failed to read inverter status.")
        elif cmd == "reset_fault":
            cfw500.reset_fault()
            if VERBOSE_LEVEL >= 2:
                print("[ACTION] Fault reset command sent.")
        elif cmd == "set_verbose":
            if len(parts) >= 2:
                level = int(parts[1])
                if level in [0, 1, 2, 3]:
                    VERBOSE_LEVEL = level
                    if VERBOSE_LEVEL >= 2:
                        print(f"[INFO] Verbosity level set to {VERBOSE_LEVEL}.")
                else:
                    if VERBOSE_LEVEL >= 2:
                        print("[ERROR] Invalid verbosity level. Use a value between 0 and 3.")
            else:
                if VERBOSE_LEVEL >= 2:
                    print("[ERROR] Specify the verbosity level (0 to 3).")
        elif cmd == "set_encoder_output":
            if len(parts) >= 2:
                mode = parts[1].lower()
                if mode in ["step", "deg"]:
                    encoder_output_mode = mode
                    if VERBOSE_LEVEL >= 2:
                        print(f"[INFO] Encoder output mode set to '{encoder_output_mode}'.")
                else:
                    if VERBOSE_LEVEL >= 2:
                        print("[ERROR] Invalid encoder output mode. Use 'step' or 'deg'.")
            else:
                if VERBOSE_LEVEL >= 2:
                    print("[ERROR] Specify the encoder output mode ('step' or 'deg').")
        elif cmd == "calibrate":
            encoder_offset = encoder_position
            if VERBOSE_LEVEL >= 2:
                print("[ACTION] Encoder calibrated. Current position set as zero.")
        elif cmd == "read_max_rpm":
            max_rpm = cfw500.read_max_rpm()
            if max_rpm is not None and VERBOSE_LEVEL >= 2:
                print(f"[INFO] Maximum RPM Read: {max_rpm} RPM")
            elif max_rpm is None and VERBOSE_LEVEL >= 2:
                print("[ERROR] Failed to read maximum RPM.")
        elif cmd == "help":
            show_manual()
        elif cmd == "exit":
            if VERBOSE_LEVEL >= 2:
                print("[INFO] Exiting the program.")
            return False  # Signal to exit
        elif cmd == "test":
            if VERBOSE_LEVEL >= 2:
                print("[INFO] Executing the default test sequence.")
            cfw500.start_motor(1000)
            await asyncio.sleep(5)
            cfw500.stop_motor()
        else:
            if VERBOSE_LEVEL >= 2:
                print("[ERROR] Unrecognized command. Type 'help' to see the list of commands.")
    except Exception as e:
        if VERBOSE_LEVEL >= 2:
            print(f"[ERROR] {e}")
    return True  # Continue running

async def main():
    global VERBOSE_LEVEL, encoder_position

    # Display the instruction manual
    show_manual()

    # Read the maximum RPM from the inverter
    max_rpm = cfw500.read_max_rpm()
    if max_rpm is not None and VERBOSE_LEVEL >= 2:
        print(f"[INFO] Maximum RPM Read: {max_rpm} RPM")
    elif max_rpm is None and VERBOSE_LEVEL >= 2:
        print("[ERROR] Failed to read maximum RPM.")

    # Read the serial interface status
    serial_state = cfw500.read_p0316()
    if VERBOSE_LEVEL >= 2:
        if serial_state == 0:
            print("[SERIAL] Serial Interface Inactive")
        elif serial_state == 1:
            print("[SERIAL] Serial Interface Active")
        elif serial_state == 2:
            print("[SERIAL] Watchdog Error on Serial Interface")
        else:
            print(f"[SERIAL] Unknown State ({serial_state})")

    # Check for faults
    fault = cfw500.check_fault()
    if VERBOSE_LEVEL >= 2:
        if fault:
            print("[ALERT] The inverter is in FAULT.")
        else:
            print("[INFO] The inverter is NOT in fault.")

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
        delay=100      # Delay in ms between callbacks
    )

    # Zero Endstop Detection on GPIO18
    zero_pin = Pin(18, Pin.IN, Pin.PULL_UP)
    zero_pin.irq(trigger=Pin.IRQ_FALLING, handler=zero_position_callback)

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
        print("\n[INFO] Program interrupted by user.")
finally:
    asyncio.new_event_loop()
