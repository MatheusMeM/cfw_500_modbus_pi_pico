# relay_pico/main.py - Modbus RTU Master (Network 1)

import sys
import time
import uselect
from machine import UART, Pin
from umodbus.serial import Serial as ModbusRTUMaster # Assuming library is in lib folder or installed

# --- Constants ---
# Network 1: Relay (Master) <-> Main (Slave)
MAIN_PICO_SLAVE_ADDR = 1
RS485_UART_ID = 0
RS485_BAUDRATE = 19200 # Ensure this matches Main Pico's UART1 - REDUCED for testing
RS485_TX_PIN_NUM = 0  # GPIO0
RS485_RX_PIN_NUM = 1  # GPIO1
RS485_DE_RE_PIN_NUM = 2 # GPIO2

# Modbus Register Definitions (Matching Main Pico Slave Map)
# Holding Registers (Write to Main)
REG_CMD = 100         # Command Register (1=START, 2=STOP, 3=REVERSE, 4=RESET_FAULT, 5=CALIBRATE)
REG_TARGET_RPM = 101  # Target RPM
REG_VERBOSE = 102     # Verbosity Level (0-3)
REG_ENC_MODE = 103    # Encoder Output Mode (0=steps, 1=degrees)
# Input Registers (Read from Main)
REG_CURRENT_RPM = 0   # Current Motor RPM (x10)
REG_VFD_STATUS = 1    # VFD Status Word (P0680)
REG_ENC_POS_STEPS = 2 # Encoder Position (Steps)
REG_ENC_POS_DEG = 3   # Encoder Position (Degrees x100)
REG_FAULT_FLAG = 4    # Fault Detected Flag (0/1)
REG_HOMING_FLAG = 5   # Homing Completed Flag (0/1)
REG_OFFSET_STEPS = 6  # Calibrated Offset (Steps)

# --- Hardware Initialization ---
rs485_de_re_pin = Pin(RS485_DE_RE_PIN_NUM, Pin.OUT)
rs485_de_re_pin.value(0) # Start in receive mode

# Initialize UART for Modbus Master
# Note: umodbus library handles UART initialization details if passed pins
modbus_master = ModbusRTUMaster(
    pins=(RS485_TX_PIN_NUM, RS485_RX_PIN_NUM), # TX, RX
    baudrate=RS485_BAUDRATE,
    # Default bits=8, parity=None, stop=1 should be suitable
    ctrl_pin=RS485_DE_RE_PIN_NUM,
    uart_id=RS485_UART_ID # Removed invalid timeout argument
)

print("[INFO] Relay Pico initialized as Modbus Master. Waiting for communication...")

# --- Polling Setup ---
poller = uselect.poll()
poller.register(sys.stdin, uselect.POLLIN) # Poll USB input

# --- Main Loop ---
def run_master():
    last_status_read_time = time.ticks_ms()
    status_read_interval = 5000 # Read status from Main Pico every 5 seconds (ms) - Increased

    while True:
        # --- Check for commands from PC (USB) ---
        if poller.poll(0): # Check stdin immediately, non-blocking
            pc_command = sys.stdin.readline()
            if pc_command:
                pc_command = pc_command.strip()
                print(f"[PC CMD RX] {pc_command}")
                parts = pc_command.split()
                if not parts:
                    continue
                cmd = parts[0].lower()

                try:
                    # Translate PC command to Modbus Write
                    # --- !!! START OF MODIFICATION FOR RELAY PICO 'start' COMMAND !!! ---
                    if cmd == "start":
                        if len(parts) > 1: # RPM is provided
                            rpm = int(parts[1])
                            print(f"[MODBUS TX] Writing START command and RPM={rpm}")
                            time.sleep_ms(20)
                            # Send command 1 and the specified RPM
                            modbus_master.write_multiple_registers(MAIN_PICO_SLAVE_ADDR, REG_CMD, [1, rpm])
                            time.sleep_ms(20)
                        else: # No RPM provided with "start"
                            print(f"[MODBUS TX] Writing START command (using Main Pico's current Target RPM)")
                            time.sleep_ms(20)
                            # Send only command 1 to REG_CMD. Main Pico will use its stored REG_TARGET_RPM.
                            modbus_master.write_single_register(MAIN_PICO_SLAVE_ADDR, REG_CMD, 1)
                            time.sleep_ms(20)
                    # --- !!! END OF MODIFICATION FOR RELAY PICO 'start' COMMAND !!! ---
                    elif cmd == "stop":
                        print(f"[MODBUS TX] Writing STOP command")
                        time.sleep_ms(20) # Delay before write
                        modbus_master.write_single_register(MAIN_PICO_SLAVE_ADDR, REG_CMD, 2)
                        time.sleep_ms(20) # Delay after write
                    # --- !!! START OF MODIFICATION FOR RELAY PICO 'reverse' COMMAND (similar to start) !!! ---
                    elif cmd == "reverse":
                        if len(parts) > 1: # RPM is provided
                            rpm = int(parts[1])
                            print(f"[MODBUS TX] Writing REVERSE command and RPM={rpm}")
                            time.sleep_ms(20)
                            modbus_master.write_multiple_registers(MAIN_PICO_SLAVE_ADDR, REG_CMD, [3, rpm])
                            time.sleep_ms(20)
                        else: # No RPM provided with "reverse"
                            print(f"[MODBUS TX] Writing REVERSE command (using Main Pico's current Target RPM)")
                            time.sleep_ms(20)
                            modbus_master.write_single_register(MAIN_PICO_SLAVE_ADDR, REG_CMD, 3)
                            time.sleep_ms(20)
                        # --- !!! END OF MODIFICATION FOR RELAY PICO 'reverse' COMMAND !!! ---
                    elif cmd == "set_speed":
                         if len(parts) > 1:
                            rpm = int(parts[1])
                            print(f"[MODBUS TX] Writing Target RPM={rpm}")
                            # Send command 0 (no action) but update RPM
                            time.sleep_ms(20) # Delay before write
                            modbus_master.write_multiple_registers(MAIN_PICO_SLAVE_ADDR, REG_CMD, [0, rpm])
                            time.sleep_ms(20) # Delay after write
                         else:
                             print("[ERROR] Specify RPM for set_speed")
                    elif cmd == "reset_fault":
                        print(f"[MODBUS TX] Writing RESET_FAULT command")
                        time.sleep_ms(20) # Delay before write
                        modbus_master.write_single_register(MAIN_PICO_SLAVE_ADDR, REG_CMD, 4)
                        time.sleep_ms(20) # Delay after write
                    elif cmd == "calibrate":
                        print(f"[MODBUS TX] Writing CALIBRATE command")
                        time.sleep_ms(20) # Delay before write
                        modbus_master.write_single_register(MAIN_PICO_SLAVE_ADDR, REG_CMD, 5)
                        time.sleep_ms(20) # Delay after write
                    elif cmd == "home":
                        print(f"[MODBUS TX] Writing HOME command")
                        time.sleep_ms(20) # Delay before write
                        modbus_master.write_single_register(MAIN_PICO_SLAVE_ADDR, REG_CMD, 6) # Home is 6
                        time.sleep_ms(20) # Delay after write
                    elif cmd == "rotate":
                        if len(parts) > 1:
                            try:
                                angle = int(parts[1])
                                print(f"[MODBUS TX] Writing ROTATE command with angle={angle} degrees")
                                time.sleep_ms(20) # Delay before write
                                # Command 7 is ROTATE, with angle parameter
                                modbus_master.write_multiple_registers(MAIN_PICO_SLAVE_ADDR, REG_CMD, [7, angle])
                                time.sleep_ms(20) # Delay after write
                            except ValueError:
                                print("[ERROR] Angle must be an integer value")
                        else:
                            print("[ERROR] Specify an angle for rotation in degrees")
                    elif cmd == "set_verbose":
                        if len(parts) > 1:
                            level = int(parts[1])
                            if 0 <= level <= 3:
                                print(f"[MODBUS TX] Writing Verbosity Level={level}")
                                time.sleep_ms(20) # Delay before write
                                modbus_master.write_single_register(MAIN_PICO_SLAVE_ADDR, REG_VERBOSE, level)
                                time.sleep_ms(20) # Delay after write
                            else:
                                print("[ERROR] Verbosity level must be 0-3")
                        else:
                            print("[ERROR] Specify verbosity level (0-3)")
                    elif cmd == "set_encoder_output":
                         if len(parts) > 1:
                            mode_str = parts[1].lower()
                            mode_val = 0 if mode_str == "step" else 1 if mode_str == "deg" else -1
                            if mode_val != -1:
                                print(f"[MODBUS TX] Writing Encoder Mode={mode_val} ({mode_str})")
                                time.sleep_ms(20) # Delay before write
                                modbus_master.write_single_register(MAIN_PICO_SLAVE_ADDR, REG_ENC_MODE, mode_val)
                                time.sleep_ms(20) # Delay after write
                            else:
                                print("[ERROR] Invalid encoder mode. Use 'step' or 'deg'")
                         else:
                             print("[ERROR] Specify encoder mode ('step' or 'deg')")
                    # Read commands are handled by periodic status read below
                    elif cmd in ["read_speed", "status", "read_offset", "read_max_rpm", "help", "exit", "test"]:
                         # These commands now primarily trigger reads or are local info
                         print(f"[INFO] Command '{cmd}' acknowledged. Status/data read periodically.")
                         if cmd == "help":
                             # Show help locally on PC side if needed, Main Pico doesn't need it
                             print("--- Relay Pico Help ---")
                             print("Commands are sent to Main Pico via Modbus.")
                             print("Status (speed, VFD status, encoder) is read periodically.")
                             print("Available commands to send: start [rpm], stop, reverse [rpm], set_speed [rpm], reset_fault, calibrate, home, rotate [angle], set_verbose [0-3], set_encoder_output [step|deg]")
                    else:
                        print(f"[ERROR] Unknown command: {cmd}")

                except Exception as e:
                    print(f"[ERROR] Modbus TX Error: {e}")

        # --- Periodically read status from Main Pico ---
        current_time = time.ticks_ms()
        if time.ticks_diff(current_time, last_status_read_time) >= status_read_interval:
            last_status_read_time = current_time
            try:
                # Read multiple input registers in one go
                print("[MODBUS RX] Reading status registers from Main Pico...")
                time.sleep_ms(20) # Delay before read
                input_regs = modbus_master.read_input_registers(
                    slave_addr=MAIN_PICO_SLAVE_ADDR,
                    starting_addr=REG_CURRENT_RPM,
                    register_qty=7 # Read all defined input registers
                )
                time.sleep_ms(20) # Delay after read attempt

                if input_regs:
                    # Process and print received data to PC (USB)
                    current_rpm = input_regs[REG_CURRENT_RPM - REG_CURRENT_RPM] / 10.0
                    vfd_status = input_regs[REG_VFD_STATUS - REG_CURRENT_RPM]
                    enc_steps = input_regs[REG_ENC_POS_STEPS - REG_CURRENT_RPM]
                    # Handle potential signed value for steps if needed (assuming unsigned for now)
                    enc_deg = input_regs[REG_ENC_POS_DEG - REG_CURRENT_RPM] / 100.0
                    fault_flag = bool(input_regs[REG_FAULT_FLAG - REG_CURRENT_RPM])
                    homing_flag = bool(input_regs[REG_HOMING_FLAG - REG_CURRENT_RPM])
                    offset_steps = input_regs[REG_OFFSET_STEPS - REG_CURRENT_RPM]

                    print(f"[STATUS RX] RPM: {current_rpm:.1f}, VFD: 0x{vfd_status:04X}, EncSteps: {enc_steps}, EncDeg: {enc_deg:.2f}, Fault: {fault_flag}, Homing: {homing_flag}, Offset: {offset_steps}")

                else:
                    print("[ERROR] Modbus RX Error: No response from Main Pico status read.")

            except Exception as e:
                print(f"[ERROR] Modbus RX Error: {e}")

        time.sleep_ms(20) # Small delay to prevent tight loop hammering

# --- Run ---
if __name__ == "__main__":
    try:
        run_master()
    except KeyboardInterrupt:
        print("\n[INFO] Relay Pico program interrupted by user.")
    finally:
        # Cleanup if needed
        pass
