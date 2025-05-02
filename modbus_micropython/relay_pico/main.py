# relay_pico/main.py - Modbus RTU Master (Network 1)

import sys
import time
import asyncio # Added for async operations
import select  # Added for reliable stdin check
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
REG_MAX_RPM = 7       # Max RPM from VFD (P0208) - Added missing register definition

# --- Modbus Statistics Tracking (Optional but Recommended) ---
class ModbusStatistics:
   """Track Modbus communication statistics"""
   def __init__(self):
       self.reset()

   def reset(self):
       """Reset all statistics"""
       self.total_tx = 0
       self.total_rx = 0
       self.tx_errors = 0
       self.rx_errors = 0
       self.crc_errors = 0
       self.timeouts = 0
       self.retries = 0
       self.avg_response_time_ms = 0.0
       self.max_response_time_ms = 0
       self._response_time_sum = 0

   def record_tx(self, success=True):
       self.total_tx += 1
       if not success: self.tx_errors += 1

   def record_rx(self, success=True, crc_error=False, response_time_ms=None):
       if success:
           self.total_rx += 1
           if response_time_ms is not None:
               self._response_time_sum += response_time_ms
               self.avg_response_time_ms = self._response_time_sum / self.total_rx
               if response_time_ms > self.max_response_time_ms:
                   self.max_response_time_ms = response_time_ms
       else:
           self.rx_errors += 1
           if crc_error: self.crc_errors += 1

   def record_timeout(self):
       self.timeouts += 1

   def record_retry(self):
       self.retries += 1

   def get_success_rate(self):
       if self.total_tx == 0: return 100.0
       return (self.total_rx / self.total_tx) * 100

   def get_stats_dict(self):
       return {
           "tx": self.total_tx, "rx": self.total_rx, "tx_errors": self.tx_errors,
           "rx_errors": self.rx_errors, "crc_errors": self.crc_errors, "timeouts": self.timeouts,
           "retries": self.retries, "avg_response_ms": self.avg_response_time_ms,
           "max_response_ms": self.max_response_time_ms, "success_rate": self.get_success_rate()
       }

   def __str__(self):
       return (f"Modbus Stats: TX={self.total_tx}, RX={self.total_rx}, "
               f"Errors={self.tx_errors+self.rx_errors}(CRC:{self.crc_errors},TO:{self.timeouts}), "
               f"Retries={self.retries}, Success={self.get_success_rate():.1f}%, "
               f"AvgTime={self.avg_response_time_ms:.1f}ms, MaxTime={self.max_response_time_ms}ms")

master_stats = ModbusStatistics()

# --- Hardware Initialization ---
# rs485_de_re_pin = Pin(RS485_DE_RE_PIN_NUM, Pin.OUT) # Removed: Let library handle DE/RE via ctrl_pin
# rs485_de_re_pin.value(0) # Start in receive mode

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

# --- Async Read Helper ---
async def read_registers_with_retry(modbus_func, slave_addr, starting_addr, register_qty, max_retries=3):
   """Async Modbus read with exponential back-off and statistics"""
   retry = 0
   delay_ms = 50 # Initial delay
   last_exception = None

   while retry < max_retries:
       start_time = time.ticks_ms()
       try:
           master_stats.record_tx() # Record TX attempt
           # The library handles DE/RE timing via ctrl_pin, no manual sleep needed here
           result = modbus_func( # Call the actual read function (e.g., modbus_master.read_input_registers)
               slave_addr=slave_addr,
               starting_addr=starting_addr,
               register_qty=register_qty
           )
           response_time = time.ticks_diff(time.ticks_ms(), start_time)
           master_stats.record_rx(success=True, response_time_ms=response_time) # Record RX success
           return result # Success
       except Exception as e:
           last_exception = e
           response_time = time.ticks_diff(time.ticks_ms(), start_time)
           # Basic error type checking
           err_str = str(e).lower()
           if "timeout" in err_str or "no data received" in err_str:
                master_stats.record_timeout()
                is_timeout = True
           else:
                is_timeout = False

           if "crc" in err_str:
                master_stats.record_rx(success=False, crc_error=True, response_time_ms=response_time)
           elif not is_timeout: # Avoid double-counting timeouts as RX errors
                master_stats.record_rx(success=False, response_time_ms=response_time)

           print(f"[WARN] Modbus read failed (Attempt {retry+1}/{max_retries}): {e}")
           retry += 1
           if retry < max_retries:
               master_stats.record_retry() # Record retry
               print(f"[INFO] Retrying in {delay_ms}ms...")
               await asyncio.sleep_ms(delay_ms) # Use async sleep
               delay_ms = min(delay_ms * 2, 500) # Exponential back-off, capped
           else:
                print(f"[ERROR] Modbus read failed after {max_retries} attempts.")

   # All retries failed
   return None # Or raise last_exception

# --- Stdin Check Helper ---
def check_stdin_input():
   """Non-blocking read from stdin using select"""
   # Check if stdin has data waiting (0 timeout = non-blocking)
   if select.select([sys.stdin], [], [], 0)[0]:
       # Read the waiting line
       line = sys.stdin.readline()
       return line.strip() if line else None
   return None

# --- Main Async Loop ---
async def run_master():
    last_status_read_time = time.ticks_ms()
    status_read_interval = 5000 # Read status from Main Pico every 5 seconds (ms) - Increased

    while True:
        # --- Check for commands from PC (USB) ---
        pc_command = check_stdin_input() # Use non-blocking select-based check
        if pc_command:
            # pc_command = pc_command.strip() # Already stripped in check_stdin_input
                print(f"[PC CMD RX] {pc_command}")
                parts = pc_command.split()
                if not parts:
                    continue
                cmd = parts[0].lower()

                try:
                    # Translate PC command to Modbus Write
                    if cmd == "start":
                        rpm = int(parts[1]) if len(parts) > 1 else 1000
                        print(f"[MODBUS TX] Writing START command and RPM={rpm}")
                        # time.sleep_ms(20) # Removed: Let library handle DE/RE timing
                        modbus_master.write_multiple_registers(MAIN_PICO_SLAVE_ADDR, REG_CMD, [1, rpm])
                        # time.sleep_ms(20) # Removed: Let library handle DE/RE timing
                    elif cmd == "stop":
                        print(f"[MODBUS TX] Writing STOP command")
                        # time.sleep_ms(20) # Removed: Let library handle DE/RE timing
                        modbus_master.write_single_register(MAIN_PICO_SLAVE_ADDR, REG_CMD, 2)
                        # time.sleep_ms(20) # Removed: Let library handle DE/RE timing
                    elif cmd == "reverse":
                        rpm = int(parts[1]) if len(parts) > 1 else 1000
                        print(f"[MODBUS TX] Writing REVERSE command and RPM={rpm}")
                        # time.sleep_ms(20) # Removed: Let library handle DE/RE timing
                        modbus_master.write_multiple_registers(MAIN_PICO_SLAVE_ADDR, REG_CMD, [3, rpm])
                        # time.sleep_ms(20) # Removed: Let library handle DE/RE timing
                    elif cmd == "set_speed":
                         if len(parts) > 1:
                            rpm = int(parts[1])
                            print(f"[MODBUS TX] Writing Target RPM={rpm}")
                            # Send command 0 (no action) but update RPM
                            # time.sleep_ms(20) # Removed: Let library handle DE/RE timing
                            modbus_master.write_multiple_registers(MAIN_PICO_SLAVE_ADDR, REG_CMD, [0, rpm])
                            # time.sleep_ms(20) # Removed: Let library handle DE/RE timing
                         else:
                             print("[ERROR] Specify RPM for set_speed")
                    elif cmd == "reset_fault":
                        print(f"[MODBUS TX] Writing RESET_FAULT command")
                        # time.sleep_ms(20) # Removed: Let library handle DE/RE timing
                        modbus_master.write_single_register(MAIN_PICO_SLAVE_ADDR, REG_CMD, 4)
                        # time.sleep_ms(20) # Removed: Let library handle DE/RE timing
                    elif cmd == "calibrate":
                        print(f"[MODBUS TX] Writing CALIBRATE command")
                        # time.sleep_ms(20) # Removed: Let library handle DE/RE timing
                        modbus_master.write_single_register(MAIN_PICO_SLAVE_ADDR, REG_CMD, 5)
                        # time.sleep_ms(20) # Removed: Let library handle DE/RE timing
                    elif cmd == "set_verbose":
                        if len(parts) > 1:
                            level = int(parts[1])
                            if 0 <= level <= 3:
                                print(f"[MODBUS TX] Writing Verbosity Level={level}")
                                # time.sleep_ms(20) # Removed: Let library handle DE/RE timing
                                modbus_master.write_single_register(MAIN_PICO_SLAVE_ADDR, REG_VERBOSE, level)
                                # time.sleep_ms(20) # Removed: Let library handle DE/RE timing
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
                                # time.sleep_ms(20) # Removed: Let library handle DE/RE timing
                                modbus_master.write_single_register(MAIN_PICO_SLAVE_ADDR, REG_ENC_MODE, mode_val)
                                # time.sleep_ms(20) # Removed: Let library handle DE/RE timing
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
                             print("Available commands to send: start [rpm], stop, reverse [rpm], set_speed [rpm], reset_fault, calibrate, set_verbose [0-3], set_encoder_output [step|deg]")
                    else:
                        print(f"[ERROR] Unknown command: {cmd}")

                except Exception as e:
                    print(f"[ERROR] Modbus TX Error: {e}")

        # --- Periodically read status from Main Pico ---
        current_time = time.ticks_ms()
        if time.ticks_diff(current_time, last_status_read_time) >= status_read_interval:
            last_status_read_time = current_time
            try:
                # Read multiple input registers using the async retry helper
                print("[MODBUS RX] Reading status registers from Main Pico...")
                input_regs = await read_registers_with_retry(
                    modbus_master.read_input_registers, # Pass the function to call
                    slave_addr=MAIN_PICO_SLAVE_ADDR,
                    starting_addr=REG_CURRENT_RPM,
                    register_qty=8 # CORRECTED: Read all 8 defined input registers
                )
                # No manual sleep needed here, handled by retry logic and main loop sleep

                if input_regs and len(input_regs) == 8: # Check response validity
                    # Process and print received data to PC (USB)
                    # Adjust indices based on starting address (REG_CURRENT_RPM = 0)
                    current_rpm = input_regs[0] / 10.0
                    vfd_status = input_regs[1]
                    enc_steps = input_regs[2] # Consider signed conversion if needed
                    enc_deg = input_regs[3] / 100.0
                    fault_flag = bool(input_regs[4])
                    homing_flag = bool(input_regs[5])
                    offset_steps = input_regs[6] # Consider signed conversion if needed
                    max_rpm = input_regs[7] # Added Max RPM read

                    print(f"[STATUS RX] RPM: {current_rpm:.1f}, VFD: 0x{vfd_status:04X}, EncSteps: {enc_steps}, EncDeg: {enc_deg:.2f}, Fault: {fault_flag}, Homing: {homing_flag}, Offset: {offset_steps}, MaxRPM: {max_rpm}")

                else:
                    # Error message already printed by read_registers_with_retry on failure
                    print("[ERROR] Modbus RX Error: Failed to read status registers after retries or invalid response length.")
                    # Optionally log stats on persistent failure
                    print(f"[STATS] {master_stats}")


            except Exception as e:
                # Catch potential errors in processing the response, not the read itself
                print(f"[ERROR] Processing Main Pico status failed: {e}")
                # Log stats on error
                print(f"[STATS] {master_stats}")


        await asyncio.sleep_ms(50) # Main loop sleep, adjust as needed

# --- Run ---
if __name__ == "__main__":
    try:
        asyncio.run(run_master())
    except KeyboardInterrupt:
        print("\n[INFO] Relay Pico program interrupted by user.")
    finally:
        # Cleanup if needed
        pass
