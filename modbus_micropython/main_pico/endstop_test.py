# main_pico_Z_POLL_test.py
# Script to test Z-pin (GPIO18) by direct polling while VFD runs motor.
# NO IRQ WILL BE USED FOR THE Z-PIN in this script.

import uasyncio as asyncio
import time
from machine import Pin
from cfw500_modbus import CFW500Modbus # Assumes cfw500_modbus.py is in the root or lib
from utils import print_verbose       # Assumes utils.py with print_verbose is present

# --- Configuration ---
# VFD Communication
VFD_UART_ID = 0
VFD_TX_PIN_NUM = 0
VFD_RX_PIN_NUM = 1
VFD_DE_RE_PIN_NUM = 2
VFD_SLAVE_ADDRESS = 1
VFD_TEST_RPM = 1500      # A slow, safe speed for this test
MOTOR_RUN_DURATION_S = 30 # How long to run the motor

# Z-Pin (Endstop)
ZERO_ENDSTOP_PIN_NUM = 18

# Polling Configuration
POLL_INTERVAL_MS = 1 # Poll very frequently to try and catch changes

# --- Global State for this Test ---
internal_state = {'VERBOSE_LEVEL': 3} 

# --- Hardware Initialization ---
vfd_master = CFW500Modbus(VFD_UART_ID, VFD_TX_PIN_NUM, VFD_RX_PIN_NUM, VFD_DE_RE_PIN_NUM, VFD_SLAVE_ADDRESS)
print_verbose("[INFO] VFD Master initialized.", 0)

# Z-Pin (Endstop) - configured with internal pull-up
zero_pin = Pin(ZERO_ENDSTOP_PIN_NUM, Pin.IN, Pin.PULL_UP)
print_verbose(f"[INFO] Zero Endstop Pin ({ZERO_ENDSTOP_PIN_NUM}) initialized with PULL_UP for polling.", 0)
print_verbose("*** RECOMMENDATION: Ensure an external 4.7kOhm pull-up resistor is on the Z-pin for better noise immunity, even for polling. ***", 0)

# --- Test Coroutine ---
async def run_z_pin_polling_test():
    print_verbose("--- Z-PIN POLLING TEST SCRIPT ---", 0)

    # Ensure motor is stopped initially
    try:
        print_verbose("[TEST] Ensuring motor is stopped initially...", 1)
        vfd_master.stop_motor()
        await asyncio.sleep_ms(3500) 
        print_verbose("[TEST] Initial motor stop command sent and settled.", 1)
    except Exception as e:
        print_verbose(f"[ERROR] Failed initial VFD stop: {e}", 0)
        return

    # Start Motor
    print_verbose(f"[TEST] Starting motor forward at {VFD_TEST_RPM} RPM...", 1)
    try:
        vfd_master.start_motor(VFD_TEST_RPM)
        print_verbose("[TEST] Motor start command sent.", 1)
    except Exception as e:
        print_verbose(f"[ERROR] Failed to start motor: {e}", 0)
        return

    last_pin_state = zero_pin.value()
    print_verbose(f"[POLL TEST] Initial Z-pin state before run: {last_pin_state}", 1)
    
    high_to_low_transitions = 0
    low_to_high_transitions = 0
    
    start_time_ms = time.ticks_ms()
    loop_count = 0

    print_verbose("Timestamp(ms)\tLoop#\tZ_Pin_State\tComment", 0)

    try:
        while time.ticks_diff(time.ticks_ms(), start_time_ms) < (MOTOR_RUN_DURATION_S * 1000):
            current_pin_state = zero_pin.value()
            loop_count += 1
            
            log_entry = f"{time.ticks_diff(time.ticks_ms(), start_time_ms)}\t{loop_count}\t{current_pin_state}"

            if current_pin_state != last_pin_state:
                if last_pin_state == 1 and current_pin_state == 0: # Falling edge
                    high_to_low_transitions += 1
                    log_entry += f"\tFALLING_EDGE ({high_to_low_transitions})"
                elif last_pin_state == 0 and current_pin_state == 1: # Rising edge
                    low_to_high_transitions += 1
                    log_entry += f"\tRISING_EDGE ({low_to_high_transitions})"
                else: # Should not happen if binary
                    log_entry += f"\tUNEXPECTED_CHANGE from {last_pin_state}"
                
                # Print every change
                print_verbose(log_entry, 2) 
                last_pin_state = current_pin_state
            else:
                # Optionally, print periodically even if no change, to show it's alive
                if loop_count % (1000 // POLL_INTERVAL_MS) == 0: # Roughly every 1 second
                    print_verbose(log_entry + "\t(No Change)", 3)


            await asyncio.sleep_ms(POLL_INTERVAL_MS)

    except KeyboardInterrupt:
        print_verbose("Polling loop interrupted.",0)
    except Exception as e:
        print_verbose(f"Error during polling: {e}",0)
    finally:
        print_verbose("\n--- POLLING SUMMARY ---", 0)
        print_verbose(f"Total High-to-Low (Falling) transitions: {high_to_low_transitions}", 0)
        print_verbose(f"Total Low-to-High (Rising) transitions: {low_to_high_transitions}", 0)
        print_verbose(f"Total polling loops: {loop_count}", 0)

        print_verbose("[TEST] Stopping motor...", 1)
        try:
            vfd_master.stop_motor()
            await asyncio.sleep_ms(3500) 
            print_verbose("[TEST] Motor stopped.", 1)
        except Exception as e:
            print_verbose(f"[ERROR] Failed to stop motor: {e}", 0)

    print_verbose("--- Z-PIN POLLING TEST SCRIPT COMPLETE ---", 0)

# --- Main Execution ---
if __name__ == "__main__":
    try:
        asyncio.run(run_z_pin_polling_test())
    except KeyboardInterrupt:
        print_verbose("\n[INFO] Test script interrupted by user.", 0)
    except Exception as e:
        print_verbose(f"[FATAL] Unhandled exception in test script: {e}", 0)
        import sys
        sys.print_exception(e)
    finally:
        try:
            print_verbose("[CLEANUP] Attempting final motor stop.",1)
            if 'vfd_master' in locals() and vfd_master is not None:
                vfd_master.stop_motor()
        except Exception:
            pass 
        asyncio.new_event_loop()