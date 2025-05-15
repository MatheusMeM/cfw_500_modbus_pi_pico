# main_pico_Z_IRQ_test_v2.py
# Comprehensive test for Z-pin IRQ with VFD motor control,
# including variable blanking periods and more detailed logging.

import uasyncio as asyncio
import time
from machine import Pin
from cfw500_modbus import CFW500Modbus
from utils import print_verbose # Assuming utils.py with print_verbose is present

# --- Configuration ---
# Encoder Parameters (CRITICAL - SET THESE TO YOUR ACTUAL ENCODER)
PPR = 2000 # Pulses Per Revolution for your E6B2-CWZ6C 2000P/R
# Note: MAX_STEPS for A/B quadrature is PPR*4, but Z-pulse is once per rev.

# VFD Communication
VFD_UART_ID = 0
VFD_TX_PIN_NUM = 0
VFD_RX_PIN_NUM = 1
VFD_DE_RE_PIN_NUM = 2
VFD_SLAVE_ADDRESS = 1

# Test Speeds & Durations
VFD_TEST_RPMS = [150, 300, 600] # Test with a few different slow speeds
MOTOR_RUN_DURATION_S = 20    # How long to run the motor in each test phase (to ensure Z-pulse can pass)
                               # At 1 RPM (60s/rev), 20s is 1/3 of a rev.
                               # At 150 RPM, Z-pulse passes >2 times per second.
SETTLE_TIME_MS = 3500        # Time for motor to settle after stop

# Z-Pin (Endstop)
ZERO_ENDSTOP_PIN_NUM = 18

# --- Global State for this Test ---
internal_state = {'VERBOSE_LEVEL': 3} 
endstop_event = asyncio.Event()
z_pin_irq_fire_count = 0
last_irq_time_us = 0
event_set_time_us = 0

# --- Hardware Initialization ---
vfd_master = CFW500Modbus(VFD_UART_ID, VFD_TX_PIN_NUM, VFD_RX_PIN_NUM, VFD_DE_RE_PIN_NUM, VFD_SLAVE_ADDRESS)
print_verbose("[INFO] VFD Master initialized.", 0)

zero_pin = Pin(ZERO_ENDSTOP_PIN_NUM, Pin.IN, Pin.PULL_UP)
print_verbose(f"[INFO] Zero Endstop Pin ({ZERO_ENDSTOP_PIN_NUM}) initialized with PULL_UP.", 0)
print_verbose("*** RECOMMENDATION: Use an external 4.7kOhm pull-up resistor on the Z-pin to 3.3V for better noise immunity. ***", 0)

# --- IRQ Handler ---
def z_pin_irq_handler(pin_obj):
    global z_pin_irq_fire_count, last_irq_time_us, event_set_time_us
    now = time.ticks_us()
    delta = time.ticks_diff(now, last_irq_time_us) if last_irq_time_us != 0 else 0
    last_irq_time_us = now
    
    z_pin_irq_fire_count += 1
    
    # Conditional print to avoid flooding if IRQs are very frequent
    if z_pin_irq_fire_count < 10 or (z_pin_irq_fire_count % 50 == 0) :
        print_verbose(f"!!! Z-IRQ Fired! Count: {z_pin_irq_fire_count}, PinStateAfter: {pin_obj.value()}, us_since_last: {delta}us", 1)
    
    if not endstop_event.is_set(): # Only record time for the first set
        event_set_time_us = now
    endstop_event.set()

# --- Test Coroutine ---
async def run_z_pin_test_cycle(test_rpm, blanking_period_ms):
    global z_pin_irq_fire_count, last_irq_time_us, event_set_time_us
    
    print_verbose(f"\n--- TEST CYCLE: RPM={test_rpm}, Blanking={blanking_period_ms}ms ---", 0)
    z_pin_irq_fire_count = 0
    last_irq_time_us = 0
    event_set_time_us = 0
    endstop_event.clear()

    initial_pin_state = zero_pin.value()
    print_verbose(f"[CYCLE] Zero Pin state BEFORE IRQ attach: {initial_pin_state}", 2)

    zero_pin.irq(trigger=Pin.IRQ_FALLING, handler=z_pin_irq_handler)
    print_verbose("[CYCLE] IRQ attached.", 2)

    # Check if IRQ fired just by attaching (pin already low or noise)
    await asyncio.sleep_ms(10) # Brief moment for an immediate false trigger
    if endstop_event.is_set():
        print_verbose(f"[CYCLE WARNING] Event set just after IRQ attach. Count: {z_pin_irq_fire_count}. PinState: {zero_pin.value()}", 1)
        endstop_event.clear() # Reset for motor start
        z_pin_irq_fire_count = 0 
        last_irq_time_us = 0

    motor_start_command_time_us = 0
    try:
        print_verbose(f"[CYCLE] Starting motor forward at {test_rpm} RPM...", 1)
        motor_start_command_time_us = time.ticks_us()
        vfd_master.start_motor(test_rpm)
        print_verbose("[CYCLE] Motor start command sent.", 1)

        if blanking_period_ms > 0:
            print_verbose(f"[CYCLE] Entering {blanking_period_ms}ms blanking period...", 2)
            await asyncio.sleep_ms(blanking_period_ms)
            
            if endstop_event.is_set():
                time_to_false_trigger_us = time.ticks_diff(event_set_time_us, motor_start_command_time_us)
                print_verbose(f"[CYCLE INFO] Event was set by noise during blanking (after {time_to_false_trigger_us/1000:.2f}ms from motor cmd). Clearing. IRQ Count: {z_pin_irq_fire_count}", 1)
                endstop_event.clear()
                # Don't reset z_pin_irq_fire_count here, let it accumulate to see total noise
            print_verbose("[CYCLE] Blanking period complete. Actively listening for true Z-pulse.", 2)
        else:
            print_verbose("[CYCLE] No blanking period. Listening immediately.", 2)

        # Wait for Z-pulse event or timeout
        run_duration_ms = MOTOR_RUN_DURATION_S * 1000
        print_verbose(f"[CYCLE] Waiting for Z-pulse event from rotation (max {MOTOR_RUN_DURATION_S} seconds)...", 1)
        
        initial_irq_count_post_blanking = z_pin_irq_fire_count
        start_wait_time_us = time.ticks_us()

        try:
            await asyncio.wait_for_ms(endstop_event.wait(), run_duration_ms)
            time_to_trigger_us = time.ticks_diff(event_set_time_us, start_wait_time_us) # Time from start of wait
            total_time_from_motor_cmd_us = time.ticks_diff(event_set_time_us, motor_start_command_time_us)
            
            print_verbose(f"[SUCCESS CYCLE] Z-pulse event received post-blanking!", 0)
            print_verbose(f"  Trigger time from wait start: {time_to_trigger_us/1000:.2f}ms")
            print_verbose(f"  Trigger time from motor cmd: {total_time_from_motor_cmd_us/1000:.2f}ms")
            print_verbose(f"  Pin state at event: {zero_pin.value()}, Total IRQs in cycle: {z_pin_irq_fire_count}")

        except asyncio.TimeoutError:
            irqs_during_run = z_pin_irq_fire_count - initial_irq_count_post_blanking
            print_verbose(f"[TIMEOUT CYCLE] Timeout waiting for Z-pulse during rotation (post-blanking).", 0)
            print_verbose(f"  Pin state at timeout: {zero_pin.value()}, IRQs during this run phase: {irqs_during_run}, Total IRQs in cycle: {z_pin_irq_fire_count}")

    except Exception as e:
        print_verbose(f"[ERROR CYCLE] Exception: {e}", 0)
    finally:
        print_verbose("[CYCLE] Stopping motor...", 1)
        try:
            vfd_master.stop_motor()
            await asyncio.sleep_ms(SETTLE_TIME_MS)
            print_verbose("[CYCLE] Motor stopped.", 1)
        except Exception as e:
            print_verbose(f"[ERROR CYCLE] Failed to stop motor: {e}", 0)
        finally:
            zero_pin.irq(handler=None)
            print_verbose("[CYCLE] IRQ detached.", 2)
    
    print_verbose(f"--- END OF CYCLE: RPM={test_rpm}, Blanking={blanking_period_ms}ms ---", 0)
    await asyncio.sleep_ms(2000) # Pause between different test configurations

async def main_test_routine():
    print_verbose("--- ADVANCED Z-PIN IRQ TEST SCRIPT ---", 0)
    
    # Ensure motor is stopped initially
    try:
        print_verbose("[SETUP] Ensuring motor is stopped initially...", 1)
        vfd_master.stop_motor()
        await asyncio.sleep_ms(SETTLE_TIME_MS)
    except Exception as e:
        print_verbose(f"[ERROR SETUP] Failed initial VFD stop: {e}", 0)
        return

    blanking_periods = [0, 250, 500, 750, 1000] # Test different blanking periods in ms

    for rpm in VFD_TEST_RPMS:
        for bp_ms in blanking_periods:
            # Before each cycle, ensure the Z-pulse is not currently active if possible
            # (Since motor is heavy, this means we rely on previous stop position)
            current_z_state = zero_pin.value()
            print_verbose(f"\nPreparing for RPM: {rpm}, Blanking: {bp_ms}ms. Current Z-pin: {current_z_state}", 1)
            if current_z_state == 0:
                print_verbose("WARNING: Z-pin is LOW before test cycle. Consider VFD jog if available or ignore this cycle's Z-active start.", 1)
                # Ideally, jog motor slightly if Z is active, but VFD lib doesn't have simple jog.
                # For now, we proceed and note the initial state.
            
            await run_z_pin_test_cycle(rpm, bp_ms)
            if not await safe_yield(1000): # Pause between major parameter changes
                 return # User interrupted

    print_verbose("--- ADVANCED Z-PIN IRQ TEST SCRIPT COMPLETE ---", 0)

async def safe_yield(duration_ms):
    try:
        await asyncio.sleep_ms(duration_ms)
        return True
    except KeyboardInterrupt:
        print_verbose("Yield interrupted by user.",0)
        raise # Re-raise to be caught by main try-except
    except Exception as e:
        print_verbose(f"Error during safe_yield: {e}",0)
        return False


# --- Main Execution ---
if __name__ == "__main__":
    try:
        asyncio.run(main_test_routine())
    except KeyboardInterrupt:
        print_verbose("\n[INFO] Test script interrupted by user.", 0)
    except Exception as e:
        print_verbose(f"[FATAL] Unhandled exception in test script: {e}", 0)
        import sys
        sys.print_exception(e) # Print full traceback for unhandled
    finally:
        try:
            print_verbose("[CLEANUP] Attempting final motor stop.",1)
            if 'vfd_master' in locals() and vfd_master is not None:
                vfd_master.stop_motor()
        except Exception:
            pass 
        asyncio.new_event_loop()