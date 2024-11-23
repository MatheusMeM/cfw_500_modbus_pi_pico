# commands.py

import uasyncio as asyncio
from utils import print_verbose, show_manual, state, save_configuration
from motor_control import initialize_cfw500
from cfw500_modbus import CFW500Modbus

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

async def process_command(command, cfw500):
    """Processes a single command."""
    print(f"[DEBUG] Processing command: {command}")
    parts = command.strip().split()
    if not parts:
        return True  # Continue running
    cmd = parts[0].lower()

    try:
        if cmd == "start":
            rpm = float(parts[1]) if len(parts) >= 2 else 1000  # Default value
            cfw500.start_motor(rpm)
            if state['VERBOSE_LEVEL'] >= 2:
                print_verbose(f"[ACTION] Starting the motor at {rpm} RPM.", 2)
        elif cmd == "stop":
            cfw500.stop_motor()
            if state['VERBOSE_LEVEL'] >= 2:
                print_verbose("[ACTION] Stopping the motor.", 2)
        elif cmd == "reverse":
            rpm = float(parts[1]) if len(parts) >= 2 else 1000  # Default value
            cfw500.reverse_motor(rpm)
            if state['VERBOSE_LEVEL'] >= 2:
                print_verbose(f"[ACTION] Reversing the motor at {rpm} RPM.", 2)
        elif cmd == "set_speed":
            if len(parts) >= 2:
                rpm = float(parts[1])
                cfw500.set_speed_reference(rpm)
                if state['VERBOSE_LEVEL'] >= 2:
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
            state_p0680 = cfw500.read_p0680()
            if state_p0680 is not None:
                print_verbose(f"[STATUS] P0680 = 0x{state_p0680:04X}", 2, override=True)
                if state['VERBOSE_LEVEL'] == 3:
                    for bit in range(16):
                        if state_p0680 & (1 << bit):
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
                    state['VERBOSE_LEVEL'] = level
                    print_verbose(f"[INFO] Verbosity level set to {state['VERBOSE_LEVEL']}.", 2, override=True)
                else:
                    print_verbose("[ERROR] Invalid verbosity level. Use a value between 0 and 3.", 2, override=True)
            else:
                print_verbose("[ERROR] Specify the verbosity level (0 to 3).", 2, override=True)
        elif cmd == "set_encoder_output":
            if len(parts) >= 2:
                mode = parts[1].lower()
                if mode in ["step", "deg"]:
                    state['encoder_output_mode'] = mode
                    save_configuration()  # Save the new setting
                    print_verbose(f"[INFO] Encoder output mode set to '{state['encoder_output_mode']}' and saved.", 2, override=True)
                else:
                    print_verbose("[ERROR] Invalid encoder output mode. Use 'step' or 'deg'.", 2, override=True)
            else:
                print_verbose("[ERROR] Specify the encoder output mode ('step' or 'deg').", 2, override=True)
        elif cmd == "calibrate":
            state['encoder_offset'] = state['encoder_position']
            save_configuration()
            print_verbose("[ACTION] Encoder calibrated. Current position set as zero.", 2, override=True)
        elif cmd == "read_offset":
            print_verbose(f"[INFO] Encoder offset: {state['encoder_offset']}", 2, override=True)
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
