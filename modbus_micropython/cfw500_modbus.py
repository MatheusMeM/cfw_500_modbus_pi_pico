# cfw500_modbus.py

import time
from umodbus.serial import Serial as ModbusRTUMaster

class CFW500Modbus:
    def __init__(self, uart_id, tx_pin, rx_pin, de_re_pin, slave_address=1):
        self.slave_address = slave_address
        self.modbus_master = ModbusRTUMaster(
            pins=(tx_pin, rx_pin),
            baudrate=19200,
            data_bits=8,
            stop_bits=1,
            parity=None,
            ctrl_pin=de_re_pin,
            uart_id=uart_id
        )
        self.max_RPM = 1700.0  # Default value, can be updated

    def read_p0316(self):
        """Reads the Serial Interface Status (P0316)."""
        response = self.modbus_master.read_holding_registers(
            slave_addr=self.slave_address,
            starting_addr=316,
            register_qty=1
        )
        if response and len(response) == 1:
            state = response[0]
            return state
        else:
            return None

    def read_p0680(self):
        """Reads the Logic Status (P0680) and returns the raw value."""
        response = self.modbus_master.read_holding_registers(
            slave_addr=self.slave_address,
            starting_addr=680,
            register_qty=1
        )
        if response and len(response) == 1:
            state = response[0]
            return state
        else:
            return None

    def check_fault(self):
        """Checks if the inverter is in fault via bit 15 of P0680."""
        state = self.read_p0680()
        if state is not None:
            fault = bool(state & (1 << 15))  # Bit 15 indicates fault
            return fault
        else:
            return None

    def start_motor(self, rpm_reference):
        """Starts the motor with the specified RPM reference."""
        control_word = 0x0007  # Bits 0, 1, 2 set to start forward
        self.modbus_master.write_single_register(
            slave_addr=self.slave_address,
            register_address=682,
            register_value=control_word
        )
        self.set_speed_reference(rpm_reference)

    def stop_motor(self):
        """Stops the motor."""
        control_word = 0x0002  # Bit 1 enabled, others cleared
        self.modbus_master.write_single_register(
            slave_addr=self.slave_address,
            register_address=682,
            register_value=control_word
        )

    def reverse_motor(self, rpm_reference):
        """Reverses the motor direction with the specified RPM reference."""
        control_word = 0x0003  # Bits 0 and 1 set to start reverse
        self.modbus_master.write_single_register(
            slave_addr=self.slave_address,
            register_address=682,
            register_value=control_word
        )
        self.set_speed_reference(rpm_reference)

    def set_speed_reference(self, rpm_value):
        """Sets the speed reference (P0208) based on RPM."""
        if self.max_RPM == 0:
            raise ValueError("Maximum RPM is zero. Cannot set speed.")

        # Ensure RPM is within limits
        rpm_value = max(0, min(rpm_value, self.max_RPM))

        # Map RPM to Modbus value considering 1700 RPM = 8192 (0x2000)
        speed_reference_value = int((rpm_value / self.max_RPM) * 8192)

        # Limit the reference value within 16-bit signed integer limits
        speed_reference_value = max(-32768, min(speed_reference_value, 32767))

        self.modbus_master.write_single_register(
            slave_addr=self.slave_address,
            register_address=683,
            register_value=speed_reference_value
        )

    def read_current_speed(self):
        """Reads the Current Speed (P0681) and returns RPM."""
        num_pole_pairs = 2  # Adjust according to the motor used

        # Read the motor's nominal frequency (P0403)
        response_nominal_freq = self.modbus_master.read_holding_registers(
            slave_addr=self.slave_address,
            starting_addr=403,  # P0403
            register_qty=1
        )
        if not response_nominal_freq or len(response_nominal_freq) != 1:
            return None
        nominal_freq = response_nominal_freq[0]

        # Read P0681
        response = self.modbus_master.read_holding_registers(
            slave_addr=self.slave_address,
            starting_addr=681,  # P0681
            register_qty=1
        )
        if response and len(response) == 1:
            p0681_value = response[0]
            # Convert to 16-bit signed integer
            if p0681_value >= 0x8000:
                p0681_value -= 0x10000

            # Calculate speed in Hz
            motor_speed_hz = (p0681_value / 8192) * nominal_freq

            # Calculate RPM
            rpm = motor_speed_hz * (60 / num_pole_pairs)

            return rpm
        else:
            return None

    def reset_fault(self):
        """Sends a command to reset faults."""
        control_word = 0x0080  # Control Word to Reset Fault (Bit 7)
        self.modbus_master.write_single_register(
            slave_addr=self.slave_address,
            register_address=682,
            register_value=control_word
        )
        time.sleep(0.1)
        # Clear Bit 7
        control_word = 0x0000
        self.modbus_master.write_single_register(
            slave_addr=self.slave_address,
            register_address=682,
            register_value=control_word
        )

    def read_max_rpm(self):
        """Reads the maximum RPM (P0208)."""
        response = self.modbus_master.read_holding_registers(
            slave_addr=self.slave_address,
            starting_addr=208,  # P0208
            register_qty=1
        )
        if response and len(response) == 1:
            max_rpm_raw = response[0]
            self.max_RPM = float(max_rpm_raw)
            return self.max_RPM
        else:
            return None
