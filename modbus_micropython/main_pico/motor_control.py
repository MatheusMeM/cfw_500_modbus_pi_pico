# motor_control.py

import time
from cfw500_modbus import CFW500Modbus
from utils import print_verbose

# Initialize Modbus and CFW500
def initialize_cfw500(UART0_ID, TX_PIN_NUM, RX_PIN_NUM, DE_RE_PIN, SLAVE_ADDRESS):
    cfw500 = CFW500Modbus(
        uart_id=UART0_ID,
        tx_pin=TX_PIN_NUM,
        rx_pin=RX_PIN_NUM,
        de_re_pin=DE_RE_PIN,
        slave_address=SLAVE_ADDRESS
    )
    return cfw500

# Add any motor control related functions here if needed
