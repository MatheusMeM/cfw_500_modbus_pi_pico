# relay.py

import sys
import time
from machine import UART, Pin
import uselect

# UART0 Configuration (RS485 Communication)
RS485_UART_ID = 0
RS485_TX_PIN = 0
RS485_RX_PIN = 1
RS485_DE_RE_PIN = Pin(2, Pin.OUT)
RS485_DE_RE_PIN.value(0)  # Initially set to receive mode

# Initialize UART0
rs485_uart = UART(RS485_UART_ID, baudrate=115200, tx=RS485_TX_PIN, rx=RS485_RX_PIN)
rs485_uart.init(bits=8, parity=None, stop=1)  # Use 1 stop bit for higher baud rates

print("[INFO] Relay Pico initialized. Waiting for communication...")

# Create a poll object
poller = uselect.poll()
poller.register(sys.stdin, uselect.POLLIN)
poller.register(rs485_uart, uselect.POLLIN)

def relay():
    rx_buffer = b''
    while True:
        events = poller.poll(100)
        for fd, event in events:
            if fd is sys.stdin:
                # Data from PC (USB serial)
                data = sys.stdin.readline()
                if data:
                    RS485_DE_RE_PIN.value(1)  # Enable transmitter
                    rs485_uart.write(data.encode('utf-8'))
                    rs485_uart.flush()
                    time.sleep(0.01)
                    RS485_DE_RE_PIN.value(0)  # Enable receiver
            elif fd is rs485_uart:
                # Data from RS485
                data = rs485_uart.read()
                if data:
                    rx_buffer += data
                    while b'\n' in rx_buffer:
                        line, rx_buffer = rx_buffer.split(b'\n', 1)
                        try:
                            message = line.decode('utf-8')
                            print(message)
                        except Exception as e:
                            print(f"[ERROR] Exception during decoding: {e}")
        time.sleep(0.01)  # Small delay to prevent high CPU usage

if __name__ == "__main__":
    try:
        relay()
    except KeyboardInterrupt:
        print("\n[INFO] Relay Pico program interrupted by user.")
