import uasyncio as asyncio
from machine import Pin
from encoder import Encoder

# Callback function to handle encoder changes
def encoder_callback(value, delta):
    print("Encoder value:", value, "Delta:", delta)

async def main():
    # Configure pins 16 and 17 with internal pull-ups
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

    while True:
        await asyncio.sleep_ms(1000)  # Main loop can perform other tasks

# Run the main function
try:
    asyncio.run(main())
finally:
    asyncio.new_event_loop()
