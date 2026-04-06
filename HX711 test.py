# HX711 load cell amplifier test script for Raspberry Pi Pico (MicroPython)
from hx711_pio import HX711
from machine import Pin
import time

# Configure data and clock pins for HX711
pin_DT  = Pin(0, Pin.IN, pull=Pin.PULL_DOWN)   # Data pin (GP0, input with pull-down)
pin_SCK = Pin(1, Pin.OUT)                        # Clock pin (GP1, output)

# Initialize HX711 with 128x gain on channel A, using PIO state machine 0
hx = HX711(pin_SCK, pin_DT, gain=128, state_machine=0)

# Power cycle the HX711 to ensure a clean startup
hx.power_down()
time.sleep_ms(100)
hx.power_up()
time.sleep_ms(500)   # Wait for HX711 to stabilize after power-up

# Tare the scale: average 15 readings to establish the zero offset
hx.tare(times=15)
raw_tare = hx.read()   # Store the tare (zero) reference value

try:
    while True:
        # Read sensor and subtract tare offset; divide by 1000 to reduce magnitude
        raw = hx.read()/1000 - raw_tare/1000
        #if raw != 8388607 and raw != -8388608:  # Filter out overflow/underflow sentinel values
        #    print(f"Raw: {raw}")
        print(f"Raw: {raw}")
        time.sleep_ms(5)   # Short delay between readings (~200 Hz)
except KeyboardInterrupt:
    print("Stopped")