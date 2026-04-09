# This script tests the TB6600 stepper driver by allowing manual control of the motor using buttons. It also serves as a basic system check to confirm that the motor, driver, and wiring are functional before running the full insert cycle.
#
from machine import Pin
import time

# TB6600 driver pins — common-anode wiring:
#   PUL+ → 3V3,  PUL- → GP0
#   DIR+ → 3V3,  DIR- → GP1
#   ENA+/ENA- — leave unplugged; driver enables by default
# Pico drives the minus pins active LOW to signal the driver
pin_STEP = Pin(0, Pin.OUT, value=1)  # PUL-: idle HIGH; pulse LOW to step
pin_DIR  = Pin(1, Pin.OUT, value=1)  # DIR-: HIGH=forward, LOW=reverse

# MakerPi onboard buttons (active LOW - reads 0 when pressed)
btn_fwd = Pin(20, Pin.IN, pull=Pin.PULL_UP)  # GP20: step forward
btn_rev = Pin(21, Pin.IN, pull=Pin.PULL_UP)  # GP21: step reverse

STEP_DELAY_US = 100  # Adjust for speed; lower = faster
DIR_SETUP_US  = 20   # TB6600 needs DIR stable before step pulse

print("Running - GP20=forward, GP21=reverse")

while True:
    if btn_fwd.value() == 0:
        pin_DIR.value(1)
        time.sleep_us(DIR_SETUP_US)  # let DIR settle before pulsing
        pin_STEP.low()
        time.sleep_us(10)
        pin_STEP.high()
        time.sleep_us(STEP_DELAY_US)

    elif btn_rev.value() == 0:
        pin_DIR.value(0)
        time.sleep_us(DIR_SETUP_US)  # let DIR settle before pulsing
        pin_STEP.low()
        time.sleep_us(10)
        pin_STEP.high()
        time.sleep_us(STEP_DELAY_US)
