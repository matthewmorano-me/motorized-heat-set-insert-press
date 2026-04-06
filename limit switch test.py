from machine import Pin
import time

# TB6600 driver — same wiring as TB6600 test.py
#   PUL+ → 3V3,  PUL- → GP0
#   DIR+ → 3V3,  DIR- → GP1
pin_STEP = Pin(0, Pin.OUT, value=1)  # idle HIGH; pulse LOW to step
pin_DIR  = Pin(1, Pin.OUT, value=1)  # HIGH=forward(up), LOW=reverse(down)

# Limit switches — NC wired to GND (open = triggered, PULL_UP otherwise)
# Adjust GP numbers to match your wiring
limit_top = Pin(2, Pin.IN, pull=Pin.PULL_UP)   # GP2: triggered when at top
limit_bot = Pin(3, Pin.IN, pull=Pin.PULL_UP)   # GP3: triggered when at bottom

STEP_DELAY_US = 65  # lower = faster
DIR_SETUP_US  = 20   # DIR must be stable before pulsing (TB6600 requirement)

def step_once():
    pin_STEP.low()
    time.sleep_us(10)
    pin_STEP.high()
    time.sleep_us(STEP_DELAY_US)

def move_to_top():
    """Drive forward until the top limit switch fires."""
    print("Moving to top...")
    pin_DIR.value(1)
    time.sleep_us(DIR_SETUP_US)
    while limit_top.value() == 0:   # 0 = NC closed (not triggered); 1 = open (triggered)
        step_once()
    print("Top reached.")

def move_to_bottom():
    """Drive reverse until the bottom limit switch fires."""
    print("Moving to bottom...")
    pin_DIR.value(0)
    time.sleep_us(DIR_SETUP_US)
    while limit_bot.value() == 0:   # 0 = NC closed (not triggered); 1 = open (triggered)
        step_once()
    print("Bottom reached.")

# --- Main sequence ---

try:
    while True:

        move_to_top()
        time.sleep_ms(300)   # brief pause at top

        move_to_bottom()
        time.sleep_ms(300)   # brief pause at bottom

except KeyboardInterrupt:
    print("Stopped")