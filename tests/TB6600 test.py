# This script tests the TB6600 stepper driver by allowing manual control of the motor using buttons. It also serves as a basic system check to confirm that the motor, driver, and wiring are functional before running the full insert cycle.
#
from machine import Pin, Timer
import time

# TB6600 driver pins — common-anode wiring:
#   PUL+ → 3V3,  PUL- → GP0
#   DIR+ → 3V3,  DIR- → GP1
#   ENA+/ENA- — leave unplugged; driver enables by default
# Pico drives the minus pins; driver triggers on edges, so a square wave steps it
pin_STEP = Pin(0, Pin.OUT, value=0)
pin_DIR  = Pin(1, Pin.OUT, value=1)  # DIR-: HIGH=forward, LOW=reverse

# MakerPi onboard buttons (active LOW - reads 0 when pressed)
btn_fwd = Pin(20, Pin.IN, pull=Pin.PULL_UP)  # GP20: step forward
btn_rev = Pin(21, Pin.IN, pull=Pin.PULL_UP)  # GP21: step reverse

# Limit switches (NC, pull-up; value()==1 when triggered)
lim_top = Pin(16, Pin.IN, pull=Pin.PULL_UP)  # GP16: blocks forward (UP)
lim_bot = Pin(17, Pin.IN, pull=Pin.PULL_UP)  # GP17: blocks reverse (DOWN)

# ---------------------------------------------------------------------------
# Mechanical: 200 steps/rev * 8 microsteps / 5 mm pitch = 320 steps/mm
# ---------------------------------------------------------------------------
STEPS_PER_MM = 200 * 8 // 5
SPEED_MMPS   = 3           # Adjust jog speed in mm/s
DIR_SETUP_US = 20           # TB6600 needs DIR stable before step pulse

# ---------------------------------------------------------------------------
# Timer-driven stepper pulses. Callback toggles STEP; Timer freq is 2x step rate.
# ---------------------------------------------------------------------------
step_tmr = Timer()
_step_state = 0

def _step_cb(_t):
    global _step_state
    _step_state ^= 1
    pin_STEP.value(_step_state)

def start_stepper(mmps):
    freq_hz = int(2 * mmps * STEPS_PER_MM)
    step_tmr.init(freq=freq_hz, mode=Timer.PERIODIC, callback=_step_cb)

def stop_stepper():
    global _step_state
    try:
        step_tmr.deinit()
    except Exception:
        pass
    _step_state = 0
    pin_STEP.value(0)

# ---------------------------------------------------------------------------
# Jog loop. Track current direction so we only (re)start the Timer on changes.
# ---------------------------------------------------------------------------
DIR_NONE, DIR_FWD, DIR_REV = 0, 1, -1
current = DIR_NONE

print(f"Running at {SPEED_MMPS} mm/s - GP20=UP, GP21=DOWN")

try:
    while True:
        want_fwd = btn_fwd.value() == 0 and lim_top.value() == 0
        want_rev = btn_rev.value() == 0 and lim_bot.value() == 0

        if want_fwd and current != DIR_FWD:
            stop_stepper()
            pin_DIR.value(1)
            time.sleep_us(DIR_SETUP_US)
            start_stepper(SPEED_MMPS)
            current = DIR_FWD
        elif want_rev and current != DIR_REV:
            stop_stepper()
            pin_DIR.value(0)
            time.sleep_us(DIR_SETUP_US)
            start_stepper(SPEED_MMPS)
            current = DIR_REV
        elif not want_fwd and not want_rev and current != DIR_NONE:
            stop_stepper()
            current = DIR_NONE

        time.sleep_ms(5)
finally:
    stop_stepper()
