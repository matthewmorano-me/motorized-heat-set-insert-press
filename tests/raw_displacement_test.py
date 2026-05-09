'''
Raw displacement test — no fixture, no force sensing.

Homes to the top limit switch, descends a fixed distance, then holds so
the displacement can be measured with calipers. Press GP20 to retract.
Use this to verify the stepper/ballscrew deliver accurate displacement
before attributing error to the fixture or load cell bracket.

Stepper is driven by machine.Timer (no PIO, no libraries).
GPIO per System Wiring Diagram 09APR2026.pdf.
'''

import time
from machine import Pin, Timer

# ---------------------------------------------------------------------------
# Pins
# ---------------------------------------------------------------------------
PIN_STEP, PIN_DIR = 0, 1
PIN_TOP, PIN_BOT  = 16, 17
PIN_STOP_BTN      = 20

# ---------------------------------------------------------------------------
# Mechanical: 200 steps/rev * 8 microsteps / 5 mm pitch = 320 steps/mm
# ---------------------------------------------------------------------------
STEPS_PER_MM = 200 * 8 // 5

# ---------------------------------------------------------------------------
# Motion
# ---------------------------------------------------------------------------
DESCENT_MM          = 4.0
DESCENT_SPEED_MMPS  = 1.0

HOMING_FAST_MMPS    = 8.0
HOMING_SLOW_MMPS    = 1.0
HOMING_BACKOFF_MM   = 2.0
POSITION_SPEED_MMPS = 4.0

HOMING_TIMEOUT_MS   = 30000

DIR_UP   = 1
DIR_DOWN = 0

# ---------------------------------------------------------------------------
# Hardware init
# ---------------------------------------------------------------------------
step_pin = Pin(PIN_STEP, Pin.OUT, value=0)
dir_pin  = Pin(PIN_DIR,  Pin.OUT, value=DIR_UP)
lim_top  = Pin(PIN_TOP,  Pin.IN, pull=Pin.PULL_UP)
lim_bot  = Pin(PIN_BOT,  Pin.IN, pull=Pin.PULL_UP)
stop_btn = Pin(PIN_STOP_BTN, Pin.IN, pull=Pin.PULL_UP)

# ---------------------------------------------------------------------------
# Stepper
# ---------------------------------------------------------------------------
step_tmr = Timer()
_step_state = 0

def _step_cb(_t):
    global _step_state
    _step_state ^= 1
    step_pin.value(_step_state)

def start_stepper(mmps):
    step_tmr.init(freq=int(2 * mmps * STEPS_PER_MM),
                  mode=Timer.PERIODIC, callback=_step_cb)

def stop_stepper():
    global _step_state
    try: step_tmr.deinit()
    except Exception: pass
    _step_state = 0
    step_pin.value(0)

def move_mm(distance_mm, speed_mmps, direction):
    dir_pin.value(direction); time.sleep_us(5)
    start_stepper(speed_mmps)
    time.sleep(distance_mm / speed_mmps)
    stop_stepper()

def _run_until_top(mmps, timeout_ms):
    dir_pin.value(DIR_UP); time.sleep_us(5)
    start_stepper(mmps)
    deadline = time.ticks_add(time.ticks_ms(), timeout_ms)
    while lim_top.value() != 1:
        if time.ticks_diff(time.ticks_ms(), deadline) >= 0:
            stop_stepper()
            raise RuntimeError("Homing timeout")
    stop_stepper()

def home_to_top():
    _run_until_top(HOMING_FAST_MMPS, HOMING_TIMEOUT_MS)
    time.sleep_ms(100)
    move_mm(HOMING_BACKOFF_MM, HOMING_SLOW_MMPS, DIR_DOWN)
    time.sleep_ms(100)
    _run_until_top(HOMING_SLOW_MMPS, HOMING_TIMEOUT_MS)

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
try:
    print("Homing to top...")
    home_to_top()

    print(f"\nHolding at top. Measure now, then press GP20 to descend.")
    while stop_btn.value() == 1:
        time.sleep_ms(20)
    while stop_btn.value() == 0:
        time.sleep_ms(20)
    time.sleep_ms(100)

    print(f"Descending {DESCENT_MM} mm at {DESCENT_SPEED_MMPS} mm/s...")
    move_mm(DESCENT_MM, DESCENT_SPEED_MMPS, DIR_DOWN)

    print(f"\nHolding at {DESCENT_MM} mm. Measure now, then press GP20 to retract.")
    while stop_btn.value() == 1:
        time.sleep_ms(20)
    while stop_btn.value() == 0:
        time.sleep_ms(20)
    time.sleep_ms(100)

    print("Retracting to top...")
    home_to_top()
    print("Done.")

except KeyboardInterrupt:
    print("\nInterrupted — retracting for safety.")
    try:
        stop_stepper()
        home_to_top()
    except Exception as e:
        print(f"Retract failed: {e}")

finally:
    stop_stepper()
