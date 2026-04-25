'''
Insert force characterization (spring-fixture proxy).

Ram attachment on the load cell pushes a spring fixture that stands in for
a real heat-set insert. Contact is detected from the force signal, then the
ram advances DESCENT_DISTANCE_MM further while logging.

Cycle:
  1. Home crosshead to top limit switch (fast approach, backoff, slow re-approach)
  2. Descend HOME_OFFSET_MM to stage the ram just above the spring fixture
  3. Wait for GP20 press to arm
  4. Descend slowly; on first force > CONTACT_FORCE_THRESHOLD, advance
     DESCENT_DISTANCE_MM more without stopping the stepper
  5. Retract to top limit

No prints during motion — print() jitters the Timer ISR. Status only at phase
boundaries; force samples go straight to CSV.

Safety:
  - GP20 (onboard MakerPi button, active LOW) aborts descent and retracts
  - Bottom limit switch is polled on every force sample during descent
  - Ctrl+C at any phase cleans up and retracts

Stepper is driven by machine.Timer (no PIO, no libraries). HX711 owns PIO0 SM0.
GPIO per System Wiring Diagram 09APR2026.pdf.
'''

import time
from machine import Pin, Timer
from hx711_pio import HX711

# ---------------------------------------------------------------------------
# Pins
# ---------------------------------------------------------------------------
PIN_STEP, PIN_DIR     = 0, 1
PIN_HX_DT, PIN_HX_SCK = 10, 11
PIN_TOP, PIN_BOT      = 16, 17
PIN_STOP_BTN          = 20

# ---------------------------------------------------------------------------
# Mechanical: 200 steps/rev * 8 microsteps / 5 mm pitch = 320 steps/mm
# ---------------------------------------------------------------------------
STEPS_PER_MM = 200 * 8 // 5       # VERIFY microstep count against TB6600 DIPs

# ---------------------------------------------------------------------------
# Motion tuning (mm and mm/s)
# ---------------------------------------------------------------------------
HOME_OFFSET_MM      = 5.0
DESCENT_DISTANCE_MM = 4.0         # distance advanced past contact

HOMING_FAST_MMPS    = 8.0
HOMING_SLOW_MMPS    = 1.0
HOMING_BACKOFF_MM   = 2.0
POSITION_SPEED_MMPS = 4.0
DESCENT_SPEED_MMPS  = 2.0

CONTACT_FORCE_THRESHOLD = 5.0     # tared scaled-force units (raw_counts / 1000).
                                   # Follow-me deadband = 20.0 in these units; 1.0
                                   # tripped on stepper-startup noise before contact.

DIR_UP   = 1                      # flip if wiring drives the stage the wrong way
DIR_DOWN = 0

RAIL_GUARD       = 100
HOMING_TIMEOUT_MS = 30000
_here    = __file__.rsplit('/', 1)[0] if '/' in __file__ else '.'
LOG_FILE = _here + '/insert_force_log.csv'

# ---------------------------------------------------------------------------
# Hardware init
# ---------------------------------------------------------------------------
step_pin = Pin(PIN_STEP, Pin.OUT, value=0)
dir_pin  = Pin(PIN_DIR,  Pin.OUT, value=DIR_UP)
lim_top  = Pin(PIN_TOP,  Pin.IN, pull=Pin.PULL_UP)
lim_bot  = Pin(PIN_BOT,  Pin.IN, pull=Pin.PULL_UP)
stop_btn = Pin(PIN_STOP_BTN, Pin.IN, pull=Pin.PULL_UP)

hx = HX711(Pin(PIN_HX_SCK, Pin.OUT),
           Pin(PIN_HX_DT,  Pin.IN, pull=Pin.PULL_DOWN),
           gain=128, state_machine=0)

hx.power_down(); time.sleep_ms(100); hx.power_up(); time.sleep_ms(500)
print("Taring load cell (keep it unloaded)...")
hx.tare(times=15)
raw_tare = hx.read()
print("Tare done.")

# ---------------------------------------------------------------------------
# Timer-driven stepper pulses. Callback toggles STEP; Timer freq is 2x step rate.
# ---------------------------------------------------------------------------
step_tmr = Timer()
_step_state = 0
_step_count = 0

def _step_cb(_t):
    global _step_state, _step_count
    _step_state ^= 1
    step_pin.value(_step_state)
    if _step_state == 1:   # rising edge = one step delivered
        _step_count += 1

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
    step_pin.value(0)

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def read_force():
    while True:
        val = hx.read()
        if val >=  8388607 - RAIL_GUARD:  continue
        if val <= -8388608 + RAIL_GUARD:  continue
        return (val - raw_tare) / 1000

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
# Descent with contact detection. Single stepper run: slow approach until
# force trips threshold, then advance DESCENT_DISTANCE_MM more at the same
# speed. No prints inside the loop — they jitter the Timer ISR.
# ---------------------------------------------------------------------------
def descend_and_log(t0):
    print(f"Descending at {DESCENT_SPEED_MMPS} mm/s. Waiting for contact "
          f"(threshold={CONTACT_FORCE_THRESHOLD}). Ctrl+C or bottom limit to stop.")

    global _step_count
    _step_count = 0

    dir_pin.value(DIR_DOWN); time.sleep_us(5)
    start_stepper(DESCENT_SPEED_MMPS)

    stop_deadline    = None   # set once contact is detected
    steps_at_contact = None   # step count snapshot at contact

    try:
        while True:
            if lim_bot.value() == 1:
                break

            f = read_force()
            t = time.ticks_diff(time.ticks_ms(), t0)

            if stop_deadline is None and f >= CONTACT_FORCE_THRESHOLD:
                push_ms = int(DESCENT_DISTANCE_MM / DESCENT_SPEED_MMPS * 1000)
                stop_deadline    = time.ticks_add(time.ticks_ms(), push_ms)
                steps_at_contact = _step_count
                print(f"Contact detected  t={t/1000:.2f}s  force={f:+.2f}")

            if steps_at_contact is not None:
                disp = (_step_count - steps_at_contact) / STEPS_PER_MM
                log_f.write(f'{t},{disp:.3f},{f:.2f}\n')

            if (stop_deadline is not None and
                time.ticks_diff(time.ticks_ms(), stop_deadline) >= 0):
                print(f"Target advance reached ({DESCENT_DISTANCE_MM} mm)")
                break
    finally:
        stop_stepper()

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
log_f = open(LOG_FILE, 'w')
log_f.write('time_ms,displacement_mm,force\n')
print(f"Logging to {LOG_FILE}")
t0 = time.ticks_ms()

try:
    print("Homing to top...")
    home_to_top()

    print(f"Positioning {HOME_OFFSET_MM} mm below top...")
    move_mm(HOME_OFFSET_MM, POSITION_SPEED_MMPS, DIR_DOWN)

    print("\nReady. Press GP20 to begin descent (Ctrl+C to abort).")
    while stop_btn.value() == 1:
        time.sleep_ms(20)
    while stop_btn.value() == 0:        # wait for release so it doesn't trip the abort check
        time.sleep_ms(20)
    time.sleep_ms(100)                  # debounce

    descend_and_log(t0)

    print("\nHolding position. Measure now, then press GP20 to retract.")
    while stop_btn.value() == 1:
        time.sleep_ms(20)
    while stop_btn.value() == 0:
        time.sleep_ms(20)
    time.sleep_ms(100)

    print("Retracting to top...")
    home_to_top()

    print("\nDone.")
    print(f"Copy log to PC:  mpremote fs cp :{LOG_FILE} .")

except KeyboardInterrupt:
    print("\nInterrupted — retracting for safety.")
    try:
        stop_stepper()
        home_to_top()
    except Exception as e:
        print(f"Retract failed: {e}")
    print(f"Copy log to PC:  mpremote fs cp :{LOG_FILE} .")

finally:
    stop_stepper()
    try:
        log_f.flush(); log_f.close()
    except Exception: pass
    hx.power_down()
    print("HX711 powered down.")
