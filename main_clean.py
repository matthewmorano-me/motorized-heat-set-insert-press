# CLEAN VERSION — same behavior as main.py, restructured for readability.
'''
Motorized heat-set insert press — production cycle.

Cycle:
  1. Home crosshead to top limit (fast, backoff, slow re-approach)
  2. Descend HOME_OFFSET_MM to stage above the insert
  3. Wait for GP20 press to arm
  4. Approach -> dwell at contact -> seat DESCENT_DISTANCE_MM past contact
  5. Retract to top limit

Force/displacement is logged to CSV through the descent.
GP13 LOW pauses the cycle and parks at top until HIGH.
Bottom limit aborts any phase. Ctrl+C cleans up and retracts.

No prints inside motion loops — print() jitters the Timer ISR.
Stepper is driven by machine.Timer. HX711 owns PIO0 SM0.
GPIO per System Wiring Diagram 09APR2026.pdf.
'''

import time
from machine import Pin, Timer
from hx711_pio import HX711

# ---------------------------------------------------------------------------
# Pins
# ---------------------------------------------------------------------------
PIN_STEP, PIN_DIR     = 0, 1       # Stepper pins
PIN_HX_DT, PIN_HX_SCK = 10, 11     # Load Cell HX711 pins
PIN_PHOT              = 13         # LM393 photoresistor digital out; LOW = abort
PIN_FAN               = 15         # Fan control pin (active HIGH via transistor)
PIN_TOP, PIN_BOT      = 16, 17     # Limit Switches
PIN_START_BTN         = 20         # Start button for press cycle on MakerPi board


STEPS_PER_MM = 200 * 8 // 5        # 200 steps/rev * 8 microsteps / 5 mm pitch = 320 steps/mm


# ---------------------------------------------------------------------------
# Motion tuning (mm and mm/s)
# ---------------------------------------------------------------------------
HOME_OFFSET_MM      = 5.0          # standby offset below top limit before press
DESCENT_DISTANCE_MM = 3.0          # press displacement; set equal to insert length
HOMING_BACKOFF_MM   = 2.0          # backoff between fast and slow homing passes

HOMING_FAST_MMPS    = 8.0          # fast approach to top limit
HOMING_SLOW_MMPS    = 1.0          # slow re-approach after backoff
APPROACH_SPEED_MMPS = 2.0          # pre-contact descent through air
SEATING_SPEED_MMPS  = 0.25         # post-contact seating push

CONTACT_FORCE_THRESHOLD = 10.0     # tared scaled-force units (raw_counts / 1000)

HEAT_SOAK_DWELL_MS  = 3000         # hold at contact while plastic melts

# Empirical lead-screw correction. Tune so commanded == measured.
DISP_CAL = 1

DIR_UP   = 1
DIR_DOWN = 0

RAIL_GUARD        = 100
HOMING_TIMEOUT_MS = 30000
_here    = __file__.rsplit('/', 1)[0] if '/' in __file__ else '.'
LOG_FILE = _here + '/press_log.csv'

# ---------------------------------------------------------------------------
# Hardware init
# ---------------------------------------------------------------------------
step_pin  = Pin(PIN_STEP, Pin.OUT, value=0)
dir_pin   = Pin(PIN_DIR,  Pin.OUT, value=DIR_UP)
lim_top   = Pin(PIN_TOP,  Pin.IN, pull=Pin.PULL_UP)
lim_bot   = Pin(PIN_BOT,  Pin.IN, pull=Pin.PULL_UP)
start_btn = Pin(PIN_START_BTN, Pin.IN, pull=Pin.PULL_UP)
phot      = Pin(PIN_PHOT, Pin.IN)         # LM393 module drives the line
fan_pin   = Pin(PIN_FAN, Pin.OUT, value=0)

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
    '''Timer ISR: toggle STEP at 2x step rate, count rising edges. Keep tiny — no print/alloc.'''
    global _step_state, _step_count
    _step_state ^= 1
    step_pin.value(_step_state)
    if _step_state == 1:
        _step_count += 1

def start_stepper(mmps):
    '''Arm the periodic Timer for stepping at `mmps`. Caller sets DIR_PIN first.'''
    freq_hz = int(2 * mmps * STEPS_PER_MM)
    step_tmr.init(freq=freq_hz, mode=Timer.PERIODIC, callback=_step_cb)

def stop_stepper():
    '''Stop the Timer and drive STEP low. _step_count is preserved for callers.'''
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
class PhotAbort(Exception):
    pass

def check_phot():
    '''Raise PhotAbort if GP13 (LM393) reads LOW. Polled in every motion loop.'''
    if phot.value() == 0:
        raise PhotAbort

def read_force():
    '''Return one tared force sample, rejecting HX711 rail-saturated reads.'''
    while True:
        val = hx.read()
        if val >=  8388607 - RAIL_GUARD:  continue
        if val <= -8388608 + RAIL_GUARD:  continue
        return (val - raw_tare) / 1000

def move_mm(distance_mm, speed_mmps, direction):
    '''Open-loop timed move. Blocks on time.sleep, so cannot be PhotAborted.'''
    dir_pin.value(direction); time.sleep_us(5)
    start_stepper(speed_mmps)
    time.sleep(distance_mm / speed_mmps)
    stop_stepper()

def _run_until_top(mmps, timeout_ms):
    '''Drive UP until top limit trips, or raise RuntimeError on timeout.'''
    dir_pin.value(DIR_UP); time.sleep_us(5)
    start_stepper(mmps)
    deadline = time.ticks_add(time.ticks_ms(), timeout_ms)
    while lim_top.value() != 1:
        if time.ticks_diff(time.ticks_ms(), deadline) >= 0:
            stop_stepper()
            raise RuntimeError("Homing timeout")
    stop_stepper()

def home_to_top():
    '''Two-stage homing: fast approach, backoff, slow re-approach for repeatable zero.'''
    _run_until_top(HOMING_FAST_MMPS, HOMING_TIMEOUT_MS)
    time.sleep_ms(100)
    move_mm(HOMING_BACKOFF_MM, HOMING_SLOW_MMPS, DIR_DOWN)
    time.sleep_ms(100)
    _run_until_top(HOMING_SLOW_MMPS, HOMING_TIMEOUT_MS)

def log_row(t, disp, f):
    '''Append one CSV row. No flush — would jitter the Timer ISR.'''
    log_f.write(f'{t},{disp:.3f},{f:.2f}\n')

# ---------------------------------------------------------------------------
# Press cycle phases. Each phase has one termination condition; safety polling
# (bottom limit, photoresistor) lives inside the phase that needs it.
# No prints inside the polling loops — they jitter the Timer ISR.
# ---------------------------------------------------------------------------
def approach_until_contact(t0):
    '''Step DOWN until force >= threshold. Returns (steps, t_ms, force) at contact,
    or None if bottom limit tripped first. May raise PhotAbort.'''
    dir_pin.value(DIR_DOWN); time.sleep_us(5)
    start_stepper(APPROACH_SPEED_MMPS)
    while True:
        if lim_bot.value() == 1:
            stop_stepper()
            return None
        check_phot()
        f = read_force()
        if f >= CONTACT_FORCE_THRESHOLD:
            stop_stepper()
            t = time.ticks_diff(time.ticks_ms(), t0)
            return _step_count, t, f

def dwell(t0):
    '''Hold position for HEAT_SOAK_DWELL_MS, logging force at displacement 0.
    Returns True on completion, False if bottom limit tripped. May raise PhotAbort.'''
    deadline = time.ticks_add(time.ticks_ms(), HEAT_SOAK_DWELL_MS)
    while time.ticks_diff(time.ticks_ms(), deadline) < 0:
        if lim_bot.value() == 1:
            return False
        check_phot()
        f = read_force()
        t = time.ticks_diff(time.ticks_ms(), t0)
        log_row(t, 0.0, f)
    return True

def seat_to_depth(t0, steps_at_contact):
    '''Step DOWN at seating speed until DESCENT_DISTANCE_MM past contact, logging
    displacement and force. Returns True on depth reached, False on bottom limit.
    May raise PhotAbort.'''
    stop_at_step = steps_at_contact + int(DESCENT_DISTANCE_MM * STEPS_PER_MM * DISP_CAL)
    dir_pin.value(DIR_DOWN); time.sleep_us(5)
    start_stepper(SEATING_SPEED_MMPS)
    while _step_count < stop_at_step:
        if lim_bot.value() == 1:
            stop_stepper()
            return False
        check_phot()
        f = read_force()
        t = time.ticks_diff(time.ticks_ms(), t0)
        disp = (_step_count - steps_at_contact) / STEPS_PER_MM
        log_row(t, disp, f)
    stop_stepper()
    return True

def press_cycle(t0):
    '''Run one full descent: approach -> dwell -> seat. Stepper guaranteed off on exit.'''
    print(f"Approaching at {APPROACH_SPEED_MMPS} mm/s. Waiting for contact "
          f"(threshold={CONTACT_FORCE_THRESHOLD}). Ctrl+C or bottom limit to stop.")

    global _step_count
    _step_count = 0

    try:
        contact = approach_until_contact(t0)
        if contact is None:
            return
        steps_at_contact, t, f = contact
        print(f"Contact detected  t={t/1000:.2f}s  force={f:+.2f}  "
              f"dwell {HEAT_SOAK_DWELL_MS/1000:.0f}s")

        if not dwell(t0):
            return

        print(f"Seating at {SEATING_SPEED_MMPS} mm/s")
        if not seat_to_depth(t0, steps_at_contact):
            return

        delivered = _step_count - steps_at_contact
        print(f"Seating depth reached: {delivered} steps "
              f"= {delivered/STEPS_PER_MM:.3f} mm commanded")
    finally:
        stop_stepper()

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
log_f = open(LOG_FILE, 'w')
log_f.write('time_ms,displacement_mm,force\n')
print(f"Logging to {LOG_FILE}")
print(f"Limit switches at startup: top={lim_top.value()} bot={lim_bot.value()} (expect both 0 when not pressed)")
fan_pin.on()
t0 = time.ticks_ms()

try:
    while True:
        try:
            print("Homing to top...")
            home_to_top()

            print(f"Positioning {HOME_OFFSET_MM} mm below top...")
            move_mm(HOME_OFFSET_MM, HOMING_SLOW_MMPS, DIR_DOWN)

            print("\nReady. Press GP20 to begin cycle (Ctrl+C to exit, GP13 low to pause).")
            while start_btn.value() == 1:
                check_phot()
                time.sleep_ms(20)
            while start_btn.value() == 0:
                check_phot()
                time.sleep_ms(20)
            time.sleep_ms(100)

            press_cycle(t0)

            print("Retracting to top...")
            home_to_top()

            print("\nDone.")
            print(f"Copy log to PC:  mpremote fs cp :{LOG_FILE} .")
            break

        except PhotAbort:
            print("\nGP13 low. Retracting and holding... Insert part to resume.")
            try:
                stop_stepper()
                home_to_top()
            except Exception as e:
                print(f"Retract failed: {e}")
            while phot.value() == 0:
                time.sleep_ms(100)
            print("GP13 high. Resuming.")

except KeyboardInterrupt:
    print("\nInterrupted. Retracting...")
    try:
        stop_stepper()
        home_to_top()
    except Exception as e:
        print(f"Retract failed: {e}")
    print(f"Copy log to PC:  mpremote fs cp :{LOG_FILE} .")

finally:
    stop_stepper()
    fan_pin.off()
    try:
        log_f.flush(); log_f.close()
    except Exception: pass
    hx.power_down()
    print("HX711 powered down.")
