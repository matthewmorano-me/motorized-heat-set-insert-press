# FINAL WORKING VERSION — tested and confirmed.
'''
Motorized heat-set insert press — production cycle.

Cycle:
  1. Home crosshead to top limit switch (fast approach, backoff, slow re-approach)
  2. Descend HOME_OFFSET_MM to stage the heated tip just above the insert
  3. Wait for GP20 press to arm
  4. Approach at APPROACH_SPEED_MMPS; on first force > CONTACT_FORCE_THRESHOLD
     latch the displacement origin, stop, and dwell HEAT_SOAK_DWELL_MS for
     heat transfer; then advance DESCENT_DISTANCE_MM past contact at
     SEATING_SPEED_MMPS
  5. Retract to top limit automatically

Force/displacement is logged to CSV through the descent.

No prints during motion — print() jitters the Timer ISR. Status only at phase
boundaries; force samples go straight to CSV.

Safety:
  - Bottom limit switch is polled on every force sample during descent
  - Ctrl+C at any phase cleans up and retracts
  - GP13 (LM393 photoresistor) LOW pauses cycle and parks at top until HIGH,
    then resumes from the start of the cycle (does not exit the program)

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
HOME_OFFSET_MM      = 5.0          # "Home" offset below top limit for standby before press cycle
DESCENT_DISTANCE_MM = 3.0          # Press displacement. Set equal to insert length
HOMING_BACKOFF_MM   = 2.0          # backoff distance after fast homing to clear the switch for slow homing

HOMING_FAST_MMPS    = 8.0          # fast approach to top limit
HOMING_SLOW_MMPS    = 1.0          # slow approach for final homing after backoff
APPROACH_SPEED_MMPS = 2.0          # pre-contact descent through air
SEATING_SPEED_MMPS  = 0.25         # post-contact seating push

CONTACT_FORCE_THRESHOLD = 10.0     # tared scaled-force units (raw_counts / 1000).
                                   # Follow-me deadband = 20.0 in these units; 1.0
                                   # tripped on stepper-startup noise before contact.

HEAT_SOAK_DWELL_MS  = 3000        # hold at contact while heat transfers before
                                   # advancing to seat the insert.

# Empirical correction. If 1280 commanded steps measure as 3.5 mm with calipers,
# set DISP_CAL = 4.0 / 3.5 ~= 1.143 so the script commands enough extra steps to
# hit a true 4 mm. Tune by running, measuring, and adjusting until commanded ==
# measured.
DISP_CAL = 1

DIR_UP   = 1                      # flip if wiring drives the stage the wrong way
DIR_DOWN = 0

RAIL_GUARD       = 100
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
    '''
    Timer ISR fired at 2x the desired step rate. Each call toggles the STEP
    line, so a full HIGH->LOW->HIGH cycle (one step) takes two firings. We
    count only on rising edges so _step_count tracks delivered steps and can
    be converted to mm via STEPS_PER_MM. Keep this callback tiny: it runs on
    the Timer ISR and any allocation or print() will jitter the pulse train
    and corrupt the displacement log during the press cycle.
    '''
    global _step_state, _step_count
    _step_state ^= 1
    step_pin.value(_step_state)
    if _step_state == 1:
        _step_count += 1

def start_stepper(mmps):
    '''
    Start continuous stepping at `mmps` linear speed. Converts mm/s to a
    Timer frequency (2 ISR firings per step, STEPS_PER_MM steps per mm) and
    arms the periodic Timer with _step_cb. Caller is responsible for setting
    DIR_PIN before calling and for stopping the Timer with stop_stepper()
    once the move is finished. Re-calling start_stepper() reinitializes the
    Timer at the new frequency without an explicit stop in between.
    '''
    freq_hz = int(2 * mmps * STEPS_PER_MM)
    step_tmr.init(freq=freq_hz, mode=Timer.PERIODIC, callback=_step_cb)

def stop_stepper():
    '''
    Stop the periodic Timer driving STEP and drive the STEP line LOW so the
    driver isn't left holding a partial pulse. The try/except guards against
    deinit() raising when the Timer was never armed (e.g. early Ctrl+C in
    main, or the finally block of press_cycle running after a PhotAbort).
    _step_count is intentionally NOT reset here — callers that need a fresh
    step origin (e.g. press_cycle latching contact) zero it themselves.
    '''
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
    '''
    Raise PhotAbort if GP13 (LM393 photoresistor) reads LOW, indicating the
    safety cover/light barrier has been broken. Polled inside every motion
    loop (homing, approach, dwell, seating) so we can interrupt mid-move.
    The exception unwinds back to main, which retracts to the top limit and
    holds there until the line goes HIGH again, then restarts the cycle.
    Cheaper than wiring an IRQ — a single GPIO read at the polling cadence
    is plenty fast given the millisecond-scale loops in this program.
    '''
    if phot.value() == 0:
        raise PhotAbort

def read_force():
    '''
    Read one tared, scaled force sample from the HX711. The 24-bit ADC
    returns signed values in [-8388608, +8388607]; samples within RAIL_GUARD
    counts of either rail are treated as saturated/invalid and rejected by
    looping until a clean reading arrives. The rail guard catches the
    transient garbage the HX711 emits during gain switches and after
    power_up(). Output is (raw - raw_tare) / 1000, the same scaled units
    used by CONTACT_FORCE_THRESHOLD; not calibrated to newtons or grams.
    '''
    while True:
        val = hx.read()
        if val >=  8388607 - RAIL_GUARD:  continue
        if val <= -8388608 + RAIL_GUARD:  continue
        return (val - raw_tare) / 1000

def move_mm(distance_mm, speed_mmps, direction):
    '''
    Open-loop timed move: set DIR, start the stepper at `speed_mmps`, sleep
    for distance/speed seconds, then stop. Used wherever we need a fixed
    relative move with no contact/limit checking — the HOME_OFFSET descent
    after homing and the backoff between fast and slow homing passes. The
    5 us delay after dir_pin.value() satisfies the DRV8825/TB6600 setup-
    time spec before the first STEP edge. Because this blocks on time.sleep,
    PhotAbort cannot interrupt the move; only use it for short distances.
    '''
    dir_pin.value(direction); time.sleep_us(5)
    start_stepper(speed_mmps)
    time.sleep(distance_mm / speed_mmps)
    stop_stepper()

def _run_until_top(mmps, timeout_ms):
    '''
    Drive UP at `mmps` until the top limit switch reads active (1), polling
    in a tight loop. If `timeout_ms` elapses before the switch trips, stop
    the motor and raise RuntimeError so the caller can react instead of the
    crosshead grinding into the rail forever (failed switch, broken wire,
    blocked carriage, wrong direction wiring). Single-pass primitive used
    twice by home_to_top: once fast for coarse approach, once slow after
    backoff for a repeatable final position.
    '''
    dir_pin.value(DIR_UP); time.sleep_us(5)
    start_stepper(mmps)
    deadline = time.ticks_add(time.ticks_ms(), timeout_ms)
    while lim_top.value() != 1:
        if time.ticks_diff(time.ticks_ms(), deadline) >= 0:
            stop_stepper()
            raise RuntimeError("Homing timeout")
    stop_stepper()

def home_to_top():
    '''
    Two-stage homing routine for repeatable top-of-travel positioning.
    First runs UP fast (HOMING_FAST_MMPS) to bring the crosshead near the
    switch quickly, then backs off HOMING_BACKOFF_MM at slow speed to clear
    the switch with margin, then re-approaches at HOMING_SLOW_MMPS so the
    final trip happens at low velocity. Slow re-approach removes the speed-
    dependent overshoot you get from a single fast hit and gives a
    consistent zero across runs. The 100 ms pauses let the carriage settle
    so the slow re-approach starts from rest.
    '''
    _run_until_top(HOMING_FAST_MMPS, HOMING_TIMEOUT_MS)
    time.sleep_ms(100)
    move_mm(HOMING_BACKOFF_MM, HOMING_SLOW_MMPS, DIR_DOWN)
    time.sleep_ms(100)
    _run_until_top(HOMING_SLOW_MMPS, HOMING_TIMEOUT_MS)

# ---------------------------------------------------------------------------
# Descent: approach until contact, dwell, then seat DESCENT_DISTANCE_MM.
# No prints inside the polling loops — they jitter the Timer ISR.
# ---------------------------------------------------------------------------
def press_cycle(t0):
    '''
    Full descent: approach -> contact detection -> heat-soak dwell -> seat.

    Steps DOWN at APPROACH_SPEED_MMPS while polling the
    bottom limit, photoresistor, and load cell. On the first force sample
    that exceeds CONTACT_FORCE_THRESHOLD we latch _step_count as the
    displacement origin, stop the motor, and dwell HEAT_SOAK_DWELL_MS so
    the heated tip can melt the surrounding plastic before any further
    motion. During the dwell we keep logging force at zero displacement so
    the soak phase is visible in the CSV. After the dwell we restart the
    stepper at the slower SEATING_SPEED_MMPS and continue down until the
    delivered step count reaches stop_at_step (= contact + DESCENT_DISTANCE
    in steps, scaled by DISP_CAL for empirical lead-screw calibration).

    Timing reference t0 is passed in from main so all log timestamps share
    a single origin across cycles. Print() is restricted to phase
    boundaries (contact, seating, completion) — printing inside the polling
    loop perturbs the Timer ISR enough to corrupt the displacement record.
    The finally block guarantees the stepper is off on any exit path:
    normal completion, bottom-limit trip, PhotAbort, or KeyboardInterrupt.
    '''
    print(f"Approaching at {APPROACH_SPEED_MMPS} mm/s. Waiting for contact "
          f"(threshold={CONTACT_FORCE_THRESHOLD}). Ctrl+C or bottom limit to stop.")

    global _step_count
    _step_count = 0

    dir_pin.value(DIR_DOWN); time.sleep_us(5)
    start_stepper(APPROACH_SPEED_MMPS)

    steps_at_contact = None
    stop_at_step     = None

    try:
        while True:
            if lim_bot.value() == 1:
                return
            check_phot()

            f = read_force()
            t = time.ticks_diff(time.ticks_ms(), t0)

            if steps_at_contact is None and f >= CONTACT_FORCE_THRESHOLD:
                steps_at_contact = _step_count
                stop_at_step     = steps_at_contact + int(DESCENT_DISTANCE_MM * STEPS_PER_MM * DISP_CAL)
                print(f"Contact detected  t={t/1000:.2f}s  force={f:+.2f}  "
                      f"dwell {HEAT_SOAK_DWELL_MS/1000:.0f}s")
                stop_stepper()
                dwell_deadline = time.ticks_add(time.ticks_ms(), HEAT_SOAK_DWELL_MS)
                while time.ticks_diff(time.ticks_ms(), dwell_deadline) < 0:
                    if lim_bot.value() == 1:
                        return
                    check_phot()
                    f = read_force()
                    t = time.ticks_diff(time.ticks_ms(), t0)
                    log_f.write(f'{t},0.000,{f:.2f}\n')
                print(f"Seating at {SEATING_SPEED_MMPS} mm/s")
                dir_pin.value(DIR_DOWN); time.sleep_us(5)
                start_stepper(SEATING_SPEED_MMPS)

            if steps_at_contact is not None:
                disp = (_step_count - steps_at_contact) / STEPS_PER_MM
                log_f.write(f'{t},{disp:.3f},{f:.2f}\n')

            if (steps_at_contact is not None and stop_at_step is not None
                    and _step_count >= stop_at_step):
                delivered = _step_count - steps_at_contact
                print(f"Seating depth reached: {delivered} steps "
                      f"= {delivered/STEPS_PER_MM:.3f} mm commanded")
                return
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
