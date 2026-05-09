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

Stepper is driven by machine.Timer. HX711 owns PIO0 SM0.
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
DESCENT_SPEED_MMPS  = 0.5

CONTACT_FORCE_THRESHOLD = 5.0     # tared scaled-force units (raw_counts / 1000).
                                   # Follow-me deadband = 20.0 in these units; 1.0
                                   # tripped on stepper-startup noise before contact.

# Empirical correction. If 1280 commanded steps measure as 3.5 mm with calipers,
# set DISP_CAL = 4.0 / 3.5 ~= 1.143 so the script commands enough extra steps to
# hit a true 4 mm. Tune by running, measuring, and adjusting until commanded ==
# measured. Diagnostic line at end of descent shows commanded steps vs mm.
DISP_CAL = 1.0

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
    """
    Timer ISR (interrupt service routine) called at 2× the desired step rate.
    Each call toggles the STEP pin LOW→HIGH→LOW. The TB6600 interprets each
    rising edge as one step command, so two calls = one step. The global
    _step_count is incremented only on rising edges so it tracks actual steps
    delivered, not timer firings.
    """
    global _step_state, _step_count
    _step_state ^= 1
    step_pin.value(_step_state)
    if _step_state == 1:   # rising edge = one step delivered
        _step_count += 1

def start_stepper(mmps):
    """
    Arms the Timer to fire _step_cb at the frequency needed for a given
    linear speed in mm/s. Timer frequency is doubled because two callbacks
    (one LOW, one HIGH) produce one complete step pulse.
    """
    freq_hz = int(2 * mmps * STEPS_PER_MM)
    step_tmr.init(freq=freq_hz, mode=Timer.PERIODIC, callback=_step_cb)

def stop_stepper():
    """
    Cancels the Timer and idles the STEP pin LOW. Safe to call even if the
    Timer is already stopped. Resetting _step_state ensures the next
    start_stepper call begins cleanly on a LOW edge.
    """
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
    """
    Blocks until the HX711 returns a valid 24-bit ADC reading, then returns
    the tared, scaled force value. Readings within RAIL_GUARD counts of the
    ADC's ±full-scale limits are discarded as saturation artifacts. Dividing
    by 1000 converts raw counts to the working force units used throughout
    (not calibrated Newtons — compare against CONTACT_FORCE_THRESHOLD).
    """
    while True:
        val = hx.read()
        if val >=  8388607 - RAIL_GUARD:  continue
        if val <= -8388608 + RAIL_GUARD:  continue
        return (val - raw_tare) / 1000

def move_mm(distance_mm, speed_mmps, direction):
    """
    Blocking open-loop move: sets direction, starts the stepper, waits the
    time required to cover the distance at the given speed, then stops.
    Uses time-based control (not step counting), so actual displacement
    depends on the Timer ISR firing reliably at the correct rate.
    The 5 µs delay after setting DIR satisfies the TB6600's setup time.
    """
    dir_pin.value(direction); time.sleep_us(5)
    start_stepper(speed_mmps)
    time.sleep(distance_mm / speed_mmps)
    stop_stepper()

def _run_until_top(mmps, timeout_ms):
    """
    Drives the crosshead upward at mmps until the top limit switch triggers,
    then stops. The limit switch is NC (normally closed) and wired with a
    pull-up, so lim_top.value()==1 means the switch has opened (actuated).
    A deadline prevents infinite motion if the switch is missed or wiring
    fails — raises RuntimeError so the caller can handle it safely.
    """
    dir_pin.value(DIR_UP); time.sleep_us(5)
    start_stepper(mmps)
    deadline = time.ticks_add(time.ticks_ms(), timeout_ms)
    while lim_top.value() != 1:
        if time.ticks_diff(time.ticks_ms(), deadline) >= 0:
            stop_stepper()
            raise RuntimeError("Homing timeout")
    stop_stepper()

def home_to_top():
    """
    Three-phase homing sequence that reproducibly parks the crosshead against
    the top limit switch:
      1. Fast approach — quickly finds the switch but overshoots slightly due
         to inertia and the switch's mechanical hysteresis.
      2. Backoff — retreats HOMING_BACKOFF_MM so the switch is fully released,
         giving a consistent approach direction for phase 3.
      3. Slow re-approach — contacts the switch at low speed to minimise the
         positional error caused by motor coasting and switch bounce.
    This two-speed pattern is standard CNC homing practice.
    """
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
    """
    Main data-collection routine. Starts the stepper downward and polls the
    load cell on every HX711 conversion cycle (~80 ms at 10 SPS):

      - Pre-contact: discards force samples; watches for bottom limit switch
        and the abort button as safety stops.
      - Contact detection: when force exceeds CONTACT_FORCE_THRESHOLD the
        current step count is latched as the displacement origin (zero point).
        From here, displacement = (current_steps - steps_at_contact) / STEPS_PER_MM.
      - Post-contact logging: each sample is written to CSV as
        (elapsed_ms, displacement_mm, force). The stepper keeps running
        uninterrupted so the force-displacement curve has no motion transients.
      - Termination: stops automatically after DESCENT_DISTANCE_MM of travel
        past contact, or immediately if the bottom limit switch trips.

    The stepper is never stopped mid-descent to avoid velocity transients in
    the force data; stop_stepper() is called only in the finally block.
    No print() calls inside the loop — they stall the CPU long enough to
    cause noticeable Timer ISR jitter at low speeds.

    t0: ticks_ms timestamp from the start of the run, used to compute
        elapsed time for the CSV log.
    """
    print(f"Descending at {DESCENT_SPEED_MMPS} mm/s. Waiting for contact "
          f"(threshold={CONTACT_FORCE_THRESHOLD}). Ctrl+C or bottom limit to stop.")

    global _step_count
    _step_count = 0

    dir_pin.value(DIR_DOWN); time.sleep_us(5)
    start_stepper(DESCENT_SPEED_MMPS)

    steps_at_contact = None   # step count snapshot at contact
    stop_at_step     = None   # step count to stop at (contact + DESCENT_DISTANCE_MM)

    try:
        while True:
            if lim_bot.value() == 1:
                break

            f = read_force()
            t = time.ticks_diff(time.ticks_ms(), t0)

            if steps_at_contact is None and f >= CONTACT_FORCE_THRESHOLD:
                steps_at_contact = _step_count
                stop_at_step     = steps_at_contact + int(DESCENT_DISTANCE_MM * STEPS_PER_MM * DISP_CAL)
                print(f"Contact detected  t={t/1000:.2f}s  force={f:+.2f}")

            if steps_at_contact is not None:
                disp = (_step_count - steps_at_contact) / STEPS_PER_MM
                log_f.write(f'{t},{disp:.3f},{f:.2f}\n')

            if (steps_at_contact is not None and stop_at_step is not None
                    and _step_count >= stop_at_step):
                delivered = _step_count - steps_at_contact
                print(f"Target advance reached: {delivered} steps "
                      f"= {delivered/STEPS_PER_MM:.3f} mm commanded")
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
    print("\nInterrupted. Retracting...")
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