'''
ME106 Project: Motorized heat-set insert press

Main Components:
- Raspberry Pi Pico microcontroller
- Stepper motor with TB6600 driver for vertical motion
- Fan for cooling stepper driver 
- HX711 load cell amplifier for force measurement   
- LM393 photoresistor sensor to detect part presence and abort if no part is detected
- Two limit switches for homing and safety (top and bottom)

Program flow:
  1. Home crosshead to top limit
  2. Descend HOME_OFFSET_MM to stage above the insert
  3. Wait for GP20 press to start press cycle
  4. Approach -> dwell at contact -> seat DESCENT_DISTANCE_MM past contact
        Exceptions: Ctrl+C, bottom limit switch, or photoresistor LOW to abort press cycle and retract to top
  5. Retract to top limit
  6. Force-displacement (F-D) data is logged to CSV on Pico local storage.
'''

import time
from machine import Pin, Timer
from hx711_pio import HX711

# ---------------------------------------------------------------------------
# Pin assignments
# ---------------------------------------------------------------------------
PIN_STEP, PIN_DIR     = 0, 1       # Stepper pins
PIN_HX_DT, PIN_HX_SCK = 10, 11     # Load Cell HX711 pins
PIN_PHOT              = 13         # LM393 photoresistor digital out; LOW = abort
PIN_FAN               = 15         # Fan control pin (active HIGH via transistor)
PIN_TOP, PIN_BOT      = 16, 17     # Limit Switches
PIN_START_BTN         = 20         # Start button for press cycle on MakerPi board

# ---------------------------------------------------------------------------
# Motion and Force variables (mm and mm/s)
# ---------------------------------------------------------------------------
HOME_OFFSET_MM      = 5.0          # standby offset below top limit before press
DESCENT_DISTANCE_MM = 3.0          # press displacement; set equal to insert length
HOMING_BACKOFF_MM   = 2.0          # backoff between fast and slow homing passes

HOMING_FAST_MMPS    = 8.0          # fast approach to top limit
HOMING_SLOW_MMPS    = 1.0          # slow re-approach after backoff
APPROACH_SPEED_MMPS = 2.0          # pre-contact descent through air
SEATING_SPEED_MMPS  = 0.25         # post-contact seating push

STEPS_PER_MM = 200 * 8 // 5        # 200 steps/rev * 8 microsteps / 5 mm pitch = 320 steps/mm

CONTACT_FORCE_THRESHOLD = 10.0     # tared scaled-force units (raw_counts / 1000), chosen empirically

HEAT_SOAK_DWELL_MS  = 3000         # hold at contact to heat up insert, chosen empirically

DIR_UP   = 1
DIR_DOWN = 0

HOMING_TIMEOUT_MS = 30000          # timeout to avoid physical crashing

pico_dir   = __file__.rsplit('/', 1)[0] if '/' in __file__ else '.'   # log file in same directory as script on Pico
LOG_FILE = pico_dir + '/press_log.csv'

# ---------------------------------------------------------------------------
# GP Pin Initialization
# ---------------------------------------------------------------------------
step_pin  = Pin(PIN_STEP, Pin.OUT, value=0)
dir_pin   = Pin(PIN_DIR,  Pin.OUT, value=DIR_UP) # initialize as UP for safety
lim_top   = Pin(PIN_TOP,  Pin.IN, pull=Pin.PULL_UP) # pull-up for NC limit switches
lim_bot   = Pin(PIN_BOT,  Pin.IN, pull=Pin.PULL_UP)
start_btn = Pin(PIN_START_BTN, Pin.IN, pull=Pin.PULL_UP) # pull-up for active LOW button on MakerPi board
phot      = Pin(PIN_PHOT, Pin.IN)         
fan_pin   = Pin(PIN_FAN, Pin.OUT, value=0) # fan initially off, turns on during press cycle

hx = HX711(Pin(PIN_HX_SCK, Pin.OUT),
           Pin(PIN_HX_DT,  Pin.IN, pull=Pin.PULL_DOWN),
           gain=128, state_machine=0) # library uses PIO state machine for non-blocking reads of load cell

hx.power_down(); time.sleep_ms(100); hx.power_up(); time.sleep_ms(500) # power cycle HX711 for stable start

print("Taring load cell (keep it unloaded)...")
hx.tare(times=15) # tare takes avg of 15 readings
raw_tare = hx.read()
print("Tare done.")

# ---------------------------------------------------------------------------
# Stepper functions
# ---------------------------------------------------------------------------
step_tmr = Timer()      # hardware timer to trigger step pulses without blocking main program
_step_state = 0         # tracks current state of STEP pin for toggling in timer callback
_step_count = 0         # counts rising edges of STEP pin to track steps taken for displacement calculation based on STEPS_PER_MM 

def _step_cb(_t):
    '''
    Hardware timer-driven GPIO toggling for flipping the STEP pin at a fixed rate set by start_stepper(), 
    similar to bitbanging but without blocking the main thread.
    Each call to _step_cb() flips the STEP pin (LOW, HIGH, LOW). 
    Each rising edge (LOW->HIGH) is a step sent to the TB6600 driver. 
    Count only rising edges so _step_count equals steps taken.
    '''
    global _step_state, _step_count      # use global variables to track state across calls since callback can't take arguments or return values
    _step_state = _step_state ^ 1        # flip between 0 and 1 each call using XOR
    step_pin.value(_step_state)
    if _step_state == 1:                 # on rising edge, count a step 
        _step_count += 1

def start_stepper(mmps):
    '''Start stepping at `mmps` mm/s. Must set dir_pin before calling.

    The timer fires at 2x step rate because each step requires two flips of the STEP pin:
    one to drive STEP high and one to drive it low again.
    '''
    freq_hz = int(2 * mmps * STEPS_PER_MM)  # calculate step frequency for desired speed
    step_tmr.init(freq=freq_hz, 
                  mode=Timer.PERIODIC, 
                  callback=_step_cb)        # start timer to run _step_cb at fixed frequency

def stop_stepper():
    '''Stop the Timer and drive STEP low. _step_count is preserved for callers.'''
    global _step_state
    try:
        step_tmr.deinit()       # stop timer to halt stepping and leaves STEP pin at its last state
    except Exception:           
        pass
    _step_state = 0             # ensure STEP state is LOW when stopped to avoid unintended steps if timer restarts
    step_pin.value(0)           # drive STEP GP pin LOW

# ---------------------------------------------------------------------------
# Helper Functions
# ---------------------------------------------------------------------------
class PhotAbort(Exception):
    '''Exception raised by check_phot().'''
    pass

def check_phot():
    '''Raise PhotAbort exception if GP13 reads LOW (no part on base)'''
    if phot.value() == 0:                           # phot LOW means no part detected, so raise exception to abort press cycle and retract
        raise PhotAbort 

def read_force():
    '''Return one tared force sample, rejecting HX711 
    values within 100 counts of upper/lower max values of load cell.'''
    while True:
        val = hx.read()
        if val >=  8388607 - 100:  continue         # HX711 randomly reads max values, this filters out random max readings
        if val <= -8388608 + 100:  continue
        return (val - raw_tare) / 1000 

def move_mm(distance_mm, speed_mmps, direction):
    '''Move a specific distance at a given speed and direction.'''
    dir_pin.value(direction); time.sleep_us(5)      # sleep after changing direction to allow TB6600 to register before stepping
    start_stepper(speed_mmps)
    time.sleep(distance_mm / speed_mmps)
    stop_stepper()

def _run_until_top(mmps, timeout_ms):
    '''Run stepper in UP direction until top limit switch contacted.'''
    dir_pin.value(DIR_UP); time.sleep_us(5)         # sleep after changing direction to allow TB6600 to register before stepping
    start_stepper(mmps)
    deadline = time.ticks_add(time.ticks_ms(), timeout_ms)
    while lim_top.value() != 1:                     # loop until top limit switch is triggered (active HIGH)
        if time.ticks_diff(time.ticks_ms(), deadline) >= 0:
            stop_stepper()
            raise RuntimeError("Homing timeout")
    stop_stepper()

def home_to_top():
    '''Home to top limit switch in two stages: 
    first a fast approach, then backoff and approach top limit slowly for accuracy.'''

    _run_until_top(HOMING_FAST_MMPS, HOMING_TIMEOUT_MS)     # fast approach to trigger limit switch
    time.sleep_ms(100)
    move_mm(HOMING_BACKOFF_MM, HOMING_SLOW_MMPS, DIR_DOWN)  # backoff to avoid mechanical hysteresis and allow for more accurate homing on second pass
    time.sleep_ms(100)
    _run_until_top(HOMING_SLOW_MMPS, HOMING_TIMEOUT_MS)     # slow approach for accurate homing position

def log_row(t, disp, f):
    '''Append one CSV row to log file with time (ms), displacement (mm), and force.'''
    log_f.write(f'{t},{disp:.3f},{f:.2f}\n')                # log time, displacement, and force

# ---------------------------------------------------------------------------
# Press cycle phases helper functions
# Each phase has termination condition: CTRL+C OR bottom limit
# Photoresistor exception is in the main loop instead of helper functions since it can occur at any phase
# ---------------------------------------------------------------------------
def approach_until_contact(t0):
    '''Step DOWN until force >= threshold. Returns (steps, t_ms, force) at contact,
    or None if bottom limit tripped first.'''
    dir_pin.value(DIR_DOWN); time.sleep_us(5)
    start_stepper(APPROACH_SPEED_MMPS)
    while True:
        if lim_bot.value() == 1:                            # bottom limit to avoid crashing into base
            stop_stepper()
            return None
        check_phot()                                        # check photoresistor to abort if no part detected
        f = read_force()
        if f >= CONTACT_FORCE_THRESHOLD:                    # contact detected based on force threshold
            stop_stepper()
            t = time.ticks_diff(time.ticks_ms(), t0)
            return _step_count, t, f

def dwell(t0):
    '''Hold position for HEAT_SOAK_DWELL_MS, logging force at displacement 0.
    Returns True on completion, False if bottom limit tripped. May raise PhotAbort.'''
    deadline = time.ticks_add(time.ticks_ms(), HEAT_SOAK_DWELL_MS)
    while time.ticks_diff(time.ticks_ms(), deadline) < 0:   # loop until dwell time has elapsed, checking for bottom limit and phot abort
        if lim_bot.value() == 1:
            return False
        check_phot()                                        # check photoresistor to abort if no part detected
        f = read_force()
        t = time.ticks_diff(time.ticks_ms(), t0)
        log_row(t, 0.0, f)
    return True

def seat_to_depth(t0, steps_at_contact):
    '''Move DOWN at seating speed until DESCENT_DISTANCE_MM past contact, logging
    displacement and force. Returns True on depth reached, False on bottom limit to avoid crashing.
    '''
    stop_at_step = steps_at_contact + int(DESCENT_DISTANCE_MM * STEPS_PER_MM)
    dir_pin.value(DIR_DOWN); time.sleep_us(5)
    start_stepper(SEATING_SPEED_MMPS)
    while _step_count < stop_at_step:                       # loop until desired depth reached, checking for bottom limit and phot abort
        if lim_bot.value() == 1:
            stop_stepper()
            return False
        check_phot()                                        # check photoresistor to abort if no part detected
        f = read_force()
        t = time.ticks_diff(time.ticks_ms(), t0)
        disp = (_step_count - steps_at_contact) / STEPS_PER_MM
        log_row(t, disp, f)
    stop_stepper()
    return True

def press_cycle(t0):
    '''
    Main function to run one full descent: approach -> dwell -> seat. Stepper guaranteed off on exit.
    '''
    print(f"Approaching at {APPROACH_SPEED_MMPS} mm/s. Waiting for contact "
          f"(threshold={CONTACT_FORCE_THRESHOLD}). Ctrl+C or bottom limit to stop.")

    global _step_count             # set global variable to track steps taken during press cycle
    _step_count = 0

    try:
        contact = approach_until_contact(t0)    # approach until contact, get step count at contact and time of contact for reference
        if contact is None:                     
            return
        steps_at_contact, t, f = contact
        print(f"Contact detected  t={t/1000:.2f}s  force={f:+.2f}"
              f"dwell {HEAT_SOAK_DWELL_MS/1000:.0f}s")

        if not dwell(t0):                       # dwell at contact to heat up insert and zero out displacement
            return

        print(f"Pressing at {SEATING_SPEED_MMPS} mm/s")  
        if not seat_to_depth(t0, steps_at_contact):     # seat to depth past contact, logging F-D data, until insert depth reached
            return

        delivered = _step_count - steps_at_contact
        print(f"Insert depth reached: {delivered} steps "
              f"= {delivered/STEPS_PER_MM:.3f} mm commanded") # show step count and calculated displacement in mm for reference
    finally:                    
        stop_stepper()

# ---------------------------------------------------------------------------
# Main Loop
# ---------------------------------------------------------------------------
log_f = open(LOG_FILE, 'w')
log_f.write('time_ms,displacement_mm,force\n')      # open log file and write header row for time, displacement, and force

print(f"Logging to {LOG_FILE}")
print(f"Limit switches at startup: top={lim_top.value()} bot={lim_bot.value()} (expect both 0 when not pressed)")
fan_pin.on()                                        # 2nd actutator for project: always ON during operation to cool down stepper driver
t0 = time.ticks_ms()                                # reference start time for press cycle, used for timestamping log data and calculating elapsed time during cycle

try:            
    while True:
        try:
            print("Homing to top...")
            home_to_top()

            print(f"Positioning {HOME_OFFSET_MM} mm below top...")
            move_mm(HOME_OFFSET_MM, HOMING_SLOW_MMPS, DIR_DOWN)

            print("\nReady. Press GP20 to begin cycle (Ctrl+C to exit, remove part to pause).")
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

            print("\nPress cycle complete.")
            print(f"Copy F-D data to PC in Thonny.")
            break

        except PhotAbort:
            print("\nNo part detected. Retracting to top... Insert part to resume.")
            stop_stepper()
            home_to_top()
            while phot.value() == 0:
                time.sleep_ms(100)
            print("Part detected. Resuming.")

except KeyboardInterrupt:
    print("\nInterrupted. Retracting...")
    stop_stepper()
    home_to_top()


finally:
    stop_stepper()
    fan_pin.off()
    try:
        log_f.flush(); log_f.close()    # ensure log is saved even if file close fails
    except Exception: pass 
    hx.power_down()