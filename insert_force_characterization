'''
Heat-set insert force characterization.

Cycle:
  1. Home crosshead to top limit switch
  2. Descend HOME_OFFSET_MM to stage just above workpiece
  3. Prompt operator to turn on soldering iron + press Enter
  4. Descend DESCENT_DISTANCE_MM slowly while logging force to CSV
  5. Retract to top limit

Safety:
  - GP20 (onboard MakerPi button, active LOW) aborts descent and retracts
  - Bottom limit switch is polled on every force sample during descent
  - Ctrl+C at any phase cleans up and retracts

Install on the Pico first:
    mpremote mip install github:bikeNomad/micropython-rp2-smartStepper

GPIO per System Wiring Diagram 09APR2026.pdf.
'''

import asyncio
import time
from machine import Pin
from hx711_pio import HX711
from smartstepper import SmartStepper
from smartstepper.homing import home

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
DESCENT_DISTANCE_MM = 4.0         # insert depth 4 mm (exact heat-set insert depth)

HOMING_FAST_MMPS    = 4.0
HOMING_SLOW_MMPS    = 0.5
POSITION_SPEED_MMPS = 2.5
DESCENT_SPEED_MMPS  = 1.0
MAX_SPEED_MMPS      = 6.0
ACCEL_MMPS2         = 30.0

STATUS_PERIOD_MS = 200
RAIL_GUARD       = 100
LOG_FILE         = 'insert_force_log.csv'

# ---------------------------------------------------------------------------
# Hardware init
# ---------------------------------------------------------------------------
lim_top  = Pin(PIN_TOP,  Pin.IN, pull=Pin.PULL_UP)
lim_bot  = Pin(PIN_BOT,  Pin.IN, pull=Pin.PULL_UP)
stop_btn = Pin(PIN_STOP_BTN, Pin.IN, pull=Pin.PULL_UP)

stepper = SmartStepper(stepPin=PIN_STEP, dirPin=PIN_DIR, accelCurve='smooth2')
stepper.stepsPerUnit = STEPS_PER_MM
stepper.maxSpeed     = MAX_SPEED_MMPS
stepper.acceleration = ACCEL_MMPS2

hx = HX711(Pin(PIN_HX_SCK, Pin.OUT),
           Pin(PIN_HX_DT,  Pin.IN, pull=Pin.PULL_DOWN),
           gain=128, state_machine=1)   # SmartStepper claims SM0 (pulse gen) + SM4 (step counter); use SM1

# ---------------------------------------------------------------------------
# HX711 startup + tare
# ---------------------------------------------------------------------------
hx.power_down(); time.sleep_ms(100); hx.power_up(); time.sleep_ms(500)
print("Taring load cell (keep it unloaded)...")
hx.tare(times=15)
raw_tare = hx.read()
print("Tare done.")

# ---------------------------------------------------------------------------
# Abort button (armed only during descent)
# ---------------------------------------------------------------------------
abort_flag = False
def _stop_btn_irq(_pin):
    global abort_flag
    abort_flag = True

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def read_force():
    while True:
        val = hx.read()
        if val >=  8388607 - RAIL_GUARD:  continue
        if val <= -8388608 + RAIL_GUARD:  continue
        return (val - raw_tare) / 1000

def home_top():
    # 3-phase homing from smartstepper.homing: fast approach, backoff, slow re-approach
    asyncio.run(home(stepper, lim_top,
                     fastSpeed=HOMING_FAST_MMPS, slowSpeed=HOMING_SLOW_MMPS,
                     direction='up', activeState=1, timeout=30))

def move_to(distance_mm, speed_mmps):
    stepper.maxSpeed = speed_mmps
    stepper.moveTo(distance_mm, relative=True)
    stepper.waitEndOfMove()
    stepper.maxSpeed = MAX_SPEED_MMPS

# ---------------------------------------------------------------------------
# Descent with live logging + abort polling
# ---------------------------------------------------------------------------
def descend_and_log(log_f, t0):
    global abort_flag
    abort_flag = False
    stop_btn.irq(trigger=Pin.IRQ_FALLING, handler=_stop_btn_irq)

    print(f"Descending {DESCENT_DISTANCE_MM} mm at {DESCENT_SPEED_MMPS} mm/s. "
          f"Press GP20 to abort.")

    stepper.maxSpeed = DESCENT_SPEED_MMPS
    stepper.moveTo(DESCENT_DISTANCE_MM, relative=True)

    ramp_t = DESCENT_SPEED_MMPS / ACCEL_MMPS2
    budget_ms = int((DESCENT_DISTANCE_MM / DESCENT_SPEED_MMPS + 2 * ramp_t) * 1000) + 500
    deadline  = time.ticks_add(time.ticks_ms(), budget_ms)

    last_status = time.ticks_ms()
    try:
        while True:
            if abort_flag:
                print("ABORT pressed — stopping descent."); break
            if lim_bot.value() == 1:
                print("Bottom limit hit — stopping descent."); break
            if time.ticks_diff(time.ticks_ms(), deadline) >= 0:
                print("Target descent distance reached."); break

            f = read_force()
            t = time.ticks_diff(time.ticks_ms(), t0)
            log_f.write(f'{t},{f:.2f}\n')

            now = time.ticks_ms()
            if time.ticks_diff(now, last_status) >= STATUS_PERIOD_MS:
                last_status = now
                print(f"  t={t/1000:6.2f}s  force={f:+6.2f}N")
                log_f.flush()
    finally:
        stepper.stop(emergency=True)
        stepper.waitEndOfMove()
        stepper.maxSpeed = MAX_SPEED_MMPS
        stop_btn.irq(handler=None)

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
log_f = open(LOG_FILE, 'w')
log_f.write('time_ms,force_N\n')
t0 = time.ticks_ms()
print(f"Logging to {LOG_FILE}")

try:
    print("Homing to top...")
    home_top()

    print(f"Positioning {HOME_OFFSET_MM} mm below top...")
    move_to(HOME_OFFSET_MM, POSITION_SPEED_MMPS)

    try:
        input("\nTurn on the soldering iron. Press Enter to begin descent "
              "(Ctrl+C to abort): ")
    except EOFError:
        raise KeyboardInterrupt

    descend_and_log(log_f, t0)

    print("Retracting to top...")
    home_top()

    print(f"\nDone. Log saved to {LOG_FILE}.")

except KeyboardInterrupt:
    print("\nInterrupted — retracting for safety.")
    try:
        stepper.stop(emergency=True); stepper.waitEndOfMove()
        home_top()
    except Exception as e:
        print(f"Retract failed: {e}")

finally:
    try:
        stepper.stop(emergency=True); stepper.waitEndOfMove(); stepper.disable()
    except Exception: pass
    try:
        log_f.flush(); log_f.close()
    except Exception: pass
    hx.power_down()
    print("HX711 powered down.")
