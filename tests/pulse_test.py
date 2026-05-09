# Slow-pulse TB6600 diagnostic. 25 steps/sec for 200 steps (~8 sec of motion).
# Bypasses the Timer to rule out startup-stall and Timer-config issues.
from machine import Pin
import time

step = Pin(0, Pin.OUT, value=0)
dirp = Pin(1, Pin.OUT, value=1)
time.sleep_ms(10)

print("Pulsing 200 steps at 25 Hz. Watch the motor.")
for _ in range(200):
    step.value(1); time.sleep_ms(20)
    step.value(0); time.sleep_ms(20)
print("Done.")
