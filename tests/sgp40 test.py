from machine import Pin, I2C
from utime import sleep, sleep_us
from bme280 import SGP40

i2c = I2C(1,scl=Pin(27), sda=Pin(26), freq=40000) #all sensor connected through I2C
air_quality = SGP40(i2c, 0x59)

while True:
  Air_quality = air_quality.measure_raw()
  print("Air quality = ",Air_quality)
  sleep(0.1)
  
  