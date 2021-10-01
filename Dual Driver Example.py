from machine import SPI, Pin
import time, math

from TMC5160 import TMC5160

"""Testing Routine
"""
    
print("Starting...")

led = Pin(25, Pin.OUT)
led.low()

spi = SPI(0)
spi.init(baudrate=4000000, firstbit=SPI.MSB, bits=8)

stepper1 = TMC5160(spi, 5, 0)
stepper1.enable()
stepper1.setCurrent(1, 10)

stepper2 = TMC5160(spi, 15, 0)
stepper2.enable()
stepper2.setCurrent(1.8, 10)




# Do a general move
stepper1.setAutoRamp(speed=500000, accel=30000) #500000 speed is a good amount
stepper2.setAutoRamp(speed=200000, accel=50000) #500000 speed is a good amount

stepper1.moveToPos(51200 * 10)
stepper2.moveToPos(51200 * 2)
while stepper1.getStatus()['positionReached'] == False:
    while stepper2.getStatus()['positionReached'] == False:
        time.sleep_ms(1)


print()
print("mid 1: ",stepper1.getStatus())
print("mid 2: ",stepper2.getStatus())

#time.sleep(1)

stepper1.moveToPos(0)
stepper2.moveToPos(0)
while stepper1.getStatus()['positionReached'] == False:
    while stepper2.getStatus()['positionReached'] == False:
        time.sleep_ms(1)

print()
print("1: ", stepper1.getStatus())
print("2: ", stepper2.getStatus())



stepper1.disable()
stepper2.disable()



