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

stepperC = TMC5160(spi, 12, 0)
stepperD = TMC5160(spi, 13, 0)

stepperA = TMC5160(spi, 10, 0)
stepperA.enable()
stepperA.setCurrent(0.75, 15)

stepperB = TMC5160(spi, 11, 0)
stepperB.enable()
stepperB.setCurrent(0.75, 15)

stepperA.setStepMode(stepperA.MicroStep64)
stepperB.setStepMode(stepperB.MicroStep32)


# Do a general move
stepperA.setAutoRamp(speed=5000, accel=30000) #500000 speed is a good amount
stepperB.setAutoRamp(speed=50000, accel=50000) #500000 speed is a good amount

stepperA.moveToPos(200 * 64 * 1)
stepperB.moveToPos(200 * 32 * 7)
while stepperA.getStatus()['positionReached'] == False:
    while stepperB.getStatus()['positionReached'] == False:
        time.sleep_ms(1)


print()
print("mid 1: ",stepperA.getStatus())
print("mid 2: ",stepperB.getStatus())

#time.sleep(1)

stepperA.moveToPos(0)
stepperB.moveToPos(0)
while stepperA.getStatus()['positionReached'] == False:
    while stepperB.getStatus()['positionReached'] == False:
        time.sleep_ms(1)

print()
print("1: ", stepperA.getStatus())
print("2: ", stepperB.getStatus())



stepperA.disable()
stepperB.disable()



