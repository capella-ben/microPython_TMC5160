from TMC5160 import TMC5160
import time
from machine import SPI, Pin
from random import random

stepsPerRotation = 200

if __name__ == "__main__":
    """Testing Routine
    """
        
    led = Pin(25, Pin.OUT)
    led.low()

    spi = SPI(0)
    spi.init(baudrate=4000000, firstbit=SPI.MSB, bits=8)

    stepper = TMC5160(spi, 10, 0)
    
    stepper.enable()
    stepper.setCurrent(1.7, 15)
    stepper.setStepMode(stepper.MicroStep64)

    
    stepper.setAutoRamp(speed=100000, accel=8000) 
    

    for i in range(500):
        stepper.moveToPos(int(stepsPerRotation * stepper.microStep * random()) * 2)
        while stepper.getStatus()['positionReached'] == False:
            time.sleep_ms(1)
            if stepper.getStatus()['velocityReached']:  led.high()          # turn on the LED once the motor gets to full speed
            else: led.low()
        time.sleep_ms(5)



    #time.sleep(5)

    stepper.disable()











