# TMC5160 Driver
# By Ben Jackson


# Wiring:
# Pico      -    AMIS-30543
# SPIO-RX   -    DO     (4.7K pullup)
# SPIO-TX   -    DI
# SPIO-CSK  -    CLK
# 5         -    CS
# 15        -    NXT
# Ground    -    GND
# 
# Also connect the motor power and the stepper driver.


from machine import SPI, Pin
import time, math


class TMC5160:

    # Registers
    GCONF = 0x00
    GSTAT = 0X01
    DRV_CONF = 0X0A
    GLOBALSCALER = 0X0B
    IHOLD_IRUN = 0X10
    TPOWERDOWN = 0X11
    TPWMTHRS = 0X13
    RAMPMODE = 0X20
    XACTUAL = 0X21
    VACTUAL = 0X22
    VSTART = 0X23
    TZEROWAIT = 0X2C
    XTARGET = 0X2D
    SW_MODE = 0X34
    RAMP_STAT = 0X35
    A1 = 0X24
    V1 = 0X25
    AMAX = 0X26
    VMAX = 0X27
    DMAX = 0X28
    D1 = 0X2A
    VSTOP = 0X2B
    CHOPCONF = 0X6C


    maxCurrent = 2.8  # Max contiuous current capabilty of the board in Amps


    def __init__(self, spiObj: SPI, csPin: int, debug=0):

        self.cs = Pin(csPin, Pin.OUT)
        self.cs.high()

        self.spi = spiObj
        self.debug = debug

        self.writeReg(self.GCONF, 0x04)             # enable PWM mode
        self.writeReg(self.CHOPCONF, 0x0100C0)      # using the example
        self.writeReg(self.TPWMTHRS, 0x1F4)         # using the example value of 500


    def intToTwosComp(self, val, nbits):
        """Compute the 2's complement of int value val 

        Args:
            val (int): Signed Integer to convert
            nbits (int): The number of bits for the converted value

        Returns:
            int: an int that represents the raw twos complement
        """
        if val < 0:
            val = (1 << nbits) + val
        else:
            if (val & (1 << (nbits - 1))) != 0:
                # If sign bit is set.
                # compute negative value.
                val = val - (1 << nbits)
        return val


    def TwosComp2Int(self, val, nbits):
        """Compute an Integer (signed or unsigned) from a raw int (converted from bytearray)

        Args:
            val (int): Raw conversion from bytearray or bytes

        Returns:
            int: signed integer
        """
        if val & 2**(nbits-1) > 0:
            return val - 2**nbits
        else:
            return val


    def writeReg(self, address: int, value: int, signed=False):
        rb = bytearray(5)       # buffer for the data we get back

        if signed:
            value = self.intToTwosComp(value, 32)

        data = 0x80             # first bit indicates a write.  
        data = address | data   # add in the address
        data = data << 32
        data = data | value

        self.cs.low()
        self.spi.write_readinto(data.to_bytes(5, 'big'), rb)
        self.cs.high()

        if self.debug >= 4: print("Write Reg:", data.to_bytes(5, 'big'))  # .to_bytes(5, 'big', signed=False)
        if self.debug >= 4: print("       rb:", rb)
        if self.debug >= 4: print("    status:", rb[0] & 0b10)

        return rb[0] & 0b10     # return the status.


    def readReg(self, address: int, signed=False):
        rb = bytearray(5)       # buffer for the data we get back

        data = address << 32

        self.cs.low()
        self.spi.write_readinto(data.to_bytes(5, 'big'), rb)
        self.cs.high()
        self.cs.low()
        self.spi.write_readinto(data.to_bytes(5, 'big'), rb)
        self.cs.high()

        if self.debug >= 3: print(" Read Reg address:", hex(address))  # .to_bytes(5, 'big', signed=False)
        if self.debug >= 3: print("       rb:", rb)
        if self.debug >= 2: print("    status:", rb[0] & 0b10)

        status = int.from_bytes(rb, 'big') >> 32

        if signed:
            return self.TwosComp2Int(int.from_bytes(rb, 'big')  & 0xFFFFFFFF, 32), status
        else:
            return int.from_bytes(rb, 'big')  & 0xFFFFFFFF, status


    def setCurrent(self, current: float, idlePercent: float):
        """Set the max current via the globalscaler.

        Args:
            current (float): In Amps
            idlePercent (float): 0->100
        """
        # set the glocal current scale
        if current > self.maxCurrent:
            raise AttributeError("Current exceeds board capability")
        if idlePercent > 100 or idlePercent < 0:
            raise AttributeError("Idle Current Percentage must be between 0 and 100")

        m = 255/self.maxCurrent
        gs = int(math.floor((current * m)))
        self.writeReg(self.GLOBALSCALER, gs)

        # set the I_RUN and I_HOLD
        iHold = int(math.floor(((idlePercent/100) * 32) -1))
        iHold_iRun_value = iHold | (31 << 8) | (0x6 <<16)
        self.writeReg(self.IHOLD_IRUN, iHold_iRun_value)

        # set TPOWERDOWN to default (10)
        self.writeReg(self.TPOWERDOWN, 0x0A)


    def setRamp(self, vstart, a1, v1, amax, vmax, dmax, d1, vstop):
        """Configure the Ramp parameters

        Args:
            vstart (int): register value
            a1 (int): register value
            v1 (int): register value
            amax (int): register value
            vmax (int): register value
            dmax (int): register value
            d1 (int): register value
            vstop (int): register value
        """
        self.writeReg(self.VSTART, vstart)
        self.writeReg(self.A1, a1)
        self.writeReg(self.V1, v1)
        self.writeReg(self.AMAX, amax)
        self.writeReg(self.VMAX, vmax)
        self.writeReg(self.DMAX, dmax)
        self.writeReg(self.D1, d1)
        self.writeReg(self.VSTOP, vstop)
        self.writeReg(self.RAMPMODE, 0x00)


    def setAutoRamp(self, speed, accel):
        """Configure a simple ramp

        Args:
            speed (int): the max speed
            accel (int): general acceleration value
        """
        self.setRamp(vstart=0, a1=accel, v1=int(speed/2), amax=int(accel/2), vmax=speed, \
                    dmax=int((accel*1.5)/2), d1=int(accel*1.5), vstop=10)  


    def moveToPos(self, pos):
        """Move the motor to a specific position.  

        Args:
            pos (int): the position in (micro)steps

        Returns:
            int: status (1 = error)
        """
        if pos < -2147483648 or pos > 2147483648:
            raise AttributeError("Target postition out of range")
        return self.writeReg(self.XTARGET, pos, True) 


    def getPos(self):
        """Read the current motor position

        Returns:
            int: the position in (micro)steps
        """
        return self.readReg(self.XACTUAL, True)[0]


    def getStatus(self):
        """Read the current driver status

        Returns:
            dict: Dictionary of status values
        """
        x = self.readReg(self.XACTUAL, True)
        status = x[1]
        position = x[0]

        positionReached = bool(status & 0b00100000)
        velocityReached = bool(status & 0b00010000)
        standstill      = bool(status & 0b00001000)
        sg2             = bool(status & 0b00000100)
        driverError     = bool(status & 0b00000010)
        resetFlag       = bool(status & 0b00000001)

        statDict = {
            'positionReached': positionReached,
            'velocityReached': velocityReached,
            'standStill': standstill,
            'sg2': sg2,
            'driverError': driverError,
            'resetFlag': resetFlag,
            'position': position
            }
        return statDict


    def disable(self):
        """Disable the motor
        """
        self.writeReg(self.CHOPCONF, 0x0100C0)
    

    def enable(self):
        """Enable the motor
        """
        self.writeReg(self.CHOPCONF, 0x0100C3)


    def setHomePosition(self):
        """Set the current motor postion to 0
        """
        self.writeReg(self.XACTUAL, 0)


    def configHoming(self, softStop, enableLeft, enableRight, invertLeft=False, invertRight=False):
        """Set the parameters for the limit switches

        Args:
            softStop (bool): True = Soft Stop, False = Hard Stop (Emergency stop)
            enableLeft (bool): enable the left limit switch
            enableRight (bool): enable the right limit switch
            invertLeft (bool, optional): Reverse the polarity of the left switch. Defaults to False.
            invertRight (bool, optional): Reverse the polarity of the right switch. Defaults to False.
        """
        data = int(softStop) << 11
        data = (int(invertLeft) << 2) | data
        data = (int(invertRight) << 3) | data
        data = int(enableLeft) | data
        data = (int(enableRight) << 1) | data

        self.writeReg(self.SW_MODE, data)






if __name__ == "__main__":
    """Testing Routine
    """
        
    print("Starting...")

    led = Pin(25, Pin.OUT)
    led.low()

    spi = SPI(0)
    spi.init(baudrate=4000000, firstbit=SPI.MSB, bits=8)

    stepper = TMC5160(spi, 5, 0)
    stepper.enable()
    stepper.setCurrent(1.6, 10)


    # Homing to the left switch
    stepper.setAutoRamp(speed=30000, accel=50000)           # set homing speed
    stepper.configHoming(True,  True, True, True, True)     # setup the hoping options
    stepper.moveToPos(-51200 * 50)                          # start the homing move negative is to left
    time.sleep_ms(10)                                       # Give the motor time to get moving
    while stepper.getStatus()['standStill'] == False:
        time.sleep_ms(5)

    print("HOME FOUND")
    stepper.setHomePosition()
    stepper.moveToPos(0)                                    # cancel the rest of the move

    time.sleep(1)




    # Do a general move
    stepper.setAutoRamp(speed=1000000, accel=50000) #500000 speed is a good amount
    
    stepper.moveToPos(51200 * 750)
    while stepper.getStatus()['positionReached'] == False:
        time.sleep_ms(1)
        if stepper.getStatus()['velocityReached']:  led.high()          # turn on the LED once the motor gets to full speed
        else: led.low()

    print(stepper.getStatus())
    stepper.moveToPos(0)
    while stepper.getStatus()['positionReached'] == False:
        time.sleep_ms(1)
        if stepper.getStatus()['velocityReached']:  led.high()          # turn on the LED once the motor gets to full speed
        else: led.low()

    print(stepper.getStatus())
    


    stepper.disable()











