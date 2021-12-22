# TMC5160 Driver
# By Ben Jackson


# Wiring:
# Pico      -    TMC5160
# SPIO-RX   -    DO     (4.7K pullup)
# SPIO-TX   -    DI
# SPIO-CSK  -    CLK
# 15        -    CS
# 


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

    chopBaseVal = 0      # The first 24 bits of CHOPCONF
    microStepVal = 0x0
    microStep = 256

    MicroStep256 = (256, 0x0)
    MicroStep128 = (128, 0x1)
    MicroStep64 = (64, 0x2)
    MicroStep32 = (32, 0x3)
    MicroStep16 = (16, 0x4)
    MicroStep8 = (8, 0x5)
    MicroStep4 = (4, 0x6)
    MicroStep2 = (2, 0x7)
    MicroStep1 = (1, 0x8)



    maxCurrent = 2.8  # Max contiuous current capabilty of the board in Amps


    def __init__(self, spiObj: SPI, csPin: int, debug=0):
        """Initialise

        Args:
            spiObj (SPI): The SPI object to use.
            csPin (int): The cs pin number
            debug (int, optional): Enable debug. Defaults to 0.
        """

        self.cs = Pin(csPin, Pin.OUT)
        self.cs.high()

        self.spi = spiObj
        self.debug = debug

        self.writeReg(self.GCONF, 0x04)             # enable PWM mode
        self.setChopperConfig(3, 0, 13, 2, 0, False)
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


    def TwosComp2Int(self, val, nbits) -> int:
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
        """Write to a register.  Can write to a signed register.

        Args:
            address (int): The address to write to.
            value (int): The value to write
            signed (bool, optional): Convert the value to a two's compliment signed integer. Defaults to False.

        Returns:
            int: Status of the write
        """
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
        """Read a register.  Can convert a signed integer value.

        Args:
            address (int): The address to read
            signed (bool, optional): Converts a two's compliment number. Defaults to False.

        Returns:
            tuple: The value read from the register and the status.
        """
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
        if current > self.maxCurrent:
            raise AttributeError("Current exceeds board capability")
        if idlePercent > 100 or idlePercent < 0:
            raise AttributeError("Idle Current Percentage must be between 0 and 100")

        # set the glocal current scale
        m = 255/self.maxCurrent
        gs = int(math.floor((current * m)))
        self.writeReg(self.GLOBALSCALER, gs)

        # set the I_RUN and I_HOLD
        iHold = int(math.floor(((idlePercent/100) * 32) -1))
        iHold_iRun_value = iHold | (31 << 8) | (0x6 <<16)
        self.writeReg(self.IHOLD_IRUN, iHold_iRun_value)

        # set TPOWERDOWN to default (10)
        self.writeReg(self.TPOWERDOWN, 0x0A)


    def setRamp(self, vstart: int, a1: int, v1: int, amax: int, vmax: int, dmax: int, d1: int, vstop: int):
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
        assert 0 <= vstart <= 0x3FFFF,      "vstart is out or range"
        assert 0 <= a1 <= 0xFFFF,           "a1 is out of range"
        assert 0 <= v1 <= 0xFFFFF,          "v1 is out of range"
        assert 0 <= amax <= 0xFFFF,         "amax is out of range"
        assert 0 <= vmax <= 0x7FFFFF,       "vmax is out of range"
        assert 0 <= dmax <= 0xFFFF,         "dmax is out of range"
        assert 0 <= d1 <= 0xFFFF,           "d1 is out of range"
        assert 0 <= vstop <= 0x3FFFF,       "vstop is out of range"

        self.writeReg(self.VSTART, vstart)
        self.writeReg(self.A1, a1)
        self.writeReg(self.V1, v1)
        self.writeReg(self.AMAX, amax)
        self.writeReg(self.VMAX, vmax)
        self.writeReg(self.DMAX, dmax)
        self.writeReg(self.D1, d1)
        self.writeReg(self.VSTOP, vstop)
        self.writeReg(self.RAMPMODE, 0x00)


    def setAutoRamp(self, speed: int, accel: int):
        """Configure a simple ramp

        Args:
            speed (int): the max speed
            accel (int): general acceleration value
        """
        self.setRamp(vstart=0, a1=accel, v1=int(speed/2), amax=int(accel/2), vmax=speed, \
                    dmax=int((accel*1.5)/2), d1=int(accel*1.5), vstop=10)  


    def moveToPos(self, pos: int):
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
        #self.writeReg(self.CHOPCONF, 0x050100C0)
        self.writeReg(self.CHOPCONF, (self.chopBaseVal  & 0xFFFFF0) | (self.microStepVal << 24))
    

    def enable(self):
        """Enable the motor
        """
        self.writeReg(self.CHOPCONF, self.chopBaseVal  | (self.microStepVal << 24))
        #self.writeReg(self.CHOPCONF, 0x050100C3)

    def setStepMode(self, msMode: tuple):
        """Set the Microstepping mode

        Args:
            msMode (Microstep tupple values): eg: self.MicroStep256
        """
        self.microStepVal = msMode[1]
        self.microStep = msMode[0]
        self.writeReg(self.CHOPCONF, self.chopBaseVal  | (self.microStepVal << 24))

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


    def setChopperConfig(self, tOff: int, hStart: int, hEnd: int, tbl: int, tpfd: int, vHigh=False):
        """Set the Chopper Configuration Parameters. Chopper mode is always set to SpreadCycle (0). See the data sheet for full explaniations. 

        Args:   
            tOff (int): off time setting.
            hStart (int): hysteresis start value.
            hEnd (int): hysteresis end value.
            tbl (int): blank time select.
            tpfd (int): passive fast decay time.
            vHigh (bool, optional): Enable high velicity settings. Defaults to False.
        """
        assert 2 <= tOff <= 15,     "tOff Out of range: 2->15"
        assert 0 <= hStart <= 7,    "hStart Out of range: 0->7"
        assert 0 <= hEnd <= 15,     "hEnd Out of range: 0->15"
        assert 0 <= tbl <= 3,       "tbl Out of range: 0->3"
        assert 0 <= tpfd <= 15,     "tpfd Out of range: 0->15"
        
        ccVal = tOff
        ccVal = (hStart << 4) + ccVal
        ccVal = (hEnd << 7) + ccVal
        ccVal = (tbl << 15) + ccVal
        ccVal = (tpfd << 20) + ccVal
        if vHigh:               # enable vhighchm and vhighfs
            ccVal = (3 << 18) + ccVal
        ccVal = (self.microStepVal << 24) + ccVal

        self.chopBaseVal = ccVal
        self.writeReg(self.CHOPCONF, self.chopBaseVal) 






if __name__ == "__main__":
    """Testing Routine
    """
        
    print("Starting...")

    led = Pin(25, Pin.OUT)
    led.low()

    spi = SPI(0)
    spi.init(baudrate=4000000, firstbit=SPI.MSB, bits=8)

    stepperA = TMC5160(spi, 10, 0)
    #stepperB = TMC5160(spi, 11, 0)
    #stepperC = TMC5160(spi, 12, 0)
    #stepperD = TMC5160(spi, 13, 0)
    #stepperE = TMC5160(spi, 16, 0)
    #stepperF = TMC5160(spi, 15, 0)
    
    stepperA.enable()
    stepperA.setCurrent(0.7, 15)
    print(stepperA.getStatus())
    stepperA.setStepMode(stepperA.MicroStep64)

    
    # Homing to the left switch
    stepperA.setAutoRamp(speed=20000, accel=8000)           # set homing speed
    stepperA.configHoming(softStop=True,  enableLeft=True, enableRight=True, invertLeft=True, invertRight=True)     # setup the hoping options
    stepperA.moveToPos(-51200 * 50)                          # start the homing move negative is to left
    time.sleep_ms(10)                                       # Give the motor time to get moving
    while stepperA.getStatus()['standStill'] == False:
        time.sleep_ms(5)

    print("HOME FOUND")
    stepperA.setHomePosition()
    stepperA.moveToPos(0)                                    # cancel the rest of the move

    time.sleep(1)
     

    # Do a general move
    stepperA.setAutoRamp(speed=700000, accel=8000) 
    
    stepperA.moveToPos(200 * stepperA.microStep * 100)
    while stepperA.getStatus()['positionReached'] == False:
        time.sleep_ms(1)
        if stepperA.getStatus()['velocityReached']:  led.high()          # turn on the LED once the motor gets to full speed
        else: led.low()
    time.sleep(1)
    
    print(stepperA.getStatus())
    stepperA.moveToPos(0)
    while stepperA.getStatus()['positionReached'] == False:
        time.sleep_ms(1)
        if stepperA.getStatus()['velocityReached']:  led.high()          # turn on the LED once the motor gets to full speed
        else: led.low()

    print(stepperA.getStatus())


    #time.sleep(5)
    print(stepperA.readReg(stepperA.CHOPCONF))
    
    stepperA.disable()











