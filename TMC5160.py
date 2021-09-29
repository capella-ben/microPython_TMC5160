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

    GCONF = 0x00
    GSTAT = 0X01
    IFCNT = 0X02
    SLAVECONF = 0X03
    IOIN = 0X04             #THE SAME ADDRESS AS THE REGITER BELOW *** COULD BE WRONG ***
    OUTPUT = 0X04
    X_COMPARE = 0X05
    OTP_PROG = 0X06
    OTP_READ = 0X07
    FACTORY_CONF = 0X08
    SHORT_CONF = 0X09
    DRV_CONF = 0X0A
    GLOBALSCALER = 0X0B
    OFFSET_READ = 0X0C
    IHOLD_IRUN = 0X10
    TPOWERDOWN = 0X11
    TSTEP = 0X12
    TPWMTHRS = 0X13
    TCOOLTHRS = 0X14
    THIGH = 0X15
    RAMPMODE = 0X20
    XACTUAL = 0X21
    VACTUAL = 0X22
    VSTART = 0X23

    TZEROWAIT = 0X2C
    XTARGET = 0X2D
    VDCMIN = 0X33
    SW_MODE = 0X34
    RAMP_STAT = 0X35
    XLATCH = 0X36

    # USED IN THE SMAPLE CODE
    GCONF = 0X00
    IHOLDIRUN = 0X10
    TPOWERDOWN = 0X11
    TPWMTHRS = 0X13
    RAMPMODE = 0X20
    A1 = 0X24
    V1 = 0X25
    AMAX = 0X26
    VMAX = 0X27
    DMAX = 0X28
    D1 = 0X2A
    VSTOP = 0X2B
    XTARGET = 0X2D
    CHOPCONF = 0X6C





    maxCurrent = 2.8  # Max contiuous current capabilty of the board in Amps







    def __init__(self, spiObj: SPI, csPin: int, debug=0):

        self.cs = Pin(csPin, Pin.OUT)
        self.cs.high()

        self.spi = spiObj
        self.debug = debug

        self.writeReg(self.GCONF, 0x04)   # enable PWM mode
        self.writeReg(self.CHOPCONF, 0x0100C3)      # using the example
        self.writeReg(self.TPWMTHRS, 0x1F4)         # using the example value of 500


    def twos_complement(self, val, nbits):
        """Compute the 2's complement of int value val"""
        if val < 0:
            val = (1 << nbits) + val
        else:
            if (val & (1 << (nbits - 1))) != 0:
                # If sign bit is set.
                # compute negative value.
                val = val - (1 << nbits)
        return val

    def byteToIntSigned32(self, val):
        if val & 2**31 > 0:
            return val - 2**32
        else:
            return val



    def writeReg(self, address: int, value: int, signed=False):
        rb = bytearray(5)       # buffer for the data we get back

        if signed:
            value = self.twos_complement(value, 32)

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
            return self.byteToIntSigned32(int.from_bytes(rb, 'big')  & 0xFFFFFFFF), status
        else:
            return int.from_bytes(rb, 'big')  & 0xFFFFFFFF, status

    





    def setCurrent(self, current: float, idlePercent: float):
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
        
    def setRamp(self, vstart, a1, v1, amax, vmax, dmax, d1, vstop):
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
        self.setRamp(vstart=0, a1=accel, v1=int(speed/2), amax=int(accel/2), vmax=speed, \
                    dmax=int((accel*1.5)/2), d1=int(accel*1.5), vstop=10)  

    def moveToPos(self, pos):
        # pos must be in the range  -2_147_483_648 -> 2_147_483_648
        return self.writeReg(self.XTARGET, pos, True) 


    def getPos(self):
        return self.readReg(self.XACTUAL, True)[0]

    def getStatus(self):
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
        self.writeReg(self.CHOPCONF, 0x0100C0)
    
    def enable(self):
        self.writeReg(self.CHOPCONF, 0x0100C3)





"""
ToDo:
- function to reset home

"""


if __name__ == "__main__":
    print("Starting...")

    led = Pin(25, Pin.OUT)
    led.low()

    spi = SPI(0)
    spi.init(baudrate=4000000, firstbit=SPI.MSB, bits=8)

    stepper = TMC5160(spi, 5, 0)

    stepper.setCurrent(2, 10)
    stepper.setAutoRamp(speed=1000000, accel=50000)

    print(stepper.getStatus())

    stepper.moveToPos(-51200 * 100)
    while stepper.getStatus()['positionReached'] == False:
        time.sleep_ms(5)
        if stepper.getStatus()['velocityReached']:  led.high()
        else: led.low()

    print(stepper.getStatus())
    stepper.moveToPos(0)
    while stepper.getStatus()['positionReached'] == False:
        time.sleep_ms(5)
        if stepper.getStatus()['velocityReached']:  led.high()
        else: led.low()

    print(stepper.getStatus())

    stepper.disable()











