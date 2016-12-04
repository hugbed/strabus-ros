#!/usr/bin/python

import smbus

class PCA9530:
    # ==============================================================================================
    # Class constants
    # ==============================================================================================
    # Identifier of the LED0.
    LED0 = 0
    # Identifier of the LED1.
    LED1 = 1
    # Identifier of both LEDs. Useful to issue a command to both LEDs at the same time.
    LED_BOTH = 2
    
    # ID of the BLINK0
    BLINK0 = 0
    # ID of the BLINK1
    BLINK1 = 1
    
    # ==============================================================================================
    # Controller registers
    # The addresses of the different usable registers of the PCA9530 controller.
    # The non-register values represent the configurable fields of the data frames of each 
    # register. In other words, they are masks.
    # ==============================================================================================
    # Device address
    DEVICE_ADDRESS = 0x60
    # Auto-Increment flag
    AUTO_INCREMENT_FLAG = 0x10
    # The INPUT register reflects the state of the device pins.
    # READ-ONLY, writing will be acknowledged but will have no effect.
    REG_INPUT = 0x00
    INPUT_LED0 = 0x01
    INPUT_LED1 = 0x02
    # The PSC0 is used to program the period of the PWM output.
    # The period of BLINK0 = (PSC0 + 1) / 152
    REG_PSC0  = 0x01
    # The PWM0 register determines the duty cycle of BLINK0. The outputs are LOW (LED on)
    # when the count is less than the value in PWM0 and HIGH (LED off) when it is greater. If
    # PWM0 is programmed with 00h, then the PWM0 output is always HIGH (LED off).
    # The duty cycle of BLINK0 = PWM0 / 256.
    REG_PWM0  = 0x02
    # PSC1 is used to program the period of the PWM output.
    # The period of BLINK1 = (PSC1 + 1) / 152
    REG_PSC1  = 0x03
    # The PWM1 register determines the duty cycle of BLINK1. The outputs are LOW (LED on)
    # when the count is less than the value in PWM1 and HIGH (LED off) when it is greater.
    # If PWM1 is programmed with 00h, then the PWM1 output is always HIGH (LED off).
    # The duty cycle of BLINK1 = PWM1 / 256
    REG_PWM1  = 0x04
    # The LS0 LED select register determines the mode in which individual LEDs operate.
    REG_LS0 = 0x05
    LS0_LED0 = 0x03
    LS0_LED1 = 0x0C
    
    # ==============================================================================================
    # LED selector values
    # The modes in which an individual LED can be set.
    # ==============================================================================================
    # Selected LED is turned ON.
    LED_OFF    = 0x00
    # Selected LED is turned OFF.
    LED_ON     = 0x01
    # Selected LED is set to blink at PMW0 rate.
    LED_BLINK0 = 0x02
    # Selected LED is set to blink at PMW1 rate.
    LED_BLINK1 = 0x03

    # ==============================================================================================
    # Open and close functions.
    # ==============================================================================================
    # Open
    # Open a new I2C connection to the specified device.
    def open(self, device):
        # Configure I2C
        self._I2C = smbus.SMBus(device)
        #Init led states
        self._LED1 = 0
        self._LED0 = 0

    # Close
    # Close the currently opened SPI connection.
    def close(self):
        # Does currently nothing but kept here for an eventuality where it would do something.
        return

    # ==============================================================================================
    # Utility functions
    # Meant to be used as private functions.
    # ==============================================================================================
    # Device read
    # Read the value of the given register.
    def deviceRead(self, register):
        return _I2C.write_byte_data(self.DEVICE_ADDRESS, register)

    # Write a byte of data to the given register.
    def deviceWrite(self, register, data):
        self._I2C.write_byte_data(self.DEVICE_ADDRESS, register, data)

    # Write a block (more than 1 byte) of data to the given register.
    # Note that the Auto-Increment flag is used for this operation in order to configure each
    # register sequentially.
    # The specified register will then be the starting point of the sequence.
    def deviceWriteBlock(self, register, data):
        self._I2C.write_i2c_block_data(self.DEVICE_ADDRESS & self.AUTO_INCREMENT_FLAG, register, data)

    # Set the specified LED (or LEDs) to the given state and write the current states of both 
    # LEDs to the controller register.
    # Does nothing if the ID does not exist.
    def setLEDState(self, ledID, ledState):
        if (ledID == self.LED0):
            self._LED0 = ledState
        elif (ledID == self.LED1):
            self._LED1 = ledState
        elif (ledID == self.LED_BOTH):
            self._LED0 = ledState
            self._LED1 = ledState
        else:
	    print "PCA9530: Invalid LED identifier %d" % (ledID)
            return

        led0Value = self.LS0_LED0 & self._LED0
        led1Value = self.LS0_LED1 & (self._LED1 << 2)
        self.deviceWrite(self.REG_LS0, led0Value | led1Value)

    # ==============================================================================================
    # LED commands
    # ==============================================================================================
    # LED Blink 0
    # Set the specified LED (or LEDs) to blink to at PMW0 rate.
    def ledBlink0(self, ledID):
        self.setLEDState(ledID, self.LED_BLINK0)

    # LED Blink 1
    # Set the specified LED (or LEDs) to blink to at PMW1 rate.
    def ledBlink1(self, ledID):
        self.setLEDState(ledID, self.LED_BLINK1)

    # LED Off
    # Turn the specified LED (or LEDs) off.
    def ledOff(self, ledID):
        self.setLEDState(ledID, self.LED_OFF)

    # LED On
    # Turn the specified LED (or LEDs) on.
    def ledOn(self, ledID):
        self.setLEDState(ledID, self.LED_ON)

    # ==============================================================================================
    # Blink configuration commands
    # ==============================================================================================
    # Set Blink
    # Set a new value for both the period and duty cycle for the given blink ID 
    # (BLINK0 or BLINK1).
    # Using this function is more efficient than setting the period and duty cycle separately.
    # See setBlinkPeriod and setBlinkDutyCycle for the format of both values.
    def setBlink(self, blinkID, period, dutyCycle):
        if blinkID == self.BLINK0:
            register = self.REG_PCS0
        elif blinkID == self.BLINK1:
            register = self.REG_PCS1
        else:
            print "PCA9530: Invalid BLINK identifier %s" % (blinkID)
            return
        
        # Limit period to an 8 bits value.
        period = int(min(period, 0xFF))

        # Limit duty cycle to an 8 bits value.
        dutyCycle = int(min(dutyCycle, 0xFF))

        # Write block will use the register auto-increment to setup both registers.
        data = [period, duty]
        self.deviceWriteBlock(register, data)

    # Set Blink Duty Cycle
    # Set a new value for the PMW register (8 bits value) for the given blink ID.
    # The PWM register determines the duty cycle. The outputs are LOW (LED on)
    # when the count is less than the value in PWM and HIGH (LED off) when it is greater. If
    # PWM is programmed with 0x00, then the PWM output is always HIGH (LED off).
    # The duty cycle of BLINK = PWM / 255
    def setBlinkDutyCycle(self, blinkID, dutyCycle):
        if blinkID == self.BLINK0:
            register = self.REG_PMW0
        elif blinkID == self.BLINK1:
            register = self.REG_PMW1
        else:
            print "PCA9530: Invalid BLINK identifier %s" % (blinkID)
            return
        
        # Limit duty cycle to an 8 bits value.
        dutyCycle = int(min(dutyCycle, 0xFF))

        self.deviceWrite(register, dutyCycle)

    # Set Blink Period
    # Set a new value for the PSC register (8 bits value) for the given blink ID.
    # PSC is used to program the period of the PWM output.
    # The period of BLINK (in seconds) is: (PSC + 1) / 152
    def setBlinkPeriod(self, blinkID, period):
        if blinkID == self.BLINK0:
            register = self.REG_PCS0
        elif blinkID == self.BLINK1:
            register = self.REG_PCS1
        else:
            print "PCA9530: Invalid BLINK identifier %s" % (blinkID)
            return
        
        # Limit to an 8 bits value.
        period = int(min(period, 0xFF))

        self.deviceWrite(register, period)
