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
    
    # ==============================================================================================
    # Controller registers
    # The addresses of the different usable registers of the PCA9530 controller.
    # The non-register values represent the configurable fields of the data frames of each 
    # register. In other words, they are masks.
    # ==============================================================================================
    # Device address
    DEVICE_ADDRESS = 0xC0
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
    PSC0_VALUE = 0xFF
    # The PWM0 register determines the duty cycle of BLINK0. The outputs are LOW (LED on)
    # when the count is less than the value in PWM0 and HIGH (LED off) when it is greater. If
    # PWM0 is programmed with 00h, then the PWM0 output is always HIGH (LED off).
    # The duty cycle of BLINK0 = PWM0 / 256.
    REG_PWM0  = 0x02
    PMW0_VALUE = 0xFF
    # PSC1 is used to program the period of the PWM output.
    # The period of BLINK1 = (PSC1 + 1) / 152
    REG_PSC1  = 0x03
    PSC1_VALUE = PSC0_VALUE
    # The PWM1 register determines the duty cycle of BLINK1. The outputs are LOW (LED on)
    # when the count is less than the value in PWM1 and HIGH (LED off) when it is greater.
    # If PWM1 is programmed with 00h, then the PWM1 output is always HIGH (LED off).
    # The duty cycle of BLINK1 = PWM1 / 256
    REG_PWM1  = 0x04
    PMW1_VALUE = PMW0_VALUE
    # The LS0 LED select register determines the mode in which individual LEDs operate.
    REG_LS0 = 0x05
    LS0_LED0 = 0x02
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
    def open(device):
        # Configure I2C
        self._I2C = smbus.SMBus(device)

    # Close
    # Close the currently opened SPI connection.
    def close():
        # Does currently nothing but kept here for an eventuality where it would do something.

    # ==============================================================================================
    # Utility functions
    # Meant to be used as private functions.
    # ==============================================================================================
    # Device read
    # Read the value of the given register.
    def deviceRead(self, register):
        return _I2C.write_byte_data(DEVICE_ADDRESS, register)
        
    # Write a byte of data to the given register.
    def deviceWrite(self, register, data):
        _I2C.write_byte_data(DEVICE_ADDRESS, register, data)
        
    # Write a block (more than 1 byte) of data to the given register.
    # Note that the Auto-Increment flag is used for this operation in order to configure each
    # register sequentially.
    # The specified register will then be the starting point of the sequence.
    def deviceWriteBlock(self, register, data):
        _I2C.write_i2c_block_data(DEVICE_ADDRESS & AUTO_INCREMENT_FLAG, register, data)
        
    # Set the specified LED (or LEDs) to the given state and write the current states of both 
    # LEDs to the controller register.
    # Does nothing if the ID does not exist.
    def setLEDState(self, ledID, ledState):
        if (ledID == LED0):
            self._LED0 = ledState
        elif (ledID == LED1):
            self._LED1 = ledState
        elif (ledID = LED_BOTH):
            self._LED0 = ledState
            self._LED1 = ledState
        else:
            return
        
        led0Value = LS0_LED0 & self._LED0
        led1Value = LS0_LED1 & (self._LED1 << 2)
        deviceWrite(REG_LS0, led0Value | led1Value)

    # ==============================================================================================
    # LED commands
    # ==============================================================================================
    # LED Blink 0
    # Set the specified LED (or LEDs) to blink to at PMW0 rate.
    def ledBlink0(self, LED_ID):
        setLEDState(ledID, LED_BLINK0)
        
    # LED Blink 1
    # Set the specified LED (or LEDs) to blink to at PMW1 rate.
    def ledBlink1(self, LED_ID):
        setLEDState(ledID, LED_BLINK1)
    
    # LED Off
    # Turn the specified LED (or LEDs) off.
    def ledOff(self, ledID):
        setLEDState(ledID, LED_OFF)

    # LED On
    # Turn the specified LED (or LEDs) on.
    def ledOn(self, LED_ID):
        setLEDState(ledID, LED_ON)
        
    # ==============================================================================================
    # Blink configuration commands
    # ==============================================================================================
    # Set Blink 0
    # Set a new value for both the period and duty cycle of BLINK0 (PSC0 and PMW0 registers).
    # Using this function is more efficient than setting the period and duty cycle separately.
    # See setBlink0Period and setBlink0DutyCycle for the format of both values.
    def setBlink0(self, period, dutyCycle):
        # Limit period to an 8 bits value.
        if (period > 0xFF):
            period = 0xFF
        
        # Limit duty cycle to an 8 bits value.
        if (dutyCycle > 0xFF):
            dutyCycle = 0xFF
        
        data = [PSC0_VALUE & period, PMW0_VALUE & dutyCycle]
        deviceWriteBlock(REG_PCS0, data)

    
    # Set Blink 0 Duty Cycle
    # Set a new value for the PMW0 register (8 bits value).
    # The PWM0 register determines the duty cycle of BLINK0. The outputs are LOW (LED on)
    # when the count is less than the value in PWM0 and HIGH (LED off) when it is greater. If
    # PWM0 is programmed with 0x00, then the PWM0 output is always HIGH (LED off).
    # The duty cycle of BLINK0 = PWM0 / 256
    def setBlink0DutyCycle(self, dutyCycle):
        # Limit to an 8 bits value.
        if (dutyCycle > 0xFF):
            dutyCycle = 0xFF
            
        deviceWrite(REG_PMW0, PMW0_VALUE & dutyCycle)
        
    # Set Blink 0 Period
    # Set a new value for the PSC0 register (8 bits value).
    # PSC0 is used to program the period of the PWM0 output.
    # The period of BLINK0 (in seconds) is: (PSC0 + 1) / 152
    def setBlink0Period(self, period):
        # Limit to an 8 bits value.
        if (period > 0xFF):
            period = 0xFF
            
        deviceWrite(REG_PCS0, PSC0_VALUE & period)
        
    # Set Blink 1
    # Set a new value for both the period and duty cycle of BLINK1 (PSC1 and PMW1 registers).
    # Using this function is more efficient than setting the period and duty cycle separately.
    # See setBlink1Period and setBlink1DutyCycle for the format of both values.
    def setBlink1(self, period, dutyCycle):
        # Limit period to an 8 bits value.
        if (period > 0xFF):
            period = 0xFF
        
        # Limit duty cycle to an 8 bits value.
        if (dutyCycle > 0xFF):
            dutyCycle = 0xFF
        
        data = [PSC1_VALUE & period, PMW1_VALUE & dutyCycle]
        deviceWriteBlock(REG_PCS1, data)
        
    # Set Blink 1 Duty Cycle
    # Set a new value for the PMW1 register (8 bits value).
    # The PWM1 register determines the duty cycle of BLINK1. The outputs are LOW (LED on)
    # when the count is less than the value in PWM1 and HIGH (LED off) when it is greater. If
    # PWM1 is programmed with 0x00, then the PWM1 output is always HIGH (LED off).
    # The duty cycle of BLINK1 = PWM1 / 256
    def setBlink1DutyCycle(self, dutyCycle):
        # Limit to an 8 bits value.
        if (dutyCycle > 0xFF):
            dutyCycle = 0xFF
            
        deviceWrite(REG_PMW1, PMW1_VALUE & dutyCycle)
        
    # Set Blink 1 Period
    # Set a new value for the PSC1 register (8 bits value).
    # PSC1 is used to program the period of the PWM1 output.
    # The period of BLINK1 (in seconds) is: (PSC1 + 1) / 152
    def setBlink1Period(self, period):
        # Limit to an 8 bits value.
        if (period > 0xFF):
            period = 0xFF
            
        deviceWrite(REG_PCS1, PSC1_VALUE & period)
        

    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    