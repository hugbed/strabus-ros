#! /usr/bin/env python

# Sleep function
from time import sleep

# SpiDev
import spidev

if __name__ == '__main__':
    # Open the given SPI device
    device = spidev.SpiDev()
    device.open(0, 0)
    
    # Configure SPI
    device.bits_per_word = 8
    device.max_speed_hz = 5000000
    device.mode = 0b11
    
    sendCmd3(, 400)
    bytes = [0x41, 0x00, 0x01, 0x90]
    device.xfer2(bytes)
    
    sleep(3)
    
    device.close()
