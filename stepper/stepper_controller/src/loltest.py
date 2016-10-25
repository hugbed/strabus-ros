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
    device.max_speed_hz = 1000000
    device.mode = 0b11
    
    # Read register MAX_SPEED with 3 nops for response.
    request = [0x27, 0x00, 0x00, 0x00]
    responseBytes = device.xfer2(request)
    
    # Return only useful 3 bytes.
    response = responseBytes[3]
    response = Response | responseBytes[2] << 8
    response = Response | responseBytes[1] << 16
    
    print "Max speed: %d" % (response)
    
    device.close()
