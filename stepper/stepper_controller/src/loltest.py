#! /usr/bin/env python

# Sleep function
from time import sleep

# Stepper controller
from L6470_lib import L6470

if __name__ == '__main__':
    # Open SPI controller.
    _controller = L6470()
    _controller.open(0, 0)

    # Issue a run command
    _controller.run(L6470.DIR_CLOCKWISE, 100.0)
    
    # Wait a bit
    sleep(2)
    
    # Softly stop the stepper
    _controller.softStop()
    
    # Close the SPI connection
    _controller.close()