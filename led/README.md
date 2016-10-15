# Nodes to control and test leds. 

Nodes :
  * led_controller_node : turn on, off and blink up to 2 leds.

#### LED Controller

Start node:
```
rosrun led_controller led_controller_node
```

This node requires publishers on:
 * /led_controller_command : issue a command to an LED (on, off, blink)
 * /led_controller_config : configure a blinking parameter


The launch file will simply start the node:
```
roslaunch led_controller led_controller.launch
```


### Messages format
## led_controller_command
# id
Used to indicate to which LED the command is targetted at:
 * 0: LED 0
 * 1: LED 1
 * 2: Both LEDs

Using a different ID will result in the command being ignored.
 
# command
The actual command:
 * on: turn on the LED
 * off: turn off the LED
 * blink0: set the LED to blink at BLINK0 rate
 * blink1: set the LED to blink at BLINK1 rate

Using a command name will result in the command being ignored.

## led_controller_config
Not supported for now.