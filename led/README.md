# Nodes to control and test leds. 

Nodes :
  * led_controller_node : Control up to 2 LEDs by issuing commands and configuring blink registers. The commands and configuration options are listed below.

## LED Controller node

Start node:
```
rosrun led_controller led_controller_node.py
```

This node requires publishers on:
 * /led_controller_topic : issue actions to the LED controller.

The launch file will simply start the node:
```
roslaunch led_controller led_controller.launch
```

### Messages format
The messages are encoded in JSON. Each message represents an action to run on the LED controller.

#### command action
```
{
    "action" : "command",
    "parameters" : [
        {
            "id" : "LED0",
            "command" : "BLINK1"
        }
    ]
}
```

##### id
Used to indicate to which LED the command is targetted at:
 * LED0: LED 0
 * LED1: LED 1
 * BOTH: Both LEDs

Using a different ID will result in the action being ignored.
 
##### command
The actual command:
 * on: turn on the LED
 * off: turn off the LED
 * blink0: set the LED to blink at BLINK0 rate
 * blink1: set the LED to blink at BLINK1 rate

Using a different command name will result in the action being ignored.

#### config action
```
{
    "action" : "config",
    "parameters" : [
        {
            "id" : "BLINK0",
            "config" : "duty-cycle",
			"value" : 128
        }
    ]
}
```

##### id
Used to indicate which BLINK register will be configured:
 * BLINK0: blink register 0
 * BLINK1: blink register 1

Using a different ID will result in the action being ignored.
 
##### config
The parameter to configure on the controller:
 * duty-cycle: the duty cycle of the blink signal (8-bits value). This value represents, on a scale from 0 to 255, the amount of time during which the LED is on at the beginning of a period.
 * period: the period of the blink signal (8-bits value). The configured value is according to the formula: period (in seconds) = (value + 1) / 152

Using a different config name will result in the action being ignored.

##### value
The value to give to the configured controller parameter.