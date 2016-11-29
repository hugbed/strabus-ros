# Nodes to control and test the inter-pupillary rail

Nodes :
  * rail_controller : Control the inter-pupillary rail by issuing commands to this node.

## Rail controller node

Start node:
```
rosrun rail_controller rail_controller_node.py
```

This node listens to publishers on:
 * /motor/inter_eye/command : issue commands to the rail controller.

The launch file will simply start the node:
```
roslaunch rail_controller rail_controller.launch
```

### Messages format
The messages are encoded in JSON. Each message represents a command to run on the rail controller.

#### run command
```
{
    "command" : "run",
    "parameters" : {
        "direction" : "clockwise",
        "speed" : 800.0
    }
}
```

##### direction (string)
Used to indicate the direction of the rotation:
 * clockwise: the stepper will run in a clockwise direction
 * counter-clockwise: the stepper will run in a counter-clockwise direction

Using a different direction name will result in the command being ignored.

##### speed (float)
Used to indicate at which speed the stepper will run.
This value should be between the configured MIN_SPEED (default 0 step/s) and MAX_SPEED (default 991.8 step/s). Otherwise, the stepper's speed is clamped to one of these values.

#### move command
```
{
    "command" : "move",
    "parameters" : {
        "direction" : "clockwise",
        "steps" : 450
    }
}
```

##### direction (string)
Used to indicate the direction of the rotation:
 * clockwise: the stepper will run in a clockwise direction
 * counter-clockwise: the stepper will run in a counter-clockwise direction

Using a different direction name will result in the command being ignored.

##### steps (int)
Used to indicate the number of microsteps by which the stepper will move.
The steps value is in agreement with the selected step mode; the parameter
value unit is equal to the selected step mode (full, half, quarter, etc.).

#### goTo command
```
{
    "command" : "goTo",
    "parameters" : {
        "position" : 680
    }
}
```

##### position (int)
Used to indicate at which position the stepper will go to.
Note that the position value is always in agreement with the selected step mode; the
parameter value unit is equal to the selected step mode (full, half, quarter, etc.).

#### stop command
Stop the movement of the rail. If it is not moving, the command does nothing.

```
{
    "command" : "stop",
    "parameters" : {
            "type" : "soft"
    }
}
```

##### type (string)
Used to indicate the type of stop to apply to the rail:
 * hard: the rail will stop immediately
 * soft: the rail will stop after a brief deceleration

Using a different stop type name will result in the command being ignored.

## SPI Notes :
The SPI stepper motor controller can be either `/dev/spidev0.0` or `/dev/spidev0.1`. 
This has to be modified accordingly in:

```
rail_controller/src/rail_controller_node.py
```
