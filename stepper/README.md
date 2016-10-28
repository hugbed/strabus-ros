# Nodes to control and test the inter-pupillary rail stepper 

Nodes :
  * stepper_controller : Control the stepper motor by issuing commands to this node. The commands are listed below.
  
## Stepper Controller node

Start node:
```
rosrun stepper_controller stepper_controller_node
```

This node listens to publishers on:
 * /stepper_controller_run : rotate the stepper at a given speed in a given direction, indefinitely.
 * /stepper_controller_move : rotate the stepper by the given amount of steps in the given direction.
 * /stepper_controller_goTo : rotate the stepper targetting a given absolute step position, using the shortest path.
 * /stepper_controller_stop : stop the motion of the stepper.

The launch file will simply start the node:
```
roslaunch stepper_controller stepper_controller.launch
```

### Messages format
#### stepper_controller_run
##### speed (float32)
Used to indicate at which speed the stepper will run.
This value should be between the configured MIN_SPEED (default 0 step/s) and MAX_SPEED (default 991.8 step/s). Otherwise, the stepper's speed is clamped to one of these values.

##### direction (string)
Used to indicate the direction of the rotation:
 * clockwise: the stepper will run in a clockwise direction
 * counter-clockwise: the stepper will run in a counter-clockwise direction
 
Using a different direction name will result in the command being ignored.

#### stepper_controller_move
##### steps (int32)
Used to indicate the number of microsteps by which the stepper will move.
The steps value is in agreement with the selected step mode; the parameter
value unit is equal to the selected step mode (full, half, quarter, etc.).

##### direction (string)
Used to indicate the direction of the rotation:
 * clockwise: the stepper will run in a clockwise direction
 * counter-clockwise: the stepper will run in a counter-clockwise direction
 
Using a different direction name will result in the command being ignored.

#### stepper_controller_goTo
##### position (int32)
Used to indicate at which position the stepper will go to.
Note that the position value is always in agreement with the selected step mode; the
parameter value unit is equal to the selected step mode (full, half, quarter, etc.).

#### stepper_controller_stop
##### type (string)
Used to indicate the type of stop to apply to the stepper:
 * hard: the stepper will stop immediately
 * soft: the stepper will stop after a deceleration as configured by the DEC register
 
Using a different stop type name will result in the command being ignored.

## SPI Notes : 
The SPI device consisting of the stepper can be either `/dev/spidev0.0` or `/dev/spidev0.1`. This has to be modified accordingly in:

```
stepper_controller/src/stepper_controller_node.py
```



