# Nodes to control and test the rail stepper 

Nodes :
  * stepper_controller : Control the stepper motor by issuing commands to this node. The commands are listed below.
  * stepper-test-client : Simple node used to test the controller node.
  
## Stepper Controller node

Start node:
```
rosrun stepper_controller stepper_controller_node
```

This node listens to publishers on:
 * /stepper_controller_run : issue a command to an LED (on, off, blink)
 * /stepper_controller_goTo : configure a blinking parameter


The launch file will simply start the node:
```
roslaunch stepper_controller stepper_controller.launch
```

### Messages format
#### stepper_controller_run
##### speed
Used to indicate at which speed the stepper will run.
This value should be between the configured MIN_SPEED (default 0 step/s) and MAX_SPEED (default 991.8 step/s). Otherwise, the stepper's speed is clamped to one of these values.

##### direction
Used to indicate the direction of the rotation:
 * clockwise: the stepper will run in a clockwise direction
 * counter-clockwise: the stepper will run in a counter-clockwise direction
 
Using a different direction name will result in the command being ignored.

#### stepper_controller_goTo
##### position
Used to indicate at which position the stepper will go to.
Note that the position value is always in agreement with the selected step mode; the
parameter value unit is equal to the selected step mode (full, half, quarter, etc.).

## SPI Notes : 
The SPI device consisting of the stepper can be either `/dev/spidev0.0` or `/dev/spidev0.1`. This has to be modified accordingly in:

```
stepper_controller/src/stepper_controller.py
```



