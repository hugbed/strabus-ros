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

#### Move command
Move the rail in the given direction until a stop command is issued or the rail arrives to a limit.

```
{
    "command" : "move",
    "parameters" : {
        "direction" : "OPEN"
    }
}
```

##### direction (string)
Used to indicate the direction of the movement:
 * OPEN: the rail will open.
 * CLOSE: the rail will close.
 
A different direction name will result in the command being ignored.

#### Move By command
Move the rail by the given amount of distance, in micrometers.

```
{
    "command" : "moveBy",
    "parameters" : {
        "distance" : 4500.0,
        "direction" : "CLOSE"
    }
}
```

##### distance (float)
Used to indicate the amount of distance by which the rail will move, in micrometers.
This value is associated with the inter-pupillary distance, meaning that each optical arm
will move by half of this distance.
 
A distance of 0 will result in the command being ignored.

##### direction (string)
Used to indicate the direction of the movement:
 * OPEN: the rail will open.
 * CLOSE: the rail will close.
 
A different direction name will result in the command being ignored.

#### Move To command
```
{
    "command" : "moveTo",
    "parameters" : {
        "position" : 35000.0
    }
}
```

##### position (float)
Used to indicate the inter-pupillary distance to which the rail will move, in micrometers.

If this position is outside of the inter-pupillary limits (42.8mm to 104mm), the rail will 
simply stop at the corresponding limit.

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
