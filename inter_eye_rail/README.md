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
```
{
    "command" : "move",
    "parameters" : {
        "speed" : 800.0
    }
}
```

##### speed (float)
Used to indicate at which speed the rail will move, in micrometers/s.
If this value should exceed the configured MIN_SPEED (default 0 step/s) and MAX_SPEED 
(default 991.8 step/s) of the stepper motor, the stepper's speed is clamped to one of these 
values.

Note that the sign of the speed value determines the direction of the movement:
 * speed > 0: the rail will open; the optical arms will move away from each other.
 * speed < 0: the rail will close; the optical arms will move towards each other.
 
A speed of 0 will result in the command being ignored.

#### Move By command
```
{
    "command" : "moveBy",
    "parameters" : {
        "distance" : 4500.0
    }
}
```

##### distance (float)
Used to indicate the amount of distance by which the rail will move, in micrometers.
This value is associated with the inter-pupillary distance, meaning that each optical arm
will move by half of this distance.

Note that the sign of the distance value determines the direction of the movement:
 * distance > 0: the rail will open; the optical arms will move away from each other.
 * distance < 0: the rail will close; the optical arms will move towards each other.
 
A distance of 0 will result in the command being ignored.

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
Used to indicate the amount of distance by which the rail will move, in micrometers.
This value is associated with the inter-pupillary distance, meaning that each optical arm
will move by half of this distance.

Note that the sign of the distance value determines the direction of the movement:
 * distance > 0: the rail will open; the optical arms will move away from each other.
 * distance < 0: the rail will close; the optical arms will move towards each other.
 
A distance of 0 will result in the command being ignored.
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
