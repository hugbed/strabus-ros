# Nodes to control and test the rail stepper 

Nodes :
  * stepper-node : Server node listening to different actions that the stepper can accomplish.
  * stepper-test-client : Simple client node used to test the server node.

### SPI Notes : 
The SPI device consisting of the stepper can be either `/dev/spidev0.0` or `/dev/spidev1.1`. This has to be modified accordingly in:

```
stepper-node/src/StepperNode.py
```
