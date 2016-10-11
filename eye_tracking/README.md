# Nodes to track pupils

Nodes :
  * eye_tracker_node : display an image fullscreen by file path, rotation angle and scale.

#### Eye tracker

Start node:
```
rosrun eye_tracking eye_tracker_node  _image_transport:=compressed
```

_image_transport needs to be set to compressed because we're subscribing to the compressed image feed

This node requires publishers on
 * /camera/image/compressed" : the camera image feed of the eye to be tracked

This node publishes on 
*/tracking/eye_tracking_feed : Video feed of the eye image with the pupil center drawn on it
*/tracking/position :Position of the pupil center
*/tracking/offset : Offset of the pupil center from the center of the image
*/tracking/filtered_position : Kalman filtered position

