# Nodes to publish and display images 

Nodes :
  * image_viewer_fs : display an image fullscreen by file path, rotation angle and scale.

#### Image Viewer Fullscreen

Start node:
```
rosrun image_viewer_fs image_viewer_fs_node
```

This node requires publishers on
 * /imageAngle : the image will be rotated with this angle (in degrees)
 * /imageScale : the image will be scaled down (0.0 < scale <= 1.0)
 * /imageFilname : the path to the image to load from disk


To test the node with dummy publishers, use the launch file:
```
roslaunch image_viewer_fs image_viewer_fs.launch
```
This will start nodes to publish the angle, the scale and the image filename (/home/jon/doge.jpg).
It will then launch the image viewer node.



### LCD Notes : 
To display the screen on the LCD, replace `/dev/fb0` to `/dev/fb1` in :

```
/usr/share/X11/xorg.conf.d/99-fbturbo.conf
```

Then reboot.

(see https://nonhazardo.us/raspberry/pi/tft/spi/device/tree/2016/05/30/raspberry_pi_zero_spi_screen.html)

