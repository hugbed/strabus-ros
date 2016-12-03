# Nodes to publish and display images 

Nodes :
  * image_viewer_fs : display an image fullscreen by file path, rotation angle and scale.

#### Image Viewer Fullscreen

Start node:
```
rosrun image_viewer_fs image_viewer_fs_node side:=left
```

The side is to publish the topics on the correct parent namespace (i.e : `/left/image/angle`)

This node requires publishers on (i.e : for side:=left)
 * /left/image/angle : the image will be rotated with this angle (in degrees)
 * /left/image/scale : the image will be scaled down (0.0 < scale <= 1.0)
 * /left/image/filename : the path to the image to load from disk
 * /left/image/show : to toggle the image display (image or black screen)


To test the node with dummy publishers, use the launch file:
```
roslaunch image_viewer_fs test_publish.launch
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

### Hide the mighty cursor
Since we have a fullscreen app running on the main display, we remove the cursor with unclutter.
Curse you, cursor!

```
$ unclutter
```
### Image viewer on boot ###
To start image viewer on boot on a raspberry pi you need to follow these steps:
1. Create a systemd service to start the ros node at launch using pi user. (See the service file in image_viewer_fs)
2. Create a script that export DISPLAY=$(Your display)
3. Profit
