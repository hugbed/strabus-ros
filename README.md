# strabus-ros

Nodes to :
image_publish : publish an image on camera/image
image_viewer_fs : view the image fullscreen.

Note : image_viewer_fs will subscribe to an angle to rotate the image.

Note : 
To display the screen on the LCD, replace `/dev/fb0` to `/dev/fb1` in :

```
/usr/share/X11/xorg.conf.d/99-fbturbo.conf
```

Then reboot.
