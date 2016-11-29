# Package to launch camera nodes

Nodes launched by `camera.launch`:
  * raspicam, publishing *on* `/camera/image/compressed`
  * web_video_server, publishing *from* `/camera/image/compressed`

#### Image Viewer Fullscreen

Start camera nodes:
```
roslaunch camera camera.launch
```
