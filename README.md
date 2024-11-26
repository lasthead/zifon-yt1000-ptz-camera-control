# zifon-yt1000-ptz-camera-control
Zifon YT-1000 Pan-Tilt-Zoom Head based on the Arduino controller (ESP-32)

Demonstration video:
https://youtu.be/_K3INTDjFJ0

Hi everyone! This is small modification for pan-tilt head Zifon Yt-1000 (you can find it on aliexpress).

By default, this device has limitations. 
For example, there is no possibility to move diagonally, there is no zoom control of the connected camera

So, I added a ESP32-controller to the device, which is controlled using a joystick from the ps4.
Also, to control the camera zoom, an electronic key was made from a 2N3904 transistor that controls the camcorder via the LANC interface

sources:

https://featherbear.cc/blog/post/zifon-yt-1000-wifi-acu

http://controlyourcamera.blogspot.com/2011/02/arduino-powered-lanc-remote.html

https://github.com/AlexNe/arduino_lanc_sample/blob/master/LANC-Interface1.png
