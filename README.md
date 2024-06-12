# Turkish_Flag_Turtlesim
<p align="center">
  <img src=https://github.com/MAli7319/Turkish_Flag_Turtlesim/blob/main/gif_V4.gif  width="400" height="400"/>
</p>


You can find the video: https://youtu.be/xglPSse_C2U
```
ros2 service call /spawn turtlesim/srv/Spawn "{'x': 3.0, 'y': 8.0, 'theta': 125, 'name': "turtle2"}"
```
```
ros2 service call /turtle1/set_pen turtlesim/srv/SetPen "{'r': 0, 'g': 0, 'b': 0, 'width': 5, 'off': 0}"
```
```
ros2 service call /kill turtlesim/srv/Kill "{'name': "turtle1"}"
```
```
ros2 param set /turtlesim  background_r 250
ros2 param set /turtlesim  background_g 0
ros2 param set /turtlesim  background_b 0
```
