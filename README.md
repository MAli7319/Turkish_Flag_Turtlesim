# Turkish_Flag_Turtlesim
<p align="center">
  <img src=https://github.com/MAli7319/Turkish_Flag_Turtlesim/blob/main/gif_V4.gif  width="400" height="400"/>
</p>


### You can find the video: https://youtu.be/xglPSse_C2U


## How to Run:
* Clone this repository into your workspace
  * ```
    git clone https://github.com/MAli7319/Turkish_Flag_Turtlesim.git
    ```
* Compile the files
  * ```
    colcon build --symlink-install
    ```
* Launch the turtlesim
  * ```
    ros2 run turtlesim turtlesim_node
    ```
* Set the background params to obtain the red color
  * ```
    ros2 param set /turtlesim  background_r 250
    ros2 param set /turtlesim  background_g 0
    ros2 param set /turtlesim  background_b 0
    ```
* Launch the main python script
  * ```
    ros2 run my_robot_controller turkish_flag_drawer
    ```
