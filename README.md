# HANDS FREE ROS DEMO 

### Environment ###
we recommend that you test code on the pc before transplant to TK1 or TX1   
* 1. Make sure you install ROS and carefully read the [Beginner Level Tutorials]( http://wiki.ros.org/ROS/Tutorials )   
* 2. run Documentation/environment_config.sh to install some dependent packages     
* 3. Compilation : catkin_make      
* 4.  run example 

        roslaunch handsfree_hw handsfree_hw.launch      
        roslaunch handsfree_hw  keyboard_teleop.launch      
        
 then you can remote control robot.

### Installation from Scratch ###
There is now an install.sh script(in Documentation), which can be executed (bash install.sh). It installs everything required.

The script is short and not complicated, so you can also use it as a manual.

If you want to use the install script, it is sufficient to [download it directly](https://raw.githubusercontent.com/HANDS-FREE/handsfree/master/Documentation/install.sh). There is no need to clone this repository then, as the script will do that for you.
