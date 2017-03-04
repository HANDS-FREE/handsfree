# HANDS FREE ROS DEMO 

### Environment ###
we recommend that you test code on the pc before transplant to TK1 or TX1   
* 1. Make sure you install ROS and carefully read the [Beginner Level Tutorials]( http://wiki.ros.org/ROS/Tutorials )   
* 2. run Documentation/environment_config.sh to install some dependent packages     
* 3. change handsfree_hw/src/main.cpp row 7 "/home/kedou/ros_workspace/HANDS_FREE_WS/src/handsfree_hw/config.txt" to your    own Path   
* 4. Compilation : catkin_make      
* 5.  run example 
   
        roslaunch handsfree_hw handsfree_hw.launch      
        roslaunch handsfree_hw  keyboard_teleop.launch      
        
 then you can remote control robot.


