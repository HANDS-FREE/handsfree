# HandsFree ROS 

## Environment 
we recommend that you test code on the x86 pc before transplant to TK1 or TX1/2   
* 1. Make sure you install ROS and carefully read the [Beginner Level Tutorials](http://wiki.ros.org/ROS/Tutorials) 
* 2. Build an ROS package to configure the system
* 3. Follow [HandsFree Tutorials](http://wiki.hfreetech.org/docs/FAQ/environment_config.html) to install some dependent packages     
* 4. Compilation : catkin_make      
* 5.  run example 

        roslaunch handsfree_hw handsfree_hw.launch      
        roslaunch handsfree_hw keyboard_teleop.launch      
        
 then you can remote control robot.

## Building an ROS Package
As long as all of the system dependencies of your package are installed, we can now build your new package.
Note: If you installed ROS using apt or some other package manager, you should already have all of your dependencies.
Before continuing remember to source your environment setup file if you have not already. On Ubuntu it would be something like this:

#source /opt/ros/%YOUR_ROS_DISTRO%/setup.bash
$ source /opt/ros/kinetic/setup.bash             # For Kinetic for instance

## Using catkin_make
catkin_make is a command line tool which adds some convenience to the standard catkin workflow. You can imagine that catkin_make combines the calls to cmake and make in the standard CMake workflow.
Usage:

## In a catkin workspace
        $ catkin_make [make_targets] [-DCMAKE_VARIABLES=...]
        For people who are unfamiliar with the standard CMake workflow, it breaks down as follows:
        Note: If you run the below commands it will not work, as this is just an example of how CMake generally works.


## In a CMake project
        $ mkdir build
        $ cd build
        $ cmake ..
        $ make
        $ make install  # (optionally)
        This process is run for each CMake project. In contrast catkin projects can be built together in workspaces. Building zero to many catkin packages in a workspace follows this work flow:


## In a catkin workspace
        $ catkin_make
        $ catkin_make install  # (optionally)
        The above commands will build any catkin projects found in the src folder. This follows the recommendations set by REP128. If your source code is in a different place, say my_src then you would call catkin_make like this:


        Note: If you run the below commands it will not work, as the directory my_src does not exist.


## In a catkin workspace
        $ catkin_make --source my_src
        $ catkin_make install --source my_src  # (optionally)
        For more advanced uses of catkin_make see the documentation: catkin/commands/catkin_make

## Building Your Package

        If you are using this page to build your own code, please also take a look at the later tutorials (C++)/(Python) since you may need to modify CMakeLists.txt.

        You should already have a catkin workspace and a new catkin package called beginner_tutorials from the previous tutorial, Creating a Package. Go into the catkin workspace if you are not already there and look in the src folder:


        $ cd ~/catkin_ws/
        $ ls src
        You should see that there is a folder called beginner_tutorials which you created with catkin_create_pkg in the previous tutorial. We can now build that package using catkin_make:


        $ catkin_make
        You should see a lot of output from cmake and then make

 ## Existing Features
 ## 2dnav 
 Allows for the robot to naviagte on a 2d plane.
 ## Camera 
 Allows for remote access of the robots camera
 ## Controller 
 Allows for the robot to be virtually controlled
 ## Vision 
 Allows for the robot to be configured to detect certain items
 

##  Community: 
* [HandsFree Github](https://github.com/HANDS-FREE)    
* [HandsFree Wiki](http://wiki.hfreetech.org/)    
* Email: hands_free@126.com     


