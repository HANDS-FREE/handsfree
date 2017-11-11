#!/binsh

# Install the prerequisites for the ROS By Example code, Volume 1

sudo apt-get install ros-kinetic-turtlebot-bringup \
 ros-kinetic-openni-* \
ros-kinetic-openni2-* ros-kinetic-freenect-*  \
ros-kinetic-laser-*  \
ros-kinetic-audio-common  \
 ros-kinetic-slam-gmapping \
ros-kinetic-joystick-drivers python-rosinstall \
ros-kinetic-orocos-kdl ros-kinetic-python-orocos-kdl \
python-setuptools ros-kinetic-dynamixel-motor-* \
libopencv-dev python-opencv ros-kinetic-vision-opencv \
ros-kinetic-depthimage-to-laserscan  \
ros-kinetic-turtlebot-teleop ros-kinetic-move-base \
ros-kinetic-map-server ros-kinetic-fake-localization \
ros-kinetic-amcl git subversion mercurial \
ros-kinetic-hokuyo3d
