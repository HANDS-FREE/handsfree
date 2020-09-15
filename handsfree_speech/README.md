# HandsFree机器人ROS Kinetic配置pocketsphinx语音

## 安装支持库
sudo apt-get install ros-kinetic-audio-common libasound2 gstreamer0.10-* python-gst0.10

sudo apt-get install pocketsphinx* pocketsphinx-en-us

cd handsfree_speech/install/

dpkg -i libsphinxbase1_0.8-6_amd64.deb

dpkg -i libpocketsphinx1_0.8-5_amd64.deb

dpkg -i gstreamer0.10-pocketsphinx_0.8-5_amd64.deb

## 编译

cd {工作空间}

catkin_make

## 运行

运行机器人驱动节点:

roslaunch handsfree_hw handsfree_hw.launch 

运行语音控制节点:

roslaunch pocketsphinx voice_navigation.launch

### 相关参考:
* www.cnblogs.com/TooyLee/p/7739783.html
* blog.csdn.net/ppp2006/article/details/22151825
* blog.csdn.net/qiaocuiyu/article/details/52093509

hmm表示隐马尔可夫声学模型，lm表示language model语言模型

## TIAGo Robot with ROS Kinetic and Pocketsphinx

To avoid false voice commands and noise interference, it is proposed to use a keyword "tiago". First, TIAGo robot waits for the keyword. Then it waits for the voice command. If the voice command is correct, the mobile base is running through that command. The following voice commands have been used:

* backward  : {"backward", "go backward", "move backward", "back", "move back"}
* forward   : {"forward", "go forward", "move forward"}
* turn left : {"turn left", "left"}
* turn right: {"turn right", "right"}
* stop      : {"stop", "halt"}
* faster    : {"faster"}
* slower    : {"slower"}
* quater    : {"quarter speed", "quarter"}
* half      : {"half speed", "half"}
* full      : {"full speed", "full"}

Thanks to Michael Ferguson (https://github.com/mikeferguson/pocketsphinx), we can implement voice comnands for robot with ROS using the pocketsphinx package. After that, in (https://github.com/sunmaxwll/pocketsphinx) the author proposes voice commands for ROS Kinetic. Finally, more details can be found in Goebel (2015).

## References
Goebel, R. P. (2015). ROS by example. Lulu. com.
