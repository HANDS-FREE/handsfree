## 机器人跟踪二维码使用教程

### 准备工作
1. 把config/Markers_0_2.png 打印到A4纸上
2. 确保使用的是HandsFree带摄像头的机器人底盘
3. 确保使用的摄像头型号是华硕Xtion Pro

###　方法一: 一键启动
roslaunch handsfree_ar_tags robot_follow_ar_marker.launch    

###　方法二: 一步步启动
#### 第一步:运行底盘驱动和摄像头驱动   　　
运行底盘驱动节点:     
roslaunch handsfree_hw handsfree_hw.launch      

运行摄像头驱动节点:      
roslaunch handsfree_camera xtion.launch　　　　　

#### 第二步:运行二维码识别节点和二维码空间3D坐标可视化 　　
运行二维码识别节点:      
roslaunch handsfree_ar_tags ar_indiv_rgb_camera.launch     

启动rviz，可视化显示识别到的二维码3D位置和姿态:     
rosrun rviz rviz -d `rospack find handsfree_ar_tags`/rviz/ar_tags.rviz     

Topic查看识别到的二维码坐标:    
rostopic echo /ar_pose_marker     

#### 第三步:机器人跟踪二维码
运行机器人跟踪二维码节点:       
roslaunch handsfree_ar_tags ar_follower.launch      

最后，把A4纸的二维码放在摄像头前方,然后机器人就可以跟踪二维码了，同时可以通过可视化界面查看是否识别到二维码
