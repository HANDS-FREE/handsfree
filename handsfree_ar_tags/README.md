## 二维码识别包使用教程

### 准备二维码
把config/Markers_0_2.png 打压到A4纸上

### 识别二维码     
运行摄像头驱动节点:      
roslaunch handsfree_camera xtion.launch

运行二维码识别节点:      
roslaunch handsfree_ar_tags ar_indiv_rgb_camera.launch     

启动rviz可视化识别到的二维码:     
rosrun rviz rviz -d `rospack find handsfree_ar_tags`/ar_tags.rviz     

Topic查看识别到的二维码:    
rostopic echo /ar_pose_marker     

### 机器人跟踪二维码
运行底盘驱动节点:     
roslaunch handsfree_hw handsfree_hw.launch      

运行机器人跟踪节点:       
roslaunch handsfree_ar_tags ar_follower.launch      

把A4纸的二维码放在摄像头前方,然后机器人就可以跟踪二维码了
