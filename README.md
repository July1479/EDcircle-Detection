# EDcircle-Detection

系统版本： Ubuntu 20.04.1 

ROS版本： ROS noetic 

依赖的库： cv_bridge    image_transport  message_filters nav_msgs  roscpp   rospy  std_msgs   geometry_msgs   edcircles

```c++
git clone https:July1479/EDcircle-Detection.git
cd AdaptWing_justread
catkin_make -j2 -DCMAKE_EXPORT_COMPILE_COMMANDS=Yes  
source devel/setup.bash

```

开启相机可视化界面

```c++
realsense-review
```

Running on a real quadrotor

```c++
roslaunch realsense2_camera rs_camera.launch    ##开启相机节点
## 开启一个新的终端
roslaunch circle_detection_node circle_detection_node.launch   ##开启circle_detection_node 节
```

查看传回的圆心坐标信息

```
##开启一个新终端
rostopic echo /circle_detection_node/center
```

