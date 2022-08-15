
### 控制无人机 启动RosLaunch 或 关闭Rosnode

**！！！待核实！！！**

0. 自启动脚本之一，用于开启其他脚本
   
1. mav launch  合集

2. 订阅 `/mavcomm/receive/console` 

  消息类型 `mavcomm_msgs::console` [msgs完整定义](../mavcomm_msgs/Readme_msgs.md#consolemsg)


```
# mavcomm_msgs::console

uint8 command
uint8 flag
uint8 type1
uint8 type2
```

---
### 标志位 含义 * flag *

- `flag = 1`
  启动 launch

- `flag = 3`
  关闭 node 

---
### 指令编号 定义 * command *

1. mavros
-	roslaunch mavros px4
```
- flag == 3		rosnode kill /mavros
- flag == 1	 
	- type1 = 
	1 ACM0	2 USB0	3 THS0	4 THS1	5 THS2 
	- type2 = 
    1 57600	2 921600
```

2. uav_ctrl
-	roslaunch px4_control ctrl
```
- flag == 3		rosnode kill /uav_ctrl
- flag == 1	 
	- type1 = type2 
```

3. vision_pose 
-	roslaunch px4_control vision_pose
```
- flag == 3		rosnode kill /vision_pose
- flag == 1	 
	- type1 = flag_1vrpn_2vio_3both
	1 2 3
	- type2 = vio_odomTopic
    1 /camera/odom/sample	2 /vins_fusion/odometry
```

4. csi_cam_0
-	roslaunch jetson_csi_cam
```
- flag == 3		rosnode kill /csi_cam_0
- flag == 1	 
	- type1 = type2 - width height
	1 1080	2 1920	3 3264
	- type2 = vio_odomTopic
    1 720	2 1080	3 2464
```

5. usb_cam
-	roslaunch usb_cam usb_cam0_480P.launch
```
- flag == 3		rosnode kill /usb_cam
- flag == 1	 
	- type1 = type2 - image_width image_height
	1 640	2 960	3 1440	4 2560
	- type2 = vio_odomTopic
    1 480	2 720	3 1080	4 1920
```

6. realsense_cam
-	roslaunch realsense2_camera rs_camera.launch
```
- flag == 3		rosnode kill /camera/realsense2_camera
- flag == 1	 
	- type1 = type2
```

