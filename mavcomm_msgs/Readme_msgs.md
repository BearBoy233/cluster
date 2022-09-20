
### 无人机集群通用 ROS Msgs

ROS msgs定义、 Ros Topic & Mavlink 关联说明.

> 消息修改后 **/mavcomm_msgs/msg**、 **mavlink/macomm.xml** 以及 **关联到的cpp** 都需要修改.  

```
通用格式
std_msgs/Header header
uint8 sysid     # 发送端 ID 
uint8 compid    # 接收端 ID 
#ID [100-地面站]   [99-所有无人机]

*** ***
*** ***
```

---
## Mavlink.msg 

```
# mavlink_framing_t enum
uint8 FRAMING_OK = 1
uint8 FRAMING_BAD_CRC = 2
uint8 FRAMING_BAD_SIGNATURE = 3

# stx values
uint8 MAVLINK_V10 = 254
uint8 MAVLINK_V20 = 253

std_msgs/Header header
uint8 framing_status

uint8 magic		# STX byte
uint8 len
uint8 incompat_flags
uint8 compat_flags
uint8 seq
uint8 sysid
uint8 compid
uint32 msgid		# 24-bit message id
uint16 checksum
uint64[] payload64
uint8[] signature	# optional signature
```

- 功能
1. 用于 [mavcomm 无人机集群通信](../mavcomm/Readme_mavcomm.md)

> Mavlink > **通用格式**

| No | 发布端 | Ros Topic | 接收端 | 备注 |
| ---- |---- |---- |---- | ---- |



---
## Heartbeat.msg 

心跳包、无人机基本状态确认

```
std_msgs/Header header
uint8 sysid     # 发送端 ID 
uint8 compid    # 接收端 ID 
#ID [100-地面站]   [99-所有无人机]

uint8 px4_mode
uint8 px4_state
uint8 px4_sys_state
uint8 ctrl_state
float32 battery_cell_voltage
float32 battery_percentage
uint8 gps_fix_type
uint8 gps_satellites_visible
```

- 功能
1. px4_mode:
   - /mavros/state/#mode (srt => int)

2. px4_sys_state:
   - /mavros/state/#system_state

3. px4_state: 8bit [7...0] 
	- [7 connected |6 armed |5 guided |4 manual_input]
    /mavros/state/#XXX [7-4] 
    - [3 battery_on ]
    /mavros/battery/#cell_voltage.size()
	- [2 XX |1 XX |0 XX ]
    Used Later

4. ctrl_state:
    - /mavcomm/uav_state_machine (std_msgs::UInt8)
    Details show in [uav_ctrl.cpp#enum FlightState](../px4_offb/src/uav_ctrl.cpp)

5. 单节电池电压
   
6. 电量百分比

7. GPS 可见星数

8. GPS 定位状态 

> Mavlink > **#0 HEARTBEAT**

| No | 发布端 | Ros Topic | 接收端 | 备注 |
| ---- |---- |---- |---- | ---- |
|1 |px4_bridge.cpp |/mavcomm/send/heartbeat |mavcomm.cpp |飞机端 收集mavros |
|2 |mavcomm.cpp  |/mavcomm/receive/heartbeat |state_uav_qnode.cpp |GCS 订阅&解析 |


---
## Console.msg

```
std_msgs/Header header
uint8 sysid     # 发送端 ID 
uint8 compid    # 接收端 ID 
#ID [100-地面站]   [99-所有无人机]

uint8 command
uint8 flag
uint8 type1
uint8 type2
```

- 功能
1. 启动 无人机 机载电脑的 其他 roslaunch 
2. 关闭 rosnode

> Mavlink > **#40 CONSOLE**

| No | 发布端 | Ros Topic | 接收端 | 备注 |
| ---- |---- |---- |---- | ---- |
|1 |console_qnode.cpp |/mavcomm/send/console |mavcomm.cpp |GCS 发布&控制 |
|2 |mavcomm.cpp  |/mavcomm/receive/console |mavconsole.cpp |飞机端 订阅&解析 |

启动脚本 & 标志位的含义 [mavconsole (ROS Package)](../mavconsole/Readme_console.md)


---
## Console_monitor.msg

```
std_msgs/Header header
uint8 sysid     # 发送端 ID 
uint8 compid    # 接收端 ID 
#ID [100-地面站]   [99-所有无人机]

uint8 param1
uint8 param2
uint8 param3
uint8 param4
```

- 功能
1. 监控机载电脑端 ROS Node 情况

> Mavlink > **#41 CONSOLE_monitor**

| No | 发布端 | Ros Topic | 接收端 | 备注 |
| ---- |---- |---- |---- | ---- |
|1 |mavconsole.cpp  |/mavcomm/send/console_monitor |mavcomm.cpp |飞机端 发布&控制 |
|2 |mavcomm.cpp  |/mavcomm/receive/console_monitor |console_qnode.cpp |GCS 订阅&解析 |

Param 标志位的含义 [mavconsole.cpp (ROS Package)](../mavconsole/src/mavconsole.cpp)


--- ---
## ChangeState.msg

无人机状态机切换

```
std_msgs/Header header
uint8 sysid     # 发送端 ID 
uint8 compid    # 接收端 ID 
#ID [100-地面站]   [99-所有无人机]

uint8 node_state
uint8 param
```

- 功能
1. 切换 无人机 px4_crtl 状态机

> Mavlink > **#2 CHANGESTATE**

| No | 发布端 | Ros Topic | 接收端 | 备注 |
| ---- |---- |---- |---- | ---- |
|1 | console_qnode.cpp |/mavcomm/send/changestate |mavcomm.cpp |GCS 发布&控制 |
|2 |mavcomm.cpp  |/mavcomm/receive/changestate |[uav_ctrl.cpp](../px4_offb/src/uav_ctrl.cpp) |飞机端 订阅&解析 |

> node_state 定义&Details show in [uav_ctrl.cpp#enum FlightState](../px4_offb/src/uav_ctrl.cpp)


--- ---
## local_pos_enu.msg

飞行控制ENU

```
std_msgs/Header header
uint8 sysid     # 发送端 ID 
uint8 compid    # 接收端 ID 
#ID [100-地面站]   [99-所有无人机]

float32 x
float32 y
float32 z
float32 yaw (rad)
uint8 flag
```

> Mavlink 
> **#1001 LOCAL_POSITION_ENU**      无 flag     99 用于编队飞行 获取邻居无人机位置
> **#1101 SET_LOCAL_POSITION_ENU**  
> **#1112 BACK_HOME_POS_ENU**       无 flag
> **#1121 SET_HOME_POS_ENU**        无 flag yaw

| 编号 | No | 发布端 | Ros Topic | 接收端 | 备注 |
| ---- | ---- |---- |---- |---- | ---- |
|#1001 |1 |px4_bridge.cpp |/mavcomm/send/loc_pos_enu |mavcomm.cpp |飞机端 收集mavros |
|#1001 |2 |mavcomm.cpp |/mavcomm/receive/loc_pos_enu |formation_widget.cpp + uav_ctrl.cpp |GCS 订阅&解析 + 飞机端订阅&解析|
| --- | --- | --- | --- | --- | --- |
|#1101 |1 |console_qnode.cpp(flag=1) |/mavcomm/send/set_loc_pos_enu | mavcomm.cpp |GCS 设置loc点(flag=1) |
|#1101 |2 |mavcomm.cpp |/mavcomm/receive/set_loc_pos_enu |uav_ctrl.cpp(flag=1) |飞机端 设置目标点(flag=1) |
|#1101 |3 |formation_widget.cpp(flag=2) |/mavcomm/send/set_loc_pos_enu | mavcomm.cpp |GCS 设置编队偏差(flag=2) |
|#1101 |4 |mavcomm.cpp |/mavcomm/receive/set_loc_pos_enu |uav_ctrl.cpp + px4_bridge.cpp(flag=2) |飞机端 设置编队偏差(flag=2) |
|#1101 |5 |uav_ctrl.cpp(flag=3) |/mavcomm/send/set_loc_pos_enu | mavcomm.cpp |飞机端 应答编队偏差设置(flag=3) |
|#1101 |6 |mavcomm.cpp |/mavcomm/receive/set_loc_pos_enu |formation_widget.cpp(flag=3) |GCS 确认编队偏差已设置(flag=3) |
| --- | --- | --- | --- | --- | --- |
|#1112 |1 |??? |/mavcomm/send/back_home_pos_enu |mavcomm.cpp |飞机端 返回Home点 |
|#1112 |2 |mavcomm.cpp |/mavcomm/receive/back_home_pos_enu |??? |GCS 订阅&解析 |
| --- | --- | --- | --- | --- | --- |
|#1121 |1 |??? |/mavcomm/send/set_home_pos_enu |mavcomm.cpp |GCS 设置Home点 |
|#1121 |2 |mavcomm.cpp |/mavcomm/receive/set_home_pos_enu |??? |飞机端 订阅&解析 |

--- ---
## global_pos_int.msg

用于地图显示 & 初始位置ENU偏差确认

注意转换 [* 1E7]

```
std_msgs/Header header
uint8 sysid     # 发送端 ID 
uint8 compid    # 接收端 ID 
#ID [100-地面站]   [99-所有无人机]

int32 lat
int32 lon
int32 alt
uint16 hdg_yaw (cdeg [0-360))
uint8 flag
```

> Mavlink 
> **#1002 GLOBAL_POSITION_INT**         无 flag
> **#1102 SET_GLOBAL_POSITION_INT**  
> **#1113 BACK_HOME_POS_GPS_INT**       无 flag
> **#1122 SET_HOME_POS_GPS_INT**        无 flag hdg_yaw

| 编号 | No | 发布端 | Ros Topic | 接收端 | 备注 |
| ---- | ---- |---- |---- |---- | ---- |
|#1002 |1 |px4_bridge.cpp |/mavcomm/send/gps_pos |mavcomm.cpp |飞机端 收集mavros |
|#1002 |2 |mavcomm.cpp |/mavcomm/receive/gps_pos |bd_map_qnode.cpp |GCS 订阅&解析 |
| --- | --- | --- | --- | --- | --- |
|#1102 |1 |??? |/mavcomm/send/set_gps_pos |mavcomm.cpp |GCS 设置loc点 |
|#1102 |2 |mavcomm.cpp |/mavcomm/receive/set_gps_pos |??? |飞机端 设置目标点 |
| --- | --- | --- | --- | --- | --- |
|#1113 |1 |??? |/mavcomm/send/back_home_gps_pos |mavcomm.cpp |飞机端 返回Home点 |
|#1113 |2 |mavcomm.cpp |/mavcomm/receive/back_home_gps_pos |??? |GCS 订阅&解析 |
| --- | --- | --- | --- | --- | --- |
|#1122 |1 |??? |/mavcomm/send/set_home_gps_pos |mavcomm.cpp |GCS 设置Home点 |
|#1122 |2 |mavcomm.cpp |/mavcomm/receive/set_home_gps_pos |??? |飞机端 订阅&解析 |


--- ---
## get_param.msg

```
std_msgs/Header header
uint8 sysid     # 发送端 ID 
uint8 compid    # 接收端 ID 
#ID [100-地面站]   [99-所有无人机]

uint8 param
```

> Mavlink 
> **#1111 GET_HOME_POSITION**

| 编号 | No | 发布端 | Ros Topic | 接收端 | 备注 |
| ---- |---- |---- |---- | ---- | ---- |
|#1111 |1 | ??? |/mavcomm/send/get_home_pos |mavcomm.cpp |GCS 发布&控制 |
|#1111 |2 |mavcomm.cpp  |/mavcomm/receive/get_home_pos |TODO |飞机端 订阅&解析 |

**TODO param 定义**





--- ---
## kcf_target_pos.msg

KCF_tracker 发布的目标的位置

```
std_msgs/Header header
uint8 sysid     # 发送端 ID 
uint8 compid    # 接收端 ID 
#ID [100-地面站]   [99-所有无人机]

uint8 state     # kcftracker_widget.cpp
                # 0-追踪失败
                # 1-追踪中
                # 2-追踪误差大
                # 3-已定位，并发送给其他无人机
                # 4-地面站发送框选，等待无人机响应

float32 x
float32 y
# z = 0, yaw = 0;
uint8 target_no     # kcf_tracker 目标编号
uint8 flag          # /1-本机仅定位,(compid=追踪无人机编号) /2-本机持续追踪,(compid=地面站)

```

> Mavlink 
> **#701 kcf_target_pos**

| 编号 | No | 发布端 | Ros Topic | 接收端 | 备注 |
| ---- |---- |---- |---- | ---- | ---- |
|#701 |1 |kcf_tracker.cpp |/mavcomm/send/kcf_target_pos |mavcomm.cpp  |无人机 发布&控制 |
|#701 |2 |mavcomm.cpp  |/mavcomm/receive/kcf_target_pos |kcftracker_widget.cpp /uav_crtl.cpp |地面站/飞机端 订阅&解析 |


--- ---
## kcf_set_target.msg

KCF_tracker 设置框选的位置
```
std_msgs/Header header
uint8 sysid     # 发送端 ID 
uint8 compid    # 接收端 ID 
#ID [100-地面站]   [99-所有无人机]

uint8 state     # 1-on 0-off
uint8 target_no # 目标编号

uint16 x
uint16 y
uint16 width
uint16 height
```

> Mavlink 
> **#702 kcf_set_target**

| 编号 | No | 发布端 | Ros Topic | 接收端 | 备注 |
| ---- |---- |---- |---- | ---- | ---- |
|#701 |1 |kcftracker_widget.cpp |/mavcomm/send/kcf_set_target |mavcomm.cpp  |无人机 发布&控制 |
|#701 |2 |mavcomm.cpp  |/mavcomm/receive/kcf_set_target |kcf_tracker.cpp |地面站/飞机端 订阅&解析 |




# MISSION 部分 TBC 暂时以 MAVmission/task.cpp 为主

--- ---
## mission_info.msg

mission_info 任务告知
```
std_msgs/Header header
uint8 sysid     # 发送端 ID 
uint8 compid    # 接收端 ID 
#ID [100-地面站]   [99-所有无人机]

uint8 flag
uint8 mission_num
uint8 param1
uint8 param2
```

> Mavlink 
> **#800 mission_info**

| 编号 | No | 发布端 | Ros Topic | 接收端 | 备注 |
| ---- |---- |---- |---- | ---- | ---- |
|#800 |1 |mission_widget.cpp |/mavcomm/send/mission_info |mavcomm.cpp  |地面站 发布&任务 |
|#800 |2 |mavcomm.cpp  |/mavcomm/receive/mission_info |uav_mission.cpp |飞机端 订阅&解析 |

uint8 flag
    # 1 - 任务设置 初始化 [有回应]
    uint8 mission_num 总任务数
    uint8 param1 random1
    uint8 param2 random2 # 用于判断无人机的mission是否相同

    # 2 - 任务设置完 校验 [有回应]
    uint8 mission_num 总任务数
    uint8 param1 random1
    uint8 param2 random2 # 用于判断无人机的mission是否相同

    # 3 - 读取&保存任务 [有回应]
    uint8 param1 1读取 2保存
    uint8 param2 保存文件名

    # 4 - 任务开始 [无回应]
    uint8 mission_num 总任务数
    uint8 param1 1顺序执行/2同步执行

    # 5 - 跳到 #N任务 并开始执行  [无回应]
    uint8 mission_num #任务N
    uint8 param1 1顺序执行/2同步执行

    # 6 - 任务暂停  [无回应]
    uint8 param1 1暂停&悬停 /2暂停&原地降落 /3暂停&起飞位置降落 



(TODO) 是否 需要应答 ? (部分)

--- ---
## mission_info_back.msg
mission_info_back 任务告知 无人机的应答指令

```
std_msgs/Header header
uint8 sysid     # 发送端 ID 
uint8 compid    # 接收端 ID 
#ID [100-地面站]   [99-所有无人机]

uint8 flag
uint8 mission_num
uint8 param1
uint8 param2
```

> Mavlink 
> **#801 mission_info_back**

| 编号 | No | 发布端 | Ros Topic | 接收端 | 备注 |
| ---- |---- |---- |---- | ---- | ---- |
|#801 |1 |uav_mission.cpp |/mavcomm/send/mission_info |mavcomm.cpp  |飞机端 发布&任务 |
|#801 |2 |mavcomm.cpp  |/mavcomm/receive/mission_info |mission_widget.cpp |地面站 订阅&解析 |

TODO 定义

-  应答
uint8 flag
    # 1 - 无人机回应 告知本机已经在 等待接收任务的状态了
    uint8 mission_num 总任务数
    uint8 param1 random1
    uint8 param2 random2 # 用于判断无人机的mission是否相同

    # 2 - 无人机回应 任务设置完 校验
    3校验正确 4校验错误 (flag_mission_set)
    uint8 mission_num 3/4
    uint8 param1 3/4
    uint8 param2 3/4

    # 3 - 无人机回应 读取&保存 正确 （flag_mission_set=2)
    uint8 mission_num 总任务数
    uint8 param1 random1
    uint8 param2 random2 # 用于判断无人机的mission是否相同

    # 4 - 无人机回应 读取&保存 错误 （flag_mission_set=5)
    uint8 param1 random1 # 1读 2写
    uint8 param2 random2 # 5 





> Mavlink 
> **#802 mission_set**

| 编号 | No | 发布端 | Ros Topic | 接收端 | 备注 |
| ---- |---- |---- |---- | ---- | ---- |
|#802 |1 |mission_widget.cpp |/mavcomm/send/mission_info |mavcomm.cpp  |地面站 设置&任务 |
|#802 |2 |mavcomm.cpp  |/mavcomm/receive/mission_info |uav_mission.cpp |无人机 订阅&解析 |

-  参数
    mission_no      任务编号
    uav_no          该任务 无人机编号
    x               ENU x
    y               ENU y
    z               ENU z
    yaw             ENU yaw
    flag            标志位
    [bit 7-0]
        7- 0-自动下一个任务 1不自动下一个
        6- 1-任务完成后，需要延迟时间 param1 (s)
        5- 1-任务完成后，需要延迟时间 param1 *10 (s) [ max=42min ]

    mission_task    任务类型
        #1 起飞
        uint8_t param1
        uint8_t param2
        uint8_t param3
        #2 降落
        #3 定点
        #4 编队
        #5 任务循环
