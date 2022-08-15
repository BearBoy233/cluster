
### mavcomm 无人机集群通信

- 功能 

1.  ROS Topic <-----> Customize Mavlink Serial
2.  ROS话题 与 自定义Mavlink协议数据的 转换、发布&接收
3.  通用(不区分地面站&飞机端) 自定义mavlink消息传输
4.  [plugins] 飞机端订阅mavros话题

TODO
1.  add Func(UDP 发送和接收)

--- ---
### 信息传递示意流

- 订阅 & 打包 & 发送
`(/mavcomm/send/...)` -> `mavcomm.cpp` -> `[Serial Device]`

- 接收 & 解析 & 发布
`[Serial Device]` -> `mavcomm.cpp`  -> `(/mavcomm/receive/...)`

### 测试 两串口模块对发

1. roslaunch mavcomm transmission_test.launch
2. rosrun mavcomm uart_test
3. rostopic pub -1 /test_mavcomm/flag std_msgs/UInt8 "data: 1"
4. rostopic pub -1 /test_mavcomm/flag std_msgs/UInt8 "data: 0"
--- ---
### ROS Topic & ROS msgs & mavlink协议 彼此间的关系

> [mavcomm_msgs](../mavcomm_msgs/Readme_msgs.md)


### roslaunch 部分参数含义

`roslaunch mavcomm mavcomm.launch`

  - serial_port 串口设备 端口名称

  - baudrate 串口设备串口波特率

  - Flag_ats153 

    仅针对P900 串口数传使用：
    
    其在一对多工作模式下(1地面站+多无人机)，地面站需要开启ATS153=1，串口数据包在最开始会加上一段数据，显示来源的设备地址，使得地面站能够区分消息来源于哪一个飞机。
    其在多对多(Mesh)模式下同理，地面+飞机都需要开启。
