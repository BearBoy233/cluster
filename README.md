# cluster

[cluster 地面站](https://github.com/BearBoy233/cluster_gcs)

[cluster 集群飞机端](https://github.com/BearBoy233/cluster)


### 依赖

- 硬件

```
  串口数传 Microhard P900 ( 需要配置为 Mesh 模式 )
```

- 软件

``` 
ros-melodic-serial

ros-melodic-mavros

opencv 3/4
```

### Quick Setup 

1. 编译

```
1. 使用 ./rospkg_compile.sh 编译， 并添加 `.bashrc` 路径


2、 飞机端，将 `TODO` 设置为开机自启

  $ gnome-session-properties

  选择 `TODO` 文件

```

2. 新建 `common.yaml`文件

需要新建 `cluster/common.yaml` 文件，并填入下面内容.

注: launch文件会读取，系统内若节点的 `my_id` 一致会 Error.

```
# common config parameter

# my_id 	本设备编号 	/100-地面站 	/99-所有无人机
# /system_id 发送端编号	/companion_id 接收端编号
my_id: 100

# Flag_1ShowRn 	输出测试 /0-不输出 /1-输出关键 /2-全输出 
Flag_1ShowRn: 1
```

- 注1: common.yaml 通用参数 介绍

  - `my_id` 本设备编号

```
对应 通信数据包中的 `/system_id` 发送端编号;
    
注1: 暂时只考虑10架，理论上最大（0xFF =255）- 2

注2: (TODO) 编组， 如取 200-240 为该组无人机所接收的指令

具体的数字含义

  1-10  - 无人机
    
  100   - 地面站 	
    
  99    - 所有节点 (此处取用) 
```

  - `Flag_1ShowRn` 程序测试输出

```
  /0  - 不输出 
    
  /1  - 输出关键 
    
  /2  - 全输出 
```

### cluster pkg 各程序包简介

- [mavcomm_msgs](./mavcomm_msgs/Readme_msgs.md)

  集群系统的自定义消息包 ROS Msg

  - 定义 ROS MSG 类型，与 mavlink 相匹配

- [mavcomm](./mavcomm/Readme_mavcomm.md)

  集群系统，通信程序包。

  - ROS Msg 和 自定义MAVLink数据包 之间的转换。
    ROS Topic <=> Customize Mavlink msg Serial
  - [plugins/px4_bridge] 订阅 mavros 主要信息，并转为 heartbeat
  - (TODO) TCP/UDP功能

- [mavconsole](./mavconsole/Readme_console.md)

  - Roslaunch 启动
  - ROS Node 监控 & 关闭

- [px4_offb](./px4_offb/Readme_px4ctrl.md)

  - uav_ctrl    无人机底层控制 
  
  - uav_mission 任务&航点节点

  - vision_pose 为PX4提供 外部定位 (Mocap /VINS /UWB...)

- [mavmission](./mavmission/Readme_mission.md)

  - mission    

    1. 任务设置
    
      系统初次使用，需通过地面站完成任务的 1设置 & 2校准 / 3读取 / 4保存
  
    2. 任务执行

      ```
      TBD
      
      顺序执行 (串行?)
      同步执行 (并行?)
      ```

      - 航点飞行

      ```
        ENU_LOC - 单机
        ENU-LOC - 编队
        
      TODO
        GPS - 单机
        GPS - 编队
      ```

      - 编队
      
      ```
        ENU 为基准
        
      TODO
        领机坐标系 为基准
      ```

      - 避障
      
      ```
        Intel D4XX 相机 深度图 + Fast_plan
      ```
      
      - 追踪 

      ```
        KCF 框选
      ```

- [gen_mavlink/额外](./gen_mavlink/Readme_mavlink.md) 

  - 生成 Customize Mavlink V2 消息 (Mavcomm)
  - 与 mavcomm_msgs 对应
  
  备注1: Mavlink是一种用于小型无人载具的通信协议。

  [github/mavlink](https://github.com/mavlink/mavlink)
