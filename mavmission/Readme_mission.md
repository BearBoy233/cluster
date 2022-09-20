# 任务模块 / mav_mission

### 概述

- mission 主要功能   

```
    主模块&调度
```

- 功能模块

[任务模块/taskpart](#task-part-模块) 

[任务模块/taskpart](#task-part-模块) 

---
#### task-part 模块

[参考/mavlink-services-mission](https://mavlink.io/en/services/mission.html)

- diff

  一对一的设置模式 => 一对多设置 (增加效率)


- 主要功能

```
    任务设置 (gcs->uav)
    任务执行 (任务进度获取)
    任务保存/读取 (json 格式)
```

- 基本的执行流程 [mission调用对应函数]

```Mermaid

stateDiagram-v2

初始设置 --> 任务信息

任务信息 --> [gcs]数传传入
任务信息 --> [uav]本地读取

[gcs]数传传入 --> 集群任务信息校验
[uav]本地读取 --> 集群任务信息校验

集群任务信息校验 --> [uav]本地存储
集群任务信息校验 --> 等待任务执行 : 校验无误
集群任务信息校验 --> [gcs]数传传入 : 校验有误


任务执行 --> 从头开始
任务执行 --> 从节点M开始

从头开始 --> [同步/异步]执行
从节点M开始 --> [同步/异步]执行

[同步/异步]执行 --> 结束
[同步/异步]执行 --> 暂停

暂停 --> 结束
暂停 --> [同步/异步]执行
暂停 --> 从节点M开始 

结束 --> [*]

```

- **上传任务到无人机&校验**

```Mermaid

sequenceDiagram

participant gcs
participant uav1
participant uavN

Note over gcs, uavN : 发送初始化

gcs ->> uav1 : mission_info (flag=1)
gcs ->> uavN : 

gcs -->> gcs : start timeout [TODO-Auto]
uav1 -->> uav1 : current_mission_state=MISSION_SETTING
uavN -->> uavN : current_mission_state=MISSION_SETTING

uav1 -->> gcs : mission_back_info(flag=MISSION_SETTING)
uavN -->> gcs : 


Note over gcs, uavN : 遍历发送 mission_set

gcs ->> uav1 : mission_set(0)
gcs ->> uavN : 
Note over gcs : mission_set [0，M)
gcs ->> uav1 : mission_set(M)
gcs ->> uavN : 


Note over gcs, uavN : 校验

loop 遍历校验
    gcs->>uav1 : mission_info (flag=2)
    gcs->>uavN : 
    Note over uav1, uavN : 遍历mis_array
    
    Note over uav1 : 缺少mission_set(?)
    uav1 -->> uav1 : current_mission_state=MISSION_CHECK_FAIL_INCOMPLETE
    uav1 -->> gcs : mission_back_info(flag=MISSION_CHECK_FAIL_INCOMPLETE)[param=?]
    gcs ->> uav1 : mission_set(?)
    
    Note over uavN : 完整mission_set
    uavN -->> uavN : current_mission_state=MISSION_CHECKED
    uavN -->> gcs : mission_back_info(flag=MISSION_CHECKED)
end




```


### [TBC] mission 模块

`node_state`各状态的切换逻辑

```Mermaid

stateDiagram-v2

[*] --> WAITING_FOR_HOME_POSE

WAITING_FOR_HOME_POSE --> IDLE : received_homePose_flag 

IDLE --> [*]
IDLE --> TAKING_OFF

TAKING_OFF --> TAKEOFF_FAIL : !takeoff() 

TAKING_OFF --> POS_EXECUTION
TAKING_OFF --> Vel_EXECUTION

POS_EXECUTION --> Vel_EXECUTION
POS_EXECUTION --> LANDING

Vel_EXECUTION --> POS_EXECUTION
Vel_EXECUTION --> LANDING

LANDING --> LANDED : autoland()
LANDED --> IDLE : disarm()

note left of [*] : /mavcomm/receive/changestate 实现切换

```



