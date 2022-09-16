000# 任务模块

# mav_mission

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



