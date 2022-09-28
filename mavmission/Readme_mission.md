# 任务模块 / mav_mission

### 概述

- mission 主要功能   

```
  主模块&调度
```

- 功能模块

[任务管理模块/taskpart](./doc/task.md)

```
  无人机端任务缓存 
  任务完整性校验
  任务本地存储/读取
```

[编队控制模块/formationpart](./doc/formation.md)

```
  编队阵型设置 (gcs->uav)
  编队控制 
  编队位置告知 (ENU - 编队偏差)
```


### [TBC] mission 模块

```Mermaid

stateDiagram-v2

任务执行 --> 从头开始
任务执行 --> 从节点M开始

从头开始 --> [同步/异步]执行
从节点M开始 --> [同步/异步]执行

[同步/异步]执行 --> 结束
[同步/异步]执行 --> 暂停

暂停 --> 结束
暂停 --> [同步/异步]执行
暂停 --> 从节点M开始 

```

### [TBC] 各状态的切换逻辑
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



