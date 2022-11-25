# 任务模块 / mav_mission

### 概述

- mission 主要功能   

```
  主模块&调度
  task含义的解析&执行
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
  编队阵型设置 (gcs->uav) (TDL uav-leader->uav)
  编队控制 
  ## 无人机位置告知 (ENU) [/mavcomm/px4_bridge.cpp里]
```


### mission模块-while()逻辑

通过 cb rostopic `/mavcomm/receive/mission_`

```Mermaid 

stateDiagram-v2 

开始 --> 任务执行 

开始 --> 任务输入(task_part) 
任务输入(task_part) --> 开始 

任务执行 --> 从节点M开始 

任务执行 --> 从头开始 
从头开始 --> [同步/异步]执行 

从节点M开始 --> [同步/异步]执行 

[同步/异步]执行 --> 结束 
[同步/异步]执行 --> 暂停 

暂停 --> 结束 
暂停 --> [同步/异步]执行 
暂停 --> 从节点M开始 

```

### parses_current_mission_task()逻辑 

task_part 任务的具体解析和执行




#### 任务执行 [ mission_exec_cb 切换逻辑]

```Mermaid

stateDiagram-v2

[*] --> MISSION_STATE_CHECKED : task_part完成校验

MISSION_STATE_CHECKED --> 任务Init : mission_exec.instruct_status=1(从n开始) 

任务Init --> 子任务解析执行parses_current_mission_task()

子任务解析执行parses_current_mission_task() --> 子任务完成判断

子任务完成判断 --> 下一个任务 

下一个任务 --> 子任务解析执行parses_current_mission_task()

子任务解析执行parses_current_mission_task() --> 暂停悬停 : mission_exec.instruct_status=2(暂停) 

暂停悬停 --> 子任务解析执行parses_current_mission_task() : mission_exec.instruct_status=3(继续已暂停的任务) 

子任务解析执行parses_current_mission_task() --> 紧急降落 : mission_exec.instruct_status=4(紧急降落) 

子任务解析执行parses_current_mission_task() --> 结束降落 : mission_exec.instruct_status=5(结束降落) 

子任务解析执行parses_current_mission_task() --> 结束悬停 : mission_exec.instruct_status=6(结束悬停)

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



