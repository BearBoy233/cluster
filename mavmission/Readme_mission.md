# 任务模块

# mav_mission

### 概述

- mission   

    主模块 调度

    rostopic sub


- task_part 

    任务设置

    任务执行


# task_part 模块



```Mermaid

flowchart TD;
A-->B;
A-->C;
B-->D;
C-->D;

```


### mission 模块

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




