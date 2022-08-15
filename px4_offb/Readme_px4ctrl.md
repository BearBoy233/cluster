# 无人机控制 & vision_pose

# px4_offb

### uav_ctrl

`node_state`各状态的切换逻辑

```Mermaid

stateDiagram-v2

[*] --> WAITING_FOR_HOME_POSE

WAITING_FOR_HOME_POSE --> IDLE : received_homePose_flag 

IDLE --> [*]
IDLE --> TAKING_OFF

TAKING_OFF --> TAKEOFF_FAIL : !takeoff() 

TAKING_OFF --> POS_EXECUTION
POS_EXECUTION --> LANDING

TAKING_OFF --> Vel_EXECUTION
Vel_EXECUTION --> LANDING

LANDING --> LANDED : autoland()
LANDED --> IDLE : disarm()

note left of [*] : /mavcomm/receive/changestate 实现切换

```
