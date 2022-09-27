
## 任务模块 / mav_mission

[任务模块-目录/mission](././../Readme_mission.md)

---
## formation-part 模块


---
### 主要功能

```
  编队阵型设置 (gcs->uav)
  编队控制 
  编队位置告知 (ENU - 编队偏差)
```

### TBD
#### 流程和状态切换 [mission调用]

```Mermaid

stateDiagram-v2

[*] --> 初始化 : MISSION_STATE_NAN

初始化 --> 任务信息 : mission_info(MISSION_INFO_SET_INIT)

任务信息 --> [gcs]数传传入 : MISSION_STATE_SETTING
任务信息 --> [uav]本地读取 : MISSION_STATE_LOADED

[gcs]数传传入 --> 集群任务信息校验 : mission_info(MISSION_INFO_CHECK)
[uav]本地读取 --> 集群任务信息校验 : mission_info(MISSION_INFO_CHECK)

集群任务信息校验 --> [gcs]数传传入 : MISSION_STATE_CHECK_FAIL

集群任务信息校验 --> 等待任务执行 : MISSION_STATE_CHECKED
等待任务执行 --> [uav]本地存储 : 

等待任务执行 --> [*]

```

#### 编队偏差设置

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
