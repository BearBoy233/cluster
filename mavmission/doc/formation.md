
## 任务模块 / mav_mission

[任务模块-目录/mission](././../Readme_mission.md)

---
## formation-part 模块


### 主要功能

```
  编队阵型设置 (gcs->uav)
  编队控制(/mavros/setpoint_velocity)
```

- [一致性编队控制算法(一阶)]

$$ u_i(t) = 1/M * \sum_{j=0}^{n}{a_{ij}} [ x_{j}(t) - x_{i}(t) - ( \delta_{j} - \delta_{i} ) ] $$

$i$ $j$       无人机编号
$n$           无人机总数 
$u_i$         无人机 $i$ 控制指令
$a_{ij}$      通信拓扑
$x_{i}$       无人机 $i$ 位置 (ENU)
$\delta_{i}$  无人机 $i$ 编队偏差

- **TODO编队模式多样性** 

  √ 相对ENU不变
  
  × 相对领机偏航不变

  √ 分组编队而不是全部
  
  × 编队变化（尺度大小/队形状态…）

  ? 多Leader
    
  × 根据实际位置就近分配位置?


#### 流程和状态切换 [mission调用]

```Mermaid

stateDiagram-v2

[*] --> 初始化 : FORMATION_STATE_NAN

初始化 --> 编队阵型设置 : [gcs/uav-leader]formation_info(FORMATION_INFO_SET)

编队阵型设置 --> 编队阵型校验 : MISSION_STATE_SETTING

编队阵型校验 --> 编队阵型设置 : MISSION_STATE_CHACK_FAILED

编队阵型校验 --> 等待编队任务执行 : MISSION_STATE_CHECKED

等待编队任务执行 --> 队形集结 : formation_info(FORMATION_INFO_START)

队形集结 --> 编队执行任务 : 跟leader领机的航迹

编队执行任务 --> 任务完成 : (TODO)安全解散

任务完成 --> [*]

```

#### 编队阵型设置

```Mermaid

sequenceDiagram

participant gcs
participant uav1
participant uavN

Note over gcs : gcs也可以是uav-leader(TODO)

Note over gcs, uavN : 编队阵型输入设置

gcs ->> gcs : 编队阵型输入
gcs ->> gcs : 编队阵型校验(x&y不能相同)

Note over gcs, uavN : 编队阵型设置初始化

gcs ->> uav1 : formation_info(FORMATION_INFO_SET_INIT)
gcs ->> uavN : 

uav1 -->> uav1 : current_formation_state=FORMATION_STATE_SETTING
uavN -->> uavN : current_formation_state=FORMATION_STATE_SETTING

uav1 -->> gcs : formation_back_info(flag=FORMATION_STATE_SETTING)
uavN -->> gcs : 

Note over gcs, uavN : 遍历发送 formation_set

gcs ->> uav1 : formation_set(0)
gcs ->> uavN : 
Note over gcs: formation_set [0，M)
gcs ->> uav1 : formation_set(M)
gcs ->> uavN : 

Note over gcs, uavN : 校验

loop 遍历校验
    gcs->>uav1 : formation_info(FORMATION_INFO_CHECK)
    gcs->>uavN : 
    Note over uav1, uavN : formation_array
    
    Note over uav1 : 缺少 formation_array(?)
    uav1 -->> uav1 : current_formation_state=FORMATION_STATE_CHECK_FAIL
    uav1 -->> gcs : formation_back_info(flag=FORMATION_STATE_CHECK_FAIL)[check_setbit]
    gcs ->> uav1 : formation_set(?)
    
    Note over uavN : formation_array
    uavN -->> uavN : current_formation_state=FORMATION_STATE_CHECKED
    uavN -->> gcs : current_formation_state(flag=FORMATION_STATE_CHECKED)[check_setbit]
end

Note over gcs, uavN : 等待编队飞行

```


#### formation_ctrl_all(执行编队流程)

```Mermaid

stateDiagram-v2

[*] --> 初始化 : FORMATION_STATE_RUN_FORMING

初始化 --> 编队阵型形成 : formation_array[NNN][FORMATION_GROUP_PPP].flag

编队阵型形成 --> 分布式编队算法 : flag[0]=0

编队阵型形成 --> 按照offset形成编队 : flag[0]=1&flag[1]=0

编队阵型形成 --> 按照offset，一架架依次形成编队 : flag[0]=1&flag[1]=1

按照offset形成编队 --> 分布式编队算法

按照offset，一架架依次形成编队 --> 分布式编队算法

分布式编队算法 --> 编队阵型形成 : 编队队形切换

分布式编队算法 --> 任务完成 : (TODO)安全解散

任务完成 --> [*]

```

