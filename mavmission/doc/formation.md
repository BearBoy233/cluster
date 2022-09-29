
## 任务模块 / mav_mission

[任务模块-目录/mission](././../Readme_mission.md)

---
## formation-part 模块


### 主要功能

```
  编队阵型设置 (gcs->uav)
  编队控制 
  编队位置告知 (ENU - 编队偏差)
```

- 一致性编队控制算法(一阶)

$$ u_i(t) = \sum_{j=0}^{n}{a_{ij}} [ x_{j}(t) - x_{i}(t) - ( \delta_{j} - \delta_{i} ) ] $$

$i$ $j$       无人机编号
$n$           无人机总数 
$u_i$         无人机 $i$ 控制指令
$a_{ij}$      通信拓扑
$x_{i}$       无人机 $i$ 位置 (ENU)
$\delta_{i}$  无人机 $i$ 编队偏差

- **TODO编队模式多样性** 

  √ 相对ENU不变
  
  × 相对领机偏航不变

  × 分组编队而不是全部
  
  × 编队变化（尺度大小/队形状态…）

  × 多Leader?
    
  × 根据实际位置寻找最佳位置?


#### (TBC) 大致流程 [mission调用]

```Mermaid

stateDiagram-v2

[*] --> 初始化 : 

初始化 --> 编队阵型设置 : set_local_pos_enu_cb(flag=2)

编队阵型设置 --> 等待编队任务开始 : 

编队阵型设置 --> 广播飞机位置(已经减去了编队偏差) : 

等待编队任务开始 --> 队形集结（可选） : (TODO)避碰 & 同时/依次序集结 

队形集结（可选） --> 编队执行任务 : 跟leader领机的航迹

编队执行任务 --> 任务完成 : (TODO)安全解散

任务完成 --> [*]

```

#### 编队阵型设置 TBCG

```Mermaid

sequenceDiagram

participant gcs
participant uav1
participant uavN

Note over gcs, uavN : 阵型输入设置

gcs ->> gcs : 阵型输入
gcs ->> gcs : 阵型校验(x&y不能相同)

Note over gcs, uavN : 阵型设置&校验(ot_offset)

gcs ->> uav1 : mavcomm/receive/set_loc_pos_enu (flag=2)
gcs ->> uavN : 

uav1 -->> uav1 : ot_offset_?
uavN -->> uavN : ot_offset_?

uav1 -->> gcs : mavcomm/send/set_loc_pos_enu(flag=3)
uavN -->> gcs : 

gcs ->> gcs : 确保每架飞机都设置了 

Note over gcs, uavN : 等待编队飞行

```



