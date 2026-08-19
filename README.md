# openShennie

基于 **ESP32-P4 + micro-ROS + ROS2 + OrangePi（Audio2Face 本地推理）** 的仿人面部表情机器人。
下位机驱动 26 个 DS-S009 智能舵机实现面部表情，上位机通过 ROS2 话题链路下发 blendshape → landmark → 舵机角度，可接入本地模型推流（Audio2Face）或手动 GUI 调试。

---

## 1. 系统架构与数据链路

```
┌── 线路 A：OrangePi 推流 ────────────────────────────────────────────┐
│  OrangePi 5 Plus          UDP :9999     PC (192.168.31.122)         │
│  Audio2Face RKNN 推理   ────────────►  a2f_ros2_bridge.py           │
│  (claire / mark 模型)     61 维 blendshape                           │
└──────────────────────────────────────────────┬──────────────────────┘
┌── 线路 B：PC 本机推流 ────────────────────────┤
│  PC 本机 Audio2Face 推理（GPU/ONNX）          │
│  → 脚本直接发布 blendshape                   │
└──────────────────────────────────────────────┤
                                               ▼ /blendshape_robot (61 维)
                                     blendshape2landmark 节点
                                               │ /landmark_positions_batch (25 维)
                                               ▼
                                         landmark2angle 节点
                                               │ /servo_positions_batch (26 维, 0–4095)
                                               ▼
                                        micro_ros_agent (serial)
                                               │ USB Serial/JTAG
                                               ▼
                                         ESP32-P4（节点 esp32_servo）
                                            micro_ros_task → servo_task
                                               │ UART1 @ 1Mbps (TX=GPIO18, RX=GPIO19)
                                               ▼
                                        26 × DS-S009 舵机（30Hz 控制）
```

- **PC 端**：micro_ros_agent 桥接 ROS2 网络与 ESP32；`pkg_facebot_controller` 提供转换节点和调试 GUI。
- **推流源（二选一，见第 6 节）**：
  - **线路 A（OrangePi）**：Audio2Face 模型 RKNN 本地推理，UDP 推流到 PC，经 bridge 进入 ROS2
  - **线路 B（PC 本机）**：PC 上直接跑 Audio2Face 推理，脚本直接发布 `/blendshape_robot`
  - 两条线路共用下游全部链路，**同一时间只跑一条**；调试时 GUI 也能直接发 `/blendshape_robot`
- **ESP32 端**：FreeRTOS 双任务 — `servo_task`（30Hz 舵机控制，EMA 平滑 + 同步写）与 `micro_ros_task`（micro-ROS 通信）。

---

## 2. 目录结构

| 目录 | 说明 |
|---|---|
| `main/` | 固件主程序：`main.cpp`、`servo_task.cpp`（舵机 30Hz 循环）、`micro_ros_task.cpp`（ROS2 通信）、`servo_config.cpp`（26 个舵机 ID/角度配置）、`usjt_transport.c`（USB Serial/JTAG 传输层） |
| `components/ds_servo/` | DS-S009 舵机驱动组件（协议 V4，见 `_doc/DSServo/Protocol-V4.md`） |
| `components/micro_ros/` | micro-ROS for ESP-IDF 组件（含自定义消息 `servo_msgs`） |
| `ros2_ws/` | ROS2 工作空间：`micro_ros_agent`、`micro_ros_msgs`、`pkg_facebot_controller` |
| `_doc/` | 文档与配置文件（舵机 ID、blendshape 映射、landmark 定义、约束 JSON） |
| `_ref_code/openFacebot/` | 参考代码（开源 openFacebot 项目） |
| `log/`、`install/`、`saved_figures/` | 构建/运行产物 |

---

## 3. 硬件接线

- **控制板**：ESP32-P4 开发板
- **舵机**：26 × DS-S009，串行总线接 **UART1**（TX=GPIO18，RX=GPIO19），波特率 **1Mbps**（出厂 115200，固件会自动改）
- **与 PC 通信**：板载 USB Serial/JTAG 口（micro-ROS 走此口）
- 舵机需独立电源，功率不足会导致舵机离线/电压异常

> 💡 **接线顺序**：USB 先接 1 口、后接 2 口（顺序不对会导致串口枚举/供电异常）

### 舵机 ID 表（26 个，顺序即批量话题的数据顺序）

| ID | 名称 | ID | 名称 | ID | 名称 |
|---|---|---|---|---|---|
| 1 | 右内眉 BrowInner_Right | 8 | 左下眼睑 EyelidLower_Left | 23 | 右嘴角下 MouthCornerDown_Right |
| 2 | 左内眉 BrowInner_Left | 13 | 眼睛 Pitch Eye_Pitch | 24 | 左嘴角下 MouthCornerDown_Left |
| 3 | 右外眉 BrowOuter_Right | 14 | 眼睛 Yaw Eye_Yaw | 25 | 上唇前后 LipUpper_FrontBack |
| 4 | 左外眉 BrowOuter_Left | 17 | 下巴开合 Jaw_Open | 26 | 下唇前后 LipLower_FrontBack |
| 5 | 右上眼睑 EyelidUpper_Right | 18 | 下巴右 Jaw_Right | 27 | 下唇上下 LipLower_UpDown |
| 6 | 左上眼睑 EyelidUpper_Left | 19 | 下巴左 Jaw_Left | 33 | 右脸颊 Cheek_Right |
| 7 | 右下眼睑 EyelidLower_Right | 21 | 右嘴角上 MouthCornerUp_Right | 34 | 左脸颊 Cheek_Left |
| | | 22 | 左嘴角上 MouthCornerUp_Left | 35 | 鼻翼 NoseWing |
| | | | | 41/42/43 | 脖子 右/左/中 Neck_* |

> 09–12、15–16、20、28–32、36–40、44 为预留空位。完整说明见 `_doc/openshennie_servoID.md`。

---

## 4. ESP32-P4 固件

### 4.1 环境要求

- ESP-IDF **v5.4.4**（支持 ESP32-P4），并已配置 `idf.py` 环境
- 依赖组件由 IDF 组件管理器自动拉取（`esp_wifi_remote`、`esp_hosted` 等）

### 4.2 编译与烧录

```bash
idf.py set-target esp32p4      # 首次需设置目标芯片
idf.py build                   # 编译
idf.py -p COMx flash           # 烧录（COMx 换成实际的 USB Serial/JTAG 端口）
idf.py -p COMx monitor         # 查看串口日志（Ctrl+] 退出）
```

> ⚠️ 运行 micro_ros_agent 时**不要同时开 monitor**，串口会被占用。

### 4.3 上电自检（重要）

固件启动后会自动扫描舵机，日志输出类似：

```
[初始化] 目标波特率 1000000, 扫描...
[初始化] 在线: 26/26
[初始化] 全部就绪
[舵机] 26个舵机就绪, 30Hz
[通信] 等待上位机连接...
```

- 若部分舵机波特率仍是出厂 115200，固件会**自动改到 1Mbps 并提示断电重启**：

  ```
  [初始化] 波特率已修改, 请给舵机断电再上电!
  ```

  此时请给**舵机电源**断电再上电（ESP32 不用），固件会每 2 秒重扫直到全部在线。
- 若某些 ID 在所有波特率下都无响应，检查该舵机接线/供电。
- 通信任务会一直等待上位机 micro-ROS Agent，连接成功后打印 `[通信] 已连接到上位机`。

### 4.4 运行机制

- 位置指令先入目标表，`servo_task` 以 30Hz 做 EMA 平滑（约 5 帧达 90%），|Δ| < 3 不发送（死区），达到阈值的舵机一次性同步写总线
- 反馈为增量轮询（每周期读 1 个舵机的位置/温度/状态），连续 3 次读取失败判离线
- **温度保护**：任一舵机 ≥ 70°C 自动急停（关闭全部扭矩），降温到 65°C 以下后需手动恢复
- 开机默认**手动模式**；急停状态下仅接受"恢复"指令

---

## 5. ROS2 上位机（PC）

### 5.1 构建工作空间

```bash
cd ros2_ws
colcon build                    # 或只编控制包: colcon build --packages-select pkg_facebot_controller
```

环境加载：
- **Windows PowerShell**：`. .\install\local_setup.ps1`
- **Windows cmd**：`call install\setup.bat`
- **Linux**：`source install/setup.bash`

### 5.2 启动 micro-ROS Agent（先启动！）

```bash
# Linux（Docker，实际使用）：--dev 换成实际的 USB Serial/JTAG 端口
sudo docker run -it --rm --privileged --net=host -v /dev:/dev \
    microros/micro-ros-agent:humble serial --dev /dev/ttyACM1 -b 921600


> USB Serial/JTAG 为虚拟串口，波特率参数实际影响很小（此处统一 `-b 921600`）。
> 连接成功后日志出现 `session established`，且 ESP32 侧打印 `[通信] 已连接到上位机`。

### 5.3 话题总览

| 话题 | 类型 | 方向 | 内容 |
|---|---|---|---|
| `/blendshape_robot` | Float32MultiArray | 推流源（线路 A/B 或 GUI）→ | 61 维 ARKit blendshape（0–1） |
| `/landmark_positions_batch` | Float32MultiArray | PC 内 | 25 维 landmark 参数 p |
| `/servo_positions_batch` | Float32MultiArray | PC → ESP32 | **26 维舵机位置（0–4095），顺序与第 3 节 ID 表一致** |
| `/servo_command` | servo_msgs/msg/ServoCommand | PC → ESP32 | 单舵机/整机指令（见 5.5） |
| `/get_servo_info` | servo_msgs/srv/GetServoInfo | 服务 | 查询全部舵机状态 |

### 5.4 启动转换节点（表情链路）

```bash
ros2 run pkg_facebot_controller blendshape2landmark   # 61 维 blendshape → 25 维 landmark
ros2 run pkg_facebot_controller landmark2angle        # 25 维 landmark → 26 维舵机角度（线性 + 五连杆 IK）
```

- 两个节点**每 5 秒检查配置 JSON 是否变化并自动热重载**，改完 `_doc` 下的配置不用重启节点
- 也可用 `ros2 launch`/脚本一键启动，各节点独立运行便于分别调试

### 5.5 servo_command 指令表

消息字段：`id`（uint8）、`position`（uint16, 0–4095）、`time_ms`（uint16）、`cmd_type`（int32）。

| cmd_type | 功能 | id | position | 示例 |
|---|---|---|---|---|
| 0 | 单舵机位置直写 | 舵机 ID | 0–4095 | 见下 |
| 253 | 扭矩开关 | 舵机 ID（254=广播） | 0=关，非 0=开 | `"{id: 254, position: 1, time_ms: 0, cmd_type: 253}"` |
| 254 | 手动/自动模式 | 1=手动，0=自动 | 忽略 | `"{id: 1, ..., cmd_type: 254}"` |
| 255 | Ping 单个舵机 | 舵机 ID | 忽略 | 结果打印在 ESP32 串口日志 |
| 263 | 急停 / 恢复 | 忽略 | 0=急停（全机关扭矩），非 0=恢复 | 见下 |

```bash
# 单舵机位置（如 17 号下巴到 2048）
ros2 topic pub -1 servo_command servo_msgs/msg/ServoCommand "{id: 17, position: 2048, time_ms: 0, cmd_type: 0}"

# 紧急停止 / 恢复
ros2 topic pub -1 servo_command servo_msgs/msg/ServoCommand "{id: 0, position: 0, time_ms: 0, cmd_type: 263}"
ros2 topic pub -1 servo_command servo_msgs/msg/ServoCommand "{id: 0, position: 1, time_ms: 0, cmd_type: 263}"

# 批量位置（26 个值，中间省略）
ros2 topic pub -r 20 servo_positions_batch std_msgs/msg/Float32MultiArray "{data: [2048, 2048, ...]}"
```

### 5.6 查询舵机状态（Service）

```bash
ros2 service call /get_servo_info servo_msgs/srv/GetServoInfo "{}"
```

返回各舵机的 `ids / positions / temperatures / statuses` 数组。

### 5.7 调试 GUI

| 命令 | 说明 |
|---|---|
| `ros2 run pkg_facebot_controller controller_angle_GUI` | 26 舵机角度滑块，直接发 `servo_positions_batch`（20Hz，可开关自动发送） |
| `ros2 run pkg_facebot_controller controller_landmark_GUI` | 25 landmark 滑块，发 `landmark_positions_batch`（走 landmark2angle 链路） |
| `ros2 run pkg_facebot_controller controller_blendshape_GUI` | 61 blendshape 滑块，发 `blendshape_robot`（走完整表情链路） |
| `ros2 run pkg_facebot_controller servo_visualizer` | 订阅 `servo_positions_batch` 实时曲线可视化 |

调试建议从 `controller_angle_GUI`（最底层）开始，逐级向上验证。

---

## 6. 推流线路（二选一）

两条线路在 `/blendshape_robot` 汇合，下游链路完全一致。**同一时间只跑一条**（都跑会互相覆盖表情数据）。
都需要打开blendshape2landmark以及landmark2angle节点

### 6.1 线路 A：OrangePi 推流（RKNN 板端推理）

> 设备：OrangePi 5 Plus，IP `192.168.31.151`，RKNPU 0.9.8，模型文件清单详见 `_doc/orangepi_model_deployment.md`

**连接方式**：

```bash
adb connect 192.168.31.151:5555     # adb shell 为 root，正常用户 orangepi
```

**模型与脚本**（均位于 `~` 即 `/home/orangepi`）：

| 文件 | 说明 |
|---|---|
| `a2f_sender_claire.py` | **claire 模型 UDP 推流**（25 FPS 目标，实测 ~18Hz，推流脚本主用） |
| `run_infer.py` | 单次推理自测 |
| `bench_a2f.py` 等 | NPU 性能测试 |
| `audio2face_claire.rknn` / `audio2face_v2.rknn` | claire / mark 模型权重 |
| `postproc_claire.npz` / `postproc_optimized.npz` | 后处理矩阵 |

**启动推流**：

```bash
# 方式一：手动（推荐，便于看日志）
cd /home/orangepi && python3 a2f_sender_claire.py
# 或 mark 模型：python3 a2f_sender_mark.py

# 方式二：systemd 服务（当前已禁用开机自启，按需启动）
sudo systemctl start a2f-stream
```

**PC 端配套**：需先启动 `a2f_ros2_bridge.py`（收 UDP :9999 → 发布 `/blendshape_robot`）：

```bash
python3 /home/laid/a2f_ros2_bridge.py    # PC (192.168.31.122) 上运行
```

### 6.2 线路 B：PC 本机推流

不经过 OrangePi 和 UDP，PC 上直接跑 Audio2Face 推理，脚本直接发布 `/blendshape_robot`（61 维）：



### 6.3 线路切换

- 停掉当前线路的推流脚本（或 OrangePi 上 `Ctrl+C`），再启动另一条即可
- 下游（blendshape2landmark → landmark2angle → agent → ESP32）不用重启
- 切换后表情源立即变更，无需任何重新连接操作

---

## 7. 完整启动流程（推荐顺序）

1. **硬件准备**：ESP32-P4 烧录固件、舵机总线接 UART1、给舵机上电；观察串口日志直到 `全部就绪`（波特率被修改时先给舵机断电重启一次）
2. **PC**：启动 micro-ROS Agent（5.2），确认 ESP32 打印 `[通信] 已连接到上位机`
3. **PC**：启动 `blendshape2landmark` 与 `landmark2angle`（5.4）
4. **冒烟测试**：开 `controller_blendshape_GUI` 拖动滑块，确认舵机动作；`ros2 service call /get_servo_info ...` 确认反馈正常
5. **接推流（二选一）**：
   - **线路 A（OrangePi）**：PC 启动 `a2f_ros2_bridge.py`，OrangePi 启动推流脚本（6.1）
   - **线路 B（PC 本机）**：直接运行 PC 推理发布脚本（6.2）
6. **急停演练**：发 `cmd_type 263, position 0` 确认全机扭矩关闭，再发 `position 1` 恢复

---

## 8. 常用调试命令速查

```bash
ros2 topic list                              # 列出所有话题（应含 5.3 的表）
ros2 topic echo servo_positions_batch        # 观察批量位置流
ros2 topic hz servo_positions_batch          # 查看发布频率
ros2 topic info servo_command -v             # 话题类型/发布订阅者
ros2 node list                               # 节点列表（含 esp32_servo）
ros2 service call /get_servo_info servo_msgs/srv/GetServoInfo "{}"
```

ESP32 串口日志（`idf.py monitor` 或任意串口助手，注意与 agent 不同时占用）：
- `[指令] ...` 收到 ROS2 指令
- `[Ping] 舵机X: 在线/无响应`
- `[警告] 舵机X 温度过高` → 已自动急停
- `[状态] 舵机X 离线`

---

## 9. 配置调参（`_doc/` 下 JSON，改完自动热重载）

| 文件 | 作用 | 谁在用 |
|---|---|---|
| `blendshapeID_info.json` | 61 维 blendshape → 25 维 landmark 的稀疏 delta 映射 + neutral 基准 | `blendshape2landmark` 节点 |
| `landmarkID_info.json` | 25 landmark 定义：线性映射（`linear`）与五连杆 IK（`fivebar`）→ 舵机角度 | `landmark2angle` 节点 |
| `servoID_constrain.json` | 各舵机角度约束（angle_min/max/neutral） | PC 端 `servo_config.py` / GUI |
| `openshennie_servoID.md`、`blendshapeID.md`、`landmarkID.md` | 编号与命名说明文档 | — |

修改 JSON 后保存，对应节点 5 秒内自动重载并打印 `配置已热重载`，无需重启节点。

---

## 10. 故障排查

| 现象 | 处理 |
|---|---|
| 上电后部分舵机无响应 | 查电源/接线；看串口日志列出的离线 ID；波特率被改过则先断电重启舵机 |
| Agent 一直连不上（`等待上位机连接...`） | COM 口被占用（monitor/串口助手）、COM 号错误、ROS 环境未 source |
| 发指令舵机不动 | 是否处于急停（先发恢复指令 263/1）；是否手动模式；`ros2 topic echo servo_positions_batch` 确认数据到达 agent |
| 舵机动作卡顿/掉帧 | 批量话题发布频率过高或过低（建议 20–30Hz）；PC 与 ESP32 间 USB 线质量问题 |
| 突然全部扭矩关闭 | 温度保护触发（≥70°C），降温后发 263/1 恢复 |
| 表情幅度不对 | 调 `_doc/blendshapeID_info.json` 的 delta/scale 或 `landmarkID_info.json` 的映射范围 |
| GUI 打开报错 | 确认已 source install；Windows 下用 PowerShell 的 `.\install\local_setup.ps1` |

---

## 11. 已知坑（OrangePi 端）

- orangepi 用户 `~/.local` 的 soundfile 必须为 **0.12.1**（0.14.0 与 numpy 1.21.5 不兼容，报 `'numpy._DTypeMeta' object is not subscriptable`）
- `/dev/rknpu*` 设备节点不存在是正常现象（驱动 0.9.8 不创建节点），不影响推理
- `a2f-stream.service` 已 `systemctl disable`，不会开机自启，需要时手动启动

## 参考

- 舵机协议：`_doc/DSServo/Protocol-V4.md`
- 参考实现：`_ref_code/openFacebot/`（开源项目 openFacebot）
- 模型部署细节：`_doc/orangepi_model_deployment.md`
- 整体规划：`_doc/openshennie_idf_plan.md`
