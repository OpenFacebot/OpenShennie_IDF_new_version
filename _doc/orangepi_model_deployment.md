# OrangePi 5 Plus — Audio2Face 模型本地部署文件清单

> 生成日期: 2026-08-06
> 设备: OrangePi 5 Plus (192.168.31.151, Ubuntu 22.04, 内核 6.1.43-rockchip-rk3588, RKNPU 0.9.8)
> 访问方式: `adb connect 192.168.31.151:5555`（adb shell 为 root，正常用户 `orangepi`）
> `~` = `/home/orangepi`

---

## 一、RKNN 模型文件（.rknn）

| 文件 | 大小 | 用途 |
|---|---|---|
| `~/DataPipeline_Audio2Face-main/model/audio2face_claire.rknn` | 55 MB | **claire 模型**（推流脚本在用） |
| `~/DataPipeline_Audio2Face-main/model/audio2face_claire_int8.rknn` | 55 MB | claire INT8（同尺寸备份） |
| `~/audio2face_v2.rknn` | 34 MB | **mark 模型 INT8**（`a2f_sender_mark.py` 用） |
| `~/audio2face-3d-v2.3-mark.rknn` | 34 MB | mark 模型（另一份） |
| `~/audio2face-3d-v2.3-mark_fp16.rknn` | 61 MB | mark 模型 **FP16**（精度更高、更慢） |
| `~/model/*.rknn`（yolov5s 等） | 7–8 MB | 目标检测模型，**与表情无关** |

> ONNX 中间产物 `check0/2/3_*.onnx`（各 ~158 MB）在 `~/DataPipeline_Audio2Face-main/` 根目录，仅供模型转换，运行时不需要。

## 二、推理引擎 & 推流脚本（均在 `~` 下）

| 文件 | 作用 |
|---|---|
| `a2f_engine_rknn.py` | mark 模型推理引擎 `A2FRKNNEngine`（3 核 NPU 并行，最高 ~55 FPS） |
| `a2f_sender_claire.py` | claire 模型 UDP 推流（25 FPS 目标，实测 ~18 Hz） |
| `a2f_sender_mark.py` | mark 优化引擎 UDP 推流（30 FPS 目标） |
| `a2f_sender.py` | mark 引擎 + 合成音频测试推流 |
| `a2f_sender_orig.py` / `a2f_sender_mark_orig.py` | 旧版推流 |
| `run_infer.py` | 单次推理自测 |
| `bench_a2f.py` / `bench_cores.py` / `bench_parallel.py` | NPU 性能测试 |
| `compare_board.py` | 板端与 PC 输出对比验证 |

## 三、后处理矩阵（.npz）

| 文件 | 说明 |
|---|---|
| `postproc_claire.npz` | claire 后处理（pre_bias / pre_matrix / Q_inv / sL1） |
| `postproc_optimized.npz` | mark 优化后处理 |
| `compare_data.npz` | 对比验证数据 |

> 由 `~/DataPipeline_Audio2Face-main/src/audio2face/nvidia_a2f/convert_a2f_optimized.py` 生成。

## 四、项目源码（`~/DataPipeline_Audio2Face-main/`）

- `src/audio2face/nvidia_a2f/` — 官方 CPU 版 `a2f_engine.py`、RKNN 版、`convert_a2f.py`、`convert_a2f_optimized.py`
- `src/motiontrack/`、`src/render/UE5/` — 配套工具（motion track 合并、UE5 渲染协议）
- `_data/sample/audio/*.wav` — 测试音频（audio_1sec / 4sec / 6sec，16k）
- `_data/sample/motion/*.csv` — 参考表情轨迹
- `envs/audio2face`（conda）、`.venv` — Python 环境

## 五、运行时 & 驱动（系统层）

| 组件 | 位置 / 版本 |
|---|---|
| rknnlite | `/usr/local/lib/python3.10/dist-packages/rknnlite` (2.3.0) |
| NPU 运行时库 | `/usr/lib/librknnrt.so` (2.3.0) |
| rknn_server | `/usr/bin/rknn_server`（板载调试服务，无需常驻） |
| NPU 驱动 | 内核 rknpu **0.9.8**（`orangepi-5b-rknpu-0.9.8-update` deb 包） |
| 推流 systemd 服务 | `/etc/systemd/system/a2f-stream.service` |

## 六、当前推流状态（2026-08-06）

- systemd 服务 `a2f-stream.service`：**已停止并取消开机自启**（`systemctl disable --now a2f-stream`）
- 需要时手动启动：
  ```bash
  cd /home/orangepi && python3 a2f_sender_claire.py
  # 或: sudo systemctl start a2f-stream
  ```
- 推流目标：UDP → PC `192.168.31.122:9999` → `a2f_ros2_bridge.py` → ROS2 `/blendshape_robot`
- 完整链路：板子 RKNN 推理 → UDP → bridge → blendshape2landmark → landmark2angle → micro_ros_agent → ESP32 舵机

## 七、已知坑

- orangepi 用户 `~/.local` 的 soundfile 需为 **0.12.1**（0.14.0 与 numpy 1.21.5 不兼容，报 `'numpy._DTypeMeta' object is not subscriptable`）
- `/dev/rknpu*` 设备节点不存在是正常现象（驱动 0.9.8 不创建节点），推理不受影响
