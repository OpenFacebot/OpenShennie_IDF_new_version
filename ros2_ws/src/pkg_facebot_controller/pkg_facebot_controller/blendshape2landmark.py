#!/usr/bin/env python3
"""
blendshape2landmark — 独立 ROS2 节点

订阅 blendshape (Float32MultiArray, 61 维)，
根据 blendshapeID_info.json 中的稀疏 delta 映射，
计算 landmark 位置后发布 landmark_positions_batch (Float32MultiArray, 25 维)。

转换公式:
  l = neutral + sum(bs[i] * scale[i] * (absolute[i] - neutral))

配置集中存储在 _doc/blendshapeID_info.json
"""

import json
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


# ---------------------------------------------------------------------------
#  常量
# ---------------------------------------------------------------------------
# 向上搜索找到 _doc 目录（兼容源码和安装位置）
_doc_candidate = os.path.dirname(os.path.abspath(__file__))
_DOC_DIR = ''
for _ in range(15):
    _test = os.path.join(_doc_candidate, '_doc')
    if os.path.isdir(_test):
        _DOC_DIR = _test
        break
    _next = os.path.dirname(_doc_candidate)
    if _next == _doc_candidate:
        break
    _doc_candidate = _next
_BS_CONFIG_PATH = os.path.join(_DOC_DIR, 'blendshapeID_info.json')
_LM_CONFIG_PATH = os.path.join(_DOC_DIR, 'landmarkID_info.json')

_NUM_BLENDSHAPES = 61


# ---------------------------------------------------------------------------
#  配置加载
# ---------------------------------------------------------------------------

def load_blendshape_config(json_path: str = _BS_CONFIG_PATH) -> dict:
    """加载 blendshapeID_info.json，返回完整配置字典。"""
    try:
        with open(json_path, 'r') as f:
            return json.load(f)
    except (FileNotFoundError, json.JSONDecodeError) as e:
        print(f"[blendshape2landmark] 无法加载配置 {json_path}: {e}")
        return {}


def load_landmark_config(json_path: str = _LM_CONFIG_PATH) -> dict:
    """加载 landmarkID_info.json（用于获取 landmark 范围做 clamp）。"""
    try:
        with open(json_path, 'r') as f:
            return json.load(f)
    except (FileNotFoundError, json.JSONDecodeError) as e:
        print(f"[blendshape2landmark] 无法加载 landmark 配置 {json_path}: {e}")
        return {}


def _build_clamp_ranges(lm_config: dict) -> dict[int, tuple[float, float]]:
    """
    从 landmarkID_info.json 构建 {landmark_id: (lower, upper)} clamp 表。
    线性 landmark 用 p_lower/p_upper，五连杆用 p_range。
    """
    ranges: dict[int, tuple[float, float]] = {}

    # 线性 landmark
    for lid_str, lm in lm_config.get('linear', {}).items():
        lid = int(lid_str)
        lo = lm.get('p_lower', 0.0)
        hi = lm.get('p_upper', 1.0)
        ranges[lid] = (min(lo, hi), max(lo, hi))

    # 五连杆 landmark (x 和 y 分别记录)
    for fb in lm_config.get('fivebar', {}).values():
        pr = fb.get('p_range', {})
        lx_id = int(fb.get('landmark_x_id', -1))
        ly_id = int(fb.get('landmark_y_id', -1))
        if lx_id >= 0:
            x_lo = pr.get('x_lower', -10.0)
            x_hi = pr.get('x_upper', 10.0)
            ranges[lx_id] = (min(x_lo, x_hi), max(x_lo, x_hi))
        if ly_id >= 0:
            y_lo = pr.get('y_lower', -10.0)
            y_hi = pr.get('y_upper', 10.0)
            ranges[ly_id] = (min(y_lo, y_hi), max(y_lo, y_hi))

    return ranges


# ---------------------------------------------------------------------------
#  模块级配置（节点启动时加载，支持热重载）
# ---------------------------------------------------------------------------

_bs_config: dict = {}
_lm_config: dict = {}
_neutral: list[float] = []
_landmark_ids_order: list[int] = []
_num_landmarks: int = 0
_clamp_ranges: dict[int, tuple[float, float]] = {}
_lid_to_idx: dict[int, int] = {}


def _refresh_module_config():
    """重新读取配置并刷新模块级变量。"""
    global _bs_config, _lm_config, _neutral, _landmark_ids_order
    global _num_landmarks, _clamp_ranges, _lid_to_idx

    _bs_config = load_blendshape_config()
    _lm_config = load_landmark_config()

    meta = _bs_config.get('meta', {})
    _landmark_ids_order = meta.get('landmark_ids_order', [])
    _num_landmarks = len(_landmark_ids_order)
    _lid_to_idx = {lid: i for i, lid in enumerate(_landmark_ids_order)}

    _neutral = list(_bs_config.get('neutral', [0.0] * _num_landmarks))
    if len(_neutral) != _num_landmarks:
        print(f"[blendshape2landmark] 警告: neutral 长度 {len(_neutral)} "
              f"!= landmark 数 {_num_landmarks}，将截断或补零")
        _neutral = (_neutral + [0.0] * _num_landmarks)[:_num_landmarks]

    _clamp_ranges = _build_clamp_ranges(_lm_config)

    bs_dict = _bs_config.get('blendshapes', {})
    print(f"[blendshape2landmark] 配置已加载: "
          f"{_num_landmarks} 个 landmark, "
          f"{len(bs_dict)} 个有效 blendshape 映射")


def reload_config() -> bool:
    """热重载配置，返回是否成功。"""
    try:
        new_bs = load_blendshape_config()
        new_lm = load_landmark_config()
        if not new_bs:
            return False

        global _bs_config, _lm_config, _neutral, _landmark_ids_order
        global _num_landmarks, _clamp_ranges, _lid_to_idx

        _bs_config = new_bs
        _lm_config = new_lm

        meta = _bs_config.get('meta', {})
        _landmark_ids_order = meta.get('landmark_ids_order', [])
        _num_landmarks = len(_landmark_ids_order)
        _lid_to_idx = {lid: i for i, lid in enumerate(_landmark_ids_order)}

        _neutral = list(_bs_config.get('neutral', [0.0] * _num_landmarks))
        if len(_neutral) != _num_landmarks:
            _neutral = (_neutral + [0.0] * _num_landmarks)[:_num_landmarks]

        _clamp_ranges = _build_clamp_ranges(_lm_config)
        print("[blendshape2landmark] 配置已热重载")
        return True
    except Exception as e:
        print(f"[blendshape2landmark] 热重载失败: {e}")
        return False


# ---------------------------------------------------------------------------
#  转换函数
# ---------------------------------------------------------------------------

def blendshapes_to_landmarks(
    bs_values: list[float],
    bs_config: dict,
    neutral: list[float],
    lid_to_idx: dict[int, int],
    clamp_ranges: dict[int, tuple[float, float]],
) -> list[float]:
    """
    批量转换: blendshape 值 (61 维) → landmark 位置 (25 维)。

    deltas 记录的是绝对目标值 (bs=1.0 时的最终位置)，
    公式: l = neutral + sum(bs[i] * scale[i] * (absolute[i] - neutral))
    """
    result = list(neutral)
    bs_dict = bs_config.get('blendshapes', {})

    for bs_id_str, bs_entry in bs_dict.items():
        bs_id = int(bs_id_str)
        if bs_id < 0 or bs_id >= len(bs_values):
            continue

        w = bs_values[bs_id] * bs_entry.get('scale', 1.0)
        if abs(w) < 1e-9:
            continue

        absolutes = bs_entry.get('deltas', {})
        for lid_str, abs_val in absolutes.items():
            lid = int(lid_str)
            idx = lid_to_idx.get(lid)
            if idx is not None and idx < len(result):
                result[idx] += w * (abs_val - neutral[idx])

    # clamp 到各 landmark 合法范围
    for lid, idx in lid_to_idx.items():
        if idx < len(result) and lid in clamp_ranges:
            lo, hi = clamp_ranges[lid]
            result[idx] = max(lo, min(hi, result[idx]))

    return result


# ---------------------------------------------------------------------------
#  ROS2 节点
# ---------------------------------------------------------------------------

class BlendshapeToLandmark(Node):
    """
    独立 ROS2 节点:
      sub  blendshape               (Float32MultiArray, 61 个值)
      pub  landmark_positions_batch  (Float32MultiArray, 25 个值)
    """

    def __init__(self):
        super().__init__('blendshape_to_landmark')

        # 加载配置
        _refresh_module_config()

        # 发布者: landmark 位置数组
        self.landmark_pub = self.create_publisher(
            Float32MultiArray, 'landmark_positions_batch', 10)

        # 订阅者: blendshape 值
        self.bs_sub = self.create_subscription(
            Float32MultiArray, 'blendshape_robot',
            self._blendshape_callback,
            rclpy.qos.qos_profile_sensor_data)

        # 配置重载定时器 (每 5 秒检查 mtime)
        self._last_mtime: float = 0.0
        self._reload_timer = self.create_timer(5.0, self._check_config_reload)

        self.get_logger().info(
            f'blendshape2landmark 节点已启动 | '
            f'blendshape: {_NUM_BLENDSHAPES} | '
            f'landmark: {_num_landmarks}')

    def _blendshape_callback(self, msg: Float32MultiArray):
        """收到 blendshape 值 → 转换 → 发布 landmark。"""
        bs_values = list(msg.data)
        if len(bs_values) != _NUM_BLENDSHAPES:
            self.get_logger().error(
                f'blendshape 长度 {len(bs_values)} != {_NUM_BLENDSHAPES}，跳过')
            return

        landmarks = blendshapes_to_landmarks(
            bs_values, _bs_config, _neutral, _lid_to_idx, _clamp_ranges)

        out_msg = Float32MultiArray()
        out_msg.data = landmarks
        self.landmark_pub.publish(out_msg)

        self.get_logger().debug(
            f'blendshape[{len(bs_values)}] → landmark[{len(landmarks)}]')

    def _check_config_reload(self):
        """定期检查配置文件 mtime，自动热重载。"""
        try:
            mtime = os.path.getmtime(_BS_CONFIG_PATH)
            if mtime > self._last_mtime:
                self._last_mtime = mtime
                if reload_config():
                    self.get_logger().info('blendshape 配置已自动热重载')
        except OSError:
            pass


# ---------------------------------------------------------------------------
#  入口
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = BlendshapeToLandmark()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
