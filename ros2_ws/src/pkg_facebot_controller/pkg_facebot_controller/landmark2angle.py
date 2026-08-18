#!/usr/bin/env python3
"""
landmark2angle — 独立 ROS2 节点
订阅 landmark_positions_batch (Float32MultiArray)，
计算舵机角度后发布 servo_positions_batch (Float32MultiArray)。

转换逻辑:
  - 线性映射 (一维):  landmark p 值 → 舵机角度 (0-4095)
  - 五连杆 (二维 IK): landmark (px, py) → 两个舵机角度

配置集中存储在 _doc/landmarkID_info.json
"""

import json
import math
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
_CONFIG_PATH = os.path.join(_DOC_DIR, 'landmarkID_info.json')

# 26 个舵机 ID（与 ESP32 固件顺序一致，复用 servo_config 约定）
SERVO_IDS = [
    1, 2, 3, 4, 5, 6, 7, 8,
    13, 14, 17, 18, 19,
    21, 22, 23, 24, 25, 26, 27,
    33, 34, 35, 41, 42, 43
]
_NUM_SERVOS = len(SERVO_IDS)          # 26
_SERVO_ID_TO_IDX = {sid: i for i, sid in enumerate(SERVO_IDS)}

DEFAULT_NEUTRAL = 2048


# ---------------------------------------------------------------------------
#  配置加载
# ---------------------------------------------------------------------------
def load_landmark_config(json_path: str = _CONFIG_PATH) -> dict:
    """加载 landmarkID_info.json，返回完整配置字典。"""
    try:
        with open(json_path, 'r') as f:
            return json.load(f)
    except (FileNotFoundError, json.JSONDecodeError) as e:
        print(f"[landmark2angle] 无法加载配置 {json_path}: {e}")
        return {}


def _get_landmark_ids_order(config: dict) -> list[int]:
    """从配置中提取 landmark ID 顺序列表。"""
    meta = config.get('meta', {})
    return meta.get('landmark_ids_order', [])


# 模块级配置（节点启动时加载，支持热重载）
_config: dict = {}
_LANDMARK_IDS: list[int] = []
_LANDMARK_ID_TO_IDX: dict[int, int] = {}


def _refresh_module_config():
    """重新读取配置并刷新模块级变量。"""
    global _config, _LANDMARK_IDS, _LANDMARK_ID_TO_IDX
    _config = load_landmark_config()
    _LANDMARK_IDS = _get_landmark_ids_order(_config)
    _LANDMARK_ID_TO_IDX = {lid: i for i, lid in enumerate(_LANDMARK_IDS)}
    print(f"[landmark2angle] 配置已加载: "
          f"{len(_LANDMARK_IDS)} 个 landmark, "
          f"{len(_config.get('linear', {}))} 个线性, "
          f"{len(_config.get('fivebar', {}))} 个五连杆")


def reload_landmark_config() -> bool:
    """热重载配置，返回是否成功。"""
    try:
        new_config = load_landmark_config()
        if not new_config:
            return False
        global _config, _LANDMARK_IDS, _LANDMARK_ID_TO_IDX
        _config = new_config
        _LANDMARK_IDS = _get_landmark_ids_order(_config)
        _LANDMARK_ID_TO_IDX = {lid: i for i, lid in enumerate(_LANDMARK_IDS)}
        print(f"[landmark2angle] 配置已热重载")
        return True
    except Exception as e:
        print(f"[landmark2angle] 热重载失败: {e}")
        return False


# ---------------------------------------------------------------------------
#  转换函数
# ---------------------------------------------------------------------------

def linear_landmark_to_angle(
    p: float,
    p_lower: float,
    p_upper: float,
    B_lower: int,
    B_upper: int,
) -> int:
    """
    一维线性映射: landmark p 值 → 舵机角度 (0-4095)

    angle = (B_upper - B_lower) / (p_upper - p_lower) * (p - p_lower) + B_lower
    结果 clamp 到 [min(B_lower,B_upper), max(B_lower,B_upper)]
    """
    if abs(p_upper - p_lower) < 1e-9:
        return int(B_lower)
    angle = (B_upper - B_lower) / (p_upper - p_lower) * (p - p_lower) + B_lower
    lo = min(B_lower, B_upper)
    hi = max(B_lower, B_upper)
    return int(max(lo, min(hi, round(angle))))


def rad_to_servo(rad: float) -> int:
    """
    弧度 → 舵机控制值 (0-4095)
    0 rad → 0, 2π rad → 4095
    """
    val = int((rad / (2 * math.pi)) * 4095) 
    return val #max(0, min(4095, val))


def fivebar_ik_theta1(
    xc: float, yc: float,
    xa1: float, ya1: float,
    l1: float, l3: float,
) -> float:
    """
    五连杆逆运动学 θ1（B1 在 A1→C 左侧，逆时针偏转）

    θ1 = atan2(yC-yA1, xC-xA1) + acos((|C-A1|² + l1² - l3²) / (2·l1·|C-A1|))
    返回弧度。
    """
    dx = xc - xa1
    dy = yc - ya1
    dist = math.hypot(dx, dy)
    if dist < 1e-9 or l1 < 1e-9:
        return 0.0
    cos_arg = (dist * dist + l1 * l1 - l3 * l3) / (2 * l1 * dist)
    cos_arg = max(-1.0, min(1.0, cos_arg))  # 数值安全 clamp
    return math.atan2(dy, dx) + math.acos(cos_arg)


def fivebar_ik_theta2(
    xc: float, yc: float,
    xa2: float, ya2: float,
    l2: float, l4: float,
) -> float:
    """
    五连杆逆运动学 θ2（B2 在 A2→C 左侧，顺时针偏转）

    θ2 = atan2(yC-yA2, xC-xA2) - acos((|C-A2|² + l2² - l4²) / (2·l2·|C-A2|))
    返回弧度。
    """
    dx = xc - xa2
    dy = yc - ya2
    dist = math.hypot(dx, dy)
    if dist < 1e-9 or l2 < 1e-9:
        return 0.0
    cos_arg = (dist * dist + l2 * l2 - l4 * l4) / (2 * l2 * dist)
    cos_arg = max(-1.0, min(1.0, cos_arg))  # 数值安全 clamp
    return math.atan2(dy, dx) - math.acos(cos_arg)


def fivebar_landmark_to_angles(
    px: float,
    py: float,
    fb_config: dict,
) -> list[tuple[int, int]]:
    """
    五连杆 landmark (px, py) → 舵机角度列表。

    处理流程:
      1. coord_map: (px, py) → (xc, yc)  物理坐标 (mm)
      2. IK: (xc, yc) → (theta1, theta2) 弧度
      3. rad_to_servo: 弧度 → 0-4095

    返回: [(servo_id, servo_angle), ...]
      - 普通五连杆: [(servo_id_1, angle1), (servo_id_2, angle2)]
      - 下巴五连杆: [(servo_id_1, angle1), (servo_id_2a, angle2),
                     (servo_id_2b, 4095-angle2)]
        Jaw_Right 与 Jaw_Left 取相反角度保持同步。
    """
    cm = fb_config.get('coord_map', {})
    xc = cm.get('x_offset', 0) + cm.get('x_scale', 1) * px
    yc = cm.get('y_offset', 0) + cm.get('y_scale', 1) * py

    A1 = fb_config.get('A1', [0, 0])
    A2 = fb_config.get('A2', [0, 0])
    l1 = fb_config.get('l1', 0)
    l2 = fb_config.get('l2', 0)
    l3 = fb_config.get('l3', 0)
    l4 = fb_config.get('l4', 0)

    theta1 = fivebar_ik_theta1(xc, yc, A1[0], A1[1], l1, l3)
    theta2 = fivebar_ik_theta2(xc, yc, A2[0], A2[1], l2, l4)
    
    angle1 = rad_to_servo(theta1)
    angle2 = rad_to_servo(theta2)
    
    servo_id_1 = fb_config.get('servo_id_1')

    # 下巴: servo_id_2 拆分为 2a/2b，取相反角度
    if 'servo_id_2a' in fb_config and 'servo_id_2b' in fb_config:
        return [
            (servo_id_1, angle1),
            (fb_config['servo_id_2a'], angle2),
            (fb_config['servo_id_2b'], angle2),
        ]
    else:
        servo_id_2 = fb_config.get('servo_id_2')
        return [
            (servo_id_1, angle1),
            (servo_id_2, angle2),
        ]


def landmarks_to_servo_positions(
    landmark_array: list[float],
    config: dict,
) -> list[float]:
    """
    批量转换: landmark 位置数组 → 26 个舵机位置值。

    landmark_array: 按 meta.landmark_ids_order 顺序排列的 p 值数组。
    返回: 按 SERVO_IDS 顺序排列的 26 个 float 值 (0-4095)。
    未涉及的舵机保持 DEFAULT_NEUTRAL (2048)。
    """
    landmark_ids_order = _get_landmark_ids_order(config)
    id_to_idx = {lid: i for i, lid in enumerate(landmark_ids_order)}
    linear_cfg = config.get('linear', {})
    fivebar_cfg = config.get('fivebar', {})

    # 初始化所有舵机为 neutral
    servo_positions = [float(DEFAULT_NEUTRAL)] * _NUM_SERVOS

    # --- 线性映射 ---
    for lid_str, lm in linear_cfg.items():
        lid = int(lid_str)
        if lid not in id_to_idx:
            continue
        arr_idx = id_to_idx[lid]
        if arr_idx >= len(landmark_array):
            continue
        p = landmark_array[arr_idx]
        angle = linear_landmark_to_angle(
            p,
            lm.get('p_lower', 0.0),
            lm.get('p_upper', 1.0),
            lm.get('B_lower', 0),
            lm.get('B_upper', 4095),
        )
        servo_id = lm.get('servo_id')
        if servo_id is not None and servo_id in _SERVO_ID_TO_IDX:
            servo_positions[_SERVO_ID_TO_IDX[servo_id]] = float(angle)

    # --- 五连杆映射 ---
    for fb_name, fb in fivebar_cfg.items():
        lx_id_str = fb.get('landmark_x_id')
        ly_id_str = fb.get('landmark_y_id')
        if lx_id_str is None or ly_id_str is None:
            continue
        lx_id = int(lx_id_str)
        ly_id = int(ly_id_str)
        if lx_id not in id_to_idx or ly_id not in id_to_idx:
            continue
        lx_idx = id_to_idx[lx_id]
        ly_idx = id_to_idx[ly_id]
        if lx_idx >= len(landmark_array) or ly_idx >= len(landmark_array):
            continue
        px = landmark_array[lx_idx]
        py = landmark_array[ly_idx]
        angle_pairs = fivebar_landmark_to_angles(px, py, fb)
        # 后处理: 从 fivebar 配置读取 direction_factor 和 bias
        # final_angle = angle * direction_factor + bias
        dir_1  = fb.get('servo_id_1_direction_factor', 1)
        dir_2  = fb.get('servo_id_2_direction_factor', 1)
        dir_2a = fb.get('servo_id_2a_direction_factor', 1)
        dir_2b = fb.get('servo_id_2b_direction_factor', 1)
        bias_1  = fb.get('servo_id_1_bias', 0)
        bias_2  = fb.get('servo_id_2_bias', 0)
        bias_2a = fb.get('servo_id_2a_bias', 0)
        bias_2b = fb.get('servo_id_2b_bias', 0)

        for servo_id, angle in angle_pairs:
            if servo_id is not None and servo_id in _SERVO_ID_TO_IDX:
                if servo_id == fb.get('servo_id_1'):
                    final_angle = (angle * dir_1 + bias_1)
                elif servo_id == fb.get('servo_id_2'):
                    final_angle = (angle * dir_2 + bias_2)
                elif servo_id == fb.get('servo_id_2a'):
                    final_angle = (angle * dir_2a + bias_2a)
                elif servo_id == fb.get('servo_id_2b'):
                    final_angle = (angle * dir_2b + bias_2b)
                else:
                    final_angle = angle
                servo_positions[_SERVO_ID_TO_IDX[servo_id]] = float(final_angle)

    return servo_positions


# ---------------------------------------------------------------------------
#  ROS2 节点
# ---------------------------------------------------------------------------

class LandmarkToAngle(Node):
    """
    独立 ROS2 节点:
      sub  landmark_positions_batch (Float32MultiArray)
      pub  servo_positions_batch    (Float32MultiArray, 26 个值)
    """

    def __init__(self):
        super().__init__('landmark_to_angle')

        # 加载配置
        _refresh_module_config()

        # 发布者: 26 个舵机角度
        self.servo_pub = self.create_publisher(
            Float32MultiArray, 'servo_positions_batch', 10)

        # 订阅者: landmark 位置数组
        self.landmark_sub = self.create_subscription(
            Float32MultiArray, 'landmark_positions_batch',
            self._landmark_callback, 10)

        # 配置重载定时器 (每 5 秒检查一次文件 mtime)
        self._last_mtime: float = 0.0
        self._reload_timer = self.create_timer(5.0, self._check_config_reload)

        self.get_logger().info(
            f'landmark2angle 节点已启动 | '
            f'landmark 数: {len(_LANDMARK_IDS)} | '
            f'舵机数: {_NUM_SERVOS}')

    # --- 回调 ---

    def _landmark_callback(self, msg: Float32MultiArray):
        """收到 landmark 位置 → 转换 → 发布舵机角度。"""
        landmark_array = list(msg.data)
        servo_positions = landmarks_to_servo_positions(landmark_array, _config)

        out_msg = Float32MultiArray()
        out_msg.data = servo_positions
        self.servo_pub.publish(out_msg)
        self.get_logger().debug(
            f'landmark[{len(landmark_array)}] → servo[{len(servo_positions)}]')

    def _check_config_reload(self):
        """定期检查配置文件是否被修改，自动热重载。"""
        try:
            mtime = os.path.getmtime(_CONFIG_PATH)
            if mtime > self._last_mtime:
                self._last_mtime = mtime
                if reload_landmark_config():
                    self.get_logger().info('配置文件已自动热重载')
        except OSError:
            pass


# ---------------------------------------------------------------------------
#  入口
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = LandmarkToAngle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
