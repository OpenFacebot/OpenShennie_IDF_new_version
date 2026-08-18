"""
26 个 DS-S009 舵机配置
从 servoID_constrain.json 动态加载约束值
顺序与 ESP32 BATCH_SERVO_IDS 一致:
[1, 2, 3, 4, 5, 6, 7, 8, 13, 14, 17, 18, 19, 21, 22, 23, 24, 25, 26, 27, 33, 34, 35, 41, 42, 43]
"""
import json
import os

# 默认值 (0-4095)
DEFAULT_MIN = 0
DEFAULT_MAX = 4095
DEFAULT_NEUTRAL = 2048

# 舵机 ID 顺序 (与 ESP32 固件一致)
SERVO_IDS = [1, 2, 3, 4, 5, 6, 7, 8, 13, 14, 17, 18, 19,
             21, 22, 23, 24, 25, 26, 27, 33, 34, 35, 41, 42, 43]

# 加载 servoID_constrain.json
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
_json_path = os.path.join(_DOC_DIR, 'servoID_constrain.json')

try:
    with open(_json_path, 'r') as f:
        _constrain = json.load(f)
except (FileNotFoundError, json.JSONDecodeError) as e:
    print(f"[servo_config] 警告: 无法加载 {_json_path}, 使用默认值 ({e})")
    _constrain = {}


def _build_servo_list():
    """从 JSON 约束文件构建舵机列表，null 字段使用默认值"""
    result = []
    for sid in SERVO_IDS:
        key = f"{sid:02d}"
        entry = _constrain.get(key, {})
        name_cn = entry.get("name_cn", f"舵机{sid}")
        name_en = entry.get("name_en", f"servo_{sid}")
        angle_min = entry.get("angle_min")
        angle_max = entry.get("angle_max")
        angle_neutral = entry.get("angle_neutral")
        result.append((
            sid, name_cn, name_en,
            angle_min if angle_min is not None else DEFAULT_MIN,
            angle_max if angle_max is not None else DEFAULT_MAX,
            angle_neutral if angle_neutral is not None else DEFAULT_NEUTRAL,
        ))
    return result


SERVO_LIST = _build_servo_list()

# 便于查找: {id: (name_cn, name_en, angle_min, angle_max, angle_neutral)}
SERVO_MAP = {s[0]: s[1:] for s in SERVO_LIST}


def reload_constrain():
    """重新读取 servoID_constrain.json 并原地更新 SERVO_LIST / SERVO_MAP"""
    global SERVO_MAP
    try:
        with open(_json_path, 'r') as f:
            new_constrain = json.load(f)
    except (FileNotFoundError, json.JSONDecodeError) as e:
        print(f"[servo_config] 重新加载失败: {e}")
        return False

    for i, sid in enumerate(SERVO_IDS):
        key = f"{sid:02d}"
        entry = new_constrain.get(key, {})
        angle_min = entry.get("angle_min")
        angle_max = entry.get("angle_max")
        angle_neutral = entry.get("angle_neutral")
        SERVO_LIST[i] = (
            sid,
            entry.get("name_cn", SERVO_LIST[i][1]),
            entry.get("name_en", SERVO_LIST[i][2]),
            angle_min if angle_min is not None else DEFAULT_MIN,
            angle_max if angle_max is not None else DEFAULT_MAX,
            angle_neutral if angle_neutral is not None else DEFAULT_NEUTRAL,
        )

    SERVO_MAP = {s[0]: s[1:] for s in SERVO_LIST}
    print(f"[servo_config] 已从 {_json_path} 重新加载约束")
    return True
