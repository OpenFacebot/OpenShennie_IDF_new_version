#!/usr/bin/env python3
"""
controller_blendshape_GUI — Blendshape 表情控制 GUI (ROS2 节点)

通过滑块控制 61 个 ARKit blendshape 的值 (0.0-1.0)，
以 Float32MultiArray 发布到 blendshape topic，
由 blendshape2landmark 节点转换为 landmark 位置。

用法: ros2 run pkg_facebot_controller controller_blendshape_GUI
"""

import json
import os
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import tkinter as tk


# ---------------------------------------------------------------------------
#  常量
# ---------------------------------------------------------------------------
_DOC_DIR = '/home/user/Documents/openShennie_IDF/_doc'
_BS_CONFIG_PATH = os.path.join(_DOC_DIR, 'blendshapeID_info.json')

_NUM_BLENDSHAPES = 61

# 完整 blendshape 名称表 (ID → (英文名, 中文名))
BLENDSHAPE_NAMES: dict[int, tuple[str, str]] = {
    0:  ("EyeBlinkLeft",          "左眼眨眼"),
    1:  ("EyeLookDownLeft",       "左眼下看"),
    2:  ("EyeLookInLeft",         "左眼内看"),
    3:  ("EyeLookOutLeft",        "左眼外看"),
    4:  ("EyeLookUpLeft",         "左眼看上"),
    5:  ("EyeSquintLeft",         "左眼眯眼"),
    6:  ("EyeWideLeft",           "左眼睁大"),
    7:  ("EyeBlinkRight",         "右眼眨眼"),
    8:  ("EyeLookDownRight",      "右眼下看"),
    9:  ("EyeLookInRight",        "右眼内看"),
    10: ("EyeLookOutRight",       "右眼外看"),
    11: ("EyeLookUpRight",        "右眼看上"),
    12: ("EyeSquintRight",        "右眼眯眼"),
    13: ("EyeWideRight",          "右眼睁大"),
    14: ("JawForward",            "下巴前伸"),
    15: ("JawLeft",               "下巴左移"),
    16: ("JawRight",              "下巴右移"),
    17: ("JawOpen",               "下巴张开"),
    18: ("MouthClose",            "嘴唇闭合"),
    19: ("MouthFunnel",           "嘴唇漏斗"),
    20: ("MouthPucker",           "嘴唇撅起"),
    21: ("MouthLeft",             "嘴左移"),
    22: ("MouthRight",            "嘴右移"),
    23: ("MouthSmileRight",       "右微笑"),
    24: ("MouthSmileLeft",        "左微笑"),
    25: ("MouthFrownLeft",        "左苦脸"),
    26: ("MouthFrownRight",       "右苦脸"),
    27: ("MouthDimpleLeft",       "左酒窝"),
    28: ("MouthDimpleRight",      "右酒窝"),
    29: ("MouthStretchLeft",      "左拉伸"),
    30: ("MouthStretchRight",     "右拉伸"),
    31: ("MouthRollLower",        "下唇内卷"),
    32: ("MouthRollUpper",        "上唇内卷"),
    33: ("MouthShrugLower",       "下唇耸起"),
    34: ("MouthShrugUpper",       "上唇耸起"),
    35: ("MouthPressLeft",        "左抿嘴"),
    36: ("MouthPressRight",       "右抿嘴"),
    37: ("MouthLowerDownLeft",    "左下唇下拉"),
    38: ("MouthLowerDownRight",   "右下唇下拉"),
    39: ("MouthUpperUpLeft",      "左上唇上提"),
    40: ("MouthUpperUpRight",     "右上唇上提"),
    41: ("BrowDownLeft",          "左眉下压"),
    42: ("BrowDownRight",         "右眉下压"),
    43: ("BrowInnerUp",           "内眉上抬"),
    44: ("BrowOuterUpLeft",       "左外眉上抬"),
    45: ("BrowOuterUpRight",      "右外眉上抬"),
    46: ("CheekPuff",             "鼓腮"),
    47: ("CheekSquintLeft",       "左颧眯眼"),
    48: ("CheekSquintRight",      "右颧眯眼"),
    49: ("NoseSneerLeft",         "左鼻冷笑"),
    50: ("NoseSneerRight",        "右鼻冷笑"),
    51: ("TongueOut",             "吐舌"),
    52: ("HeadYaw",               "头偏转"),
    53: ("HeadPitch",             "头俯仰"),
    54: ("HeadRoll",              "头侧倾"),
    55: ("LeftEyeYaw",            "左眼偏转"),
    56: ("LeftEyePitch",          "左眼俯仰"),
    57: ("LeftEyeRoll",           "左眼翻滚"),
    58: ("RightEyeYaw",           "右眼偏转"),
    59: ("RightEyePitch",         "右眼俯仰"),
    60: ("RightEyeRoll",          "右眼翻滚"),
}

# GUI 分组定义: (组名, [blendshape_id, ...])
LEFT_GROUPS = [
    ("左眼",    list(range(0, 7))),
    ("右眼",    list(range(7, 14))),
    ("眉毛",    list(range(41, 46))),
    ("头部",    list(range(52, 55))),
]

RIGHT_GROUPS = [
    ("下颌",    list(range(14, 18))),
    ("嘴部",    list(range(18, 41))),
    ("脸颊/鼻/舌", list(range(46, 52))),
    ("眼球注视", list(range(55, 61))),
]


# ---------------------------------------------------------------------------
#  配置加载
# ---------------------------------------------------------------------------

def load_bs_config(json_path: str = _BS_CONFIG_PATH) -> dict:
    """加载 blendshapeID_info.json。"""
    try:
        with open(json_path, 'r') as f:
            return json.load(f)
    except (FileNotFoundError, json.JSONDecodeError) as e:
        print(f"[controller_blendshape_GUI] 无法加载配置 {json_path}: {e}")
        return {}


# ---------------------------------------------------------------------------
#  ROS2 节点
# ---------------------------------------------------------------------------

class BlendshapeGuiNode(Node):
    """发布 blendshape 值，由 blendshape2landmark 节点转换为 landmark。"""

    def __init__(self, config: dict):
        super().__init__('blendshape_gui')

        self.config = config

        # 发布者
        self.bs_pub = self.create_publisher(
            Float32MultiArray, 'blendshape', 10)

        # 当前所有 blendshape 值 (61 个, 默认 0.0)
        self.bs_values: list[float] = [0.0] * _NUM_BLENDSHAPES

        # GUI 变量引用 (在 create_gui 中填充)
        self.double_vars: list[tk.DoubleVar | None] = []
        self.auto_send = True

    def publish_blendshapes(self):
        """发布当前所有 blendshape 值。"""
        msg = Float32MultiArray()
        msg.data = [float(v) for v in self.bs_values]
        self.bs_pub.publish(msg)

    def publish_blendshapes_manual(self):
        """手动发送（带终端打印）。"""
        t0 = time.time()
        self.publish_blendshapes()
        elapsed_ms = (time.time() - t0) * 1000
        self.get_logger().info(
            f'[发送全部] 已发布 {_NUM_BLENDSHAPES} 个 blendshape | '
            f'耗时 {elapsed_ms:.2f}ms')
        print(f'[发送全部] 已发布 {_NUM_BLENDSHAPES} 个 blendshape | '
              f'耗时 {elapsed_ms:.2f}ms', flush=True)

    def all_to_zero(self):
        """所有 blendshape 归零。"""
        for i in range(_NUM_BLENDSHAPES):
            self.bs_values[i] = 0.0
            if i < len(self.double_vars) and self.double_vars[i] is not None:
                self.double_vars[i].set(0.0)
        self.publish_blendshapes()
        self.get_logger().info('全部归零')

    def reload_config(self):
        """热重载 blendshapeID_info.json。"""
        new_config = load_bs_config()
        if not new_config:
            self.get_logger().warn('加载配置失败')
            print('[加载配置] 加载失败', flush=True)
            return
        self.config = new_config
        self.get_logger().info('配置已重载')
        print('[加载配置] 已重载', flush=True)

    def on_slider_change(self, idx: int, val: float):
        """slider 回调：更新 bs_values。"""
        self.bs_values[idx] = float(val)


# ---------------------------------------------------------------------------
#  GUI 构建
# ---------------------------------------------------------------------------

def _build_bs_row(
    parent: tk.Frame,
    node: BlendshapeGuiNode,
    bs_id: int,
):
    """构建单个 blendshape 行: #ID 名称 [slider] 数值"""
    frame = tk.Frame(parent)
    frame.pack(fill=tk.X, padx=3, pady=1)

    name_en, name_cn = BLENDSHAPE_NAMES.get(bs_id, (f"bs_{bs_id}", f"bs_{bs_id}"))

    # 标签: #ID 中文名 (英文名)
    tk.Label(
        frame, text=f"#{bs_id:02d} {name_cn} ({name_en})", width=28,
        anchor='w', font=('TkDefaultFont', 9)
    ).pack(side=tk.LEFT)

    # slider (DoubleVar)
    var = tk.DoubleVar(value=0.0)
    node.double_vars[bs_id] = var

    scale = tk.Scale(
        frame, from_=0.0, to=1.0,
        orient=tk.HORIZONTAL, variable=var,
        length=160, resolution=0.01,
        tickinterval=0, showvalue=0,
        command=lambda val, idx=bs_id: node.on_slider_change(idx, float(val))
    )
    scale.pack(side=tk.LEFT)

    # 数值标签
    lbl = tk.Label(
        frame, text="0.00",
        width=6, anchor='w', font=('TkDefaultFont', 9)
    )
    lbl.pack(side=tk.LEFT, padx=2)

    # 实时更新标签
    def make_updater(var=var, label=lbl):
        def _update(*_):
            label.config(text=f"{var.get():.2f}")
        return _update

    var.trace_add('write', make_updater())


def create_gui(node: BlendshapeGuiNode) -> tk.Tk:
    root = tk.Tk()
    root.title("Blendshape 表情控制器")

    # 预分配 DoubleVar 列表
    node.double_vars = [None] * _NUM_BLENDSHAPES

    # --- 顶部按钮区 ---
    top = tk.Frame(root)
    top.pack(pady=4)
    tk.Button(
        top, text="全部归零", bg="lightblue",
        command=node.all_to_zero
    ).pack(side=tk.LEFT, padx=5)
    tk.Button(
        top, text="发送全部", bg="lightgreen",
        command=node.publish_blendshapes_manual
    ).pack(side=tk.LEFT, padx=5)
    tk.Button(
        top, text="加载配置", bg="orange",
        command=node.reload_config
    ).pack(side=tk.LEFT, padx=5)

    auto_btn = tk.Button(
        top, text="自动发送: ON", bg="lime green",
        fg="black", font=('TkDefaultFont', 10, 'bold'), width=14
    )

    def toggle_auto():
        node.auto_send = not node.auto_send
        if node.auto_send:
            auto_btn.config(text="自动发送: ON", bg="lime green", fg="black")
        else:
            auto_btn.config(text="自动发送: OFF", bg="lightgray", fg="black")

    auto_btn.config(command=toggle_auto)
    auto_btn.pack(side=tk.LEFT, padx=5)

    # 分隔线
    tk.Frame(root, height=2, bg='gray').pack(fill=tk.X, padx=5, pady=2)

    # --- 可滚动主体区域 ---
    canvas_frame = tk.Frame(root)
    canvas_frame.pack(fill=tk.BOTH, expand=True)
    canvas = tk.Canvas(canvas_frame, highlightthickness=0)
    scrollbar = tk.Scrollbar(canvas_frame, orient=tk.VERTICAL, command=canvas.yview)
    canvas.configure(yscrollcommand=scrollbar.set)
    scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
    canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

    content = tk.Frame(canvas)
    canvas_win = canvas.create_window((0, 0), window=content, anchor='nw')

    def _on_content_config(event):
        canvas.configure(scrollregion=canvas.bbox('all'))

    def _on_canvas_config(event):
        canvas.itemconfig(canvas_win, width=event.width)

    content.bind('<Configure>', _on_content_config)
    canvas.bind('<Configure>', _on_canvas_config)

    # 鼠标滚轮
    canvas.bind_all('<MouseWheel>',
                    lambda e: canvas.yview_scroll(int(-1 * (e.delta / 120)), 'units'))
    canvas.bind_all('<Button-4>', lambda e: canvas.yview_scroll(-1, 'units'))
    canvas.bind_all('<Button-5>', lambda e: canvas.yview_scroll(1, 'units'))

    # --- 双列布局 ---
    columns = tk.Frame(content)
    columns.pack(fill=tk.BOTH, expand=True)

    left_col = tk.Frame(columns)
    left_col.grid(row=0, column=0, sticky='n', padx=5, pady=2)
    right_col = tk.Frame(columns)
    right_col.grid(row=0, column=1, sticky='n', padx=5, pady=2)

    # ---- 左列 ----
    for group_name, bs_ids in LEFT_GROUPS:
        tk.Label(
            left_col, text=f"--- {group_name} ---",
            font=('TkDefaultFont', 9, 'bold')
        ).pack(pady=(6, 2))
        for bs_id in bs_ids:
            _build_bs_row(left_col, node, bs_id)

    # ---- 右列 ----
    for group_name, bs_ids in RIGHT_GROUPS:
        tk.Label(
            right_col, text=f"--- {group_name} ---",
            font=('TkDefaultFont', 9, 'bold')
        ).pack(pady=(6, 2))
        for bs_id in bs_ids:
            _build_bs_row(right_col, node, bs_id)

    root.geometry("780x520")
    return root


# ---------------------------------------------------------------------------
#  入口
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)

    config = load_bs_config()
    node = BlendshapeGuiNode(config)
    root = create_gui(node)

    def spin_once():
        rclpy.spin_once(node, timeout_sec=0.05)
        root.after(50, spin_once)

    spin_once()

    def auto_send_tick():
        """20Hz 无条件自动发送。"""
        if node.auto_send:
            node.publish_blendshapes()
        root.after(20, auto_send_tick)

    auto_send_tick()
    root.mainloop()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
