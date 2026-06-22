#!/usr/bin/env python3
"""
controller_landmark_GUI — Landmark 位点控制 GUI (ROS2 节点)

通过滑块控制各面部 landmark 位点的位置值 (p)，
以 Float32MultiArray 发布到 landmark_positions_batch topic，
由 landmark2angle 节点转换为舵机角度。

用法: ros2 run pkg_facebot_controller controller_landmark_GUI
"""

import json
import os
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import tkinter as tk

from .landmark2angle import (
    load_landmark_config,
    _get_landmark_ids_order,
    linear_landmark_to_angle,
)


# ---------------------------------------------------------------------------
#  常量
# ---------------------------------------------------------------------------
_DOC_DIR = '/home/user/Documents/openShennie_IDF/_doc'
_CONFIG_PATH = os.path.join(_DOC_DIR, 'landmarkID_info.json')


# ---------------------------------------------------------------------------
#  GUI 分组定义（硬编码布局顺序，从配置中读取参数）
# ---------------------------------------------------------------------------

# 线性 landmark 分组: (组名, [landmark_id, ...])
LINEAR_GROUPS = [
    ("眉毛 & 眼睑", [1, 2, 3, 4, 5, 6, 7, 8]),
    ("眼球",        [13, 14]),
    ("嘴唇",        [25, 26, 27]),
    ("中庭",        [33, 34, 35]),
    ("脖子",        [41, 42, 43]),
]

# 五连杆 landmark 分组: (组名, fb_config_key)
FIVEBAR_GROUPS = [
    ("五连杆 - 右嘴角", "right_mouth"),
    ("五连杆 - 左嘴角", "left_mouth"),
    ("五连杆 - 下巴",   "jaw"),
]


# ---------------------------------------------------------------------------
#  工具函数
# ---------------------------------------------------------------------------

def _build_linear_lookup(config: dict) -> dict[int, dict]:
    """构建 {landmark_id: config_entry} 快速查找表。"""
    return {int(k): v for k, v in config.get('linear', {}).items()}


def _fmt_p(p: float, p_lower: float) -> str:
    """格式化 p 值显示。"""
    return f"{p:+.2f}" if p_lower < 0 else f"{p:.2f}"


def _fmt_p_with_angle(p: float, lm_cfg: dict) -> str:
    """格式化 p 值 + 预估舵机角度（仅线性 landmark）。"""
    p_str = _fmt_p(p, lm_cfg.get('p_lower', 0.0))
    angle = linear_landmark_to_angle(
        p,
        lm_cfg.get('p_lower', 0.0),
        lm_cfg.get('p_upper', 1.0),
        lm_cfg.get('B_lower', 0),
        lm_cfg.get('B_upper', 4095),
    )
    deg = angle * 360.0 / 4095
    return f"{p_str}  (→ {angle:4d} / {deg:5.1f}°)"


# ---------------------------------------------------------------------------
#  ROS2 节点
# ---------------------------------------------------------------------------

class LandmarkGuiNode(Node):
    """发布 landmark 位置值，由 landmark2angle 节点转换为舵机角度。"""

    def __init__(self, config: dict):
        super().__init__('landmark_gui')

        self.config = config
        self.landmark_ids = _get_landmark_ids_order(config)
        self.num_landmarks = len(self.landmark_ids)

        # 发布者
        self.landmark_pub = self.create_publisher(
            Float32MultiArray, 'landmark_positions_batch', 10)

        # 当前所有 landmark 的 p 值（按 landmark_ids 顺序）
        self.landmark_values: list[float] = []
        for lid in self.landmark_ids:
            lid_str = f"{lid:02d}"
            linear_cfg = config.get('linear', {}).get(lid_str)
            if linear_cfg:
                self.landmark_values.append(linear_cfg.get('p_neutral', 0.5))
            else:
                # 五连杆 landmark，从 fivebar 配置中查找
                p_neutral = 0.0
                for fb in config.get('fivebar', {}).values():
                    if int(fb.get('landmark_x_id', -1)) == lid:
                        p_neutral = fb.get('p_range', {}).get('x_neutral', 0.0)
                        break
                    if int(fb.get('landmark_y_id', -1)) == lid:
                        p_neutral = fb.get('p_range', {}).get('y_neutral', 0.0)
                        break
                self.landmark_values.append(p_neutral)

        # GUI 变量引用（在 create_gui 中填充）
        self.double_vars: list[tk.DoubleVar] = []
        self.auto_send = True

    def publish_landmarks(self):
        """发布当前所有 landmark 值。"""
        msg = Float32MultiArray()
        msg.data = [float(v) for v in self.landmark_values]
        self.landmark_pub.publish(msg)

    def publish_landmarks_manual(self):
        """手动发送（带终端打印）。"""
        t0 = time.time()
        self.publish_landmarks()
        elapsed_ms = (time.time() - t0) * 1000
        self.get_logger().info(
            f'[发送全部] 已发布 {self.num_landmarks} 个 landmark | '
            f'耗时 {elapsed_ms:.2f}ms')
        print(f'[发送全部] 已发布 {self.num_landmarks} 个 landmark | '
              f'耗时 {elapsed_ms:.2f}ms', flush=True)

    def all_to_neutral(self):
        """所有 landmark 回到 p_neutral。"""
        for i, lid in enumerate(self.landmark_ids):
            lid_str = f"{lid:02d}"
            linear_cfg = self.config.get('linear', {}).get(lid_str)
            if linear_cfg:
                pn = linear_cfg.get('p_neutral', 0.5)
            else:
                pn = 0.0
                for fb in self.config.get('fivebar', {}).values():
                    if int(fb.get('landmark_x_id', -1)) == lid:
                        pn = fb.get('p_range', {}).get('x_neutral', 0.0)
                        break
                    if int(fb.get('landmark_y_id', -1)) == lid:
                        pn = fb.get('p_range', {}).get('y_neutral', 0.0)
                        break
            self.landmark_values[i] = pn
            if i < len(self.double_vars) and self.double_vars[i] is not None:
                self.double_vars[i].set(pn)
        self.publish_landmarks()
        self.get_logger().info('全部归中')

    def reload_config(self):
        """热重载 landmarkID_info.json。"""
        new_config = load_landmark_config(_CONFIG_PATH)
        if not new_config:
            self.get_logger().warn('加载配置失败')
            print('[加载配置] 加载失败', flush=True)
            return
        self.config = new_config
        self.get_logger().info('配置已重载（需重启 GUI 以更新布局）')
        print('[加载配置] 已重载，布局将在下次启动时更新', flush=True)

    def on_slider_change(self, idx: int, val: float):
        """slider 回调：更新 landmark_values。"""
        self.landmark_values[idx] = float(val)


# ---------------------------------------------------------------------------
#  GUI 构建
# ---------------------------------------------------------------------------

def _build_linear_row(
    parent: tk.Frame,
    node: LandmarkGuiNode,
    arr_idx: int,
    lid: int,
    lm_cfg: dict,
):
    """构建单个线性 landmark 行: #ID 名称 [slider] 数值(角度)"""
    frame = tk.Frame(parent)
    frame.pack(fill=tk.X, padx=3, pady=1)

    p_lower = lm_cfg.get('p_lower', 0.0)
    p_upper = lm_cfg.get('p_upper', 1.0)
    p_neutral = lm_cfg.get('p_neutral', 0.5)
    name_cn = lm_cfg.get('name_cn', f'landmark_{lid}')

    # 标签: #ID 名称
    tk.Label(
        frame, text=f"#{lid:02d} {name_cn}", width=16,
        anchor='w', font=('TkDefaultFont', 9)
    ).pack(side=tk.LEFT)

    # slider (DoubleVar)
    var = tk.DoubleVar(value=p_neutral)
    node.double_vars[arr_idx] = var

    resolution = 0.01
    scale = tk.Scale(
        frame, from_=p_lower, to=p_upper,
        orient=tk.HORIZONTAL, variable=var,
        length=180, resolution=resolution,
        tickinterval=0, showvalue=0,
        command=lambda val, idx=arr_idx: node.on_slider_change(idx, float(val))
    )
    scale.pack(side=tk.LEFT)

    # 数值标签（显示 p 值 + 预估角度）
    lbl = tk.Label(
        frame, text=_fmt_p_with_angle(p_neutral, lm_cfg),
        width=28, anchor='w', font=('TkDefaultFont', 9)
    )
    lbl.pack(side=tk.LEFT, padx=2)

    # 实时更新标签
    def make_updater(var=var, label=lbl, cfg=lm_cfg):
        def _update(*_):
            label.config(text=_fmt_p_with_angle(var.get(), cfg))
        return _update

    var.trace_add('write', make_updater())


def _build_fivebar_row(
    parent: tk.Frame,
    node: LandmarkGuiNode,
    arr_idx_x: int,
    arr_idx_y: int,
    fb_name: str,
    fb_cfg: dict,
):
    """构建五连杆 landmark 行（X/Y 双 slider）。"""
    p_range = fb_cfg.get('p_range', {})
    x_lower = p_range.get('x_lower', -1.0)
    x_upper = p_range.get('x_upper', 1.0)
    y_lower = p_range.get('y_lower', -1.0)
    y_upper = p_range.get('y_upper', 1.0)
    x_neutral = p_range.get('x_neutral', 0.0)
    y_neutral = p_range.get('y_neutral', 0.0)

    display_name = fb_cfg.get('name_cn', fb_name)

    # --- X 行 ---
    fx = tk.Frame(parent)
    fx.pack(fill=tk.X, padx=3, pady=1)
    tk.Label(
        fx, text=f"{display_name} X", width=16,
        anchor='w', font=('TkDefaultFont', 9)
    ).pack(side=tk.LEFT)

    var_x = tk.DoubleVar(value=x_neutral)
    node.double_vars[arr_idx_x] = var_x
    tk.Scale(
        fx, from_=x_lower, to=x_upper,
        orient=tk.HORIZONTAL, variable=var_x,
        length=180, resolution=0.01,
        tickinterval=0, showvalue=0,
        command=lambda val, idx=arr_idx_x: node.on_slider_change(idx, float(val))
    ).pack(side=tk.LEFT)

    lbl_x = tk.Label(
        fx, text=_fmt_p(x_neutral, x_lower),
        width=10, anchor='w', font=('TkDefaultFont', 9)
    )
    lbl_x.pack(side=tk.LEFT, padx=2)

    def make_x_updater(var=var_x, label=lbl_x, lo=x_lower):
        def _update(*_):
            label.config(text=_fmt_p(var.get(), lo))
        return _update
    var_x.trace_add('write', make_x_updater())

    # --- Y 行 ---
    fy = tk.Frame(parent)
    fy.pack(fill=tk.X, padx=3, pady=1)
    tk.Label(
        fy, text=f"{display_name} Y", width=16,
        anchor='w', font=('TkDefaultFont', 9)
    ).pack(side=tk.LEFT)

    var_y = tk.DoubleVar(value=y_neutral)
    node.double_vars[arr_idx_y] = var_y
    tk.Scale(
        fy, from_=y_lower, to=y_upper,
        orient=tk.HORIZONTAL, variable=var_y,
        length=180, resolution=0.01,
        tickinterval=0, showvalue=0,
        command=lambda val, idx=arr_idx_y: node.on_slider_change(idx, float(val))
    ).pack(side=tk.LEFT)

    lbl_y = tk.Label(
        fy, text=_fmt_p(y_neutral, y_lower),
        width=10, anchor='w', font=('TkDefaultFont', 9)
    )
    lbl_y.pack(side=tk.LEFT, padx=2)

    def make_y_updater(var=var_y, label=lbl_y, lo=y_lower):
        def _update(*_):
            label.config(text=_fmt_p(var.get(), lo))
        return _update
    var_y.trace_add('write', make_y_updater())


def _get_landmark_arr_idx(node: LandmarkGuiNode, lid: int) -> int:
    """获取 landmark ID 在数组中的索引。"""
    return node.landmark_ids.index(lid)


def create_gui(node: LandmarkGuiNode) -> tk.Tk:
    root = tk.Tk()
    root.title("Landmark 位点控制器")

    # 预分配 DoubleVar 列表
    node.double_vars = [None] * node.num_landmarks

    # --- 顶部按钮区 ---
    top = tk.Frame(root)
    top.pack(pady=4)
    tk.Button(
        top, text="全部归中", bg="lightblue",
        command=node.all_to_neutral
    ).pack(side=tk.LEFT, padx=5)
    tk.Button(
        top, text="发送全部", bg="lightgreen",
        command=node.publish_landmarks_manual
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

    # --- 布局: 两列 ---
    # 左列: 线性 (眉眼 + 眼球) + 五连杆
    # 右列: 线性 (嘴唇 + 中庭 + 脖子)
    linear_lookup = _build_linear_lookup(node.config)
    fivebar_cfg = node.config.get('fivebar', {})

    columns = tk.Frame(content)
    columns.pack(fill=tk.BOTH, expand=True)

    left_col = tk.Frame(columns)
    left_col.grid(row=0, column=0, sticky='n', padx=5, pady=2)
    right_col = tk.Frame(columns)
    right_col.grid(row=0, column=1, sticky='n', padx=5, pady=2)

    # ---- 左列 ----
    # 线性: 眉毛 & 眼睑, 眼球
    for group_name, lids in LINEAR_GROUPS[:2]:
        tk.Label(
            left_col, text=f"--- {group_name} ---",
            font=('TkDefaultFont', 9, 'bold')
        ).pack(pady=(6, 2))
        for lid in lids:
            lm_cfg = linear_lookup.get(lid)
            if lm_cfg is None:
                continue
            arr_idx = _get_landmark_arr_idx(node, lid)
            _build_linear_row(left_col, node, arr_idx, lid, lm_cfg)

    # 五连杆 (全部放左列底部)
    tk.Frame(left_col, height=2, bg='gray').pack(fill=tk.X, padx=3, pady=4)
    for group_name, fb_key in FIVEBAR_GROUPS:
        fb = fivebar_cfg.get(fb_key)
        if fb is None:
            continue
        tk.Label(
            left_col, text=f"--- {group_name} ---",
            font=('TkDefaultFont', 9, 'bold')
        ).pack(pady=(6, 2))
        lx_id = int(fb.get('landmark_x_id', 0))
        ly_id = int(fb.get('landmark_y_id', 0))
        arr_idx_x = _get_landmark_arr_idx(node, lx_id)
        arr_idx_y = _get_landmark_arr_idx(node, ly_id)
        _build_fivebar_row(left_col, node, arr_idx_x, arr_idx_y, fb_key, fb)

    # ---- 右列 ----
    # 线性: 嘴唇, 中庭, 脖子
    for group_name, lids in LINEAR_GROUPS[2:]:
        tk.Label(
            right_col, text=f"--- {group_name} ---",
            font=('TkDefaultFont', 9, 'bold')
        ).pack(pady=(6, 2))
        for lid in lids:
            lm_cfg = linear_lookup.get(lid)
            if lm_cfg is None:
                continue
            arr_idx = _get_landmark_arr_idx(node, lid)
            _build_linear_row(right_col, node, arr_idx, lid, lm_cfg)

    root.geometry("780x520")
    return root


# ---------------------------------------------------------------------------
#  入口
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)

    config = load_landmark_config(_CONFIG_PATH)
    if not config:
        print(f"[controller_landmark_GUI] 无法加载配置 {_CONFIG_PATH}，退出")
        rclpy.shutdown()
        return

    node = LandmarkGuiNode(config)
    root = create_gui(node)

    def spin_once():
        rclpy.spin_once(node, timeout_sec=0.05)
        root.after(50, spin_once)

    spin_once()

    def auto_send_tick():
        """20Hz 无条件自动发送。"""
        if node.auto_send:
            node.publish_landmarks()
        root.after(20, auto_send_tick)

    auto_send_tick()
    root.mainloop()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
