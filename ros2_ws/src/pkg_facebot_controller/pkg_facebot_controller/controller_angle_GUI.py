#!/usr/bin/env python3
"""
facebot_controller — DS-S009 舵机批量控制 GUI
通过 servo_positions_batch (Float32MultiArray) 发送 26 个舵机位置
用法: ros2 run pkg_facebot_controller facebot_controller
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import tkinter as tk
import time

from .servo_config import SERVO_LIST, DEFAULT_NEUTRAL, reload_constrain


def fmt_pos(pos):
    """格式化位置值 (0-4095) + 角度参考"""
    deg = pos * 360.0 / 4095
    return f"{pos:4d} ({deg:5.1f}°)"


class FacebotController(Node):
    def __init__(self):
        super().__init__('facebot_controller')

        self.batch_pub = self.create_publisher(
            Float32MultiArray, 'servo_positions_batch', 10)

        # 当前舵机位置 (26个，0-4095)
        self.positions = [s[5] for s in SERVO_LIST]  # neutral 值

        # IntVar 列表 (slider 绑定用，在 create_gui 中填充)
        self.vars: list[tk.IntVar] = []

        # Scale 控件引用 (用于动态更新范围)
        self.scales: list[tk.Scale] = []

        # 自动发送模式 (默认开启)
        self.auto_send = True

    def publish_batch(self):
        """将 26 个位置值打包成 Float32MultiArray 发布"""
        t0 = time.time()
        msg = Float32MultiArray()
        msg.data = [float(p) for p in self.positions]
        self.batch_pub.publish(msg)
        elapsed_ms = (time.time() - t0) * 1000
        self.get_logger().info(
            f'已发布 {len(self.positions)} 个舵机位置 | 耗时 {elapsed_ms:.2f}ms')

    def publish_batch_manual(self):
        """“发送全部”按钮调用：发布并打印总执行时间到终端"""
        t0 = time.time()
        msg = Float32MultiArray()
        msg.data = [float(p) for p in self.positions]
        self.batch_pub.publish(msg)
        elapsed_ms = (time.time() - t0) * 1000
        self.get_logger().info(f'[发送全部] 回调已触发 | 总执行时间 {elapsed_ms:.2f}ms')
        print(f'[发送全部] 已发布 {len(self.positions)} 个舵机位置 | 总执行时间 {elapsed_ms:.2f}ms', flush=True)

    def all_to_neutral(self):
        """全部归中 — 同时更新 IntVar 让 slider 移动"""
        for i, (sid, name_cn, name_en, pmin, pmax, pneut) in enumerate(SERVO_LIST):
            self.positions[i] = pneut
            if i < len(self.vars):
                self.vars[i].set(pneut)
        self.publish_batch()
        self.get_logger().info('全部归中')

    def load_constrain(self):
        """重新加载 servoID_constrain.json，更新滑块范围，并将所有舵机移至 neutral"""
        if reload_constrain():
            for i, (sid, name_cn, name_en, pmin, pmax, pneut) in enumerate(SERVO_LIST):
                if i < len(self.scales) and self.scales[i] is not None:
                    self.scales[i].config(from_=pmin, to=pmax)
                # 设置 neutral 位置
                self.positions[i] = pneut
                if i < len(self.vars) and self.vars[i] is not None:
                    self.vars[i].set(pneut)
            self.publish_batch()
            self.get_logger().info('已重新加载 constrain 约束，全部归中')
            print('[加载constrain] 约束已更新，全部归中', flush=True)
        else:
            self.get_logger().warn('加载 constrain 失败')
            print('[加载constrain] 加载失败', flush=True)

    def on_slider_change(self, idx, val):
        """slider 回调: 仅更新 position，发送由 25Hz 定时器负责"""
        self.positions[idx] = int(float(val))


def _build_servo_row(parent, node, i, sid, name_cn, name_en, pmin, pmax, pneut):
    """构建单个舵机行: 名称 | slider | 数值"""
    frame = tk.Frame(parent)
    frame.pack(fill=tk.X, padx=3, pady=1)

    # 舵机名: #ID 中文名
    tk.Label(frame, text=f"#{sid:2d} {name_cn}", width=12,
             anchor='w', font=('TkDefaultFont', 9)).pack(side=tk.LEFT)

    # 位置 slider (0-4095)
    var = tk.IntVar(value=pneut)
    node.vars[i] = var

    scale = tk.Scale(frame, from_=pmin, to=pmax,
                     orient=tk.HORIZONTAL,
                     variable=var, length=180, resolution=1,
                     tickinterval=0, showvalue=0,
                     command=lambda val, idx=i: node.on_slider_change(idx, val))
    scale.pack(side=tk.LEFT)
    node.scales[i] = scale

    # 数值标签
    lbl = tk.Label(frame, text=fmt_pos(pneut), width=16, anchor='w',
                   font=('TkDefaultFont', 9))
    lbl.pack(side=tk.LEFT, padx=2)

    # 实时更新数值标签
    def make_label_updater(var=var, label=lbl):
        def _update(*a):
            label.config(text=fmt_pos(var.get()))
        return _update

    var.trace_add('write', make_label_updater())



def create_gui(node: FacebotController):
    root = tk.Tk()
    root.title("Facebot 舵机控制器  (0-4095)")

    # 预分配 vars 列表 (26 个)
    node.vars = [None] * len(SERVO_LIST)
    node.scales = [None] * len(SERVO_LIST)

    # 顶部按钮区
    top = tk.Frame(root)
    top.pack(pady=4)
    tk.Button(top, text="全部归中", bg="lightblue",
              command=node.all_to_neutral).pack(side=tk.LEFT, padx=5)
    tk.Button(top, text="发送全部", bg="lightgreen",
              command=node.publish_batch_manual).pack(side=tk.LEFT, padx=5)
    tk.Button(top, text="加载constrain", bg="orange",
              command=node.load_constrain).pack(side=tk.LEFT, padx=5)

    # 自动发送 toggle 按钮
    auto_btn = tk.Button(top, text="自动发送: ON", bg="lime green",
                         fg="black", font=('TkDefaultFont', 10, 'bold'),
                         width=14)

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

    # 可滚动区域
    canvas_frame = tk.Frame(root)
    canvas_frame.pack(fill=tk.BOTH, expand=True)

    canvas = tk.Canvas(canvas_frame, highlightthickness=0)
    scrollbar = tk.Scrollbar(canvas_frame, orient=tk.VERTICAL, command=canvas.yview)
    canvas.configure(yscrollcommand=scrollbar.set)

    scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
    canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

    # 内容容器 (放在 canvas 里)
    content = tk.Frame(canvas)
    canvas_win = canvas.create_window((0, 0), window=content, anchor='nw')

    def _on_content_config(event):
        canvas.configure(scrollregion=canvas.bbox('all'))

    def _on_canvas_config(event):
        canvas.itemconfig(canvas_win, width=event.width)

    content.bind('<Configure>', _on_content_config)
    canvas.bind('<Configure>', _on_canvas_config)

    # 鼠标滚轮绑定
    def _on_mousewheel(event):
        canvas.yview_scroll(int(-1 * (event.delta / 120)), 'units')

    canvas.bind_all('<MouseWheel>', _on_mousewheel)
    canvas.bind_all('<Button-4>', lambda e: canvas.yview_scroll(-1, 'units'))
    canvas.bind_all('<Button-5>', lambda e: canvas.yview_scroll(1, 'units'))

    # 分组: 左列 13 个 (眉毛+眼睑+眼球+下颌), 右列 13 个 (嘴唇+中庭+脖子)
    mid = 13
    left_servos = SERVO_LIST[:mid]
    right_servos = SERVO_LIST[mid:]

    # 双列容器
    columns = tk.Frame(content)
    columns.pack(fill=tk.BOTH, expand=True)

    # 左列标题
    left_hdr = tk.Label(columns, text="--- 眉眼 / 眼球 / 下颌 ---",
                        font=('TkDefaultFont', 9, 'bold'))
    # 右列标题
    right_hdr = tk.Label(columns, text="--- 嘴唇 / 中庭 / 脖子 ---",
                         font=('TkDefaultFont', 9, 'bold'))

    # 用 grid 布局两列
    left_hdr.grid(row=0, column=0, sticky='w', padx=5, pady=2)
    right_hdr.grid(row=0, column=1, sticky='w', padx=5, pady=2)

    left_col = tk.Frame(columns)
    left_col.grid(row=1, column=0, sticky='n', padx=3)
    right_col = tk.Frame(columns)
    right_col.grid(row=1, column=1, sticky='n', padx=3)

    # 填充左列
    for j, (sid, name_cn, name_en, pmin, pmax, pneut) in enumerate(left_servos):
        _build_servo_row(left_col, node, j, sid, name_cn, name_en, pmin, pmax, pneut)

    # 填充右列
    for j, (sid, name_cn, name_en, pmin, pmax, pneut) in enumerate(right_servos):
        _build_servo_row(right_col, node, mid + j, sid, name_cn, name_en, pmin, pmax, pneut)

    # 窗口大小: 不足以显示全部内容，需要滚动
    root.geometry("720x500")
    return root


def main(args=None):
    rclpy.init(args=args)
    node = FacebotController()
    root = create_gui(node)

    def spin_once():
        rclpy.spin_once(node, timeout_sec=0.05)
        root.after(50, spin_once)

    spin_once()

    def auto_send_tick():
        """20Hz 无条件自动发送"""
        if node.auto_send:
            node.publish_batch()
        root.after(50, auto_send_tick)

    auto_send_tick()
    root.mainloop()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
