# Landmark ID

机器人驱动位点 ID 与对应名称关系

## 编号规则



## 位点列表

### 上半张脸 — 眉毛 & 眼睑

| ID | 中文名称 | EN Name | 范围 |
|----|----------|---------|---------|
| 01 | p_右内眉 | p_BrowInner_Right |（0,1）|
| 02 | p_左内眉 | p_BrowInner_Left |（0,1）|
| 03 | p_右外眉 | p_BrowOuter_Right |（0,1）|
| 04 | p_左外眉 | p_BrowOuter_Left |（0,1）|
| 05 | p_右上眼睑 | p_EyelidUpper_Right |（0,1）|
| 06 | p_左上眼睑 | p_EyelidUpper_Left |（0,1）|
| 07 | p_右下眼睑 | p_EyelidLower_Right |（0,1）|
| 08 | p_左下眼睑 | p_EyelidLower_Left |（0,1）|
| — | *09–12 预留* | | |

### 上半脸 — 眼球（中心）

| ID | 中文名称 | EN Name | 范围 |
|----|----------|---------|---------|
| 13 | p_眼睛pitch | p_Eye_Pitch |（-1,1）|
| 14 | p_眼睛yaw | p_Eye_Yaw |（-1,1）|
| — | *15–16 预留* | | |

### 下半脸 — 下颌运动

| ID | 中文名称 | EN Name |
|----|----------|---------|
| 17 | p_下巴-前后 | p_Jaw_X |
| 18 | p_下巴-上下 | p_Jaw_Y |
| — | *19-20 预留* | |

### 下半脸 — 嘴唇周围

| ID | 中文名称 | EN Name | 范围 |
|----|----------|---------|---------|
| 21 | p_右嘴角-前后 | p_MouthCornerRight_X | 待定 |
| 22 | p_右嘴角-上下 | p_MouthCornerRight_Y | 待定 |
| 23 | p_左嘴角-前后 | p_MouthCornerLeft_X | 待定 |
| 24 | p_左嘴角-上下 | p_MouthCornerLeft_Y | 待定 |
| 25 | p_上嘴唇-前后 | p_LipUpper_FrontBack |（0,1）|
| 26 | p_下嘴唇-前后 | p_LipLower_FrontBack |（0,1）|
| 27 | p_下嘴唇-上下 | p_LipLower_UpDown |（0,1）|
| —  | *28-32 预留* | |

### 脖子

| ID | 中文名称 | EN Name | 范围 |
|----|----------|---------|---------|
| 41 | p_脖子_roll | p_Neck_Roll | 待定 |
| 42 | p_脖子_pitch | p_Neck_Pitch | 待定 |
| 43 | p_脖子_yaw | p_Neck_Yaw | 待定 |
| — | *44 预留* | |

## 与电机对应关系

### 线性对应（一维）

| ID | 中文名称 | EN Name | 电机名称
|----|----------|---------|---------|
| 01 | 右内眉 | p_BrowInner_Right | BrowInner_Right |
| 02 | 左内眉 | p_BrowInner_Left | BrowInner_Left |
| 03 | 右外眉 | p_BrowOuter_Right | BrowOuter_Right |
| 04 | 左外眉 | p_BrowOuter_Left | BrowOuter_Left |
| 05 | 右上眼睑 | p_EyelidUpper_Right | EyelidUpper_Right |
| 06 | 左上眼睑 | p_EyelidUpper_Left | EyelidUpper_Left |
| 07 | 右下眼睑 | p_EyelidLower_Right | EyelidLower_Right |
| 08 | 左下眼睑 | p_EyelidLower_Left | EyelidLower_Left |
| 13 | 眼睛pitch | p_Eye_Pitch | Eye_Pitch|
| 14 | 眼睛yaw | p_Eye_Yaw | Eye_Yaw|
| 25 | 上嘴唇-前后 | p_LipUpper_FrontBack | LipUpper_FrontBack |
| 26 | 下嘴唇-前后 | p_LipLower_FrontBack | LipLower_FrontBack |
| 27 | 下嘴唇-上下 | p_LipLower_UpDown |LipLower_UpDown |
| 33 | 右颧骨 | p_Cheek_Right | Cheek_Right  |
| 34 | 左颧骨 | p_Cheek_Left | Cheek_Left  |
| 35 | 鼻翼 | p_NoseWing | NoseWing |

如何从位点位置得到对应舵机角度（线性对应）：
- 位点位置范围 $(p_l, p_u)$,（0，1.0）或者 （-1,1）
- 对应角度范围 $(B_l，B_u)$，数值0-4095（对应0-360 degree）
- 计算方式：$angle = ( B_u - B_l )/(p_u-p_l) * p + B_l $, where $p \in（0，1.0）$

### 五连杆（二维）

| 位点自由度 | 电机自由度  |
|----------|----------|
| p_右嘴角-前后(p_MouthCornerRight_X) + p_右嘴角-上下(p_MouthCornerRight_Y) | 右嘴角上(MouthCornerUp_Right) + 右嘴角下(MouthCornerDown_Right) |
| p_左嘴角-前后(p_MouthCornerLeft_X) + p_左嘴角-上下(p_MouthCornerLeft_Y) | 左嘴角上(MouthCornerUp_Left) + 左嘴角下(MouthCornerDown_Left) |
| p_下巴-前后(p_Jaw_X) + p_下巴-上下(p_Jaw_Y) | 下巴开合(Jaw_Open) + {下巴右(Jaw_Right)-下巴左(Jaw_Left)} |

注意：下巴右(Jaw_Right)-下巴左(Jaw_Left) 是一个自由度，取相反的转动角度，保持同步

**五连杆的逆运动学解：**

两个固定点（电机位置，坐标已知）$A_1(x_{A1},\; y_{A1})$、$A_2(x_{A2},\; y_{A2})$，从动点 $B_1$、$B_2$，目标移动点 $C(x_C,\; y_C)$。

| 符号 | 含义 |
|------|------|
| $l_1 = \overline{A_1 B_1}$ | 主动连杆 1（电机 1 到从动点 $B_1$）|
| $l_2 = \overline{A_2 B_2}$ | 主动连杆 2（电机 2 到从动点 $B_2$）|
| $l_3 = \overline{B_1 C}$ | 从动连杆 1（$B_1$ 到目标点 $C$）|
| $l_4 = \overline{B_2 C}$ | 从动连杆 2（$B_2$ 到目标点 $C$）|
| $\theta_1$ | $\overrightarrow{A_1 B_1}$ 与 $x$ 轴夹角（电机 1 角度）|
| $\theta_2$ | $\overrightarrow{A_2 B_2}$ 与 $x$ 轴夹角（电机 2 角度）|

闭环链：$A_1 \to B_1 \to C \to B_2 \to A_2$

### 装配模式约定

为消除多解歧义，需指定装配模式（以嘴角为例）：
- $B_1$ 在 $A_1 C$ 的**左侧**（$\overrightarrow{A_1 B_1}$ 相对 $\overrightarrow{A_1 C}$ 逆时针偏转）
- $B_2$ 在 $A_2 C$ 的**左侧**（$\overrightarrow{A_2 B_2}$ 相对 $\overrightarrow{A_2 C}$ 顺时针偏转）

### 逆运动学（Inverse Kinematics）——已知 $C$ 坐标求 $\theta_1$、$\theta_2$

#### 求 $\theta_1$

记 $C$ 到 $A_1$ 的距离：

$$\|C - A_1\| = \sqrt{(x_C - x_{A1})^2 + (y_C - y_{A1})^2}$$

解存在条件（三角形 $A_1 B_1 C$ 可构成）：

$$|l_1 - l_3| \le \|C - A_1\| \le l_1 + l_3$$

取 $B_1$ 在 $A_1 C$ 右侧（逆时针偏转）：

$$\boxed{\theta_1 = \text{atan2}(y_C - y_{A1},\; x_C - x_{A1}) + \arccos\!\left( \frac{\|C-A_1\|^2 + l_1^2 - l_3^2}{2 \; l_1 \; \|C-A_1\|} \right)}$$

#### 求 $\theta_2$

记 $C$ 到 $A_2$ 的距离：

$$\|C - A_2\| = \sqrt{(x_C - x_{A2})^2 + (y_C - y_{A2})^2}$$

解存在条件（三角形 $A_2 B_2 C$ 可构成）：

$$|l_2 - l_4| \le \|C - A_2\| \le l_2 + l_4$$

取 $B_2$ 在 $A_2 C$ 左侧（顺时针偏转）：

$$\boxed{\theta_2 = \text{atan2}(y_C - y_{A2},\; x_C - x_{A2}) - \arccos\!\left( \frac{\|C-A_2\|^2 + l_2^2 - l_4^2}{2 \; l_2 \; \|C-A_2\|} \right)}$$

#### 解的唯一性

在选定装配模式下，逆运动学为**单值映射**，无需在多解间进行选择判断。

### 从位点位置到舵机角度

1. 由位点位置 $(p_x, p_y)$ 映射到目标点坐标 $(x_C, y_C)$（根据机械结构标定）
2. 代入逆运动学公式求得电机角度 $(\theta_1, \theta_2)$（单位：弧度）
3. 将弧度转换为舵机控制值：$\text{angle} = \frac{\theta \times 180}{\pi}$，再映射到 $0$–$4095$（对应 $0°$–$360°$）

### 各五连杆机构参数

| 机构 | $A_1$ 电机 | $A_2$ 电机 | $l_1$ | $l_2$ | $l_3$ | $l_4$ |
|------|-----------|-----------|-------|-------|-------|-------|
| 右嘴角 | MouthCornerUp_Right | MouthCornerDown_Right | 待定 | 待定 | 待定 | 待定 |
| 左嘴角 | MouthCornerUp_Left | MouthCornerDown_Left | 待定 | 待定 | 待定 | 待定 |
| 下巴 | Jaw_Open | Jaw_Right/Jaw_Left | 待定 | 待定 | 待定 | 待定 |

> 注：下巴机构中 Jaw_Right 与 Jaw_Left 取相反转动角度保持同步，作为一个自由度处理。
