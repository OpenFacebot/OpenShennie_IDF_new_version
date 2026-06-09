# Servo ID

机器人舵机 ID 与名称对应关系。

## 编号规则 （参考，不用严格遵守）

按面部解剖区域从上到下顺序编排，各段之间预留空位以便扩展。

- **01–16** 上半脸
  - 01–12：眉毛 & 眼睑
  - 13–16：眼球运动（中心）
- **17–40** 下半脸
  - 17–20：下颌运动
  - 21–32：嘴唇周围
  - 33–40：中庭区域（颧骨、鼻翼）
- **41-44** 脖子

对称部件采用**右奇左偶**连续编号，便于代码中配对处理。

## 舵机列表

### 上半脸 — 眉毛 & 眼睑（左右对称）

| ID | 中文名称 | EN Name |
|----|----------|---------|
| 01 | 右内眉 | BrowInner_Right |
| 02 | 左内眉 | BrowInner_Left |
| 03 | 右外眉 | BrowOuter_Right |
| 04 | 左外眉 | BrowOuter_Left |
| 05 | 右上眼睑 | EyelidUpper_Right |
| 06 | 左上眼睑 | EyelidUpper_Left |
| 07 | 右下眼睑 | EyelidLower_Right |
| 08 | 左下眼睑 | EyelidLower_Left |
| — | *09–12 预留* | |

### 上半脸 — 眼球（中心）

| ID | 中文名称 | EN Name |
|----|----------|---------|
| 13 | 眼睛pitch | Eye_Pitch |
| 14 | 眼睛yaw | Eye_Yaw |
| — | *15–16 预留* | |


### 下半脸 — 下颌运动

| ID | 中文名称 | EN Name |
|----|----------|---------|
| 17 | 下巴开合 | Jaw_Open |
| 18 | 下巴右 | Jaw_Right |
| 19 | 下巴左 | Jaw_Left |
| — | *20 预留* | |

### 下半脸 — 嘴唇周围

| ID | 中文名称 | EN Name |
|----|----------|---------|
| 21 | 右嘴角上 | MouthCornerUp_Right |
| 22 | 左嘴角上 | MouthCornerUp_Left |
| 23 | 右嘴角下 | MouthCornerDown_Right |
| 24 | 左嘴角下 | MouthCornerDown_Left |
| 25 | 上嘴唇-前后 | LipUpper_FrontBack |
| 26 | 下嘴唇-前后 | LipLower_FrontBack |
| 27 | 下嘴唇-上下 | LipLower_UpDown |
| —  | *28-32 预留* | |


### 下半脸 — 中庭区域

| ID | 中文名称 | EN Name |
|----|----------|---------|
| 33 | 右颧骨 | Cheek_Right |
| 34 | 左颧骨 | Cheek_Left |
| 35 | 鼻翼 | NoseWing |
| — | *36–40 预留* | |


### 脖子

| ID | 中文名称 | EN Name |
|----|----------|---------|
| 41 | 脖子右 | Neck_Right |
| 42 | 脖子左 | Neck_Left |
| 43 | 脖子中 | Neck_Center |
| — | *44 预留* | |