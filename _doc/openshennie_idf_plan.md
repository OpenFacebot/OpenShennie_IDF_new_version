# openshennie_idf_plan

版本：V2.0.0

# 1. Overall System

电机采用 DS-S009 智能舵机, 通讯协议参考：`_doc\DSServo\Protocol-V4.md`
电机ID定义参考：`_doc\openshennie_servoID.md`

# 2. Task Descriptions

## 



# 3. Implementation Details

## 3.1 Hardware Details	

和 ESP32-P4 通讯方式：Serial 
接口：UART1

**ESP32-P4 的 UART 分配：**
- UART
- 


## 3.2 Servo Details

MAX_SERVO_NUM = 48 (目前版本最多电机数量，包含预留位置)
SERVO_NUM = 26 （已使用电机数量，不包含预留位置）

按面部解剖区域从上到下顺序编排，各段之间预留空位以便扩展。

- **01–16** 上半脸
  - 01–12：眉毛 & 眼睑
  - 13–16：眼球运动（中心）
- **17–40** 下半脸
  - 17–20：下颌运动
  - 21–32：嘴唇周围
  - 33–40：中庭区域（颧骨、鼻翼）
- **41-44** 脖子

详见 `_doc\openshennie_servoID.md` 和 `_doc\servoID_template.json`


