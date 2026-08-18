#pragma once

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#define MAX_SERVO_COUNT      48     // 实际舵机数量上限
#define MAX_SERVO_SLOTS      64     // 舵机最大索引上限
#define CMD_QUEUE_LENGTH     32     // 指令队列长度
#define DS_BROADCAST_ID      254    // 广播ID   
#define OFFLINE_STATUS       0xFF   // 离线状态


#define DEFAULT_SMOOTHING  0.5f   // EMA 系数：约5帧(165ms)达90%
#define MIN_MOVE_TIME      20      // 最短 move_time (ms)
#define POSITION_DEADZONE  3       // 死区：|Δ| < 3 不发送

struct ServoSnapshot {
    uint8_t  id;
    int16_t  position;
    int16_t  speed;
    int16_t  voltage;
    int16_t  temperature;
    uint8_t  status;
    bool     valid;
};

enum class ServoCmdType {
    SET_TORQUE,
    EMERGENCY_STOP,
    PING,

};

struct ServoCommand {
    ServoCmdType type;
    uint8_t  id;
    bool     torque_enable;

};

struct ServoConfig {
    uint8_t id;
    char    name[32];
    uint16_t angle_min;
    uint16_t angle_max;
    uint16_t angle_neutral;
};

struct ServoMotionState{
    float current;//当前平滑位置（0-4095）
    uint16_t target;//最终目标位置
    uint16_t last_sent;//上次已发送值
    bool active;//有效目标
};

  // 按面部区域从上到下编排
  // 01-12: 眉毛&眼睑, 13-16: 眼球, 17-20: 下颌
  // 21-32: 嘴唇, 33-40: 中庭, 41-44: 脖子
constexpr uint8_t DEFAULT_SERVO_IDS[] = {
      1,  2,  3,  4,  5,  6,  7,  8,      // 眉毛&眼睑 (09-12预留)
      13, 14,                                // 眼球 (15-16预留)
      17, 18, 19,                            // 下颌 (20预留)
      21, 22, 23, 24, 25, 26, 27,          // 嘴唇 (28-32预留)
      33, 34, 35,                            // 中庭 (36-40预留)
      41, 42, 43                             // 脖子 (44预留)
  };
constexpr int DEFAULT_SERVO_COUNT = sizeof(DEFAULT_SERVO_IDS) / sizeof(DEFAULT_SERVO_IDS[0]); // =26

