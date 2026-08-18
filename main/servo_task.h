#pragma once

#include "servo_types.h"

struct ServoTaskParams {
    const ServoConfig *cfgs;
    int cfg_count;
};

void servo_task(void *arg);

// ============================================================
//  共享 API — 供 micro_ros_task 调用
//  底层实现: servo_task.cpp 的 file-static 变量 + FreeRTOS 原语
// ============================================================
void servo_share_init();                              // 初始化 mutex / queue (app_main 调用)
void servo_set_target(uint8_t id, uint16_t position); // 写入目标位置
bool servo_send_command(const ServoCommand &cmd);     // 入指令队列 (扭矩/急停/Ping)
int  servo_get_all_feedback(ServoSnapshot *out, int max_count); // 拷贝有效反馈
void servo_set_emergency_stop(bool stop);             // 置急停标志
bool servo_is_emergency_stop();                       // 读急停标志
void servo_set_manual_mode(bool manual);              // 置手动模式
bool servo_is_manual_mode();                          // 读手动模式
