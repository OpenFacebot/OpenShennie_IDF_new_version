#pragma once

#include <stdint.h>

class ServoStateDB;

struct MicroRosTaskParams {
    uint8_t servo_count;
    const uint8_t *servo_ids;
    ServoStateDB *db;       //数据中心指针，供通信任务读写
};

void micro_ros_task(void *arg);
