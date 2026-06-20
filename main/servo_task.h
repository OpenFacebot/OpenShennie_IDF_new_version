#pragma once
#include <stdint.h>

class ServoStateDB;
struct ServoConfig;

struct ServoTaskParams {
    ServoStateDB *db;
    const ServoConfig *cfgs;
    int cfg_count;
};
void servo_task(void *args);
