#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "servo_types.h"
#include "servo_config.h"
#include "servo_task.h"
#include "micro_ros_task.h"

// ============================================================
//  app_main — 组装车间
// ============================================================
extern "C" void app_main(void) {
    printf("\n===== OpenShennie ESP32-P4 舵机控制器 =====\n");

    // 1. 加载舵机配置 (26个) — static 避免栈溢出
    static ServoConfig cfgs[MAX_SERVO_SLOTS];
    int cfg_count = servo_config_init(cfgs, MAX_SERVO_SLOTS);
    printf("[主程序] 加载 %d 个舵机配置\n", cfg_count);

    // 2. 初始化共享状态 (mutex、queue)
    servo_share_init();

    // 3. 启动 servo_task — static 保证任务启动时参数仍有效
    static ServoTaskParams servo_params;
    servo_params.cfgs      = cfgs;
    servo_params.cfg_count = cfg_count;
    xTaskCreate(servo_task, "servo", 16384, &servo_params, 6, NULL);

    // 4. 启动 micro_ros_task
    xTaskCreate(micro_ros_task, "microros", 16384, NULL, 4, NULL);

    printf("[主程序] 全部就绪\n");
}
