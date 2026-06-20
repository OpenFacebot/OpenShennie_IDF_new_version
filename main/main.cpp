#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "servo_types.h"
#include "servo_state_db.h"
#include "servo_config.h"
#include "servo_task.h"
#include "micro_ros_task.h"

// ============================================================
//  app_main — 组装车间
// ============================================================
extern "C" void app_main(void) {
    printf("\n===== OpenShennie ESP32-P4 舵机控制器 =====\n");

    // 1. 加载舵机配置 (26个) — static 避免栈溢出
    static ServoConfig cfgs[48];
    int cfg_count = servo_config_init(cfgs, 48);
    printf("[主程序] 加载 %d 个舵机配置\n", cfg_count);

    // 2. 创建数据中心
    ServoStateDB *db = new ServoStateDB();

    // 3. 启动 servo_task — static 保证任务启动时参数仍有效
    static ServoTaskParams servo_params;
    servo_params.db        = db;
    servo_params.cfgs      = cfgs;
    servo_params.cfg_count = cfg_count;
    xTaskCreate(servo_task, "servo", 16384, &servo_params, 6, NULL);

    // 4. 启动 micro_ros_task
    static MicroRosTaskParams microros_params;
    microros_params.servo_count = cfg_count;
    microros_params.servo_ids   = DEFAULT_SERVO_IDS;
    microros_params.db          = db;
    xTaskCreate(micro_ros_task, "microros", 8192, &microros_params, 4, NULL);

    printf("[主程序] 全部就绪\n");
}
