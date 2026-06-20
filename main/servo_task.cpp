#include "servo_task.h"
#include "servo_state_db.h"
#include "servo_config.h"
#include "servo_types.h"
#include "DSServo.h"

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"

// ---- 硬件配置 ----
#define SERVO_UART          UART_NUM_1
#define SERVO_TX_PIN        20
#define SERVO_RX_PIN        21
#define SERVO_BAUD          1000000
#define SERVO_FACTORY_BAUD  115200
#define SERVO_FREQ_HZ       10          // 控制频率
#define TEMP_ALARM          55          // 高温报警阈值
#define TEMP_RECOVER        53          // 恢复阈值
#define OFFLINE_MISS_MAX    3           // 连续失败次数→离线

// ============================================================
//  波特率工具
// ============================================================
static uint8_t baud_to_reg(uint32_t b) {
    switch(b) {
        case 9600:   return 1;
        case 38400:  return 2;
        case 57600:  return 3;
        case 76800:  return 4;
        case 115200: return 5;
        case 128000: return 6;
        case 250000: return 7;
        case 500000: return 8;
        case 1000000: return 9;
        default:     return 5;
    }
}

static int scan_servos(DSServo &servo,uint32_t baud, const ServoConfig *cfgs, int cfg_count, uint8_t *bitmap) {
    uart_driver_delete(SERVO_UART);
    servo.begin(SERVO_UART, SERVO_TX_PIN, SERVO_RX_PIN, baud);
    vTaskDelay(pdMS_TO_TICKS(300));
    int alive = 0;
    for (int i = 0; i < cfg_count; i++) {
        bool ok = (servo.ping(cfgs[i].id) == ESP_OK);
        if (bitmap) bitmap[i] = ok ? 1 : 0;
        if (ok) alive++;
    }
    return alive;
}

static void smart_baud_init(DSServo &servo, const ServoConfig *cfgs, int cfg_count) {
    uint8_t online[48] = {0};
    bool need_power_cycle = false;

    // Step 1: 以目标波特率扫描
    printf("[初始化] 目标波特率 %d, 扫描...\n", SERVO_BAUD);
    int a = scan_servos(servo, SERVO_BAUD, cfgs, cfg_count, online);
    printf("[初始化] 在线: %d/%d\n", a, cfg_count);
    if (a == cfg_count) {
        printf("[初始化] 全部就绪\n\n");
        return;
    }

    // Step 2: 遍历所有已知波特率, 找到离线的舵机并改到目标波特率
    uint32_t known_bauds[] = {
        SERVO_FACTORY_BAUD, 1000000, 115200,
        9600, 38400, 57600, 76800, 128000, 250000, 500000
    };

    for (uint32_t try_b : known_bauds) {
        if (try_b == (uint32_t)SERVO_BAUD) continue;

        uint8_t found[48] = {0};
        int n = scan_servos(servo, try_b, cfgs, cfg_count, found);
        if (n == 0) continue;

        printf("[初始化] 波特率 %d 发现 %d 个舵机\n", (int)try_b, n);

        uint8_t bv = baud_to_reg(SERVO_BAUD);
        for (int i = 0; i < cfg_count; i++) {
            if (online[i]) continue;
            if (!found[i]) continue;
            printf("[初始化] 舵机 %d (%s): %d -> %d\n",
                   cfgs[i].id, cfgs[i].name, (int)try_b, SERVO_BAUD);
            servo.setServoBaudRate(cfgs[i].id, bv);
            vTaskDelay(pdMS_TO_TICKS(50));
            online[i] = 1;
            need_power_cycle = true;
        }

        int remain = 0;
        for (int i = 0; i < cfg_count; i++)
            if (!online[i]) remain++;
        if (remain == 0) break;
    }

    int remain = 0;
    for (int i = 0; i < cfg_count; i++)
        if (!online[i]) remain++;
    if (remain > 0) {
        printf("[初始化] %d 个舵机在所有波特率下无响应, 检查接线/电源\n", remain);
        for (int i = 0; i < cfg_count; i++)
            if (!online[i]) printf("[初始化]   舵机 %d (%s) 无响应\n", cfgs[i].id, cfgs[i].name);
    }

    if (!need_power_cycle) {
        scan_servos(servo, SERVO_BAUD, cfgs, cfg_count, nullptr);
        return;
    }

    // Step 3: 等待断电重启
    printf("[初始化] ========================================\n");
    printf("[初始化] 波特率已修改, 请给舵机断电再上电!\n");
    printf("[初始化] 等待重启中(每2秒确认一次)...\n");
    printf("[初始化] ========================================\n");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(2000));
        servo.changeHostBaudRate(SERVO_BAUD);
        vTaskDelay(pdMS_TO_TICKS(300));

        int alive = scan_servos(servo, SERVO_BAUD, cfgs, cfg_count, online);
        printf("[初始化] 扫描: %d/%d 在线\n", alive, cfg_count);

        if (alive == cfg_count) {
            printf("[初始化] 全部就绪!\n\n");
            return;
        }
        printf("[初始化] 还有 %d 个离线, 继续等待...\n", cfg_count - alive);
    }
}

static void read_one_servo(DSServo &servo, ServoStateDB *db, const ServoConfig &cfg, uint8_t *miss_cnt) {
    int16_t pos = servo.getPosition(cfg.id);
    int16_t temp = servo.getTemperature(cfg.id);

    if (pos >= 0) {
        uint8_t st = servo.getServoStatus(cfg.id);
        db->updateServo(cfg.id, pos, 0, 0, temp, st, true);
        *miss_cnt = 0;

        // 高温监测
        if (temp >= TEMP_ALARM) {
            st |= 0x40;
            db->setEmergencyStop(true);
            printf("[警告] 舵机%d 温度过高: %d°C\n", cfg.id, temp);
        }
    } else {
        // 读取失败 → 累计离线计数
        (*miss_cnt)++;
        if (*miss_cnt >= OFFLINE_MISS_MAX) {
            db->updateServo(cfg.id, 0, 0, 0, 0, OFFLINE_STATUS, false);
            printf("[状态] 舵机%d 离线\n", cfg.id);
        }
    }
}


static void process_commands(DSServo &servo, ServoStateDB *db, const ServoConfig *cfgs, int count) {
    ServoCommand cmd;
    while (db->receiveCommand(&cmd, 0)) {
        switch (cmd.type) {
        case ServoCmdType::SET_TORQUE:
            if (cmd.id == DS_BROADCAST_ID)
                for (int i = 0; i < count; i++)
                    servo.setTorque(cfgs[i].id, cmd.torque_enable);
            else
                servo.setTorque(cmd.id, cmd.torque_enable);
            break;

        case ServoCmdType::EMERGENCY_STOP:
            for (int i = 0; i < count; i++)
                servo.setTorque(cfgs[i].id, false);
            printf("[急停] 所有舵机扭矩关闭\n");
            break;

        case ServoCmdType::PING: {
            bool ok = (servo.ping(cmd.id) == ESP_OK);
            printf("[Ping] 舵机%d: %s\n", cmd.id, ok ? "在线" : "无响应");
            break;
        }

        default:
            break;
        }
    }
}


void servo_task(void *arg) {
      auto *p = (ServoTaskParams *)arg;
      DSServo servo;

      // 1. 智能波特率初始化
      smart_baud_init(servo, p->cfgs, p->cfg_count);

      // 2. 开启所有舵机扭矩
      for (int i = 0; i < p->cfg_count; i++)
          servo.setTorque(p->cfgs[i].id, true);

      printf("[舵机] %d个舵机就绪, %dHz\n", p->cfg_count, SERVO_FREQ_HZ);

      TickType_t last_wake = xTaskGetTickCount();
      const TickType_t period = pdMS_TO_TICKS(1000 / SERVO_FREQ_HZ);
      uint8_t miss_cnt[48] = {0};
      uint32_t read_idx = 0;

      while (1) {
          // ---- 队列指令 ----
          process_commands(servo, p->db, p->cfgs, p->cfg_count);

          // ---- 急停时跳过控制 ----
          if (!p->db->isEmergencyStop()) {
              // ---- 消费 target → syncWrite ----
              DSSyncWriteData sync_buf[MAX_SERVO_SLOTS + 1];
              int sync_count = 0;
              if (p->db->consumeTargets(sync_buf, MAX_SERVO_SLOTS + 1, &sync_count)) {
                  servo.syncWritePosition(sync_buf, (uint8_t)sync_count);
              }
          }

          // ---- 增量反馈 (每次读 1 个舵机) ----
          read_one_servo(servo, p->db, p->cfgs[read_idx], &miss_cnt[read_idx]);
          read_idx = (read_idx + 1) % p->cfg_count;

          vTaskDelayUntil(&last_wake, period);
      }
  }


