#include "servo_task.h"
#include "servo_config.h"
#include "DSServo.h"

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "driver/uart.h"

// ---- 硬件配置 ----
#define SERVO_UART          UART_NUM_1
#define SERVO_TX_PIN        18
#define SERVO_RX_PIN        19
#define SERVO_BAUD          1000000
#define SERVO_FACTORY_BAUD  115200
#define SERVO_FREQ_HZ       30          // 控制频率
#define TEMP_ALARM          70          // 高温报警阈值
#define TEMP_RECOVER        65          // 恢复阈值
#define OFFLINE_MISS_MAX    3           // 连续失败次数→离线

// 无目标哨兵: 0-4095 外的值表示"该舵机尚未收到目标"
#define NO_TARGET           0xFFFF

// ============================================================
//  共享状态 — 全部归 servo_task.cpp 所有
//  (替代 ServoStateDB, 直接用 FreeRTOS 原语隔离两个 Task)
// ============================================================

// 目标位置 (micro_ros 写, servo_task 锁内整拷)
static uint16_t          g_targets[MAX_SERVO_SLOTS + 1];
static SemaphoreHandle_t g_target_mutex;

// 指令队列 (micro_ros 发, servo_task 收)
static QueueHandle_t     g_cmd_queue;

// 反馈数据 (servo_task 写, micro_ros service 读)
static ServoSnapshot     g_feedback[MAX_SERVO_SLOTS + 1];
static SemaphoreHandle_t g_feedback_mutex;

// 急停 / 模式 (volatile, 单写者)
static volatile bool     g_emergency_stop;
static volatile bool     g_manual_mode;

// EMA 插值状态 (servo_task 独占, 无锁)
static ServoMotionState  g_motion[MAX_SERVO_SLOTS + 1];

// ============================================================
//  共享 API — 锁内只做 copy in/out, 不做计算
// ============================================================

void servo_share_init() {
    g_target_mutex   = xSemaphoreCreateMutex();
    g_feedback_mutex = xSemaphoreCreateMutex();
    g_cmd_queue      = xQueueCreate(CMD_QUEUE_LENGTH, sizeof(ServoCommand));

    for (int i = 0; i <= MAX_SERVO_SLOTS; i++)
        g_targets[i] = NO_TARGET;
    memset(g_feedback, 0, sizeof(g_feedback));
    memset(g_motion, 0, sizeof(g_motion));
    g_emergency_stop = false;
    g_manual_mode    = true;
}

void servo_set_target(uint8_t id, uint16_t position) {
    if (id == 0 || id > MAX_SERVO_SLOTS) return;
    xSemaphoreTake(g_target_mutex, portMAX_DELAY);
    g_targets[id] = position;
    xSemaphoreGive(g_target_mutex);
}

bool servo_send_command(const ServoCommand &cmd) {
    return xQueueSend(g_cmd_queue, &cmd, 0) == pdTRUE;
}

int servo_get_all_feedback(ServoSnapshot *out, int max_count) {
    int count = 0;
    xSemaphoreTake(g_feedback_mutex, portMAX_DELAY);
    for (int i = 1; i <= MAX_SERVO_SLOTS && count < max_count; i++) {
        if (g_feedback[i].valid) {
            out[count++] = g_feedback[i];
        }
    }
    xSemaphoreGive(g_feedback_mutex);
    return count;
}

void servo_set_emergency_stop(bool stop) {
    g_emergency_stop = stop;
}

bool servo_is_emergency_stop() {
    return g_emergency_stop;
}

void servo_set_manual_mode(bool manual) {
    g_manual_mode = manual;
}

bool servo_is_manual_mode() {
    return g_manual_mode;
}

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

// ============================================================
//  反馈读取 — 每次读 1 个舵机 (增量轮询)
// ============================================================
static void read_one_servo(DSServo &servo, const ServoConfig &cfg, uint8_t *miss_cnt) {
    int16_t pos = servo.getPosition(cfg.id);
    int16_t temp = servo.getTemperature(cfg.id);

    if (pos >= 0) {
        uint8_t st = servo.getServoStatus(cfg.id);
        xSemaphoreTake(g_feedback_mutex, portMAX_DELAY);
        g_feedback[cfg.id].id          = cfg.id;
        g_feedback[cfg.id].position    = pos;
        g_feedback[cfg.id].temperature = temp;
        g_feedback[cfg.id].status      = st;
        g_feedback[cfg.id].valid       = true;
        xSemaphoreGive(g_feedback_mutex);
        *miss_cnt = 0;

        // 高温监测
        if (temp >= TEMP_ALARM) {
            st |= 0x40;
            servo_set_emergency_stop(true);
            printf("[警告] 舵机%d 温度过高: %d°C\n", cfg.id, temp);
        }
    } else {
        // 读取失败 → 累计离线计数
        (*miss_cnt)++;
        if (*miss_cnt >= OFFLINE_MISS_MAX) {
            xSemaphoreTake(g_feedback_mutex, portMAX_DELAY);
            g_feedback[cfg.id].valid = false;
            xSemaphoreGive(g_feedback_mutex);
            printf("[状态] 舵机%d 离线\n", cfg.id);
        }
    }
}

// ============================================================
//  指令处理 — 从队列取出非位置指令执行
// ============================================================
static void process_commands(DSServo &servo, const ServoConfig *cfgs, int count) {
    ServoCommand cmd;
    while (xQueueReceive(g_cmd_queue, &cmd, 0) == pdTRUE) {
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

// ============================================================
//  servo_task — 30Hz 主循环
//  [1] 处理指令队列   [2] EMA 平滑→syncWrite   [3] 增量反馈
// ============================================================
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
    uint8_t miss_cnt[MAX_SERVO_SLOTS + 1] = {0};
    uint32_t read_idx = 0;

    while (1) {
        // ---- 1. 队列指令 ----
        process_commands(servo, p->cfgs, p->cfg_count);

        // ---- 2. 急停时跳过位置控制 ----
        if (!g_emergency_stop) {
            // 2a. 锁内拷贝目标 (持锁 < 1μs)
            uint16_t local_targets[MAX_SERVO_SLOTS + 1];
            xSemaphoreTake(g_target_mutex, portMAX_DELAY);
            memcpy(local_targets, g_targets, sizeof(g_targets));
            xSemaphoreGive(g_target_mutex);

            // 2b. 锁外 EMA 平滑 + 组包
            DSSyncWriteData sync_buf[MAX_SERVO_SLOTS + 1];
            int sync_count = 0;

            for (int i = 0; i < p->cfg_count; i++) {
                uint8_t id = p->cfgs[i].id;
                ServoMotionState &m = g_motion[id];

                bool first_activation = false;

                // 有新目标
                if (local_targets[id] != NO_TARGET) {
                    if (!m.active) {
                        // 首次目标: 以反馈位置为起点, 无反馈则直接对齐目标 (避免从0爬升)
                        m.current   = g_feedback[id].valid
                                    ? (float)g_feedback[id].position
                                    : (float)local_targets[id];
                        m.target    = local_targets[id];
                        m.active    = true;
                        first_activation = true;  // 首帧强制发送, 跳过死区
                    } else {
                        m.target = local_targets[id];
                    }
                }
                if (!m.active) continue;

                // EMA 一阶低通: 每帧向目标走剩余距离的 DEFAULT_SMOOTHING
                m.current += ((float)m.target - m.current) * DEFAULT_SMOOTHING;

                // 死区: |Δ| < POSITION_DEADZONE 不发送, 避免微抖刷总线
                // 例外: 首次激活 或 目标变更后首帧 强制发送
                int diff = (int)m.current - (int)m.last_sent;
                if (diff < 0) diff = -diff;
                if (diff >= POSITION_DEADZONE || first_activation) {
                    sync_buf[sync_count].id       = id;
                    sync_buf[sync_count].position = (uint16_t)m.current;
                    sync_buf[sync_count].time_ms  = MIN_MOVE_TIME;
                    sync_count++;
                    m.last_sent = (uint16_t)m.current;
                }
            }

            // 2c. 一次性同步写
            if (sync_count > 0)
                servo.syncWritePosition(sync_buf, (uint8_t)sync_count);
        }

        // ---- 3. 增量反馈 (每次读 1 个舵机) ----
        read_one_servo(servo, p->cfgs[read_idx], &miss_cnt[read_idx]);
        read_idx = (read_idx + 1) % p->cfg_count;

        vTaskDelayUntil(&last_wake, period);
    }
}
