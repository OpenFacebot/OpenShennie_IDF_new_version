#include "micro_ros_task.h"
#include "servo_state_db.h"
#include "servo_types.h"
#include "usjt_transport.h"

#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
// rcl 基础
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
// rclc — 按需包含，不拉 action/lifecycle
#include <rclc/executor.h>
#include <rclc/subscription.h>
#include <rclc/service.h>
#include <rclc/timer.h>
#include <rclc/node.h>
// micro-ROS transport
#include <rmw_microros/custom_transport.h>

// 类型化消息
#include <servo_msgs/msg/servo_command.h>
#include <servo_msgs/srv/get_servo_info.h>

// 批量位置消息
#include <std_msgs/msg/float32_multi_array.h>

static MicroRosTaskParams *g_params;

// ---- ROS2 对象 ----
static rcl_subscription_t   cmd_sub;
static rcl_subscription_t   batch_sub;   // 批量位置 subscriber
static rcl_service_t        svc_get_servo_info;
static rclc_executor_t      executor;
static rclc_support_t       support;
static rcl_allocator_t      allocator;
static rcl_node_t           node;

// ---- 消息实例 ----
static servo_msgs__msg__ServoCommand          cmd_msg;
static servo_msgs__srv__GetServoInfo_Request   req_msg;
static servo_msgs__srv__GetServoInfo_Response  res_msg;

// 批量位置消息缓存
static std_msgs__msg__Float32MultiArray        batch_msg;
static float batch_data_buf[DEFAULT_SERVO_COUNT];

// 批量舵机 ID 顺序 (与 servo_config.cpp / Python servo_config.py 保持一致)
static const uint8_t BATCH_SERVO_IDS[DEFAULT_SERVO_COUNT] = {
     1,  2,  3,  4,  5,  6,  7,  8,
    13, 14,
    17, 18, 19,
    21, 22, 23, 24, 25, 26, 27,
    33, 34, 35,
    41, 42, 43
};

// ============================================================
//  cmd_callback — 接收上位机 servo_command
// ============================================================

static void cmd_callback(const void *msgin)
{
    const auto *msg = (const servo_msgs__msg__ServoCommand *)msgin;
    ServoStateDB *db = g_params->db;

    // 急停状态下：只放行恢复指令，其他全忽略
    if (db->isEmergencyStop() && !(msg->cmd_type == 263 && msg->position != 0))
        return;

    switch (msg->cmd_type) {

    // ---- 位置控制 (0) : 直写 DB target ----
    case 0:
        db->setTarget(msg->id, msg->position, msg->time_ms);
        break;

    // ---- 扭矩开关 (253) : 入队由 servo_task 执行 ----
    case 253: {
        ServoCommand sc = {};
        sc.type          = ServoCmdType::SET_TORQUE;
        sc.id            = msg->id;
        sc.torque_enable = (msg->position != 0);
        db->sendCommand(sc);
        break;
    }

    // ---- 模式切换 (254) : id=1手动, id=0自动 ----
    case 254:
        db->setManualMode(msg->id != 0);
        printf("[指令] %s模式\n", msg->id ? "手动" : "自动");
        break;

    // ---- Ping (255) : 入队 ----
    case 255: {
        ServoCommand sc = {};
        sc.type = ServoCmdType::PING;
        sc.id   = msg->id;
        db->sendCommand(sc);
        break;
    }

    // ---- 急停/恢复 (263) ----
    case 263:
        db->setEmergencyStop(msg->position == 0);
        if (msg->position == 0) {
            ServoCommand sc = {};
            sc.type = ServoCmdType::EMERGENCY_STOP;
            sc.id   = DS_BROADCAST_ID;
            db->sendCommand(sc);
            printf("[指令] 紧急停止\n");
        } else {
            printf("[指令] 急停恢复\n");
        }
        break;

    default:
        break;
    }
}

// ============================================================
//  batch_callback — 接收批量舵机位置 (Float32MultiArray)
//  数组顺序与 BATCH_SERVO_IDS 一致，值范围 0–4095
// ============================================================

static void batch_callback(const void *msgin)
{
    const auto *msg = (const std_msgs__msg__Float32MultiArray *)msgin;
    ServoStateDB *db = g_params->db;

    if (db->isEmergencyStop()) return;

    int n = (int)msg->data.size;
    if (n > DEFAULT_SERVO_COUNT) n = DEFAULT_SERVO_COUNT;

    for (int i = 0; i < n; i++) {
        int pos = (int)msg->data.data[i];
        if (pos < 0)    pos = 0;
        if (pos > 4095) pos = 4095;
        db->setTarget(BATCH_SERVO_IDS[i], (uint16_t)pos, 40);
    }
}

// ============================================================
//  get_servo_info_callback — Service: 按需查询舵机状态
// ============================================================

static void get_servo_info_callback(const void *req, void *res)
{
    (void)req;
    auto *rsp = (servo_msgs__srv__GetServoInfo_Response *)res;

    ServoSnapshot snap[MAX_SERVO_SLOTS + 1];
    int count = g_params->db->getAllServos(snap, MAX_SERVO_SLOTS + 1);

    rsp->ids.data          = (uint8_t *)malloc(count * sizeof(uint8_t));
    rsp->positions.data    = (int16_t *)malloc(count * sizeof(int16_t));
    rsp->temperatures.data = (int16_t *)malloc(count * sizeof(int16_t));
    rsp->statuses.data     = (uint8_t *)malloc(count * sizeof(uint8_t));

    rsp->ids.capacity          = count;
    rsp->positions.capacity    = count;
    rsp->temperatures.capacity = count;
    rsp->statuses.capacity     = count;

    for (int i = 0; i < count; i++) {
        rsp->ids.data[i]           = snap[i].id;
        rsp->positions.data[i]     = snap[i].position;
        rsp->temperatures.data[i]  = snap[i].temperature;
        rsp->statuses.data[i]      = snap[i].status;
    }
    rsp->ids.size          = count;
    rsp->positions.size    = count;
    rsp->temperatures.size = count;
    rsp->statuses.size     = count;
    rsp->success = true;
}

// ============================================================
//  micro_ros_task — 通信任务入口
// ============================================================

void micro_ros_task(void *arg)
{
    g_params = (MicroRosTaskParams *)arg;

    // 1. 设置 USB Serial/JTAG 传输层
    rmw_uros_set_custom_transport(
        true, NULL,
        usjt_transport_open, usjt_transport_close,
        usjt_transport_write, usjt_transport_read);

    allocator = rcl_get_default_allocator();

    // 2. 等待 micro-ROS Agent（失败自动重试）
    printf("[通信] 等待上位机连接...\n");
    while (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK)
        vTaskDelay(pdMS_TO_TICKS(2000));
    printf("[通信] 已连接到上位机\n");

    // 3. 创建 Node
    rclc_node_init_default(&node, "esp32_servo", "", &support);

    // 4. Sub: servo_command — 接收控制指令
    rclc_subscription_init_default(
        &cmd_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(servo_msgs, msg, ServoCommand),
        "servo_command");

    // 5. Service: get_servo_info — 按需查询舵机状态
    rclc_service_init_default(
        &svc_get_servo_info, &node,
        ROSIDL_GET_SRV_TYPE_SUPPORT(servo_msgs, srv, GetServoInfo),
        "get_servo_info");

    // 6. Sub: servo_positions_batch — 批量位置指令 (Float32MultiArray)
    rclc_subscription_init_default(
        &batch_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "servo_positions_batch");

    // 初始化批量消息缓存
    batch_msg.data.data     = batch_data_buf;
    batch_msg.data.size     = 0;
    batch_msg.data.capacity = DEFAULT_SERVO_COUNT;

    // 7. Executor — 4 handles: cmd_sub + batch_sub + service + (reserved)
    rclc_executor_init(&executor, &support.context, 4, &allocator);
    rclc_executor_add_subscription(&executor, &cmd_sub, &cmd_msg,
                                    cmd_callback, ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &batch_sub, &batch_msg,
                                    batch_callback, ON_NEW_DATA);
    rclc_executor_add_service(&executor, &svc_get_servo_info,
                               &req_msg, &res_msg,
                               get_servo_info_callback);

    printf("[通信] 就绪 — 2 Sub | 1 Service | 批量位置 topic\n");

    // 7. 事件循环
    while (1) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(50));
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
