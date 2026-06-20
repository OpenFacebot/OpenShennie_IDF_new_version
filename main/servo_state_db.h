#pragma once        

#include "servo_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

// 前置声明 (来自 DSServo.h)
struct DSSyncWriteData;



class ServoStateDB {
public:
      ServoStateDB();
      ~ServoStateDB() = default;

      // ---- 反馈读写（servo_task 写, micro_ros 读） ----
      void updateServo(uint8_t id, int16_t pos, int16_t speed,
                       int16_t volt, int16_t temp,
                       uint8_t status, bool valid);

      ServoSnapshot getServo(uint8_t id) const;
      int getAllServos(ServoSnapshot *out, int max_count) const;

      // ---- 目标位置（micro_ros 写, servo_task 读） ----
      void setTarget(uint8_t id, uint16_t position, uint16_t time_ms);
      bool consumeTargets(DSSyncWriteData *out, int max_count, int *out_count);

      // ---- 指令队列（非位置指令） ----
      bool sendCommand(const ServoCommand &cmd);
      bool receiveCommand(ServoCommand *cmd, TickType_t timeout);

      // ---- 系统状态 ----
      bool isEmergencyStop() const;
      void setEmergencyStop(bool stop);

      bool isManualMode() const;
      void setManualMode(bool manual);

private:
      ServoSnapshot _servos[MAX_SERVO_SLOTS + 1];   // ID 做索引 (1-based)

      SemaphoreHandle_t _mutex;
      QueueHandle_t _cmd_queue;
      volatile bool _emergency_stop;
      volatile bool _manual_mode;

      // 批量目标位置
      uint16_t _target_pos [MAX_SERVO_SLOTS + 1];
      uint16_t _target_time[MAX_SERVO_SLOTS + 1];
      volatile bool _targets_dirty;
  };

