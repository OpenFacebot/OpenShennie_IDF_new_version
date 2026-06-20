
#include <string.h>
#include "servo_state_db.h"
#include "DSServo.h"  // DSSyncWriteData 定义

ServoStateDB::ServoStateDB()
    : _emergency_stop(false), _manual_mode(true), _targets_dirty(false)
{
    memset(_servos, 0, sizeof(_servos));
    memset(_target_pos,  0, sizeof(_target_pos));
    memset(_target_time, 0, sizeof(_target_time));
    _mutex = xSemaphoreCreateMutex();
    _cmd_queue = xQueueCreate(CMD_QUEUE_LENGTH, sizeof(ServoCommand));
}

//---反馈读写---
void ServoStateDB::updateServo(uint8_t id, int16_t pos, int16_t speed,
                                int16_t volt, int16_t temp,  uint8_t st, bool valid)
{
      if (id == 0 || id > MAX_SERVO_SLOTS) return;
      xSemaphoreTake(_mutex, portMAX_DELAY);

        _servos[id].id = id;
        _servos[id].position = pos;
        _servos[id].speed = speed;
        _servos[id].voltage = volt;
        _servos[id].temperature = temp;
        _servos[id].status = st;
        _servos[id].valid = valid;
   
    xSemaphoreGive(_mutex);
}

ServoSnapshot ServoStateDB::getServo(uint8_t id) const
{
    ServoSnapshot snap = {};
    xSemaphoreTake(_mutex, portMAX_DELAY);
    if (id == 0 || id > MAX_SERVO_SLOTS) {
        xSemaphoreGive(_mutex);
        return snap;
    }
    snap = _servos[id];
    xSemaphoreGive(_mutex);
    return snap;
}

int ServoStateDB::getAllServos(ServoSnapshot *out, int max_count) const
{
    int count = 0;
    xSemaphoreTake(_mutex, portMAX_DELAY);
    for (int i = 0; i <= MAX_SERVO_SLOTS && count < max_count; i++) {
        if (_servos[i].valid) {
            out[count++] = _servos[i];
        }
    }
    xSemaphoreGive(_mutex);
    return count;
}

//---指令队列---
bool ServoStateDB::sendCommand(const ServoCommand &cmd)
{
    return xQueueSend(_cmd_queue, &cmd, 0) == pdTRUE;
}

bool ServoStateDB::receiveCommand(ServoCommand *cmd, TickType_t timeout)
{
    return xQueueReceive(_cmd_queue, cmd, timeout) == pdTRUE;
}

//---急停---
bool ServoStateDB::isEmergencyStop() const
{
    return _emergency_stop;
}

void ServoStateDB::setEmergencyStop(bool stop)
{
    _emergency_stop = stop;
}
//---模式---
bool ServoStateDB::isManualMode() const
{
    return _manual_mode;
}

void ServoStateDB::setManualMode(bool manual)
{
    _manual_mode = manual;
}

//---目标位置---
void ServoStateDB::setTarget(uint8_t id, uint16_t position, uint16_t time_ms)
{
    if (id == 0 || id > MAX_SERVO_SLOTS) return;
    xSemaphoreTake(_mutex, portMAX_DELAY);
    _target_pos[id]  = position;
    _target_time[id] = time_ms;
    _targets_dirty   = true;
    xSemaphoreGive(_mutex);
}

bool ServoStateDB::consumeTargets(DSSyncWriteData *out, int max_count, int *out_count)
{
    xSemaphoreTake(_mutex, portMAX_DELAY);
    if (!_targets_dirty) {
        xSemaphoreGive(_mutex);
        return false;
    }
    int count = 0;
    for (int id = 1; id <= MAX_SERVO_SLOTS && count < max_count; id++) {
        if (_target_pos[id] > 0 || _target_time[id] > 0) {
            out[count].id       = (uint8_t)id;
            out[count].position = _target_pos[id];
            out[count].time_ms  = _target_time[id];
            count++;
        }
    }
    _targets_dirty = false;
    xSemaphoreGive(_mutex);
    if (out_count) *out_count = count;
    return count > 0;
}
