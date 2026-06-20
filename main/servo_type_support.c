// Stub type support for servo_msgs — minimal implementations
#include <string.h>
#include <rosidl_runtime_c/message_type_support_struct.h>
#include <rosidl_runtime_c/service_type_support_struct.h>

const rosidl_message_type_support_t *
rosidl_typesupport_c__get_message_type_support_handle__servo_msgs__msg__ServoCommand(void) {
    static rosidl_message_type_support_t ts;
    memset(&ts, 0, sizeof(ts));
    return &ts;
}

const rosidl_service_type_support_t *
rosidl_typesupport_c__get_service_type_support_handle__servo_msgs__srv__GetServoInfo(void) {
    static rosidl_service_type_support_t ts;
    memset(&ts, 0, sizeof(ts));
    return &ts;
}
