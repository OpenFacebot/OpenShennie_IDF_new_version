#include "servo_config.h"
#include <cstring>

int servo_config_init(ServoConfig *out, int max_count) {
    const ServoConfig table[] = {
        // ==== 眉毛 & 眼睑 (01-08) ====
          { 1,  "BrowInner_Right",    0, 4095, 2048 },
          { 2,  "BrowInner_Left",     0, 4095, 2048 },
          { 3,  "BrowOuter_Right",    0, 4095, 2048 },
          { 4,  "BrowOuter_Left",     0, 4095, 2048 },
          { 5,  "EyelidUpper_Right",  0, 4095, 2048 },
          { 6,  "EyelidUpper_Left",   0, 4095, 2048 },
          { 7,  "EyelidLower_Right",  0, 4095, 2048 },
          { 8,  "EyelidLower_Left",   0, 4095, 2048 },

          // ==== 眼球 (13-14) ====
          { 13, "Eye_Pitch",          0, 4095, 2048 },
          { 14, "Eye_Yaw",            0, 4095, 2048 },

          // ==== 下颌 (17-19) ====
          { 17, "Jaw_Open",           0, 4095, 2048 },
          { 18, "Jaw_Right",          0, 4095, 2048 },
          { 19, "Jaw_Left",           0, 4095, 2048 },

          // ==== 嘴唇 (21-27) ====
          { 21, "MouthCornerUp_Right",   0, 4095, 2048 },
          { 22, "MouthCornerUp_Left",    0, 4095, 2048 },
          { 23, "MouthCornerDown_Right", 0, 4095, 2048 },
          { 24, "MouthCornerDown_Left",  0, 4095, 2048 },
          { 25, "LipUpper_FrontBack",    0, 4095, 2048 },
          { 26, "LipLower_FrontBack",    0, 4095, 2048 },
          { 27, "LipLower_UpDown",       0, 4095, 2048 },

          // ==== 中庭 (33-35) ====
          { 33, "Cheek_Right",        0, 4095, 2048 },
          { 34, "Cheek_Left",         0, 4095, 2048 },
          { 35, "NoseWing",           0, 4095, 2048 },

          // ==== 脖子 (41-43) ====
          { 41, "Neck_Right",         0, 4095, 2048 },
          { 42, "Neck_Left",          0, 4095, 2048 },
          { 43, "Neck_Center",        0, 4095, 2048 },

    };

    int count = sizeof(table) / sizeof(table[0]);
    if (count > max_count) count = max_count;
    memcpy(out, table, count * sizeof(ServoConfig));
    return count; //返回26
}

// 以后从 NVS 加载
// → 新建一个 servo_config_load_nvs()
// → 把 memcpy(table→out) 换成 nvs_read(out)
// → 上层代码一行不改
