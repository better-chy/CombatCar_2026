#ifndef __PID_H
#define __PID_H

#include "main.h"
#include <stdint.h>

typedef struct
{
    float kp; /* 比例系数。 */
    float ki; /* 积分系数。 */
    float kd; /* 微分系数。 */
    float integral; /* 当前累计积分项。 */
    float prev_error; /* 上一次误差。 */
    float integral_limit; /* 积分项限幅。 */
    float output_limit; /* 输出限幅。 */
} AnglePid;

void AnglePid_Init(AnglePid *pid,
                   float kp,
                   float ki,
                   float kd,
                   float integral_limit,
                   float output_limit);
void AnglePid_Reset(AnglePid *pid);
float AnglePid_NormalizeDeg(float angle_deg);
float AnglePid_Update(AnglePid *pid, float target_angle_deg, float current_angle_deg);
void AnglePid_ComputeTrackSpeeds(AnglePid *pid,
                                 uint8_t reset_pid,
                                 int16_t enemy_angle_deg,
                                 int forward_speed,
                                 int *left_speed,
                                 int *right_speed);

#endif
