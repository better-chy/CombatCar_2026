#include "PID.h"

/* 将任意角度归一化到 -180 到 180 度。 */
float AnglePid_NormalizeDeg(float angle_deg)
{
    while (angle_deg > 180.0f)
    {
        angle_deg -= 360.0f;
    }

    while (angle_deg < -180.0f)
    {
        angle_deg += 360.0f;
    }

    return angle_deg;
}

/* 初始化角度 PID 参数与运行时状态。 */
void AnglePid_Init(AnglePid *pid,
                   float kp,
                   float ki,
                   float kd,
                   float integral_limit,
                   float output_limit)
{
    if (pid == NULL)
    {
        return;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->integral_limit = integral_limit;
    pid->output_limit = output_limit;
}

/* 清空角度 PID 的积分项和上一拍误差。 */
void AnglePid_Reset(AnglePid *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}

/* 计算一次角度 PID 输出。
 * target_angle_deg 为目标角度；
 * current_angle_deg 为当前角度；
 * 返回值为已完成限幅的修正输出。
 */
float AnglePid_Update(AnglePid *pid, float target_angle_deg, float current_angle_deg)
{
    float error_deg; /* 当前角度误差。 */
    float derivative; /* 当前微分项。 */
    float output; /* 当前 PID 输出。 */

    if (pid == NULL)
    {
        return 0.0f;
    }

    error_deg = AnglePid_NormalizeDeg(target_angle_deg - current_angle_deg);
    pid->integral += error_deg;

    if (pid->integral > pid->integral_limit)
    {
        pid->integral = pid->integral_limit;
    }
    else if (pid->integral < -pid->integral_limit)
    {
        pid->integral = -pid->integral_limit;
    }

    derivative = error_deg - pid->prev_error;
    output = (pid->kp * error_deg) + (pid->ki * pid->integral) + (pid->kd * derivative);

    if (output > pid->output_limit)
    {
        output = pid->output_limit;
    }
    else if (output < -pid->output_limit)
    {
        output = -pid->output_limit;
    }

    pid->prev_error = error_deg;
    return output;
}

void AnglePid_ComputeTrackSpeeds(AnglePid *pid,
                                 uint8_t reset_pid,
                                 int16_t enemy_angle_deg,
                                 int forward_speed,
                                 int *left_speed,
                                 int *right_speed)
{
    float correction;
    int base_speed;

    if (pid == NULL || left_speed == NULL || right_speed == NULL)
    {
        return;
    }

    if (reset_pid != 0U)
    {
        AnglePid_Reset(pid);
    }

    base_speed = forward_speed;

    correction = AnglePid_Update(pid, 0.0f, (float)enemy_angle_deg);
    *left_speed = base_speed - (int)correction;
    *right_speed = base_speed + (int)correction;

    /* 前进跟踪阶段保持边走边修正，不让单侧跨过 0 触发电机换向保护。 */
    if (base_speed > 0)
    {
        if (*left_speed < 0)
        {
            *left_speed = 0;
        }

        if (*right_speed < 0)
        {
            *right_speed = 0;
        }
    }
}
