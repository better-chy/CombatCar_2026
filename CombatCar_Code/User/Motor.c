#include "Motor.h"
#include "Task_Danger.h"

#include "cmsis_os.h"
#include "main.h"

/* 电机输出限幅 */
#define MOTOR_MAX_PWM       800 /* 单轮允许输出的最大 PWM，保护电机驱动和电源。 */
#define MOTOR_PWM_DEADZONE   20 /* 小于该绝对值的 PWM 视为 0，避免电机低占空比抖动。 */

volatile uint8_t g_debug_motor_kill = 0U;

static int Motor_ClampSpeed(int pwm);
static int Motor_ApplyDeadzone(int pwm);
static void Motor_ApplyOutputs(int left_pwm, int right_pwm);

void Motor_Init(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

    Motor_StopAll();
}

/* 将 PWM 限制在 90% 占空比以内，给电机驱动、电源和机械冲击留余量。 */
static int Motor_ClampSpeed(int pwm)
{
    if (pwm > MOTOR_MAX_PWM)
    {
        return MOTOR_MAX_PWM;
    }

    if (pwm < -MOTOR_MAX_PWM)
    {
        return -MOTOR_MAX_PWM;
    }

    return pwm;
}

static int Motor_ApplyDeadzone(int pwm)
{
    if (pwm > -MOTOR_PWM_DEADZONE && pwm < MOTOR_PWM_DEADZONE)
    {
        return 0;
    }

    return pwm;
}

static void Motor_ApplyOutputs(int left_pwm, int right_pwm)
{
    /* 每侧电机使用两路 PWM 控制 H 桥方向：一路负责正转，另一路负责反转。 */
    if (left_pwm > 0)
    {
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, left_pwm);
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 0);
    }
    else if (left_pwm < 0)
    {
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 0);
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, -left_pwm);
    }
    else
    {
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 0);
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 0);
    }

    if (right_pwm > 0)
    {
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, right_pwm);
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 0);
    }
    else if (right_pwm < 0)
    {
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 0);
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, -right_pwm);
    }
    else
    {
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 0);
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 0);
    }
}

void Motor_SetSpeeds(int left_speed, int right_speed)
{
    /* 检录、桌面调试或异常排查时，可通过软件开关强制禁止所有动力输出。 */
    if (COMBATCAR_MOTOR_DEBUG_FORCE_STOP != 0U || g_debug_motor_kill != 0U)
    {
        Motor_ApplyOutputs(0, 0);
        return;
    }

    left_speed = Motor_ApplyDeadzone(Motor_ClampSpeed(left_speed));
    right_speed = Motor_ApplyDeadzone(Motor_ClampSpeed(right_speed));
    TaskDanger_ApplySpeedLimitPair(&left_speed, &right_speed);
    Motor_ApplyOutputs(left_speed, right_speed);
}

void Motor_StopAll(void)
{
    Motor_SetSpeeds(0, 0);
}

void Run_Forward(int speed)
{
    if (speed < 0)
    {
        speed = -speed;
    }

    Motor_SetSpeeds(speed, speed);
}

void Run_Back(int speed)
{
    if (speed > 0)
    {
        speed = -speed;
    }

    Motor_SetSpeeds(speed, speed);
}

void Turn_Left(int left_speed, int right_speed)
{
    Motor_SetSpeeds(left_speed, right_speed);
}

void Turn_Right(int left_speed, int right_speed)
{
    Motor_SetSpeeds(left_speed, right_speed);
}
