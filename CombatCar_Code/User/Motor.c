#include "Motor.h"
#include "main.h"
#include "cmsis_os.h"

static int left_last_speed = 0;
static int right_last_speed = 0;

void Motor_Init(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

    Motor_StopAll();
}

void Motor_SetSpeed(int id, int pwm)
{
    if (pwm >= 900)
    {
        pwm = 900;
    }
    else if (pwm <= -900)
    {
        pwm = -900;
    }

    if (id == 1)
    {
        if (pwm > 0)
        {
            //反转保护
            if (left_last_speed < 0)
            {
                __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 0);
                __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 0);
                osDelay(100);
            }
            __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, pwm);
            __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 0);
            left_last_speed = pwm;
        }
        else if (pwm < 0)
        {
            if (left_last_speed > 0)
            {
                __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 0);
                __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 0);
                osDelay(100);
            }
            __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 0);
            __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, -pwm);
            left_last_speed = pwm;
        }
        else if (pwm == 0)
        {
            __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 0);
            __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 0);
            left_last_speed = 0;
        }
    }
    else if (id == 2)
    {
        if (pwm > 0)
        {
            if (right_last_speed < 0)
            {
                __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 0);
                __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 0);
                osDelay(100);
            }
            __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, pwm);
            __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 0);
            right_last_speed = pwm;
        }
        else if (pwm < 0)
        {
            if (right_last_speed > 0)
            {
                __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 0);
                __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 0);
                osDelay(100);
            }
            __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 0);
            __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, -pwm);
            right_last_speed = pwm;
        }
        else if (pwm == 0)
        {
            __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 0);
            __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 0);
            right_last_speed = 0;
        }
    }
}

void Motor_StopAll(void) // 停止所有电机
{
    Motor_SetSpeed(1, 0);
    Motor_SetSpeed(2, 0);
}

void Run_Forward(int speed)
{
    //防呆
    if (speed < 0)
    {
        speed = -speed;
    }
    Motor_SetSpeed(1, speed);
    Motor_SetSpeed(2, speed);
}

void Run_Back(int speed)
{
    // 防呆
    if (speed > 0)
    {
        speed = -speed;
    }
    Motor_SetSpeed(1, speed);
    Motor_SetSpeed(2, speed);
}

void Turn_Left(int left_speed, int right_speed)
{
    Motor_SetSpeed(1, left_speed);
    Motor_SetSpeed(2, right_speed);
}

void Turn_Right(int left_speed, int right_speed)
{
    Motor_SetSpeed(1, left_speed);
    Motor_SetSpeed(2, right_speed);
}
