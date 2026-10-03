#ifndef __TASK_DANGER_H
#define __TASK_DANGER_H

#include "main.h"

typedef enum
{
    EDGE_NONE = 0,
    EDGE_LEFT,  /* 左侧边缘传感器触发。 */
    EDGE_RIGHT, /* 右侧边缘传感器触发。 */
    EDGE_FRONT  /* 两侧同时触发，视为车头接近台沿。 */
} EdgeDangerType;

extern EdgeDangerType edge_danger_type; /* 当前硬件边缘触发类型。 */
extern volatile uint8_t g_danger_slowdown_active; /* 危险减速是否激活。 */
extern volatile uint8_t g_danger_far_edge_soft_slowdown_active; /* TRACK/ATTACK 中二级光电软限速是否激活。 */
extern volatile uint8_t g_danger_model_label; /* 危险模型最近一次输出标签。 */
extern volatile uint32_t g_danger_stack_free_words; /* DangerTask 剩余栈空间，单位为 FreeRTOS stack word。 */

/* 危险检测线程入口。 */
void Task_Danger_Run(void *argument);
/* 查询危险减速是否处于激活状态。 */
uint8_t TaskDanger_IsSlowdownActive(void);
/* 当前是否允许启用随机森林减速保护。 */
uint8_t TaskDanger_ShouldUseSlowdownProtection(void);
/* 当前是否允许启用更靠前的二级光电边缘保护。 */
uint8_t TaskDanger_ShouldUseFarEdgeProtection(void);
/* 临时屏蔽近处和远处光电边缘保护。 */
void TaskDanger_SuppressEdgeProtection(uint32_t duration_ms);
/* 查询近处和远处光电边缘保护是否处于临时屏蔽期。 */
uint8_t TaskDanger_IsEdgeProtectionSuppressed(void);
/* 对单个 PWM 做危险限速。 */
int TaskDanger_ApplySpeedLimit(int pwm);
/* 对左右轮 PWM 做危险限速。 */
void TaskDanger_ApplySpeedLimitPair(int *left_pwm, int *right_pwm);

#endif
