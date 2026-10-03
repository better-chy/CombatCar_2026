#ifndef __COMBATCAR_TUNE_CONFIG_H
#define __COMBATCAR_TUNE_CONFIG_H

/*
 * 快速调参模式。
 *
 * 1：跳过随机森林模型，减小固件体积，加快烧录速度。
 *    StageModel 固定返回“台上正常”，DangerModel 固定返回“安全”。
 *    适合调 LiDAR/索敌参数时使用。
 * 0：编译真实随机森林模型，用于正常跑车。
 *
 * 注意：正式比赛或真实跑车前必须改回 0。
 */
#define COMBATCAR_FAST_PARAM_TUNE 0U

/*
 * 电机强制停止调试。
 *
 * 1：所有 Motor_SetSpeeds 输出都会被强制置 0，适合桌面检查传感器/状态机。
 * 0：允许正常电机输出。
 *
 * 比赛前必须保持 0。
 */
#define COMBATCAR_MOTOR_DEBUG_FORCE_STOP 0U

/*
 * 阵营调试模式。
 *
 * 1：INIT 阶段跳过左右测距判队，强制按蓝方流程发车。
 * 0：使用左右激光测距自动判断蓝方/黄方起始位。
 *
 * 比赛前一般保持 0，除非现场明确需要固定蓝方流程。
 */
#define COMBATCAR_DEBUG_AUTO_TEAM 0U

/*
 * 训练/调试串口任务开关。
 *
 * 1：创建 Task_TrainLog，可用上位机采集训练帧、索敌线段可视化等调试功能。
 * 0：比赛模式下不创建 Task_TrainLog，减少无关线程和串口 debug 开销。
 */
#define COMBATCAR_ENABLE_TRAINLOG 0U

#endif
