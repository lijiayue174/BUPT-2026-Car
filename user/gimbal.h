#ifndef __gimbal_h_
#define __gimbal_h_

#include "headfile.h"

/* ============================================================================
 *  gimbal.h —— 2 自由度舵机云台控制（北邮 2026 电赛 / 任务2 定点瞄准, mode==6）
 * ----------------------------------------------------------------------------
 *  设计依据（已对照 senior 工程实测）：
 *   - PWM 底层用 ml_pwm.c 的 pwm_init / pwm_update（不改 ml_libs）。
 *   - yaw 舵机 = TIMG12 CC0 = PA14（PINMAP.md 确认唯一空闲且无冲突的 TIMG PWM 通道）。
 *   - TIMG12 @ 50Hz 时：psc=0, arr=199999, 计数 10MHz（1 计数=0.1us，周期 20ms）。
 *     pwm_update 的 duty 与脉宽关系： 脉宽us = duty * 2  ⇒  duty = 脉宽us / 2
 *     例：1500us(中位)->duty750；500us->250；2500us->1250；1300us->650；1700us->850。
 *
 *  ⚠ pitch（俯仰）舵机：TIMG12 CC1=PA24 已被右编码器占用，ml_pwm 支持的其它
 *     TIMG 通道（TIMG0/6/7/8）也都已占用（编码器/电机/按键/中断）。因此第二路
 *     舵机需要先在 SysConfig 里分配一个空闲 TIMA0/TIMA1 通道并在 ml_pwm.c 中加一个
 *     分支（PINMAP.md 方案B）。在此之前，本模块默认 **只驱动 yaw（单舵机即可上电测试）**，
 *     pitch 由 GIMBAL_PITCH_ENABLE 开关与下面两个宏接入，解决引脚后改 1 即可。
 *
 *  安全（开工七阶段·阶段6 强调）：
 *   - 初版限幅 1300~1700us，确认机械限位后再放开到 GIMBAL_US_MIN/MAX。
 *   - 不要一上来就扫全程 500~2500us，可能撞硬限位。gimbal_test_sweep_safe() 只在安全窗口扫。
 * ========================================================================== */

/* ---- 舵机通道配置（按硬件改这里，不要改函数体） ---- */
#define GIMBAL_YAW_TIMER     TIMG12
#define GIMBAL_YAW_CH        DL_TIMER_CC_0_INDEX      /* PA14 */

#define GIMBAL_PITCH_ENABLE  0                        /* 解决 pitch 引脚后改成 1 */
#define GIMBAL_PITCH_TIMER   TIMG12                   /* TODO: 改为分配好的 TIMA0/1 实例 */
#define GIMBAL_PITCH_CH      DL_TIMER_CC_1_INDEX      /* TODO: 改为对应通道（当前 PA24 与编码器冲突，禁止直接用） */

/* ---- PWM 频率与脉宽常量（单位 us） ---- */
#define GIMBAL_PWM_FREQ      50                       /* 标准舵机 50Hz / 20ms */
#define GIMBAL_US_MIN        500                      /* 物理全程下限（确认限位后才可用到） */
#define GIMBAL_US_MAX        2500                     /* 物理全程上限 */
#define GIMBAL_US_SAFE_MIN   1300                     /* 初版安全窗口下限 */
#define GIMBAL_US_SAFE_MAX   1700                     /* 初版安全窗口上限 */
#define GIMBAL_US_CENTER     1500                     /* 中位 */

/* 是否启用安全窗口限幅：调试初期=1（强制夹到1300~1700）；标定完限位后改0放开全程 */
#define GIMBAL_USE_SAFE_LIMIT 1

/* ---- 瞄准 B 点的预置脉宽（必须在真实硬件上标定！下面是占位初值） ---- */
#define GIMBAL_AIM_B_YAW_US   1500
#define GIMBAL_AIM_B_PITCH_US 1500
#define GIMBAL_CENTER_YAW_US  1500
#define GIMBAL_CENTER_PITCH_US 1500

/* ---- 对外接口 ---- */
void gimbal_init(void);                              /* 初始化 PWM，并回中位（安全） */
void gimbal_set_pulse_us(int yaw_us, int pitch_us);  /* 直接给两个舵机脉宽（带限幅） */
void gimbal_set_yaw_us(int yaw_us);                  /* 只给 yaw */
void gimbal_center(void);                            /* 回中位 */
void gimbal_aim_B(void);                             /* 瞄准 B 点固定姿态（标定后用） */

/* 安全自检扫动：在 1300~1700us 之间缓慢来回，用于上电验证舵机会动、方向正确 */
void gimbal_test_sweep_safe(void);

/* 供调试/OLED 读取的当前脉宽 */
extern int gimbal_yaw_us_now;
extern int gimbal_pitch_us_now;

#endif
