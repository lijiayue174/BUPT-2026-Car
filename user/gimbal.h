#ifndef __gimbal_h_
#define __gimbal_h_

#include "headfile.h"

/* ============================================================================
 *  gimbal.h —— two-servo gimbal control (mode7 independent safety test)
 * ----------------------------------------------------------------------------
 *  设计依据（已对照 senior 工程实测）：
 *   - PWM 底层用 ml_pwm.c 的 pwm_init / pwm_update（不改 ml_libs）。
 *   - yaw 舵机 = TIMG12 CC0 = PA14。
 *   - pitch 舵机 = TIMA1 CC1 = PA16。
 *   - TIMG12 @ 50Hz 时：psc=0, arr=199999, 计数 10MHz（1 计数=0.1us，周期 20ms）。
 *     pwm_update 的 duty 与脉宽关系： 脉宽us = duty * 2  ⇒  duty = 脉宽us / 2
 *     例：1500us(中位)->duty750；500us->250；2500us->1250；1300us->650；1700us->850。
 *
 *   - PA16 在 MSPM0G3507 头文件中可复用为 IOMUX_PINCM38_PF_TIMA1_CCP1。
 *     为避免改 ml_libs，pitch 的 TIMA1 PWM 初始化在 gimbal.c 内部完成。
 *
 *  舵机供电安全：
 *   - 舵机红线接外部 5V/6A 降压模块；
 *   - 舵机黑线/棕线接外部电源 GND；
 *   - 外部 5V/6A GND 必须和 MSPM0 GND 共地；
 *   - yaw 舵机信号线接 PA14，pitch 舵机信号线接 PA16；
 *   - 不要用 MSPM0 3.3V 给舵机供电。
 *
 *  安全（开工七阶段·阶段6 强调）：
 *   - 初版限幅 1300~1700us，确认机械限位后再放开到 GIMBAL_US_MIN/MAX。
 *   - 不要一上来就扫全程 500~2500us，可能撞硬限位。gimbal_test_sweep_safe() 只在安全窗口扫。
 * ========================================================================== */

/* ---- 舵机通道配置（按硬件改这里，不要改函数体） ---- */
#define GIMBAL_YAW_TIMER     TIMG12
#define GIMBAL_YAW_CH        DL_TIMER_CC_0_INDEX      /* PA14 / TIMG12_CCP0 */

#define GIMBAL_PITCH_ENABLE  1
#define GIMBAL_PITCH_SOFT_PWM 1                       /* PA16 GPIO software PWM */
/* Current strategy: PA16 pitch uses GPIO software PWM, not TIMA1 hardware PWM. */

/* ---- PWM 频率与脉宽常量（单位 us） ---- */
#define GIMBAL_PWM_FREQ      50                       /* 标准舵机 50Hz / 20ms */
#define GIMBAL_US_MIN        500                      /* 物理全程下限（确认限位后才可用到） */
#define GIMBAL_US_MAX        2500                     /* 物理全程上限 */
#define GIMBAL_US_SAFE_MIN   1300                     /* 初版安全窗口下限 */
#define GIMBAL_US_SAFE_MAX   1700                     /* 初版安全窗口上限 */
#define GIMBAL_US_CENTER     1500                     /* 中位 */

#define GIMBAL_YAW_CENTER_US       1500
#define GIMBAL_YAW_MIN_SAFE_US     1300
#define GIMBAL_YAW_MAX_SAFE_US     1700

#define GIMBAL_PITCH_CENTER_US     1500
#define GIMBAL_PITCH_MIN_SAFE_US   1300
#define GIMBAL_PITCH_MAX_SAFE_US   1700

#define GIMBAL_STEP_US             10
#define GIMBAL_SWEEP_INTERVAL_MS   20

/* 是否启用安全窗口限幅：调试初期=1（强制夹到1300~1700）；标定完限位后改0放开全程 */
#define GIMBAL_USE_SAFE_LIMIT 1

/* ---- 瞄准 B 点的预置脉宽（必须在真实硬件上标定！下面是占位初值） ---- */
#define GIMBAL_AIM_B_YAW_US   1500
#define GIMBAL_AIM_B_PITCH_US 1500
#define GIMBAL_CENTER_YAW_US   1500
#define GIMBAL_CENTER_PITCH_US 1500

/* ---- 对外接口 ---- */
void gimbal_init(void);                              /* 初始化 PWM，并回中位（安全） */
void gimbal_set_pulse_us(uint16_t yaw_us, uint16_t pitch_us);  /* 直接给两个舵机脉宽（带限幅） */
void gimbal_set_yaw_us(uint16_t yaw_us);             /* 只给 yaw */
void gimbal_set_pitch_us(uint16_t pitch_us);         /* 只给 pitch */
void gimbal_pitch_soft_pwm_service(void);            /* PA16 software PWM, one 20ms frame */
void gimbal_center(void);                            /* 回中位 */
void gimbal_aim_B(void);                             /* 瞄准 B 点固定姿态（标定后用） */

/* 安全自检扫动：在 1300~1700us 之间缓慢来回，用于上电验证舵机会动、方向正确 */
void gimbal_test_sweep_safe(void);

/* 供调试/OLED 读取的当前脉宽 */
extern int gimbal_yaw_us_now;
extern int gimbal_pitch_us_now;

#endif
