#include "headfile.h"
#include "gimbal.h"

/* ============================================================================
 *  gimbal.c —— 见 gimbal.h 顶部说明
 *  关键换算（TIMG12 @ 50Hz）：duty = 脉宽us / 2  （pwm_update 的 duty 量程 0~10000）
 *  若需要更高分辨率（0.1us），可直接写 CCR：
 *      DL_TimerG_setCaptureCompareValue(TIMG12, us*10, ch);
 *  本实现用 pwm_update（2us≈0.27°，瞄 A4 靶绰绰有余，且不碰 ml_libs）。
 * ========================================================================== */

int gimbal_yaw_us_now   = GIMBAL_US_CENTER;
int gimbal_pitch_us_now = GIMBAL_US_CENTER;

/* 脉宽(us) -> pwm_update 的 duty。仅对 TIMG12 50Hz 成立（arr=199999, 10MHz 计数）。 */
static inline uint32_t gimbal_us_to_duty(int us)
{
    return (uint32_t)(us / 2);
}

/* 限幅：调试初期夹到安全窗口，标定完机械限位后（GIMBAL_USE_SAFE_LIMIT=0）放开到全程。 */
static int gimbal_clamp_us(int us)
{
#if GIMBAL_USE_SAFE_LIMIT
    if (us < GIMBAL_US_SAFE_MIN) us = GIMBAL_US_SAFE_MIN;
    if (us > GIMBAL_US_SAFE_MAX) us = GIMBAL_US_SAFE_MAX;
#else
    if (us < GIMBAL_US_MIN) us = GIMBAL_US_MIN;
    if (us > GIMBAL_US_MAX) us = GIMBAL_US_MAX;
#endif
    return us;
}

void gimbal_init(void)
{
    /* TIMG12 CC0 -> PA14（yaw）。pwm_init 内部已做引脚复用与时钟配置。 */
    pwm_init(GIMBAL_YAW_TIMER, GIMBAL_YAW_CH, GIMBAL_PWM_FREQ);

#if GIMBAL_PITCH_ENABLE
    pwm_init(GIMBAL_PITCH_TIMER, GIMBAL_PITCH_CH, GIMBAL_PWM_FREQ);
#endif

    /* 上电先回中位，避免突然打到限位。 */
    gimbal_center();
}

void gimbal_set_yaw_us(int yaw_us)
{
    yaw_us = gimbal_clamp_us(yaw_us);
    gimbal_yaw_us_now = yaw_us;
    pwm_update(GIMBAL_YAW_TIMER, GIMBAL_YAW_CH, gimbal_us_to_duty(yaw_us));
}

void gimbal_set_pulse_us(int yaw_us, int pitch_us)
{
    gimbal_set_yaw_us(yaw_us);

#if GIMBAL_PITCH_ENABLE
    pitch_us = gimbal_clamp_us(pitch_us);
    gimbal_pitch_us_now = pitch_us;
    pwm_update(GIMBAL_PITCH_TIMER, GIMBAL_PITCH_CH, gimbal_us_to_duty(pitch_us));
#else
    (void)pitch_us;   /* pitch 未接入，仅记录请求值便于调试 */
    gimbal_pitch_us_now = pitch_us;
#endif
}

void gimbal_center(void)
{
    gimbal_set_pulse_us(GIMBAL_CENTER_YAW_US, GIMBAL_CENTER_PITCH_US);
}

void gimbal_aim_B(void)
{
    /* ⚠ 这两个值必须在真实赛道上对 B 点靶面标定后填入 gimbal.h 的宏。 */
    gimbal_set_pulse_us(GIMBAL_AIM_B_YAW_US, GIMBAL_AIM_B_PITCH_US);
}

/* ----------------------------------------------------------------------------
 * 非阻塞安全自检扫动：每次调用前进一步，在 1300~1700us 之间缓慢来回。
 * 用法：mode==6 时在 10ms 的 TIMG8 ISR 里每周期调一次（见集成指南）。
 * 现象：yaw 舵机缓慢左右摆动；据此确认舵机会动、方向、限位是否合理。
 * -------------------------------------------------------------------------- */
void gimbal_test_sweep_safe(void)
{
    static int us = GIMBAL_US_SAFE_MIN;
    static int dir = 1;          /* +1 向上 / -1 向下 */
    const int step = 4;          /* 每 10ms 走 4us → 全窗口约 1s 单程，平滑不抖 */

    us += dir * step;
    if (us >= GIMBAL_US_SAFE_MAX) { us = GIMBAL_US_SAFE_MAX; dir = -1; }
    if (us <= GIMBAL_US_SAFE_MIN) { us = GIMBAL_US_SAFE_MIN; dir =  1; }

    gimbal_set_yaw_us(us);
}
