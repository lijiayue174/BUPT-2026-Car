#include "headfile.h"
#include "gimbal.h"

/* ============================================================================
 *  gimbal.c —— 见 gimbal.h 顶部说明
 *
 *  Servo power safety:
 *   - servo red wire: external 5V/6A buck module;
 *   - servo black/brown wire: external power GND;
 *   - external 5V/6A GND must be connected to MSPM0 GND;
 *   - yaw servo signal wire: PA14;
 *   - pitch servo signal wire: PA16;
 *   - do not power the servo from MSPM0 3.3V.
 *   - field note: if yaw only works while pressing the breadboard GND rail,
 *     fix hardware first. The 5V buck GND, yaw GND, pitch GND, and MSPM0 GND
 *     must be tied to the same reliable ground point.
 *
 *  Servo PWM is 50Hz / 20ms. Timer count clock is 10MHz, so 1us = 10 counts.
 *  PA14 yaw writes TIMG12 CCR directly. PA16 pitch uses GPIO software PWM.
 * ========================================================================== */

int gimbal_yaw_us_now   = GIMBAL_YAW_CENTER_US;
int gimbal_pitch_us_now = GIMBAL_PITCH_CENTER_US;

static int gimbal_sweep_interval_tick = 0;
static int gimbal_sweep_yaw_us = GIMBAL_YAW_MIN_SAFE_US;
static int gimbal_sweep_pitch_us = GIMBAL_PITCH_MAX_SAFE_US;
static int gimbal_sweep_yaw_dir = 1;
static int gimbal_sweep_pitch_dir = -1;

/* Pulse width(us) -> raw compare count. 10MHz timer clock means 1us = 10 counts. */
static inline uint32_t gimbal_us_to_ccr(int us)
{
    return (uint32_t)(us * 10);
}

static int gimbal_clamp_yaw_us(int us)
{
#if GIMBAL_USE_SAFE_LIMIT
    if (us < GIMBAL_YAW_MIN_SAFE_US) us = GIMBAL_YAW_MIN_SAFE_US;
    if (us > GIMBAL_YAW_MAX_SAFE_US) us = GIMBAL_YAW_MAX_SAFE_US;
#else
    if (us < GIMBAL_US_MIN) us = GIMBAL_US_MIN;
    if (us > GIMBAL_US_MAX) us = GIMBAL_US_MAX;
#endif
    return us;
}

static int gimbal_clamp_pitch_us(int us)
{
#if GIMBAL_USE_SAFE_LIMIT
    if (us < GIMBAL_PITCH_MIN_SAFE_US) us = GIMBAL_PITCH_MIN_SAFE_US;
    if (us > GIMBAL_PITCH_MAX_SAFE_US) us = GIMBAL_PITCH_MAX_SAFE_US;
#else
    if (us < GIMBAL_US_MIN) us = GIMBAL_US_MIN;
    if (us > GIMBAL_US_MAX) us = GIMBAL_US_MAX;
#endif
    return us;
}

static void gimbal_test_reset_state(void)
{
    gimbal_sweep_interval_tick = 0;
    gimbal_sweep_yaw_us = GIMBAL_YAW_MIN_SAFE_US;
    gimbal_sweep_pitch_us = GIMBAL_PITCH_MAX_SAFE_US;
    gimbal_sweep_yaw_dir = 1;
    gimbal_sweep_pitch_dir = -1;
}

#if GIMBAL_PITCH_ENABLE
static void gimbal_pitch_soft_pwm_init(void)
{
    /*
     * PA16 GPIO software PWM. GPIO diagnosis proved PA16 reaches the header;
     * TIMA1 hardware PWM is intentionally not used. This blocking one-frame
     * service is for stopped gimbal aiming/test phases only, not line following.
     */
    DL_GPIO_initDigitalOutput(IOMUX_PINCM38);
    DL_GPIO_enableOutput(GPIOA, DL_GPIO_PIN_16);
    DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_16);
}
#endif

void gimbal_init(void)
{
    /* TIMG12 CC0 -> PA14（yaw）。pwm_init 内部已做引脚复用与时钟配置。 */
    pwm_init(GIMBAL_YAW_TIMER, GIMBAL_YAW_CH, GIMBAL_PWM_FREQ);

#if GIMBAL_PITCH_ENABLE
    gimbal_pitch_soft_pwm_init();
#endif

    /* 上电先回中位，避免突然打到限位。 */
    gimbal_center();
}

void gimbal_set_yaw_us(uint16_t yaw_us)
{
    int safe_yaw_us;

    safe_yaw_us = gimbal_clamp_yaw_us((int)yaw_us);
    gimbal_yaw_us_now = safe_yaw_us;
    /*
     * Do not use pwm_update() here: TIMG12 50Hz uses LOAD=199999, while
     * ml_pwm.c reads LOAD into uint16_t and truncates it. Write CCR directly
     * so 1500us becomes 15000 timer counts on PA14/TIMG12_CCP0.
     */
    DL_TimerG_setCaptureCompareValue(GIMBAL_YAW_TIMER,
        gimbal_us_to_ccr(safe_yaw_us),
        GIMBAL_YAW_CH);
}

void gimbal_set_pitch_us(uint16_t pitch_us)
{
    int safe_pitch_us;

    safe_pitch_us = gimbal_clamp_pitch_us((int)pitch_us);
    gimbal_pitch_us_now = safe_pitch_us;
#if GIMBAL_PITCH_ENABLE
    (void)safe_pitch_us;
#endif
}

void gimbal_pitch_soft_pwm_service(void)
{
#if GIMBAL_PITCH_ENABLE
    int high_us;
    int low_us;

    high_us = gimbal_clamp_pitch_us(gimbal_pitch_us_now);
    low_us = 20000 - high_us;
    if (low_us < 0) {
        low_us = 0;
    }

    DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_16);
    delay_us((uint32_t)high_us);
    DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_16);
    delay_us((uint32_t)low_us);
#endif
}

void gimbal_set_pulse_us(uint16_t yaw_us, uint16_t pitch_us)
{
    gimbal_set_yaw_us(yaw_us);
    gimbal_set_pitch_us(pitch_us);
}

void gimbal_center(void)
{
    gimbal_test_reset_state();
    gimbal_set_pulse_us(GIMBAL_YAW_CENTER_US, GIMBAL_CENTER_PITCH_US);
}

void gimbal_aim_B(void)
{
    /* ⚠ 这两个值必须在真实赛道上对 B 点靶面标定后填入 gimbal.h 的宏。 */
    gimbal_set_pulse_us(GIMBAL_AIM_B_YAW_US, GIMBAL_AIM_B_PITCH_US);
}

/* ----------------------------------------------------------------------------
 * Non-blocking mode7 sweep: PA14 yaw and PA16 pitch both output real servo PWM.
 * -------------------------------------------------------------------------- */
void gimbal_test_sweep_safe(void)
{
    gimbal_sweep_interval_tick++;
    if (gimbal_sweep_interval_tick < (GIMBAL_SWEEP_INTERVAL_MS / 10)) {
        return;
    }
    gimbal_sweep_interval_tick = 0;

    gimbal_sweep_yaw_us += GIMBAL_STEP_US * gimbal_sweep_yaw_dir;
    if (gimbal_sweep_yaw_us >= GIMBAL_YAW_MAX_SAFE_US) {
        gimbal_sweep_yaw_us = GIMBAL_YAW_MAX_SAFE_US;
        gimbal_sweep_yaw_dir = -1;
    } else if (gimbal_sweep_yaw_us <= GIMBAL_YAW_MIN_SAFE_US) {
        gimbal_sweep_yaw_us = GIMBAL_YAW_MIN_SAFE_US;
        gimbal_sweep_yaw_dir = 1;
    }

    gimbal_sweep_pitch_us += GIMBAL_STEP_US * gimbal_sweep_pitch_dir;
    if (gimbal_sweep_pitch_us >= GIMBAL_PITCH_MAX_SAFE_US) {
        gimbal_sweep_pitch_us = GIMBAL_PITCH_MAX_SAFE_US;
        gimbal_sweep_pitch_dir = -1;
    } else if (gimbal_sweep_pitch_us <= GIMBAL_PITCH_MIN_SAFE_US) {
        gimbal_sweep_pitch_us = GIMBAL_PITCH_MIN_SAFE_US;
        gimbal_sweep_pitch_dir = 1;
    }

    gimbal_set_pulse_us((uint16_t)gimbal_sweep_yaw_us,
        (uint16_t)gimbal_sweep_pitch_us);
}
