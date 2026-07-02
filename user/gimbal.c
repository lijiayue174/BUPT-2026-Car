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
 *
 *  关键换算（TIMG12 @ 50Hz）：duty = 脉宽us / 2  （pwm_update 的 duty 量程 0~10000）
 *  若需要更高分辨率（0.1us），可直接写 CCR：
 *      DL_TimerG_setCaptureCompareValue(TIMG12, us*10, ch);
 *  本实现用 pwm_update（2us≈0.27°，瞄 A4 靶绰绰有余，且不碰 ml_libs）。
 * ========================================================================== */

int gimbal_yaw_us_now   = GIMBAL_YAW_CENTER_US;
int gimbal_pitch_us_now = GIMBAL_PITCH_CENTER_US;

typedef enum {
    GIMBAL_TEST_HOLD_BEFORE_YAW = 0,
    GIMBAL_TEST_YAW_TO_MAX,
    GIMBAL_TEST_YAW_TO_MIN,
    GIMBAL_TEST_YAW_TO_CENTER,
    GIMBAL_TEST_HOLD_BEFORE_PITCH,
    GIMBAL_TEST_PITCH_TO_MAX,
    GIMBAL_TEST_PITCH_TO_MIN,
    GIMBAL_TEST_PITCH_TO_CENTER,
    GIMBAL_TEST_HOLD_AFTER_PITCH
} GimbalTestState;

static GimbalTestState gimbal_test_state = GIMBAL_TEST_HOLD_BEFORE_YAW;
static int gimbal_test_hold_ticks = 0;
static int gimbal_test_interval_ticks = 0;
static int gimbal_sweep_yaw_us = GIMBAL_YAW_CENTER_US;
static int gimbal_sweep_pitch_us = GIMBAL_PITCH_CENTER_US;

/* 脉宽(us) -> pwm_update 的 duty。仅对 TIMG12 50Hz 成立（arr=199999, 10MHz 计数）。 */
static inline uint32_t gimbal_us_to_duty(int us)
{
    return (uint32_t)(us / 2);
}

/* 脉宽(us) -> TIMA1/TIMG raw compare count. 10MHz timer clock means 1us = 10 counts. */
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
    gimbal_test_state = GIMBAL_TEST_HOLD_BEFORE_YAW;
    gimbal_test_hold_ticks = 0;
    gimbal_test_interval_ticks = 0;
    gimbal_sweep_yaw_us = GIMBAL_YAW_CENTER_US;
    gimbal_sweep_pitch_us = GIMBAL_PITCH_CENTER_US;
}

#if GIMBAL_PITCH_ENABLE
static void gimbal_pitch_pwm_init(void)
{
    DL_TimerA_ClockConfig clock_config = {
        .clockSel = DL_TIMER_CLOCK_BUSCLK,
        .divideRatio = DL_TIMER_CLOCK_DIVIDE_8,
        .prescale = 0
    };
    DL_TimerA_PWMConfig pwm_config = {
        .pwmMode = DL_TIMER_PWM_MODE_EDGE_ALIGN_UP,
        .period = 10000000 / GIMBAL_PWM_FREQ - 1,
        .isTimerWithFourCC = false,
        .startTimer = DL_TIMER_START
    };

    /* PA16 -> TIMA1 CCP1. PB16/PB17 are not touched. */
    DL_GPIO_initPeripheralOutputFunction(IOMUX_PINCM38, IOMUX_PINCM38_PF_TIMA1_CCP1);
    DL_GPIO_enableOutput(GPIOA, DL_GPIO_PIN_16);

    GIMBAL_PITCH_TIMER->GPRCM.RSTCTL = GPTIMER_RSTCTL_KEY_UNLOCK_W;
    DL_TimerA_enablePower(GIMBAL_PITCH_TIMER);
    DL_TimerA_setClockConfig(GIMBAL_PITCH_TIMER, &clock_config);
    DL_TimerA_initPWMMode(GIMBAL_PITCH_TIMER, &pwm_config);
    DL_TimerA_setCaptureCompareValue(GIMBAL_PITCH_TIMER, 0, GIMBAL_PITCH_CH);
    DL_TimerA_setCaptureCompareOutCtl(GIMBAL_PITCH_TIMER,
        DL_TIMER_CC_OCTL_INIT_VAL_LOW,
        DL_TIMER_CC_OCTL_INV_OUT_DISABLED,
        DL_TIMER_CC_OCTL_SRC_FUNCVAL,
        GIMBAL_PITCH_CH);
    DL_TimerA_setCaptCompUpdateMethod(GIMBAL_PITCH_TIMER,
        DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE,
        GIMBAL_PITCH_CH);
    DL_TimerA_enableClock(GIMBAL_PITCH_TIMER);
    GIMBAL_PITCH_TIMER->COMMONREGS.CCPD |= 1 << GIMBAL_PITCH_CH;
}
#endif

void gimbal_init(void)
{
    /* TIMG12 CC0 -> PA14（yaw）。pwm_init 内部已做引脚复用与时钟配置。 */
    pwm_init(GIMBAL_YAW_TIMER, GIMBAL_YAW_CH, GIMBAL_PWM_FREQ);

#if GIMBAL_PITCH_ENABLE
    gimbal_pitch_pwm_init();
#endif

    /* 上电先回中位，避免突然打到限位。 */
    gimbal_center();
}

void gimbal_set_yaw_us(uint16_t yaw_us)
{
    int safe_yaw_us;

    safe_yaw_us = gimbal_clamp_yaw_us((int)yaw_us);
    gimbal_yaw_us_now = safe_yaw_us;
    pwm_update(GIMBAL_YAW_TIMER, GIMBAL_YAW_CH, gimbal_us_to_duty(safe_yaw_us));
}

void gimbal_set_pitch_us(uint16_t pitch_us)
{
    int safe_pitch_us;

    safe_pitch_us = gimbal_clamp_pitch_us((int)pitch_us);
    gimbal_pitch_us_now = safe_pitch_us;
#if GIMBAL_PITCH_ENABLE
    DL_TimerA_setCaptureCompareValue(GIMBAL_PITCH_TIMER,
        gimbal_us_to_ccr(safe_pitch_us),
        GIMBAL_PITCH_CH);
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
 * 非阻塞安全自检扫动：每 20ms 前进一步。
 * 流程：中位保持 1s -> 只扫 yaw -> 中位保持 1s -> 只扫 pitch -> 中位保持 1s -> 循环。
 * 用法：mode==7 && set==1 时在 10ms 的 TIMG8 ISR 里每周期调一次。
 * 现象：先水平 yaw 左右摆动，后 pitch 上下摆动。
 * -------------------------------------------------------------------------- */
void gimbal_test_sweep_safe(void)
{
    const int hold_ticks = 1000 / GIMBAL_SWEEP_INTERVAL_MS;

    gimbal_test_interval_ticks++;
    if (gimbal_test_interval_ticks < (GIMBAL_SWEEP_INTERVAL_MS / 10)) {
        return;
    }
    gimbal_test_interval_ticks = 0;

    switch (gimbal_test_state) {
    case GIMBAL_TEST_HOLD_BEFORE_YAW:
        gimbal_set_pulse_us(GIMBAL_YAW_CENTER_US, GIMBAL_PITCH_CENTER_US);
        gimbal_test_hold_ticks++;
        if (gimbal_test_hold_ticks >= hold_ticks) {
            gimbal_test_hold_ticks = 0;
            gimbal_sweep_yaw_us = GIMBAL_YAW_CENTER_US;
            gimbal_test_state = GIMBAL_TEST_YAW_TO_MAX;
        }
        break;

    case GIMBAL_TEST_YAW_TO_MAX:
        gimbal_sweep_yaw_us += GIMBAL_STEP_US;
        if (gimbal_sweep_yaw_us >= GIMBAL_YAW_MAX_SAFE_US) {
            gimbal_sweep_yaw_us = GIMBAL_YAW_MAX_SAFE_US;
            gimbal_test_state = GIMBAL_TEST_YAW_TO_MIN;
        }
        gimbal_set_pulse_us((uint16_t)gimbal_sweep_yaw_us, GIMBAL_PITCH_CENTER_US);
        break;

    case GIMBAL_TEST_YAW_TO_MIN:
        gimbal_sweep_yaw_us -= GIMBAL_STEP_US;
        if (gimbal_sweep_yaw_us <= GIMBAL_YAW_MIN_SAFE_US) {
            gimbal_sweep_yaw_us = GIMBAL_YAW_MIN_SAFE_US;
            gimbal_test_state = GIMBAL_TEST_YAW_TO_CENTER;
        }
        gimbal_set_pulse_us((uint16_t)gimbal_sweep_yaw_us, GIMBAL_PITCH_CENTER_US);
        break;

    case GIMBAL_TEST_YAW_TO_CENTER:
        gimbal_sweep_yaw_us += GIMBAL_STEP_US;
        if (gimbal_sweep_yaw_us >= GIMBAL_YAW_CENTER_US) {
            gimbal_sweep_yaw_us = GIMBAL_YAW_CENTER_US;
            gimbal_test_state = GIMBAL_TEST_HOLD_BEFORE_PITCH;
        }
        gimbal_set_pulse_us((uint16_t)gimbal_sweep_yaw_us, GIMBAL_PITCH_CENTER_US);
        break;

    case GIMBAL_TEST_HOLD_BEFORE_PITCH:
        gimbal_set_pulse_us(GIMBAL_YAW_CENTER_US, GIMBAL_PITCH_CENTER_US);
        gimbal_test_hold_ticks++;
        if (gimbal_test_hold_ticks >= hold_ticks) {
            gimbal_test_hold_ticks = 0;
            gimbal_sweep_pitch_us = GIMBAL_PITCH_CENTER_US;
            gimbal_test_state = GIMBAL_TEST_PITCH_TO_MAX;
        }
        break;

    case GIMBAL_TEST_PITCH_TO_MAX:
        gimbal_sweep_pitch_us += GIMBAL_STEP_US;
        if (gimbal_sweep_pitch_us >= GIMBAL_PITCH_MAX_SAFE_US) {
            gimbal_sweep_pitch_us = GIMBAL_PITCH_MAX_SAFE_US;
            gimbal_test_state = GIMBAL_TEST_PITCH_TO_MIN;
        }
        gimbal_set_pulse_us(GIMBAL_YAW_CENTER_US, (uint16_t)gimbal_sweep_pitch_us);
        break;

    case GIMBAL_TEST_PITCH_TO_MIN:
        gimbal_sweep_pitch_us -= GIMBAL_STEP_US;
        if (gimbal_sweep_pitch_us <= GIMBAL_PITCH_MIN_SAFE_US) {
            gimbal_sweep_pitch_us = GIMBAL_PITCH_MIN_SAFE_US;
            gimbal_test_state = GIMBAL_TEST_PITCH_TO_CENTER;
        }
        gimbal_set_pulse_us(GIMBAL_YAW_CENTER_US, (uint16_t)gimbal_sweep_pitch_us);
        break;

    case GIMBAL_TEST_PITCH_TO_CENTER:
        gimbal_sweep_pitch_us += GIMBAL_STEP_US;
        if (gimbal_sweep_pitch_us >= GIMBAL_PITCH_CENTER_US) {
            gimbal_sweep_pitch_us = GIMBAL_PITCH_CENTER_US;
            gimbal_test_state = GIMBAL_TEST_HOLD_AFTER_PITCH;
        }
        gimbal_set_pulse_us(GIMBAL_YAW_CENTER_US, (uint16_t)gimbal_sweep_pitch_us);
        break;

    case GIMBAL_TEST_HOLD_AFTER_PITCH:
    default:
        gimbal_set_pulse_us(GIMBAL_YAW_CENTER_US, GIMBAL_PITCH_CENTER_US);
        gimbal_test_hold_ticks++;
        if (gimbal_test_hold_ticks >= hold_ticks) {
            gimbal_test_hold_ticks = 0;
            gimbal_test_state = GIMBAL_TEST_HOLD_BEFORE_YAW;
        }
        break;
    }
}
