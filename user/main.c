#include "headfile.h"
#include "_pid.h"
#include "key.h"
#include "gimbal.h"      /* ?? */
#include "mission.h"     /* ?? */
#include "buzzer_light.h"
int left_pwm,right_pwm,left_angle_pwm,right_angle_pwm;
int base_left_pwm,base_right_pwm,angle_pwm,angle_pwm1;
int cur_state,last_state,pre_state;
unsigned int Temp[2] = { 0 };
int turn_pwm,cnt;
//int mode,set;

uint8_t KeyNum;
static int yaw_zero_offset = 0;

static int yaw_wrap_180(int angle)
{
    while (angle > 180) angle -= 360;
    while (angle < -180) angle += 360;
    return angle;
}

static int yaw_relative_int(void)
{
    return yaw_wrap_180(yaw_angle_int - yaw_zero_offset);
}

static int yaw_display_int(void)
{
    if (set==1 && (mode==2 || mode==5)) {
        return yaw_relative_int();
    }
    return yaw_angle_int;
}

static void yaw_zero_here(void)
{
    yaw_zero_offset = yaw_angle_int;
}

/* ============================================================================
 *  gray_mapped() —— 灰度通道映射层
 * ----------------------------------------------------------------------------
 *  目的：支持灰度传感器物理反装。即使物理上从左到右是 D8~D1，代码逻辑
 *        上仍然按照 D1~D8 从左到右使用，保持一致性。
 *
 *  映射表：gray_map[逻辑通道-1] = 物理通道
 *    当前配置（整条反转）：
 *      逻辑 D1 ← 物理 digtal(8)
 *      逻辑 D2 ← 物理 digtal(7)
 *      ...
 *      逻辑 D8 ← 物理 digtal(1)
 *
 *  用法：代码中统一使用 gray_mapped(1~8)，不再直接调用 digtal()。
 *
 *  如需恢复正装：改为 {1,2,3,4,5,6,7,8}
 *  如需其他顺序：直接修改数组
 * ========================================================================== */
static const uint8_t gray_map[8] = {8, 7, 6, 5, 4, 3, 2, 1};  /* 整条反转 */

static int gray_mapped(int logic_idx)
{
    /* logic_idx: 1~8（逻辑通道，从左到右）
     * 返回：对应物理通道的灰度值（0=黑线, 1=白底） */
    if (logic_idx < 1 || logic_idx > 8) return 1;  /* 越界保护，返回白底 */
    return digtal(gray_map[logic_idx - 1]);
}

/* ============================================================================
 *  oled_debug_update() —— 统一调试显示（所有 mode 1~6 格式一致）
 * ----------------------------------------------------------------------------
 *  固定 4 行布局（不随 mode 变化）：
 *    第 1 行: M:X S:X          —— mode 和 set 状态
 *    第 2 行: G:xxxxxxxx       —— 8 路灰度传感器 D1~D8（0=黑线, 1=白底）
 *    第 3 行: yaw:XXXX         —— IMU 航向角；mode2/mode5 运行时显示相对 yaw
 *    第 4 行: L:XXXX R:XXXX    —— 左右编码器计数（每 10ms 增量）
 *
 *  每行刷新前用空格清除旧字符，避免残留。
 * ========================================================================== */
static void oled_debug_update(void)
{
    uint8_t i;

    /* 第 1 行: M:X S:X */
    OLED_ShowString(1, 1, "M:  S:  ");   /* 先清空行，避免残留 */
    OLED_ShowNum(1, 3, mode, 1);
    OLED_ShowNum(1, 7, set, 1);

    /* 第 2 行: G:xxxxxxxx（8 路灰度，逻辑顺序 D1~D8） */
    OLED_ShowString(2, 1, "G:");
    for (i = 1; i <= 8; i++) {
        OLED_ShowNum(2, (uint8_t)(2 + i), gray_mapped(i), 1);  /* 使用映射后的灰度 */
    }

    /* 第 3 行: yaw:XXXX */
    OLED_ShowString(3, 1, "yaw:    ");   /* 清空 yaw 后面的旧数字 */
    OLED_ShowSignedNum(3, 5, yaw_display_int(), 4);

    /* 第 4 行: L:XXXX R:XXXX（左右编码器） */
    OLED_ShowString(4, 1, "L:      R:      ");   /* 清空整行 */
    OLED_ShowSignedNum(4, 3, left_encoder, 4);
    OLED_ShowSignedNum(4, 10, right_encoder, 4);
}

/* ============================================================================
 *  mode 6 临时测试 —— 阶段3电机测试 + 阶段4低速循迹测试
 * ----------------------------------------------------------------------------
 *  两个测试共用电机校准参数（MOTOR_TEST_XXX 宏），确保循迹测试复用
 *  阶段3已经调好的电机方向、速度、配平参数。
 *
 *  阶段3：motor_suspended_test()     悬空电机测试
 *  阶段4：line_follow_test_run()     低速开环循迹测试（复用校准参数）
 *
 *  切换方法：修改 TIMG8_IRQHandler 里 mode==6 调用的函数名即可。
 * ========================================================================== */

/* ==================== 电机校准参数（阶段3调好，阶段4复用） ==================== */
#define MOTOR_TEST_PWM_BASE        2200   /* 基础速度，太慢不转改 2300/2400 */
#define MOTOR_TEST_LEFT_TRIM       300    /* 左轮补偿，左轮慢就加大（如 +300） */
#define MOTOR_TEST_RIGHT_TRIM      0      /* 右轮补偿，一般保持 0 */
#define MOTOR_TEST_LEFT_DIR        1      /* 左轮方向，反了改成 -1 */
#define MOTOR_TEST_RIGHT_DIR       1      /* 右轮方向，反了改成 -1 */

/* ==================== 低速循迹测试参数（阶段4专用） ==================== */
#define LINE_TEST_PWM_BASE          500    /* half-speed curve tracing for stable gray feedback */
#define LINE_TEST_PWM_MIN           0
#define LINE_TEST_PWM_MAX           3600
#define LINE_TEST_TURN_GAIN         430    /* keep enough angular speed through the arc */
#define LINE_TEST_LEFT_TURN_OFFSET  350    /* preload after the line leaves center sensors */
#define LINE_TEST_LEFT_TURN_EXTRA   300    /* extra left turn when abs(bias) >= 1.0 */
#define LINE_TEST_RIGHT_TURN_GAIN   430    /* symmetric right-turn gain */
#define LINE_TEST_RIGHT_TURN_OFFSET 350    /* preload after the line leaves center sensors */
#define LINE_TEST_RIGHT_TURN_EXTRA  300    /* extra right turn when abs(bias) >= 1.0 */
#define LINE_TEST_TURN_MAX          1600   /* avoid snapping into a perpendicular exit */
#define LINE_TEST_EDGE_TURN         1500   /* D1/D2 or D7/D8 on line: moderate recovery */
#define LINE_TEST_EDGE_PWM_BASE     300    /* half-speed recovery from edge sensors */
#define LINE_TEST_CURVE_SLOW_GAIN   180    /* larger bias -> lower dynamic base */
#define LINE_TEST_RIGHT_SLOW_GAIN   180
#define LINE_TEST_PWM_BASE_MIN      350    /* half-speed minimum while line is far from D4/D5 */
#define LINE_TEST_RIGHT_BASE_MIN    350
#define LINE_TEST_BIAS_SIGN         1      /* reverse to -1 if steering direction is wrong */
#define LINE_TEST_LOST_SHORT_COUNT  30     /* shorter last-turn hold; faster straight/turn updates */
#define LINE_TEST_LOST_PWM_BASE     325    /* half-speed search base after short line loss */

/* ==================== 测试状态变量（需在函数前声明） ==================== */
static int motor_test_tick = 0;           /* 电机测试计数器 */
static int line_last_turn = 0;            /* last valid turn PWM for short line-loss recovery */
static int line_lost_count = 0;           /* continuous all-white sample count */
static int line_last_black_cnt = 0;       /* last sample black sensor count, for mode2 task1 only */
static int mode7_diag_tick = 0;           /* mode7 hardware diag tick, 10ms per tick */
static uint8_t mode7_led_state = 0;
static uint8_t mode3_aim_started = 0;
static int mode3_current_yaw_us = GIMBAL_CENTER_YAW_US;
static int mode3_current_pitch_us = GIMBAL_CENTER_PITCH_US;

#define MODE3_AIM_STEP_US        10       /* smooth 10us per 10ms tick, avoids startup kick */

/* ==================== mode4: static gimbal drawing test, capital L ==================== */
#define MODE4_L_STEP_US          10       /* 10us per 10ms tick, smooth servo movement */
#define MODE4_L_PITCH_DELTA_US   300      /* vertical stroke length; tune on the target surface */
#define MODE4_L_YAW_DELTA_US     300      /* horizontal stroke length; tune on the target surface */
#define MODE4_L_PITCH_SIGN       1        /* change to -1 if vertical stroke goes the wrong way */
#define MODE4_L_YAW_SIGN         1        /* change to -1 if horizontal stroke goes the wrong way */
#define MODE4_L_GAP_TICKS        50       /* about 1s between pitch stroke and yaw stroke */

#define MODE4_DRAW_IDLE         0
#define MODE4_DRAW_PITCH        1
#define MODE4_DRAW_WAIT         2
#define MODE4_DRAW_YAW          3
#define MODE4_DRAW_DONE         4

static int mode4_draw_state = MODE4_DRAW_IDLE;
static uint8_t mode4_draw_started = 0;
static int mode4_gap_ticks = 0;
static int mode4_start_yaw_us = GIMBAL_CENTER_YAW_US;
static int mode4_start_pitch_us = GIMBAL_CENTER_PITCH_US;
static int mode4_current_yaw_us = GIMBAL_CENTER_YAW_US;
static int mode4_current_pitch_us = GIMBAL_CENTER_PITCH_US;
static int mode4_target_yaw_us = GIMBAL_CENTER_YAW_US;
static int mode4_target_pitch_us = GIMBAL_CENTER_PITCH_US;

/* ==================== mode2: 校赛任务一，顺时针 A-B-C-D-A ==================== */
#define TASK1_COUNT_TO_DIST       0.20f   /* same odom scale as mission.c, tune on site if distance is off */
#define TASK1_STRAIGHT_DIST_MM    1000.0f /* A-B and C-D straight section, 100cm */
#define TASK1_STRAIGHT_LEFT_PWM   2600    /* just above ml_motor.c's 2500 start floor */
#define TASK1_STRAIGHT_RIGHT_PWM  2600
#define TASK1_STRAIGHT_BALANCE_GAIN 0.0f  /* 0 disables encoder balance while tuning straight PWM */
#define TASK1_STRAIGHT_BALANCE_MAX  900
#define TASK1_STRAIGHT_BLACK_MIN_TICKS 30 /* allow black-line handoff after 0.3s straight */
#define TASK1_ARC_MIN_TICKS       80      /* ignore line loss at arc entry for 0.8s */
#define TASK1_ARC_LOST_TICKS      25      /* 0.25s all-white after arc means arc finished */
#define TASK3_ARC_MIN_TICKS       140     /* mode5: ignore short all-white gaps inside the first curve */
#define TASK3_ARC_LOST_TICKS      50      /* mode5: require 0.5s all-white before ending an arc */
#define TASK3_AB_STRAIGHT_YAW_TARGET 0
#define TASK3_CD_STRAIGHT_LEFT_PWM 2550   /* slightly slower than AB, still above the 2500 start floor */
#define TASK3_CD_STRAIGHT_RIGHT_PWM 2550
#define TASK3_CD_STRAIGHT_YAW_ABS_TARGET 180
#define YAW_STRAIGHT_GAIN         180     /* PWM correction per yaw degree */
#define YAW_STRAIGHT_MAX          2400    /* max left/right PWM correction */
#define YAW_STRAIGHT_DEADBAND     3       /* allow -3..+3 deg on straight without twitching */
#define YAW_STRAIGHT_MIN_CORR     900     /* make 1-degree yaw errors visibly correct */
#define YAW_STRAIGHT_SIGN         +1      /* measured correct with current mode5 motor output */
#define YAW_STRAIGHT_BOOST1_COUNT 3       /* same-side corrections before tier 1 boost */
#define YAW_STRAIGHT_BOOST2_COUNT 6       /* same-side corrections before tier 2 boost */
#define YAW_STRAIGHT_BOOST3_COUNT 10      /* same-side corrections before tier 3 boost */
#define YAW_STRAIGHT_BOOST1_PCT   140     /* 1.4x after repeated same-side drift */
#define YAW_STRAIGHT_BOOST2_PCT   180     /* 1.8x after longer same-side drift */
#define YAW_STRAIGHT_BOOST3_PCT   230     /* 2.3x if it still will not come back */
#define YAW_ARC_EXIT_DEADBAND     3       /* leave arc only when yaw is back near 0000 */
#define YAW_ARC_ALIGN_BASE        2600    /* above start floor so mode5 50% output still turns */
#define YAW_ARC_ALIGN_TURN        900     /* minimum yaw correction while squaring up */
#define YAW_ARC_ALIGN_MAX_TICKS   300     /* 3.0s fallback, gives the car time to square up */
#define MODE5_SPEED_PERCENT       50      /* mode5 route only: keep other modes on original motor output */
#define MODE5_MOTOR_START_PWM     2500    /* mirror ml_motor.c start compensation before scaling */

typedef enum {
    TASK1_AB_STRAIGHT = 0,
    TASK1_BC_ARC,
    TASK1_CD_STRAIGHT,
    TASK1_DA_ARC,
    TASK1_DONE
} task1_phase_t;

typedef enum {
    TASK3_AB_STRAIGHT = 0,
    TASK3_B_AIM,
    TASK3_BC_ARC,
    TASK3_CD_STRAIGHT,
    TASK3_C_AIM,
    TASK3_DA_ARC,
    TASK3_DONE
} task3_phase_t;

static task1_phase_t task1_phase = TASK1_AB_STRAIGHT;
static float task1_dist = 0.0f;
static float task1_left_dist = 0.0f;
static float task1_right_dist = 0.0f;
static int task1_phase_ticks = 0;
static int task1_arc_lost_ticks = 0;
static int task1_arc_done_count = 0;
static int task1_arc_align_ticks = 0;
static uint8_t task1_arc_seen_line = 0;
static uint8_t task1_arc_aligning = 0;
static uint8_t task1_finished = 0;
static int task1_straight_yaw_target = 0;

static task3_phase_t task3_phase = TASK3_AB_STRAIGHT;
static float task3_dist = 0.0f;
static float task3_left_dist = 0.0f;
static float task3_right_dist = 0.0f;
static int task3_phase_ticks = 0;
static int task3_straight_black_ticks = 0;
static int task3_arc_lost_ticks = 0;
static int task3_arc_align_ticks = 0;
static int task3_aim_ticks = 0;
static uint8_t task3_arc_seen_line = 0;
static uint8_t task3_arc_aligning = 0;
static uint8_t task3_finished = 0;
static int task3_straight_yaw_target = 0;
static int yaw_straight_last_error_sign = 0;
static int yaw_straight_same_sign_count = 0;

static void yaw_straight_adapt_reset(void)
{
    yaw_straight_last_error_sign = 0;
    yaw_straight_same_sign_count = 0;
}

static void task1_clear_state(void)
{
    task1_phase = TASK1_AB_STRAIGHT;
    task1_dist = 0.0f;
    task1_left_dist = 0.0f;
    task1_right_dist = 0.0f;
    task1_phase_ticks = 0;
    task1_arc_lost_ticks = 0;
    task1_arc_done_count = 0;
    task1_arc_align_ticks = 0;
    task1_arc_seen_line = 0;
    task1_arc_aligning = 0;
    task1_finished = 0;
    task1_straight_yaw_target = 0;
    yaw_straight_adapt_reset();
}

static void task3_clear_state(void)
{
    task3_phase = TASK3_AB_STRAIGHT;
    task3_dist = 0.0f;
    task3_left_dist = 0.0f;
    task3_right_dist = 0.0f;
    task3_phase_ticks = 0;
    task3_straight_black_ticks = 0;
    task3_arc_lost_ticks = 0;
    task3_arc_align_ticks = 0;
    task3_aim_ticks = 0;
    task3_arc_seen_line = 0;
    task3_arc_aligning = 0;
    task3_finished = 0;
    task3_straight_yaw_target = 0;
    yaw_straight_adapt_reset();
}

static void mode7_diag_reset(void)
{
    mode7_diag_tick = 0;
    mode7_led_state = 0;
    gpio_set(GPIOB, DL_GPIO_PIN_8, 1);    /* buzzer off */
    gpio_set(GPIOB, DL_GPIO_PIN_26, 0);   /* LED default off */
}

static void mode7_diag_run(void)
{
    mode7_diag_tick++;

    /* PB8 buzzer stays off in the current mode7 gimbal-only test. */
    gpio_set(GPIOB, DL_GPIO_PIN_8, 1);

    /* LED toggles about every 120ms, roughly 4Hz blink, to prove mode7 PWM is active. */
    if ((mode7_diag_tick % 12) == 0) {
        mode7_led_state = (uint8_t)!mode7_led_state;
        gpio_set(GPIOB, DL_GPIO_PIN_26, mode7_led_state ? 1 : 0);
    }

    if (mode7_diag_tick >= 1200) {
        mode7_diag_tick = 0;
    }
}

/* 灰度加权权重表（D1~D8，从左到右） */
static const float kLineWeight[8] = {
    3.0f, 1.5f, 0.5f, 0.1f,   /* D1~D4 */
   -0.1f, -0.5f, -1.5f, -3.0f  /* D5~D8 */
};

/* ==================== 统一电机输出函数（所有 mode6 测试必须用这些） ==================== */
/* 设置左轮 PWM（自动应用阶段3校准的方向和补偿） */
static void motor_output_left(int pwm)
{
    int actual_pwm;
    if (pwm > 0) {
        actual_pwm = pwm + MOTOR_TEST_LEFT_TRIM;  /* 正转加补偿 */
    } else if (pwm < 0) {
        actual_pwm = pwm - MOTOR_TEST_LEFT_TRIM;  /* 反转减补偿 */
    } else {
        actual_pwm = 0;
    }
    Set_left_pwm(MOTOR_TEST_LEFT_DIR * actual_pwm);
}

/* 设置右轮 PWM（自动应用阶段3校准的方向和补偿） */
static void motor_output_right(int pwm)
{
    int actual_pwm;
    if (pwm > 0) {
        actual_pwm = pwm + MOTOR_TEST_RIGHT_TRIM;  /* 正转加补偿 */
    } else if (pwm < 0) {
        actual_pwm = pwm - MOTOR_TEST_RIGHT_TRIM;  /* 反转减补偿 */
    } else {
        actual_pwm = 0;
    }
    Set_right_pwm(MOTOR_TEST_RIGHT_DIR * actual_pwm);
}

static int mode5_effective_pwm(int pwm)
{
    if (pwm > 0 && pwm < MODE5_MOTOR_START_PWM) return MODE5_MOTOR_START_PWM;
    if (pwm < 0 && pwm > -MODE5_MOTOR_START_PWM) return -MODE5_MOTOR_START_PWM;
    return pwm;
}

static int mode5_scale_pwm(int pwm)
{
    pwm = mode5_effective_pwm(pwm);
    return pwm * MODE5_SPEED_PERCENT / 100;
}

static void mode5_set_left_pwm_raw(int pwm)
{
    if (pwm > 0) {
        gpio_set(GPIOB, DL_GPIO_PIN_24, 1);
        gpio_set(GPIOB, DL_GPIO_PIN_20, 0);
        pwm_update(TIMG6, DL_TIMER_CC_0_INDEX, pwm);
    } else if (pwm == 0) {
        pwm_update(TIMG6, DL_TIMER_CC_0_INDEX, 0);
    } else {
        gpio_set(GPIOB, DL_GPIO_PIN_24, 0);
        gpio_set(GPIOB, DL_GPIO_PIN_20, 1);
        pwm_update(TIMG6, DL_TIMER_CC_0_INDEX, -pwm);
    }
}

static void mode5_set_right_pwm_raw(int pwm)
{
    if (pwm > 0) {
        gpio_set(GPIOB, DL_GPIO_PIN_19, 1);
        gpio_set(GPIOB, DL_GPIO_PIN_18, 0);
        pwm_update(TIMG6, DL_TIMER_CC_1_INDEX, pwm);
    } else if (pwm == 0) {
        pwm_update(TIMG6, DL_TIMER_CC_1_INDEX, 0);
    } else {
        gpio_set(GPIOB, DL_GPIO_PIN_19, 0);
        gpio_set(GPIOB, DL_GPIO_PIN_18, 1);
        pwm_update(TIMG6, DL_TIMER_CC_1_INDEX, -pwm);
    }
}

static void mode5_motor_output_both(int left_pwm, int right_pwm)
{
    int actual_left;
    int actual_right;

    if (left_pwm > 0) actual_left = left_pwm + MOTOR_TEST_LEFT_TRIM;
    else if (left_pwm < 0) actual_left = left_pwm - MOTOR_TEST_LEFT_TRIM;
    else actual_left = 0;

    if (right_pwm > 0) actual_right = right_pwm + MOTOR_TEST_RIGHT_TRIM;
    else if (right_pwm < 0) actual_right = right_pwm - MOTOR_TEST_RIGHT_TRIM;
    else actual_right = 0;

    actual_left = MOTOR_TEST_LEFT_DIR * actual_left;
    actual_right = MOTOR_TEST_RIGHT_DIR * actual_right;

    mode5_set_left_pwm_raw(mode5_scale_pwm(actual_left));
    mode5_set_right_pwm_raw(mode5_scale_pwm(actual_right));
}

/* 同时设置左右轮（方便统一调用） */
static void motor_output_both(int left_pwm, int right_pwm)
{
    if (mode==5 && set==1) {
        mode5_motor_output_both(left_pwm, right_pwm);
        return;
    }

    motor_output_left(left_pwm);
    motor_output_right(right_pwm);
}

/* ============================================================================
 *  car_stop_all() —— 统一安全停止函数
 * ----------------------------------------------------------------------------
 *  用途：立即停止所有电机输出，清空临时测试状态。
 *  调用时机：
 *    - set 从 1 变为 0 时
 *    - mode 切换时
 *    - 异常停止时
 *
 *  确保：无论处于哪个 mode，调用后左右轮都不转。
 * ========================================================================== */
static void car_stop_all(void)
{
    /* 立即停止电机 */
    motor_output_both(0, 0);

    /* 清空 mode6 临时测试状态 */
    motor_test_tick = 0;
    line_last_turn = 0;
    line_lost_count = 0;
    line_last_black_cnt = 0;
    task1_clear_state();
    task3_clear_state();

    /* mission 状态解除（mode5 专用，其他 mode 调用无害） */
    mission_disarm();
}

/* ============================================================================
 *  阶段3：motor_suspended_test() —— 悬空电机测试
 * ----------------------------------------------------------------------------
 *  目的：验证电机驱动、接线、方向、速度配平，车轮悬空不接触地面。
 *  用法：在 TIMG8 ISR 里 mode==6&&set==1 时每 10ms 调用一次。
 *  流程：
 *    0-2s:   左轮单独低速前进，右轮停止
 *    2-3s:   全停
 *    3-5s:   右轮单独低速前进，左轮停止
 *    5-6s:   全停
 *    6-8s:   双轮同时低速前进（物理意义上都向前滚）
 *    8-9s:   全停
 *    9-11s:  双轮同时低速后退（物理意义上都向后滚）
 *    11s+:   全停
 *
 *  注意：不使用灰度、IMU、mission、编码器闭环，纯开环 PWM 测试。
 * ========================================================================== */
static void motor_suspended_test(void)
{
    int base_pwm;

    base_pwm = MOTOR_TEST_PWM_BASE;
    motor_test_tick++;

    /* 测试流程状态机（基于 tick 计数，1 tick = 10ms） */
    if (motor_test_tick <= 200) {
        /* 0-2s: 左轮单独前进 */
        motor_output_both(base_pwm, 0);
    }
    else if (motor_test_tick <= 300) {
        /* 2-3s: 全停 */
        motor_output_both(0, 0);
    }
    else if (motor_test_tick <= 500) {
        /* 3-5s: 右轮单独前进 */
        motor_output_both(0, base_pwm);
    }
    else if (motor_test_tick <= 600) {
        /* 5-6s: 全停 */
        motor_output_both(0, 0);
    }
    else if (motor_test_tick <= 800) {
        /* 6-8s: 双轮前进（物理意义上都向前滚） */
        motor_output_both(base_pwm, base_pwm);
    }
    else if (motor_test_tick <= 900) {
        /* 8-9s: 全停 */
        motor_output_both(0, 0);
    }
    else if (motor_test_tick <= 1100) {
        /* 9-11s: 双轮后退（物理意义上都向后滚） */
        motor_output_both(-base_pwm, -base_pwm);
    }
    else {
        /* 11s+: 最后全停，保持停止状态 */
        motor_output_both(0, 0);
    }
}

static void motor_test_reset(void)
{
    motor_test_tick = 0;
    line_last_turn = 0;
    line_lost_count = 0;
    line_last_black_cnt = 0;
    motor_output_both(0, 0);
}

static int task1_sample_black_count(void);

static void task1_reset(void)
{
    motor_test_reset();
    task1_clear_state();
}

static void task3_reset(void)
{
    motor_test_reset();
    task3_clear_state();
}

static int line_test_limit_pwm(int pwm)
{
    if (pwm > LINE_TEST_PWM_MAX) return LINE_TEST_PWM_MAX;
    if (pwm < LINE_TEST_PWM_MIN) return LINE_TEST_PWM_MIN;
    return pwm;
}

/* ============================================================================
 *  阶段4：line_follow_test_run() —— 低速开环循迹测试（复用阶段3校准参数）
 * ----------------------------------------------------------------------------
 *  目的：验证灰度传感器和基本循迹算法，车轮着地沿黑线跟踪。
 *  用法：在 TIMG8 ISR 里 mode==6&&set==1 时每 10ms 调用一次。
 *  算法：
 *    1. 读取 digtal(1)~digtal(8)（0=黑线, 1=白底）
 *    2. 计算加权灰度偏差 bias（借鉴 mission.c 权重思想）
 *    3. 根据 bias 计算转向差速 turn = bias * GAIN
 *    4. left_pwm = base - turn, right_pwm = base + turn
 *    5. 通过 motor_output_left/right 输出（自动应用阶段3校准参数）
 *
 *  注意：
 *   - 不使用 mission_drive()、motorA/motorB 速度 PID、mission 状态机。
 *   - 不判断 B/C/D/A，不触发云台。
 *   - 纯开环 PWM 控制，简单加权偏差算法。
 *   - 全白丢线时停车，避免车冲出赛道。
 * ========================================================================== */
static void line_follow_test_run(void)
{
    uint8_t gray[8];
    int i, black_cnt, turn, abs_bias_x10, dynamic_base, base_min, left_pwm, right_pwm;
    int center_on_line, left_edge_on_line, right_edge_on_line;
    int turn_gain, slow_gain;
    float sum, bias, abs_bias;

    black_cnt = 0;
    sum = 0.0f;
    bias = 0.0f;

    /* 1. 读取灰度传感器（0=黑线, 1=白底），使用映射后的逻辑顺序 D1~D8 */
    for (i = 0; i < 8; i++) {
        gray[i] = gray_mapped(i + 1);  /* gray_mapped(1~8) */
    }
    center_on_line = (gray[3] == 0 || gray[4] == 0);
    left_edge_on_line = (gray[0] == 0 || gray[1] == 0);
    right_edge_on_line = (gray[6] == 0 || gray[7] == 0);

    /* 2. 计算加权偏差（只对压线的传感器 gray[i]==0 求权重平均） */
    for (i = 0; i < 8; i++) {
        if (gray[i] == 0) {  /* 检测到黑线 */
            sum += kLineWeight[i];
            black_cnt++;
        }
    }
    line_last_black_cnt = black_cnt;

    /* 3. 判断是否有线 */
    if (black_cnt == 0) {
        line_lost_count++;
        if (line_lost_count < LINE_TEST_LOST_SHORT_COUNT) {
            /* Short line loss: keep the last turn at low speed to search the line. */
            turn = line_last_turn;
            left_pwm  = LINE_TEST_LOST_PWM_BASE - turn;
            right_pwm = LINE_TEST_LOST_PWM_BASE + turn;
            left_pwm  = line_test_limit_pwm(left_pwm);
            right_pwm = line_test_limit_pwm(right_pwm);
            motor_output_both(left_pwm, right_pwm);
            return;
        }

        /* Long line loss: stop instead of driving straight out of the track. */
        motor_output_both(0, 0);
        return;
    } else {
        line_lost_count = 0;
    }

    /*
     * bias is the average weight of black-line sensors only.
     * D1..D8 weights: +3, +1.5, +0.5, +0.1, -0.1, -0.5, -1.5, -3.
     * LINE_TEST_BIAS_SIGN only changes steering polarity; gray_mapped() still
     * keeps logical D1..D8 from left to right, black=0 and white=1.
     */
    bias = LINE_TEST_BIAS_SIGN * (sum / (float)black_cnt);
    abs_bias = (bias < 0.0f) ? -bias : bias;
    abs_bias_x10 = (int)(abs_bias * 10.0f + 0.5f);

    /*
     * turn is differential PWM. Larger bias makes larger differential speed and
     * also lowers dynamic_base, so sharp curves slow down and the inner wheel
     * may fall to 0 instead of being lifted to a start threshold.
     *
     * D4/D5 are the center sensors. When either one still sees the line, avoid
     * preload, but keep the normal proportional turn so the car keeps rotating
     * through the arc instead of leaving on an early tangent.
     */
    if (bias < 0.0f) {
        turn_gain = LINE_TEST_RIGHT_TURN_GAIN;
        slow_gain = LINE_TEST_RIGHT_SLOW_GAIN;
        base_min = LINE_TEST_RIGHT_BASE_MIN;
    } else {
        turn_gain = LINE_TEST_TURN_GAIN;
        slow_gain = LINE_TEST_CURVE_SLOW_GAIN;
        base_min = LINE_TEST_PWM_BASE_MIN;
    }

    turn = (int)(bias * turn_gain);
    if (!center_on_line && bias > 0.0f) {
        turn += LINE_TEST_LEFT_TURN_OFFSET;
        if (abs_bias_x10 >= 10) {
            turn += LINE_TEST_LEFT_TURN_EXTRA;
        }
    } else if (!center_on_line && bias < 0.0f) {
        turn -= LINE_TEST_RIGHT_TURN_OFFSET;
        if (abs_bias_x10 >= 10) {
            turn -= LINE_TEST_RIGHT_TURN_EXTRA;
        }
    }

    if (!center_on_line && left_edge_on_line && turn < LINE_TEST_EDGE_TURN) {
        turn = LINE_TEST_EDGE_TURN;
    } else if (!center_on_line && right_edge_on_line && turn > -LINE_TEST_EDGE_TURN) {
        turn = -LINE_TEST_EDGE_TURN;
    }

    if (turn > LINE_TEST_TURN_MAX)  turn = LINE_TEST_TURN_MAX;
    if (turn < -LINE_TEST_TURN_MAX) turn = -LINE_TEST_TURN_MAX;
    line_last_turn = turn;

    dynamic_base = LINE_TEST_PWM_BASE - abs_bias_x10 * slow_gain / 10;
    if (dynamic_base < base_min) {
        dynamic_base = base_min;
    }
    if (!center_on_line && (left_edge_on_line || right_edge_on_line)) {
        dynamic_base = LINE_TEST_EDGE_PWM_BASE;
    }

    /* 5. 计算左右轮 PWM（dynamic_base - turn, dynamic_base + turn） */
    left_pwm  = dynamic_base - turn;
    right_pwm = dynamic_base + turn;
    left_pwm  = line_test_limit_pwm(left_pwm);
    right_pwm = line_test_limit_pwm(right_pwm);

    /* 6. 输出到电机（自动应用阶段3校准的方向和补偿） */
    motor_output_both(left_pwm, right_pwm);
}

static int task1_sample_black_count(void)
{
    int i, black_cnt;

    black_cnt = 0;
    for (i = 1; i <= 8; i++) {
        if (gray_mapped(i) == 0) {
            black_cnt++;
        }
    }
    line_last_black_cnt = black_cnt;
    return black_cnt;
}

static void task1_odom_update(void)
{
    float left_d, right_d, d;

    left_d = (float)left_encoder * TASK1_COUNT_TO_DIST;
    right_d = (float)right_encoder * TASK1_COUNT_TO_DIST;
    if (left_d > 0.0f) {
        task1_left_dist += left_d;
    }
    if (right_d > 0.0f) {
        task1_right_dist += right_d;
    }

    d = (left_d + right_d) * 0.5f;
    if (d > 0.0f) {
        task1_dist += d;
    }
}

static int yaw_normalize_error(int error)
{
    return yaw_wrap_180(error);
}

static int yaw_limit_correction(int correction)
{
    if (correction > YAW_STRAIGHT_MAX) return YAW_STRAIGHT_MAX;
    if (correction < -YAW_STRAIGHT_MAX) return -YAW_STRAIGHT_MAX;
    return correction;
}

static int yaw_abs_int(int angle)
{
    return (angle < 0) ? -angle : angle;
}

static int task3_cd_straight_yaw_error(void)
{
    int yaw;
    int error;

    yaw = yaw_relative_int();
    error = TASK3_CD_STRAIGHT_YAW_ABS_TARGET - yaw_abs_int(yaw);
    if (yaw < 0) {
        error = -error;
    }
    return error;
}

static int yaw_adaptive_straight_correction(int error)
{
    int error_sign, correction, boost_pct;

    if (error <= YAW_STRAIGHT_DEADBAND && error >= -YAW_STRAIGHT_DEADBAND) {
        yaw_straight_adapt_reset();
        return 0;
    }

    correction = yaw_limit_correction(error * YAW_STRAIGHT_GAIN * YAW_STRAIGHT_SIGN);
    if (correction > 0 && correction < YAW_STRAIGHT_MIN_CORR) {
        correction = YAW_STRAIGHT_MIN_CORR;
    } else if (correction < 0 && correction > -YAW_STRAIGHT_MIN_CORR) {
        correction = -YAW_STRAIGHT_MIN_CORR;
    }

    if (!(mode==5 && set==1)) {
        yaw_straight_adapt_reset();
        return correction;
    }

    error_sign = (error > 0) ? 1 : -1;
    if (error_sign == yaw_straight_last_error_sign) {
        if (yaw_straight_same_sign_count < 1000) {
            yaw_straight_same_sign_count++;
        }
    } else {
        yaw_straight_last_error_sign = error_sign;
        yaw_straight_same_sign_count = 1;
    }

    boost_pct = 100;
    if (yaw_straight_same_sign_count >= YAW_STRAIGHT_BOOST3_COUNT) {
        boost_pct = YAW_STRAIGHT_BOOST3_PCT;
    } else if (yaw_straight_same_sign_count >= YAW_STRAIGHT_BOOST2_COUNT) {
        boost_pct = YAW_STRAIGHT_BOOST2_PCT;
    } else if (yaw_straight_same_sign_count >= YAW_STRAIGHT_BOOST1_COUNT) {
        boost_pct = YAW_STRAIGHT_BOOST1_PCT;
    }

    return yaw_limit_correction(correction * boost_pct / 100);
}

static void yaw_drive_straight(int yaw_target)
{
    int error, correction;

    error = yaw_normalize_error(yaw_relative_int() - yaw_target);
    correction = yaw_adaptive_straight_correction(error);

    motor_output_both(TASK1_STRAIGHT_LEFT_PWM + correction,
                      TASK1_STRAIGHT_RIGHT_PWM - correction);
}

static void task3_drive_cd_straight(void)
{
    int correction;

    correction = yaw_adaptive_straight_correction(task3_cd_straight_yaw_error());
    motor_output_both(TASK3_CD_STRAIGHT_LEFT_PWM - correction,
                      TASK3_CD_STRAIGHT_RIGHT_PWM + correction);
}

static int yaw_arc_exit_is_aligned(int yaw_target)
{
    int error;

    error = yaw_normalize_error(yaw_relative_int() - yaw_target);
    return (error <= YAW_ARC_EXIT_DEADBAND && error >= -YAW_ARC_EXIT_DEADBAND);
}

static void yaw_arc_exit_align_turn(int yaw_target)
{
    int error, correction;

    error = yaw_normalize_error(yaw_relative_int() - yaw_target);
    correction = yaw_limit_correction(error * YAW_STRAIGHT_GAIN * YAW_STRAIGHT_SIGN);
    if (correction > 0 && correction < YAW_ARC_ALIGN_TURN) {
        correction = YAW_ARC_ALIGN_TURN;
    } else if (correction < 0 && correction > -YAW_ARC_ALIGN_TURN) {
        correction = -YAW_ARC_ALIGN_TURN;
    }

    motor_output_both(YAW_ARC_ALIGN_BASE - correction,
                      YAW_ARC_ALIGN_BASE + correction);
}

static void task1_next_phase(task1_phase_t next_phase)
{
    task1_phase = next_phase;
    task1_dist = 0.0f;
    task1_left_dist = 0.0f;
    task1_right_dist = 0.0f;
    task1_phase_ticks = 0;
    task1_arc_lost_ticks = 0;
    task1_arc_align_ticks = 0;
    task1_arc_seen_line = 0;
    task1_arc_aligning = 0;
    line_lost_count = 0;
    if (next_phase == TASK1_AB_STRAIGHT || next_phase == TASK1_CD_STRAIGHT) {
        task1_straight_yaw_target = 0;
        yaw_straight_adapt_reset();
    }
}

static void task1_finish(void)
{
    task1_finished = 1;
    task1_phase = TASK1_DONE;
    set = 0;
    motor_output_both(0, 0);
    beep();
}

static void task1_drive_straight(void)
{
    yaw_drive_straight(task1_straight_yaw_target);
}

static void task1_run_arc(task1_phase_t next_phase)
{
    if (task1_arc_aligning) {
        task1_arc_align_ticks++;
        if (yaw_arc_exit_is_aligned(0) ||
            task1_arc_align_ticks >= YAW_ARC_ALIGN_MAX_TICKS) {
            task1_arc_aligning = 0;
            task1_arc_done_count++;
            if (task1_arc_done_count >= 2) {
                task1_finish();
            } else {
                task1_next_phase(next_phase);
            }
        } else {
            yaw_arc_exit_align_turn(0);
        }
        return;
    }

    if (!task1_arc_seen_line) {
        if (task1_sample_black_count() == 0) {
            task1_drive_straight();
            return;
        }
        task1_arc_seen_line = 1;
        line_lost_count = 0;
    }

    line_follow_test_run();
    task1_phase_ticks++;

    if (task1_arc_seen_line && line_last_black_cnt == 0 && task1_phase_ticks > TASK1_ARC_MIN_TICKS) {
        task1_arc_lost_ticks++;
    } else {
        task1_arc_lost_ticks = 0;
    }

    if (task1_arc_lost_ticks >= TASK1_ARC_LOST_TICKS) {
        task1_arc_aligning = 1;
        task1_arc_align_ticks = 0;
    }
}

static void task1_run_once(void)
{
    if (task1_finished) {
        motor_output_both(0, 0);
        return;
    }

    task1_odom_update();

    switch (task1_phase) {
    case TASK1_AB_STRAIGHT:
        task1_phase_ticks++;
        task1_drive_straight();
        if ((task1_sample_black_count() > 0 && task1_phase_ticks >= TASK1_STRAIGHT_BLACK_MIN_TICKS) ||
            task1_dist >= TASK1_STRAIGHT_DIST_MM) {
            task1_next_phase(TASK1_BC_ARC);
        }
        break;

    case TASK1_BC_ARC:
        task1_run_arc(TASK1_CD_STRAIGHT);
        break;

    case TASK1_CD_STRAIGHT:
        task1_phase_ticks++;
        task1_drive_straight();
        if ((task1_sample_black_count() > 0 && task1_phase_ticks >= TASK1_STRAIGHT_BLACK_MIN_TICKS) ||
            task1_dist >= TASK1_STRAIGHT_DIST_MM) {
            task1_next_phase(TASK1_DA_ARC);
        }
        break;

    case TASK1_DA_ARC:
        task1_run_arc(TASK1_DONE);
        break;

    default:
        task1_finish();
        break;
    }
}

static int mode3_step_to_target(int current, int target)
{
    if (current < target) {
        current += MODE3_AIM_STEP_US;
        if (current > target) current = target;
    } else if (current > target) {
        current -= MODE3_AIM_STEP_US;
        if (current < target) current = target;
    }
    return current;
}

static void mode3_aim_prepare(void)
{
    mode3_aim_started = 1;
    mode3_current_yaw_us = gimbal_yaw_us_now;
    mode3_current_pitch_us = gimbal_pitch_us_now;
    gimbal_set_pulse_us((uint16_t)mode3_current_yaw_us,
        (uint16_t)mode3_current_pitch_us);
}

static void mode3_aim_reset(void)
{
    mode3_aim_started = 0;
    mode3_current_yaw_us = GIMBAL_CENTER_YAW_US;
    mode3_current_pitch_us = GIMBAL_CENTER_PITCH_US;
    gpio_set(GPIOB, DL_GPIO_PIN_26, 0);
    gimbal_center();
}

static void mode3_aim_run(void)
{
    motor_output_both(0, 0);

    if (!mode3_aim_started) {
        mode3_aim_prepare();
    }

    /*
     * mode3 independent aiming: reverse the 0->1 servo movement around center.
     * The final pulse still goes through gimbal_set_pulse_us() safety clamps.
     */
    mode3_current_yaw_us = mode3_step_to_target(mode3_current_yaw_us,
        (GIMBAL_CENTER_YAW_US * 2 - GIMBAL_AIM_B_YAW_US));
    mode3_current_pitch_us = mode3_step_to_target(mode3_current_pitch_us,
        (GIMBAL_CENTER_PITCH_US * 2 - GIMBAL_AIM_B_PITCH_US));

    gimbal_set_pulse_us((uint16_t)mode3_current_yaw_us,
        (uint16_t)mode3_current_pitch_us);
}

static int mode4_step_to_target(int current, int target)
{
    if (current < target) {
        current += MODE4_L_STEP_US;
        if (current > target) current = target;
    } else if (current > target) {
        current -= MODE4_L_STEP_US;
        if (current < target) current = target;
    }
    return current;
}

static void mode4_draw_reset(void)
{
    mode4_draw_started = 0;
    mode4_draw_state = MODE4_DRAW_IDLE;
    mode4_gap_ticks = 0;
    mode4_start_yaw_us = gimbal_yaw_us_now;
    mode4_start_pitch_us = gimbal_pitch_us_now;
    mode4_current_yaw_us = mode4_start_yaw_us;
    mode4_current_pitch_us = mode4_start_pitch_us;
    mode4_target_yaw_us = mode4_start_yaw_us;
    mode4_target_pitch_us = mode4_start_pitch_us;
}

static void mode4_draw_prepare(void)
{
    mode4_draw_started = 1;
    mode4_draw_state = MODE4_DRAW_PITCH;
    mode4_gap_ticks = 0;

    /*
     * mode3 set=0 is the field-calibrated starting point. Capture the current
     * gimbal pulse widths and draw an L from there: pitch stroke first, yaw
     * stroke second. Flip MODE4_L_*_SIGN if either stroke direction is wrong.
     */
    mode4_start_yaw_us = gimbal_yaw_us_now;
    mode4_start_pitch_us = gimbal_pitch_us_now;
    mode4_current_yaw_us = mode4_start_yaw_us;
    mode4_current_pitch_us = mode4_start_pitch_us;
    mode4_target_yaw_us = mode4_start_yaw_us;
    mode4_target_pitch_us =
        mode4_start_pitch_us + MODE4_L_PITCH_SIGN * MODE4_L_PITCH_DELTA_US;

    gimbal_set_pulse_us((uint16_t)mode4_current_yaw_us,
        (uint16_t)mode4_current_pitch_us);
}

static void mode4_draw_run(void)
{
    motor_output_both(0, 0);

    if (!mode4_draw_started) {
        mode4_draw_prepare();
    }

    if (mode4_draw_state == MODE4_DRAW_PITCH) {
        mode4_current_pitch_us = mode4_step_to_target(mode4_current_pitch_us,
            mode4_target_pitch_us);
        gimbal_set_pitch_us((uint16_t)mode4_current_pitch_us);
        if (mode4_current_pitch_us == mode4_target_pitch_us) {
            mode4_draw_state = MODE4_DRAW_WAIT;
            mode4_gap_ticks = MODE4_L_GAP_TICKS;
        }
    } else if (mode4_draw_state == MODE4_DRAW_WAIT) {
        gimbal_set_pulse_us((uint16_t)mode4_current_yaw_us,
            (uint16_t)mode4_current_pitch_us);
        if (mode4_gap_ticks > 0) {
            mode4_gap_ticks--;
        } else {
            mode4_draw_state = MODE4_DRAW_YAW;
            mode4_target_yaw_us =
                mode4_start_yaw_us + MODE4_L_YAW_SIGN * MODE4_L_YAW_DELTA_US;
        }
    } else if (mode4_draw_state == MODE4_DRAW_YAW) {
        mode4_current_yaw_us = mode4_step_to_target(mode4_current_yaw_us,
            mode4_target_yaw_us);
        gimbal_set_yaw_us((uint16_t)mode4_current_yaw_us);
        if (mode4_current_yaw_us == mode4_target_yaw_us) {
            mode4_draw_state = MODE4_DRAW_DONE;
        }
    } else {
        gimbal_set_pulse_us((uint16_t)mode4_current_yaw_us,
            (uint16_t)mode4_current_pitch_us);
    }
}

static void task3_odom_update(void)
{
    float left_d, right_d, d;

    left_d = (float)left_encoder * TASK1_COUNT_TO_DIST;
    right_d = (float)right_encoder * TASK1_COUNT_TO_DIST;
    if (left_d > 0.0f) {
        task3_left_dist += left_d;
    }
    if (right_d > 0.0f) {
        task3_right_dist += right_d;
    }

    d = (left_d + right_d) * 0.5f;
    if (d > 0.0f) {
        task3_dist += d;
    }
}

static void task3_next_phase(task3_phase_t next_phase)
{
    task3_phase = next_phase;
    task3_dist = 0.0f;
    task3_left_dist = 0.0f;
    task3_right_dist = 0.0f;
    task3_phase_ticks = 0;
    task3_straight_black_ticks = 0;
    task3_arc_lost_ticks = 0;
    task3_arc_align_ticks = 0;
    task3_arc_seen_line = 0;
    task3_arc_aligning = 0;
    line_lost_count = 0;
    if (next_phase == TASK3_AB_STRAIGHT || next_phase == TASK3_CD_STRAIGHT) {
        if (next_phase == TASK3_CD_STRAIGHT) {
            task3_straight_yaw_target = TASK3_CD_STRAIGHT_YAW_ABS_TARGET;
        } else {
            task3_straight_yaw_target = TASK3_AB_STRAIGHT_YAW_TARGET;
        }
        yaw_straight_adapt_reset();
    }
}

static void task3_finish(void)
{
    task3_finished = 1;
    task3_phase = TASK3_DONE;
    set = 0;
    motor_output_both(0, 0);
    DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_16);
    beep();
}

static void task3_drive_straight(void)
{
    if (task3_phase == TASK3_CD_STRAIGHT) {
        task3_drive_cd_straight();
        return;
    }

    yaw_drive_straight(task3_straight_yaw_target);
}

static uint8_t task3_run_straight_until_black(void)
{
    task3_phase_ticks++;
    task3_drive_straight();

    if (task3_phase_ticks >= TASK1_STRAIGHT_BLACK_MIN_TICKS &&
        task1_sample_black_count() > 0) {
        if (task3_straight_black_ticks < 1000) {
            task3_straight_black_ticks++;
        }
    } else {
        task3_straight_black_ticks = 0;
    }

    return (task3_straight_black_ticks >= 3);
}

static void task3_run_arc(task3_phase_t next_phase)
{
    int arc_exit_yaw_target;

    arc_exit_yaw_target = TASK3_AB_STRAIGHT_YAW_TARGET;
    if (task3_arc_aligning) {
        task3_arc_align_ticks++;
        if (yaw_arc_exit_is_aligned(arc_exit_yaw_target) ||
            task3_arc_align_ticks >= YAW_ARC_ALIGN_MAX_TICKS) {
            task3_arc_aligning = 0;
            if (next_phase == TASK3_DONE) {
                task3_finish();
            } else {
                beep();
                task3_next_phase(next_phase);
            }
        } else {
            yaw_arc_exit_align_turn(arc_exit_yaw_target);
        }
        return;
    }

    if (!task3_arc_seen_line) {
        if (task1_sample_black_count() == 0) {
            task3_drive_straight();
            return;
        }
        task3_arc_seen_line = 1;
        line_lost_count = 0;
    }

    line_follow_test_run();
    task3_phase_ticks++;

    if (task3_arc_seen_line && line_last_black_cnt == 0 && task3_phase_ticks > TASK3_ARC_MIN_TICKS) {
        task3_arc_lost_ticks++;
    } else {
        task3_arc_lost_ticks = 0;
    }

    if (task3_arc_lost_ticks >= TASK3_ARC_LOST_TICKS) {
        if (next_phase == TASK3_CD_STRAIGHT) {
            beep();
            task3_next_phase(next_phase);
        } else if (next_phase == TASK3_DONE) {
            task3_finish();
        } else {
            beep();
            task3_arc_aligning = 1;
            task3_arc_align_ticks = 0;
        }
    }
}

static void task3_run_once(void)
{
    if (task3_finished) {
        motor_output_both(0, 0);
        return;
    }

    task3_odom_update();

    switch (task3_phase) {
    case TASK3_AB_STRAIGHT:
        if (task3_run_straight_until_black()) {
            motor_output_both(0, 0);
            beep();
            gimbal_center();
            task3_next_phase(TASK3_B_AIM);
            task3_aim_ticks = 300;
        }
        break;

    case TASK3_B_AIM:
        motor_output_both(0, 0);
        gimbal_center();
        if (--task3_aim_ticks <= 0) {
            gimbal_center();
            DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_16);
            task3_next_phase(TASK3_BC_ARC);
        }
        break;

    case TASK3_BC_ARC:
        task3_run_arc(TASK3_CD_STRAIGHT);
        break;

    case TASK3_CD_STRAIGHT:
        if (task3_run_straight_until_black()) {
            beep();
            task3_next_phase(TASK3_DA_ARC);
        }
        break;

    case TASK3_C_AIM:
        motor_output_both(0, 0);
        gimbal_center();
        if (--task3_aim_ticks <= 0) {
            gimbal_center();
            DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_16);
            task3_next_phase(TASK3_DA_ARC);
        }
        break;

    case TASK3_DA_ARC:
        task3_run_arc(TASK3_DONE);
        break;

    default:
        task3_finish();
        break;
    }
}

static void gimbal_soft_pwm_background(void)
{
    if (mode==3) {
        gimbal_pitch_soft_pwm_service();
        return;
    }
    if (mode==4 && set==1) {
        mode4_draw_run();
        gimbal_pitch_soft_pwm_service();
        return;
    }
    if (mode==7 && set==1) {
        gimbal_pitch_soft_pwm_service();
        return;
    }
    if (mode==5 && set==1) {
        if (task3_phase==TASK3_B_AIM || task3_phase==TASK3_C_AIM) {
            gimbal_pitch_soft_pwm_service();
        }
    }
}

/* ============================================================================
 *  main() 和 TIMG8 ISR
 * ========================================================================== */
int main(void)
{
		system_init();  // 系统初始化 系统频率80MHZ 必须加
		delay_ms(500);  // 延时0.5s 等待电源稳定
		OLED_Init();
	  imu_init();     // imu初始化 其实就是初始化串口 这里是串口1
    key_init();

		motor_init();
		encoder_init();
	  uart0_init();
	  xunji_init();
//	track_init();
    buzzer_light_init();
	  gimbal_init();        /* ??:PA14(TIMG12 CC0) ?? yaw PWM,????? */
    mission_init();       /* ??:??? line PD??????? */
		tim_interrupt_ms_init(TIMG8, 10, 1);   // 定时器中断初始化 中断内放电机PID控制程序

	  pid_init(&motorA,DELTA_PID,10,10,5);
    pid_init(&motorB,DELTA_PID,10,10,5);
	  pid_init(&angle,POSITION_PID,0.04117,0,0.102);   //0.0405  0.1

    while (1)
	 {
		   if (!(mode==5 && set==1)) mission_disarm();   /* ????? mode5,???????? */
			 if(imu_flag)
			{
				imu_flag=0;
				imu_analysis();  //获取角度值存放在  yaw_angle_int
			}

		  KeyNum = key_GetNum();

		  /* ===== PA17 mode 键：切换模式 ===== */
		  if (KeyNum == 1)
		 {
		   mode++;
       if(mode>7)
			 {mode=1;}

       /* mode 切换时强制停止 */
       set=0;
       car_stop_all();  /* 立即停止电机，清空状态 */
       if (mode==3) {
           mode3_aim_reset();
       }
       if (mode==7) {
           mode7_diag_reset();
           gimbal_center();  /* mode7: 云台独立测试，切入时先回安全中位 */
       }
		 }

		  /* ===== PA27 set 键：启停切换 ===== */
		  if (KeyNum == 2)
		 {
			 if (set == 0) {
			     /* 当前停止 -> 启动 */
			     set = 1;
			     if (mode==1) beep();            /* mode1: 声光自检 */
			     if (mode==2) {
			         yaw_zero_here();            /* mode2: set启动瞬间作为 yaw 0000 */
			         task1_reset();              /* mode2: 校赛任务一，一圈后自动停车 */
			     }
			     if (mode==3) {
			         car_stop_all();   /* mode3: 定点瞄准，车必须静止 */
			         mode3_aim_prepare();
			     }
			     if (mode==5) {
			         yaw_zero_here();            /* mode5: set启动瞬间作为 yaw 0000 */
			         task3_reset();              /* mode5: 任务三，复用mode2路线+B点打靶 */
			     }
			     if (mode==6) motor_test_reset(); /* mode6: 复位电机测试计数器 */
			     if (mode==7) {
			         car_stop_all();   /* mode7: 启动云台测试前确保电机无残留 PWM */
			         mode7_diag_reset();
			         gimbal_center();  /* 从中位开始安全扫动 */
			     }
			 } else {
			     /* 当前运行 -> 停止 */
			     set = 0;
			     car_stop_all();  /* 立即停止电机，清空状态 */
			     if (mode==3) {
			         mode3_aim_reset();  /* mode3: 停止时回安全中位 */
			     }
			     if (mode==7) {
			         mode7_diag_reset();
			         gimbal_center();  /* mode7: 停止时云台回安全中位 */
			     }
			 }
		 }

          gimbal_soft_pwm_background();

		  /* 统一调试显示（所有 mode 1~7 格式一致，不随 mode 变化） */
		  oled_debug_update();
   }
}

void TIMG8_IRQHandler()
{
	   if(DL_TimerG_getPendingInterrupt(TIMG8) == DL_TIMER_IIDX_LOAD)
	  {
        left_encoder=-read_encoder1();
		    right_encoder=-read_encoder2();

		    if(mode==1&&set==1)
		   {
          track1();
		   }
			 	if(mode==2&&set==1)
		   {
          task1_run_once();
		   }
			 	if(mode==3&&set==1)
		   {
          mode3_aim_run();
		   }
			 	if(mode==4&&set==1)
		   {
          motor_output_both(0, 0);
		   }
			  if(mode==5&&set==1)          /* ??:????????? */
       {
          task3_run_once();
       }

        /* mode 6 临时测试：阶段3电机测试 / 阶段4低速循迹测试 */
        if(mode==6 && set==1) {
          /* 阶段3电机测试：取消下面注释，注释掉 line_follow_test_run() */
          // motor_suspended_test();

          /* 阶段4循迹测试（当前配置）：复用阶段3校准的电机参数 */
          line_follow_test_run();
        } else if(mode==6 && set==0) {
          /* mode 6 且 set=0 时，主动停止电机（安全保护） */
          motor_output_both(0, 0);
        }

        /* mode 7 硬件诊断：不跑车、不循迹、不进 mission，电机始终停止 */
        if(mode==7 && set==1) {
          car_stop_all();
          mode7_diag_run();
          gimbal_test_sweep_safe();
        } else if(mode==7 && set==0) {
          car_stop_all();
          mode7_diag_reset();
          gimbal_center();
        }

        /* 其他 mode 且 set=0 时，由各自的 trackN() 函数内部判断，或在 main 循环里已经 car_stop_all() */
	  }
}
