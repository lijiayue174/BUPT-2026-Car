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
 *    第 3 行: yaw:XXXX         —— IMU 航向角 yaw_angle_int
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
    OLED_ShowSignedNum(3, 5, yaw_angle_int, 4);

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
#define LINE_TEST_PWM_BASE          2200   /* low-speed line base PWM; if motor stalls, try 2300/2400 on site */
#define LINE_TEST_PWM_MIN           0
#define LINE_TEST_PWM_MAX           3600
#define LINE_TEST_TURN_GAIN         160    /* bias -> differential PWM gain */
#define LINE_TEST_RIGHT_TURN_GAIN   420    /* stronger gain when bias < 0, for clockwise/right curves */
#define LINE_TEST_RIGHT_TURN_OFFSET 500    /* right-turn preload once line moves to the right side */
#define LINE_TEST_RIGHT_TURN_EXTRA  300    /* extra right turn when abs(bias) >= 0.5 */
#define LINE_TEST_TURN_MAX          1800   /* max differential PWM */
#define LINE_TEST_CURVE_SLOW_GAIN   80     /* larger bias -> lower dynamic base */
#define LINE_TEST_RIGHT_SLOW_GAIN   140    /* stronger slowing when bias < 0 */
#define LINE_TEST_PWM_BASE_MIN      1600   /* minimum moving base while line is visible */
#define LINE_TEST_RIGHT_BASE_MIN    1200   /* allow lower base in 40cm clockwise/right curves */
#define LINE_TEST_BIAS_SIGN         1      /* reverse to -1 if steering direction is wrong */
#define LINE_TEST_LOST_SHORT_COUNT  20     /* 20 * 10ms: keep searching shortly after line loss */
#define LINE_TEST_LOST_PWM_BASE     1600   /* slow search base after short line loss */

/* ==================== 测试状态变量（需在函数前声明） ==================== */
static int motor_test_tick = 0;           /* 电机测试计数器 */
static int line_last_turn = 0;            /* last valid turn PWM for short line-loss recovery */
static int line_lost_count = 0;           /* continuous all-white sample count */
static int line_last_black_cnt = 0;       /* last sample black sensor count, for mode2 task1 only */
static int mode7_diag_tick = 0;           /* mode7 hardware diag tick, 10ms per tick */
static uint8_t mode7_led_state = 0;

/* ==================== mode2: 校赛任务一，顺时针 A-B-C-D-A ==================== */
#define TASK1_COUNT_TO_DIST       0.20f   /* same odom scale as mission.c, tune on site if distance is off */
#define TASK1_STRAIGHT_DIST_MM    1000.0f /* A-B and C-D straight section, 100cm */
#define TASK1_STRAIGHT_LEFT_PWM   1100    /* straight-only trim: lower left to reduce right drift */
#define TASK1_STRAIGHT_RIGHT_PWM  1850
#define TASK1_STRAIGHT_BALANCE_GAIN 0.0f  /* 0 disables encoder balance while tuning straight PWM */
#define TASK1_STRAIGHT_BALANCE_MAX  900
#define TASK1_STRAIGHT_BLACK_MIN_TICKS 30 /* allow black-line handoff after 0.3s straight */
#define TASK1_ARC_MIN_TICKS       80      /* ignore line loss at arc entry for 0.8s */
#define TASK1_ARC_LOST_TICKS      25      /* 0.25s all-white after arc means arc finished */

typedef enum {
    TASK1_AB_STRAIGHT = 0,
    TASK1_BC_ARC,
    TASK1_CD_STRAIGHT,
    TASK1_DA_ARC,
    TASK1_DONE
} task1_phase_t;

static task1_phase_t task1_phase = TASK1_AB_STRAIGHT;
static float task1_dist = 0.0f;
static float task1_left_dist = 0.0f;
static float task1_right_dist = 0.0f;
static int task1_phase_ticks = 0;
static int task1_arc_lost_ticks = 0;
static int task1_arc_done_count = 0;
static uint8_t task1_arc_seen_line = 0;
static uint8_t task1_finished = 0;

static void task1_clear_state(void)
{
    task1_phase = TASK1_AB_STRAIGHT;
    task1_dist = 0.0f;
    task1_left_dist = 0.0f;
    task1_right_dist = 0.0f;
    task1_phase_ticks = 0;
    task1_arc_lost_ticks = 0;
    task1_arc_done_count = 0;
    task1_arc_seen_line = 0;
    task1_finished = 0;
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

/* 同时设置左右轮（方便统一调用） */
static void motor_output_both(int left_pwm, int right_pwm)
{
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
    int turn_gain, slow_gain;
    float sum, bias, abs_bias;

    black_cnt = 0;
    sum = 0.0f;
    bias = 0.0f;

    /* 1. 读取灰度传感器（0=黑线, 1=白底），使用映射后的逻辑顺序 D1~D8 */
    for (i = 0; i < 8; i++) {
        gray[i] = gray_mapped(i + 1);  /* gray_mapped(1~8) */
    }

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
     * On this car, clockwise/right curves are weaker. The 40cm contest arc
     * needs earlier correction, so bias < 0 uses a preload offset instead of
     * waiting for D7/D8 to create a large averaged bias.
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
    if (bias < 0.0f) {
        turn -= LINE_TEST_RIGHT_TURN_OFFSET;
        if (abs_bias_x10 >= 5) {
            turn -= LINE_TEST_RIGHT_TURN_EXTRA;
        }
    }
    if (turn > LINE_TEST_TURN_MAX)  turn = LINE_TEST_TURN_MAX;
    if (turn < -LINE_TEST_TURN_MAX) turn = -LINE_TEST_TURN_MAX;
    line_last_turn = turn;

    dynamic_base = LINE_TEST_PWM_BASE - abs_bias_x10 * slow_gain / 10;
    if (dynamic_base < base_min) {
        dynamic_base = base_min;
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

static void task1_next_phase(task1_phase_t next_phase)
{
    task1_phase = next_phase;
    task1_dist = 0.0f;
    task1_left_dist = 0.0f;
    task1_right_dist = 0.0f;
    task1_phase_ticks = 0;
    task1_arc_lost_ticks = 0;
    task1_arc_seen_line = 0;
    line_lost_count = 0;
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
    int balance;

    balance = (int)((task1_left_dist - task1_right_dist) * TASK1_STRAIGHT_BALANCE_GAIN);
    if (balance > TASK1_STRAIGHT_BALANCE_MAX) {
        balance = TASK1_STRAIGHT_BALANCE_MAX;
    }
    if (balance < -TASK1_STRAIGHT_BALANCE_MAX) {
        balance = -TASK1_STRAIGHT_BALANCE_MAX;
    }

    motor_output_both(TASK1_STRAIGHT_LEFT_PWM - balance,
                      TASK1_STRAIGHT_RIGHT_PWM + balance);
}

static void task1_run_arc(task1_phase_t next_phase)
{
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
        task1_arc_done_count++;
        if (task1_arc_done_count >= 2) {
            task1_finish();
        } else {
            task1_next_phase(next_phase);
        }
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
			     if (mode==2) task1_reset();     /* mode2: 校赛任务一，一圈后自动停车 */
			     if (mode==5) mission_reset();   /* mode5: 复位任务状态机 */
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
			     if (mode==7) {
			         mode7_diag_reset();
			         gimbal_center();  /* mode7: 停止时云台回安全中位 */
			     }
			 }
		 }

          /*
           * PA16 pitch uses software PWM. Run one 20ms frame only while the car
           * is stopped in mode7 gimbal test; never block inside TIMG8 ISR.
           */
          if (mode==7 && set==1) {
              gimbal_pitch_soft_pwm_service();
          }

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
          track3();
		   }
			 	if(mode==4&&set==1)
		   {
          track4();
		   }
			  if(mode==5&&set==1)          /* ??:????????? */
       {
          mission_run();
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
