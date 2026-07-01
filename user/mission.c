#include "headfile.h"
#include "mission.h"
#include "xunji.h"        /* digtal() */
#include "_pid.h"         /* turn_pid/check/pid_cal/pid_out_limit/pid_t/motorA/motorB/POSITION_PID */
#include "ml_motor.h"     /* Set_left_pwm/Set_right_pwm */
#include "encoder.h"      /* left_encoder/right_encoder */
#include "buzzer_light.h" /* beep()  (同时开蜂鸣器+LED, TIMG7 自动关) */
#include "gimbal.h"       /* gimbal_aim_B/gimbal_center */

/* ============================================================================
 *  mission.c —— 巡迹到靶位联动 (mode==5) + 加权循迹统一驱动层
 *  借鉴：25E 加权偏差/串级/里程；24H 混合策略/发车保护/入弯保护。详见 mission.h。
 * ========================================================================== */

/* ===================== 灰度通道映射层 ===================== */
/* 映射表：gray_map[逻辑通道-1] = 物理通道
 * 当前配置（整条反转）：逻辑 D1~D8 = 物理 D8~D1
 * 如需恢复正装：改为 {1,2,3,4,5,6,7,8} */
static const uint8_t mission_gray_map[8] = {8, 7, 6, 5, 4, 3, 2, 1};

int mission_gray_mapped(int logic_idx)
{
    /* logic_idx: 1~8（逻辑通道，从左到右）
     * 返回：对应物理通道的灰度值（0=黑线, 1=白底） */
    if (logic_idx < 1 || logic_idx > 8) return 1;  /* 越界保护，返回白底 */
    return digtal(mission_gray_map[logic_idx - 1]);
}

/* ===================== 全局/调试量 ===================== */
task_state_t mission_state  = TASK_IDLE;
float   mission_dist     = 0.0f;
uint8_t mission_gray_bd  = 0xFF;   /* bit=0 压线, bit=1 白（与 digtal 同极性） */
uint8_t mission_nodata   = 0xFF;   /* 丢线置信计数（高=确信丢线），借 24H 思路 */
int     mission_wp_count = 0;

/* ===================== 内部状态 ===================== */
static pid_t line_pid;                 /* 外环：偏差->差速（PD） */
static float mission_bias = 0.0f;      /* 当前加权偏差 */
static int   last_bias_sign = 0;       /* 最近一次有效偏差的符号，丢线时朝它找线 */

static int   begin_protect_cnt = 0;    /* 发车保护倒计时 */
static int   stop_timer = 0;           /* 停车提示倒计时 */
static int   aim_timer = 0;            /* 云台瞄准停留倒计时 */
static int   mission_armed = 0;        /* 进入 mode5 自复位一次 */

static float last_marker_dist = -1e9f; /* 上一次关键点的里程（防抖间隔用） */

/* 8 路权重表 */
static const float kW[8] = {
    MISSION_W1, MISSION_W2, MISSION_W3, MISSION_W4,
    MISSION_W5, MISSION_W6, MISSION_W7, MISSION_W8
};

/* ===================== 灰度读取 + 加权偏差（25E 算法） ===================== */
static void mission_gray_update(void)
{
    uint8_t bd = 0x00;
    int i;
    float sum;
    int cnt;

    /* 使用映射后的灰度顺序读取（逻辑 D1~D8） */
    for (i = 0; i < 8; i++)
        if (mission_gray_mapped(i + 1)) bd |= (uint8_t)(1u << i);
    mission_gray_bd = bd;

    /* 丢线置信计数（24H GraySensorNoData）：持续全白/全黑则升，见到线则减半 */
    if ((bd == 0xFF || bd == 0x00) && mission_nodata != 0xFF) mission_nodata++;
    else if (mission_nodata != 0x00)                          mission_nodata >>= 1;

    /* 加权偏差：对所有"压线"的传感器(bit=0)求权重均值，再 *增益 */
    sum = 0.0f;
    cnt = 0;
    for (i = 0; i < 8; i++)
        if (!(bd & (1u << i))) { sum += kW[i]; cnt++; }

    if (cnt == 0) {
        mission_bias = 0.0f;                      /* 无线 -> 偏差 0（丢线另由保护处理） */
    } else {
        mission_bias = MISSION_BIAS_SIGN * (sum / (float)cnt) * MISSION_BIAS_GAIN;
        if (mission_bias > 0.5f)  last_bias_sign =  1;   /* 记住偏向，丢线时回头找 */
        if (mission_bias < -0.5f) last_bias_sign = -1;
    }
}

float mission_get_bias(void) { return mission_bias; }

/* ===================== 里程累计（25E：积分速度） ===================== */
static void mission_odom_update(void)
{
    /* left_encoder/right_encoder 已在 TIMG8 顶部取为本周期计数；前进为正。 */
    float d = ((float)left_encoder + (float)right_encoder) * 0.5f * MISSION_COUNT_TO_DIST;
    mission_dist += d;
}

/* ===================== 关键点检测 ===================== */
/* 直道<->半圆弧 交界检测（24H 法）。
 * 依据：本题赛道=2024 H 题赛道，直道为虚线(常丢线->nodata 高)、半圆弧为实线(连续有线->nodata 低)，
 *       A/B/C/D 即直道与弧的 4 个交界。检测 nodata 穿越阈值的跳变即到关键点。
 * 仅当直道确为虚线时有效；若实际直道是实线，请改用纯里程模式(见 mission.h)。 */
static uint8_t nodata_was_high = 1;     /* 上一拍是否"确信丢线"(在直道)；begin_protect/reset 也会写它 */

#if MISSION_WP_BY_TRANSITION
static int mission_transition_hit(void)
{
    int hit = 0;
    if (nodata_was_high && mission_nodata <= MISSION_ARC_ENTER) {
        /* 由直道(丢线)进入弧(有线) */
        if (mission_dist - last_marker_dist >= MISSION_WP_MIN_GAP) {
            last_marker_dist = mission_dist; hit = 1;
        }
        nodata_was_high = 0;
    } else if (!nodata_was_high && mission_nodata >= MISSION_ARC_EXIT) {
        /* 由弧(有线)回到直道(丢线) */
        if (mission_dist - last_marker_dist >= MISSION_WP_MIN_GAP) {
            last_marker_dist = mission_dist; hit = 1;
        }
        nodata_was_high = 1;
    }
    return hit;
}
#endif

/* 统一关键点判定：按 mission.h 选 跳变/里程。 */
static int mission_waypoint_reached(float dist_target)
{
#if MISSION_WP_BY_TRANSITION
    (void)dist_target;
    return mission_transition_hit();
#else  /* MISSION_WP_BY_DISTANCE */
    return (mission_dist >= dist_target);
#endif
}

/* ===================== 停车（带 PID 复位，防重启跳变） ===================== */
static void mission_stop(void)
{
    Set_left_pwm(0);
    Set_right_pwm(0);
    motorA.out = 0; motorB.out = 0;
    motorA.error[0] = motorA.error[1] = motorA.error[2] = 0;
    motorB.error[0] = motorB.error[1] = motorB.error[2] = 0;
}

/* ===================== 纯直行一拍（双轮同速，不纠偏、不检测，发车保护用） ===================== */
static void mission_drive_straight(int base)
{
    motorA.now = left_encoder;  motorA.target = base;
    motorB.now = right_encoder; motorB.target = base;
    pid_cal(&motorA); pid_cal(&motorB);
    pid_out_limit(&motorA); pid_out_limit(&motorB);
    Set_left_pwm((int)motorA.out);
    Set_right_pwm((int)motorB.out);
}

/* ===================== 加权循迹一拍（串级，借 25E MotorPidCtrl 结构） ===================== */
void mission_drive(int base)
{
    int diff;

#if MISSION_USE_HYBRID
    /* 混合策略（24H）：直道锁 IMU 航向。直道/弯道判别先用"确信丢线"近似(直道无弧线
     * 特征常丢线)，跑通后建议改为里程分段（见集成指南）。默认关闭。 */
    if (mission_nodata >= 0x40) {
        turn_pid(base, MISSION_STRAIGHT_YAW);
        return;
    }
#endif

    if (mission_nodata >= 0x40 && mission_gray_bd == 0xFF) {
        /* 入弯/丢线保护（24H Protect）：确信丢线时朝最近偏向强制找线 */
        int s = (last_bias_sign != 0) ? last_bias_sign : 1;
        diff = s * MISSION_LOST_FORCE_TURN;
    } else {
        /* 外环 line PD：偏差 -> 差速 */
        line_pid.now = mission_bias;
        line_pid.target = 0;
        pid_cal(&line_pid);
        diff = (int)line_pid.out;
        if (diff >  MISSION_TURN_MAX) diff =  MISSION_TURN_MAX;
        if (diff < -MISSION_TURN_MAX) diff = -MISSION_TURN_MAX;
    }

    /* 内环：复用 senior 已调好的双轮速度 PID（与 turn_pid 内环一致） */
    motorA.now = left_encoder;  motorA.target = base - diff;
    motorB.now = right_encoder; motorB.target = base + diff;
    pid_cal(&motorA); pid_cal(&motorB);
    pid_out_limit(&motorA); pid_out_limit(&motorB);
    Set_left_pwm((int)motorA.out);
    Set_right_pwm((int)motorB.out);
}

/* ===================== 复位 / 初始化 ===================== */
void mission_reset(void)
{
    mission_state    = TASK_TRACE_TO_B;   /* 发车即进入循迹到 B */
    mission_dist     = 0.0f;
    mission_wp_count = 0;
    mission_nodata   = 0xFF;
    mission_bias     = 0.0f;
    last_bias_sign   = 0;
    last_marker_dist = -1e9f;
    nodata_was_high  = 1;             /* 交界检测初始按"在直道"，第一段直道->弧 才算到 B */
    begin_protect_cnt = MISSION_BEGIN_PROTECT_TICKS;  /* 发车保护(24H) */
    stop_timer = 0; aim_timer = 0;

    /* 清 PID 历史，避免上次残留导致起步抽搐 */
    line_pid.error[0] = line_pid.error[1] = line_pid.error[2] = 0;
    line_pid.out = line_pid.iout = 0;
    mission_stop();
}

void mission_init(void)
{
    /* 外环 line PD（POSITION 模式，Ki=0 即纯 PD，弧线持续偏差不积分饱和） */
    pid_init(&line_pid, POSITION_PID, MISSION_LINE_KP, MISSION_LINE_KI, MISSION_LINE_KD);
    mission_armed = 0;
    mission_reset();
}

/* 解除武装：在 main() 循环里"不处于 mode5 运行(mode!=5 || set==0)"时调用一次，
 * 这样下次重新发车 mission_run() 会自动 mission_reset()，实现每次跑都从头干净开始，
 * 不依赖按键是否可靠触发（解决双通道按键 + 重跑不复位问题）。 */
void mission_disarm(void)
{
    mission_armed = 0;
}

/* ===================== 任务状态机（10ms 调用一次） ===================== */
void mission_run(void)
{
    /* 进入 mode5 后首拍自复位一次（也可在按键 set->1 时显式调 mission_reset） */
    if (!mission_armed) { mission_reset(); mission_armed = 1; }

    mission_gray_update();
    mission_odom_update();

    /* 发车保护（24H BeginProtect）：起步 1s 内翘头、灰度不准 ->
     * 双轮同速慢速直行，完全不纠偏、不检测关键点；过后再正常循迹。 */
    if (begin_protect_cnt > 0) {
        begin_protect_cnt--;
        mission_nodata   = 0xFF;        /* 保护期一律按"丢线"，且不让交界检测误触发 */
        nodata_was_high  = 1;
        last_marker_dist = mission_dist;/* 保护结束时以当前里程为基准，避免立刻误判关键点 */
        mission_drive_straight(MISSION_SPEED_SLOW);
        return;
    }

    switch (mission_state) {

    case TASK_IDLE:                      /* 正常不会停在这；保险 */
        mission_stop();
        mission_state = TASK_TRACE_TO_B;
        break;

    /* ---- A 出发，循迹到 B ---- */
    case TASK_TRACE_TO_B:
        mission_drive(MISSION_SPEED_BASE);
        if (mission_waypoint_reached(MISSION_DIST_TO_B)) {
            mission_wp_count = 1;
            beep();                       /* B 点提示 */
            stop_timer = MISSION_STOP_BEEP_TICKS;
            mission_state = TASK_STOP_AT_B;
        }
        break;

    /* ---- B 点停车 + 提示 ---- */
    case TASK_STOP_AT_B:
        mission_stop();
        if (--stop_timer <= 0) {
            gimbal_aim_B();               /* 触发云台瞄准 B 点靶 */
            aim_timer = MISSION_GIMBAL_AIM_TICKS;
            mission_state = TASK_GIMBAL_AIM;
        }
        break;

    /* ---- 云台瞄准，停留 ≤5s ---- */
    case TASK_GIMBAL_AIM:
        mission_stop();
        if (--aim_timer <= 0) {
            gimbal_center();              /* 收回云台（可按需保留瞄准姿态） */
            mission_state = TASK_TRACE_TO_C;
        }
        break;

    /* ---- 继续循迹 C ---- */
    case TASK_TRACE_TO_C:
        mission_drive(MISSION_SPEED_BASE);
        if (mission_waypoint_reached(MISSION_DIST_TO_C)) {
            mission_wp_count = 2;
            beep();                       /* C 点提示 */
            mission_state = TASK_TRACE_TO_D;
        }
        break;

    /* ---- 循迹 D ---- */
    case TASK_TRACE_TO_D:
        mission_drive(MISSION_SPEED_BASE);
        if (mission_waypoint_reached(MISSION_DIST_TO_D)) {
            mission_wp_count = 3;
            beep();                       /* D 点提示 */
            mission_state = TASK_TRACE_TO_A;
        }
        break;

    /* ---- 循迹回 A ---- */
    case TASK_TRACE_TO_A:
        mission_drive(MISSION_SPEED_BASE);
        if (mission_waypoint_reached(MISSION_DIST_TO_A)) {
            mission_wp_count = 4;
            stop_timer = MISSION_STOP_BEEP_TICKS;
            mission_state = TASK_FINISH;
        }
        break;

    /* ---- 完成：A 点停车 + 提示，保持停止 ---- */
    case TASK_FINISH:
        mission_stop();
        if (stop_timer > 0) {             /* 进入时响一次，之后保持安静停车 */
            if (stop_timer == MISSION_STOP_BEEP_TICKS) beep();
            stop_timer--;
        }
        break;

    default:
        mission_stop();
        break;
    }
}