#ifndef __mission_h_
#define __mission_h_

#include "headfile.h"

/* ============================================================================
 *  mission.h —— 巡迹到靶位联动任务（北邮 2026 电赛 / 任务3, mode==5）
 *               兼做"加权循迹"的统一驱动层（取代 track2/3/4 的巨型 if/else）
 * ----------------------------------------------------------------------------
 *  借鉴来源（均已读源码核实）：
 *   - 25E GraySensorToTurnAngle()  : 加权位置算法，8 路二值 -> 连续偏差，取代 if/else。
 *   - 25E MotorPidCtrl()           : 串级——外环(偏差->差速 PD) + 内环(每轮速度 PID)。
 *   - 25E Encoder X/V              : 里程累计，用于按距离切速度 / 兜底关键点判断。
 *   - 24H Task4 混合策略           : 直道用 IMU 锁航向狂飙、弯道用灰度循迹（与本题
 *                                    "两直道+两半圆弧"完全对应）；近弯降速。
 *   - 24H BeginProtect/Protect     : 发车翘头保护 + 入弯丢线强制找线保护。
 *
 *  控制接口复用 senior 现成实现（不重写 PID 库）：
 *   - 内环：直接用 senior 的 motorA/motorB 速度 PID（pid_cal + pid_out_limit）。
 *   - 外环：本模块新增一个 line PD（以加权偏差为误差，目标 0，输出差速）。
 *   - 直道可选直接调 senior 的 turn_pid(base, yaw_target) 锁 IMU 航向。
 *
 *  运行上下文：与 trackN 一样，在 10ms 的 TIMG8_IRQHandler 里 mode==5&&set==1 调用
 *             mission_run()。所有计时用 tick 计数（1 tick = 10ms）。
 * ========================================================================== */

/* ===================== 1. 加权循迹参数（按实测调） ===================== */
/* 8 路权重：从 D1 到 D8。压在中间->和≈0；偏一侧->和偏向该侧。来自 25E。
 * 若你的灰度 D1/D8 物理左右与符号方向不符，用 MISSION_BIAS_SIGN 翻转即可。 */
#define MISSION_W1   3.0f
#define MISSION_W2   1.5f
#define MISSION_W3   0.5f
#define MISSION_W4   0.10f
#define MISSION_W5  -0.10f
#define MISSION_W6  -0.5f
#define MISSION_W7  -1.5f
#define MISSION_W8  -3.0f
#define MISSION_BIAS_GAIN   10.0f   /* 偏差均值 -> 转角，来自 25E（*10） */
#define MISSION_BIAS_SIGN   (+1)    /* 方向不对就改成 -1（硬件标定一次） */

/* 外环 line PD（偏差->差速）。senior 100Hz；25E 的(0.8,0,0.15)是 200Hz，
 * 这里给一组保守初值，先小后大慢慢加，先 D 后大 P。 */
#define MISSION_LINE_KP     0.9f
#define MISSION_LINE_KI     0.0f    /* 弧线持续偏差，用 PD 防积分饱和（与 24H/25E 一致） */
#define MISSION_LINE_KD     0.20f
#define MISSION_TURN_MAX    200     /* 差速限幅（对应 25E SelfTurn_Maxout=200） */

/* 速度（单位=编码器计数/10ms，与 senior turn_pid 的 base 同量纲，先慢后快） */
#define MISSION_SPEED_FAST   16     /* 直道/弧线中段 */
#define MISSION_SPEED_SLOW    9     /* 近关键点/入弯 减速，提升停车与转弯稳定性 */
#define MISSION_SPEED_BASE   MISSION_SPEED_SLOW   /* 联动任务第一版统一用慢速，跑通再提速 */

/* ===================== 2. 混合策略开关（24H） ===================== */
/* 0 = 全程灰度加权循迹（最简，直道弧线统一处理，先用这个把流程跑通）
 * 1 = 直道锁 IMU 航向 + 弯道灰度（更快更直，跑通后再开，需先标好直道航向角） */
#define MISSION_USE_HYBRID   0
#define MISSION_STRAIGHT_YAW  0      /* 混合模式下第一条直道目标航向（IMU 标定后填） */

/* ===================== 3. 关键点检测方式 ===================== */
/* ★已据题目 PDF 确认：本题赛道 = 2024 H 题赛道，直道为"直线/虚线"、半圆弧为实线，
 *   全程无垂直黑标记带；线宽仅 1.8cm，8 路不可能同时全黑。故用"直道↔弧跳变"。
 *
 * TRANSITION = 直道(虚线->常丢线)↔半圆弧(实线->连续有线) 的交界跳变触发（24H 法，推荐）
 * DISTANCE   = 纯里程：到设定距离即认为到关键点（万一实际直道是实线、跳变不可用时用） */
#define MISSION_WP_BY_TRANSITION  1
#define MISSION_WP_BY_DISTANCE    0

/* 跳变检测阈值（带迟滞）：nodata<=ENTER 视为"进入弧/有线"，>=EXIT 视为"回到直道/丢线"。
 * nodata 由 mission_gray_update 维护(全白/全黑则升、有线则减半)，范围 0x00~0xFF。 */
#define MISSION_ARC_ENTER     0x20     /* 低于它=确信有线 */
#define MISSION_ARC_EXIT      0x40     /* 高于它=确信丢线 */
#define MISSION_WP_MIN_GAP    200.0f   /* 两次关键点最小里程间隔，防同一交界抖动重复计数 */

/* 纯距离模式下，A 起点到 B/C/D/A 的累计里程（里程单位 = mm 或自定，需标定） */
#define MISSION_DIST_TO_B   600.0f
#define MISSION_DIST_TO_C   1400.0f
#define MISSION_DIST_TO_D   2000.0f
#define MISSION_DIST_TO_A   2800.0f

/* ===================== 4. 里程标定（25E 公式） ===================== */
/* 每 10ms 把 (左计数+右计数)/2 * 该系数 累加为里程。先随便给个数，
 * 用"手推车一米看里程读数"标定后改这里。 */
#define MISSION_COUNT_TO_DIST   0.20f

/* ===================== 5. 保护与计时（24H） ===================== */
#define MISSION_BEGIN_PROTECT_TICKS  100   /* 发车保护 100*10ms=1s，期间忽略关键点/翘头 */
#define MISSION_LOST_FORCE_TURN      50    /* 弯道丢线时强制差速找线（符号见实现） */
#define MISSION_GIMBAL_AIM_TICKS     300   /* 云台瞄准停留 300*10ms=3s（≤5s 规则内） */
#define MISSION_STOP_BEEP_TICKS      50    /* 到点停车提示时长 */

/* ===================== 任务状态机 ===================== */
typedef enum {
    TASK_IDLE = 0,
    TASK_TRACE_TO_B,
    TASK_STOP_AT_B,
    TASK_GIMBAL_AIM,
    TASK_TRACE_TO_C,
    TASK_TRACE_TO_D,
    TASK_TRACE_TO_A,
    TASK_FINISH
} task_state_t;

/* ===================== 对外接口 ===================== */
void mission_init(void);        /* 在 main() 里 motor/encoder/灰度/云台初始化之后调用 */
void mission_run(void);         /* 在 TIMG8 ISR 里 mode==5&&set==1 每 10ms 调用 */
void mission_reset(void);       /* set 由 0->1 发车前复位状态/里程/标志（解决 change_flag1 不复位的老坑） */
void mission_disarm(void);      /* main 循环里"非 mode5 运行"时调用，保证每次重跑都干净复位 */

/* 单独的加权循迹（可绑到某个 mode 做循迹单测；不含任务状态机） */
void mission_drive(int base_speed);   /* 跑一拍灰度加权循迹 */
float mission_get_bias(void);         /* 当前加权偏差，供调试/OLED */

/* 供 OLED/调试读取 */
extern task_state_t mission_state;
extern float  mission_dist;       /* 当前累计里程 */
extern uint8_t mission_gray_bd;   /* 当前 8 路二值（bit=0 压线, bit=1 白） */
extern uint8_t mission_nodata;    /* 丢线置信计数（高=确信丢线） */
extern int    mission_wp_count;   /* 已经过的关键点数 */

#endif
