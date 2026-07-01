#include "headfile.h"
#include "_pid.h"
#include "key.h"
#include "gimbal.h"      /* ?? */
#include "mission.h"     /* ?? */
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
#define LINE_TEST_PWM_BASE         2400   /* 循迹基础速度（略高于起转阈值） */
#define LINE_TEST_TURN_GAIN        80     /* 转向增益，偏差->差速的放大系数 */
#define LINE_TEST_TURN_MAX         500    /* 最大差速限幅，防止一边突然太快 */
#define LINE_TEST_BIAS_SIGN        1      /* 偏差符号，反向改成 -1 */

/* ==================== 测试状态变量（需在函数前声明） ==================== */
static int motor_test_tick = 0;           /* 电机测试计数器 */
static float line_last_bias = 0.0f;       /* 上一帧循迹偏差 */

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
    Set_left_pwm(0);
    Set_right_pwm(0);

    /* 清空 mode6 临时测试状态 */
    motor_test_tick = 0;
    line_last_bias = 0.0f;

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
    motor_output_both(0, 0);
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
    int i, black_cnt, turn, left_pwm, right_pwm;
    float sum, bias;

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

    /* 3. 判断是否有线 */
    if (black_cnt == 0) {
        /* 全白丢线：停车，避免冲出赛道（保守策略） */
        motor_output_both(0, 0);
        return;
    } else {
        /* 有线：计算平均偏差 */
        bias = LINE_TEST_BIAS_SIGN * (sum / (float)black_cnt);
        line_last_bias = bias;  /* 记录有效偏差 */
    }

    /* 4. 根据偏差计算转向差速 */
    turn = (int)(bias * LINE_TEST_TURN_GAIN);
    if (turn > LINE_TEST_TURN_MAX)  turn = LINE_TEST_TURN_MAX;
    if (turn < -LINE_TEST_TURN_MAX) turn = -LINE_TEST_TURN_MAX;

    /* 5. 计算左右轮 PWM（base - turn, base + turn） */
    left_pwm  = LINE_TEST_PWM_BASE - turn;
    right_pwm = LINE_TEST_PWM_BASE + turn;

    /* 6. 输出到电机（自动应用阶段3校准的方向和补偿） */
    motor_output_both(left_pwm, right_pwm);
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
       if(mode==7)
			 {mode=1;}

       /* mode 切换时强制停止 */
       set=0;
       car_stop_all();  /* 立即停止电机，清空状态 */
		 }

		  /* ===== PA27 set 键：启停切换 ===== */
		  if (KeyNum == 2)
		 {
			 if (set == 0) {
			     /* 当前停止 -> 启动 */
			     set = 1;
			     if (mode==5) mission_reset();   /* mode5: 复位任务状态机 */
			     if (mode==6) motor_test_reset(); /* mode6: 复位电机测试计数器 */
			 } else {
			     /* 当前运行 -> 停止 */
			     set = 0;
			     car_stop_all();  /* 立即停止电机，清空状态 */
			 }
		 }

		  /* 统一调试显示（所有 mode 1~6 格式一致，不随 mode 变化） */
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
          track2();
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

        /* 其他 mode 且 set=0 时，由各自的 trackN() 函数内部判断，或在 main 循环里已经 car_stop_all() */
	  }
}
