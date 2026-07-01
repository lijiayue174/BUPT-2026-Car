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
		  if (KeyNum == 1)			
		 {
		   mode++;
       if(mode==7)
			 {mode=1;}
       if(set==1)
       {set=0; }				 
		 }
		  if (KeyNum == 2)			
		 {
			 set=1;
			 if (mode==5) mission_reset();   /* ??:???????????? */
		 }
			
		  /* 第1行: M:X S:X */
		  OLED_ShowString(1, 1, "M:");
		  OLED_ShowNum(1, 3, mode, 1);
		  OLED_ShowString(1, 5, "S:");
		  OLED_ShowNum(1, 7, set, 1);

		  /* 第2行: G:D1D2D3D4D5D6D7D8 */
		  {
		      uint8_t gi;
		      OLED_ShowString(2, 1, "G:");
		      for (gi = 1; gi <= 8; gi++) {
		          OLED_ShowNum(2, (uint8_t)(2 + gi), digtal(gi), 1);
		      }
		  }

		  /* 第3行: yaw */
		  OLED_ShowString(3, 1, "yaw:");
		  OLED_ShowSignedNum(3, 5, yaw_angle_int, 4);

		  /* 第4行: 编码器调试 / 任务状态 */
		  if (mode==5) {
              OLED_ShowString(4, 1, "st:");
              OLED_ShowNum(4, 4, (int)mission_state, 1);
          } else {
              OLED_ShowString(4, 1, "L:");
              OLED_ShowSignedNum(4, 3, left_encoder, 4);
              OLED_ShowString(4, 8, "R:");
              OLED_ShowSignedNum(4, 10, right_encoder, 4);
          }
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
        if(mode==6&&set==1)          /* ??:??????(????) */
       {
          gimbal_test_sweep_safe();
       }
	  } 
}









