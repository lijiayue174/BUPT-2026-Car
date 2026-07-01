#include "headfile.h"
#include "ml_gpio.h"
#include "xunji.h"
#include "ml_motor.h"
#include "pid.h"
#include "buzzer_light.h"

//int left_encoder,right_encoder,left_pwm,right_pwm,left_angle_pwm,right_angle_pwm;
//int base_left_pwm,base_right_pwm,angle_pwm;

int now_statue,last_statue,change_flag1;


void xunji_init()
{
  gpio_init(GPIOB, DL_GPIO_PIN_13, PB13, IN_UP);   // D1
	gpio_init(GPIOB, DL_GPIO_PIN_15, PB15, IN_UP);   // D2
	gpio_init(GPIOA, DL_GPIO_PIN_31, PA31, IN_UP);   // D3
	gpio_init(GPIOA, DL_GPIO_PIN_28, PA28, IN_UP);   // D4
	gpio_init(GPIOB, DL_GPIO_PIN_1,  PB1, IN_UP);   // D5
	gpio_init(GPIOB, DL_GPIO_PIN_4,  PB4, IN_UP);   // D6
	gpio_init(GPIOB, DL_GPIO_PIN_17, PB17, IN_UP);   // D7
	gpio_init(GPIOB, DL_GPIO_PIN_12, PB12, IN_UP);   // D8
}

unsigned char digtal(unsigned char channel)//1-8	  ��ȡXͨ������ֵ
{
	uint8_t value = 0;
	switch(channel) 
	{
		case 1:  
			if (gpio_get(GPIOB, DL_GPIO_PIN_13)) value = 0;
			else value = 1;  
			break;  
		case 2: 
			if(gpio_get(GPIOB, DL_GPIO_PIN_15)) value = 0;
			else value = 1;  
			break;  
		case 3: 
			if(gpio_get(GPIOA, DL_GPIO_PIN_31)) value = 0;
			else value = 1;  
			break;   
		case 4:  
			if(gpio_get(GPIOA, DL_GPIO_PIN_28)) value = 0;
			else value = 1;  
			break;   
		case 5:
			if(gpio_get(GPIOB, DL_GPIO_PIN_1)) value = 0;
			else value = 1;  
			break;
		case 6:  
			if(gpio_get(GPIOB, DL_GPIO_PIN_4)) value = 0;
			else value = 1;  
			break;  
		case 7: 
			if(gpio_get(GPIOB, DL_GPIO_PIN_17)) value = 0;
			else value = 1;  
			break;  
 		case 8: 
 			if(gpio_get(GPIOB, DL_GPIO_PIN_12)) value = 0;
 			else value = 1;  
 			break;   
	}
	return value; 
}

void track1(void)
{
       if(D1==0|D2==0|D3==0|D4==0|D5==0|D6==0|D7==0|D8==0)
	   { Set_right_pwm(0);	Set_left_pwm(0); beep(); }             //STOP          
	     else 
	    { 
          turn_pid(10,0);
			}                          //PID
	 
}

void track2(void)
{
	  if(D1==1 && D2==1 && D3==1 && D4==0 && D5==0 && D6==1 && D7==1 && D8==1)     //11100111
	{
//		sensor_bias = 0;
		Set_right_pwm(4500); Set_left_pwm(4500); 
	}
	
	 else if(D1==1 && D2==1 && D3==0 && D4==0 && D5==1 && D6==1 && D7==1 && D8==1) //11001111
	{
//		sensor_bias = -15;
		Set_right_pwm(6000); Set_left_pwm(2700); 
	}
   else if(D1==1 && D2==0 && D3==0 && D4==1 && D5==1 && D6==1 && D7==1 && D8==1)  //10011111
	{
//		sensor_bias = -25;
		Set_right_pwm(7500); Set_left_pwm(1500); 
	}
	 else if(D1==0 && D2==0 && D3==1 && D4==1 && D5==1 && D6==1 && D7==1 && D8==1)  //00111111
	{
//		sensor_bias = -35;
		Set_right_pwm(7800); Set_left_pwm(1050); 
	}
	 else if(D1==0 && D2==1 && D3==1 && D4==1 && D5==1 && D6==1 && D7==1 && D8==1)  //01111111
	{
//		sensor_bias = -45;
		Set_right_pwm(8400); Set_left_pwm(600); 
	}
	
	 // danglu
	 	else if(D1==1 && D2==1 && D3==1 && D4==0 && D5==1 && D6==1 && D7==1 && D8==1) //11101111
	{
//		sensor_bias = -5;
		Set_right_pwm(5250); Set_left_pwm(3750); 
	}
   else if(D1==1 && D2==1 && D3==0 && D4==1 && D5==1 && D6==1 && D7==1 && D8==1)   //11011111
	{
//		sensor_bias = -20;
		Set_right_pwm(6300); Set_left_pwm(3000); 
	}
	 else if(D1==1 && D2==0 && D3==1 && D4==1 && D5==1 && D6==1 && D7==1 && D8==1)   //10111111
	{
//		sensor_bias = -30;
		Set_right_pwm(7200); Set_left_pwm(1500); 
	}
	
	 else if(D1==1 && D2==1 && D3==1 && D4==1 && D5==0 && D6==1 && D7==1 && D8==1)  //11110111
	{
//		sensor_bias = 10;
		Set_right_pwm(3750); Set_left_pwm(5250);  
	}
   else if(D1==1 && D2==1 && D3==1 && D4==1 && D5==1 && D6==0 && D7==1 && D8==1)   //11111011
	{
//		sensor_bias = 20;
		Set_right_pwm(3000); Set_left_pwm(6000);  
	}
	 else if(D1==1 && D2==1 && D3==1 && D4==1 && D5==1 && D6==1 && D7==0 && D8==1)   //11111101
	{
//		sensor_bias = 30;
		Set_right_pwm(1800); Set_left_pwm(7200); 
	}

	 else if(D1==1 && D2==1 && D3==1 && D4==1 && D5==0 && D6==0 && D7==1 && D8==1)  //11110011
	{
//		sensor_bias = 15;
		Set_right_pwm(2700); Set_left_pwm(6450);  
	}
	 else if(D1==1 && D2==1 && D3==1 && D4==1 && D5==1 && D6==0 && D7==0 && D8==1)  //11111001
	{
//		sensor_bias = 25;
		Set_right_pwm(2100); Set_left_pwm(6750);  
	}
	 else if(D1==1 && D2==1 && D3==1 && D4==0 && D5==0 && D6==1 && D7==0 && D8==0)  //11111100
	{
//		sensor_bias = 35;
		Set_right_pwm(1800);  Set_left_pwm(7200);  
	}
	 else if(D1==1 && D2==1 && D3==1 && D4==1 && D5==1 && D6==1 && D7==1 && D8==0)  //11111110
	{
//		sensor_bias = 45;
		Set_right_pwm(1200); Set_left_pwm(7800);  
	}
	 else if(D1==1 && D2==1 && D3==1 && D4==1 && D5==1 && D6==1 && D7==1 && D8==1)  //11111111
   {
//	   sensor_bias = 0;
		  last_statue=now_statue;
		  now_statue=0;
   }
	   if(D1==0|D2==0|D3==0|D4==0|D5==0|D6==0|D7==0|D8==0)
	 {
		    last_statue=now_statue;
		    now_statue=1;
	 }
	        
	 		    if(now_statue!=last_statue) 
	       {
			     change_flag1++;
           beep();	
		     }
				 
				   if(change_flag1==0)
			    {
				    turn_pid(10,0);
		      }	 
					
           if(change_flag1==2)
			    {
						check(177);
				    turn_pid(10,177); 
		      }	 
			     if(change_flag1==4)
			    {
			    	Set_left_pwm(0);
   		      Set_right_pwm(0);
				    change_flag1++;
			    }
}
void track3(void)
{
	  if(D1==1 && D2==1 && D3==1 && D4==0 && D5==0 && D6==1 && D7==1 && D8==1)     //11100111
	{
//		sensor_bias = 0;
		Set_right_pwm(4500); Set_left_pwm(4500); 
	}
	
	 else if(D1==1 && D2==1 && D3==0 && D4==0 && D5==1 && D6==1 && D7==1 && D8==1) //11001111
	{
//		sensor_bias = -15;
//    Set_right_pwm(5250); Set_left_pwm(3300); 
		Set_right_pwm(6300); Set_left_pwm(2100); 
	}
   else if(D1==1 && D2==0 && D3==0 && D4==1 && D5==1 && D6==1 && D7==1 && D8==1)  //10011111
	{
//		sensor_bias = -25;
		Set_right_pwm(7050); Set_left_pwm(1500); 
	}
	 else if(D1==0 && D2==0 && D3==1 && D4==1 && D5==1 && D6==1 && D7==1 && D8==1)  //00111111
	{
//		sensor_bias = -35; 
			if(yaw_angle_int>-120&&yaw_angle_int<-90)
		{
			Set_right_pwm(1050);  Set_left_pwm(8100);  
		}
		 else
		 { Set_right_pwm(8100);  Set_left_pwm(1050); } 
		             //5000               800
	}
	
	 else if(D1==0 && D2==1 && D3==1 && D4==1 && D5==1 && D6==1 && D7==1 && D8==1)  //01111111
	{
			if(yaw_angle_int>-120&&yaw_angle_int<-90)
		{
			Set_right_pwm(300);  Set_left_pwm(9300);  
		}
		 else
		 { Set_right_pwm(9300);  Set_left_pwm(300); } 
		              //5500              500
	 }
	 // danglu
	 	else if(D1==1 && D2==1 && D3==1 && D4==0 && D5==1 && D6==1 && D7==1 && D8==1) //11101111
	{
//		sensor_bias = -5;
		Set_right_pwm(5250); Set_left_pwm(3750); 
	}
   else if(D1==1 && D2==1 && D3==0 && D4==1 && D5==1 && D6==1 && D7==1 && D8==1)   //11011111
	{
//		sensor_bias = -20;
		Set_right_pwm(6750); Set_left_pwm(1500); 
	}
	 else if(D1==1 && D2==0 && D3==1 && D4==1 && D5==1 && D6==1 && D7==1 && D8==1)   //10111111
	{
     	 if(yaw_angle_int>-120&&yaw_angle_int<-90)
		{
			Set_right_pwm(300);  Set_left_pwm(9300);  
		}
		 else
		 { Set_right_pwm(9300);  Set_left_pwm(300); } 
	}
	 else if(D1==1 && D2==1 && D3==1 && D4==1 && D5==0 && D6==1 && D7==1 && D8==1)  //11110111
	{    
//		sensor_bias = 10;
		 Set_right_pwm(2250); Set_left_pwm(6750);  
	}
   else if(D1==1 && D2==1 && D3==1 && D4==1 && D5==1 && D6==0 && D7==1 && D8==1)   //11111011
	{
//		sensor_bias = 20;
		Set_right_pwm(1200); Set_left_pwm(7800);  
	}
	 else if(D1==1 && D2==1 && D3==1 && D4==1 && D5==1 && D6==1 && D7==0 && D8==1)   //11111101
	{
//		sensor_bias = 30;
			if(yaw_angle_int>-60&&yaw_angle_int<-20)
		{
			Set_right_pwm(9300);  Set_left_pwm(300);  
		}
		 else
		 { Set_right_pwm(300);  Set_left_pwm(9300);} 
	}

	 else if(D1==1 && D2==1 && D3==1 && D4==1 && D5==0 && D6==0 && D7==1 && D8==1)  //11110011
	{
//		sensor_bias = 15;
		Set_right_pwm(1800); Set_left_pwm(7200);  
	}
	 else if(D1==1 && D2==1 && D3==1 && D4==1 && D5==1 && D6==0 && D7==0 && D8==1)  //11111001
	{
//		sensor_bias = 25;
		Set_right_pwm(1500); Set_left_pwm(7500);  
	}
	 else if(D1==1 && D2==1 && D3==1 && D4==1 && D5==1 && D6==1 && D7==0 && D8==0)  //11111100
	{
		if(yaw_angle_int>-60&&yaw_angle_int<-20)
		{
			Set_right_pwm(8700);  Set_left_pwm(900);  
		}
//		sensor_bias = 35;
		 else
		 { Set_right_pwm(900);  Set_left_pwm(8700);} 
	}
	
	 else if(D1==1 && D2==1 && D3==1 && D4==1 && D5==1 && D6==1 && D7==1 && D8==0)  //11111110
	{
//		sensor_bias = 45;
			if(yaw_angle_int>-60&&yaw_angle_int<-20)
		{
			Set_right_pwm(9300);  Set_left_pwm(300);  
		}
		 else
		 { Set_right_pwm(300);  Set_left_pwm(9300);} 
		  
	}
	
	  else if(D1==1 && D2==1 && D3==1 && D4==1 && D5==1 && D6==1 && D7==1 && D8==1)  //11111111
   {
		  last_statue=now_statue;
		  now_statue=0;
   }
	 
	   if(D1==0|D2==0|D3==0|D4==0|D5==0|D6==0|D7==0|D8==0)
	 {
		    last_statue=now_statue;
		    now_statue=1;
	 }
	        
	 		    if(now_statue!=last_statue) 
	       {
			     change_flag1++;
           beep();	
		     }
/****************************************************************************************
				                          ��һȦ
*****************************************************************************************/
			 if(change_flag1==0) {turn_pid(10,-42);}	 
			 if(change_flag1==2){check(-137); turn_pid(10,-137);}	 
					
					  if(change_flag1==4)
			    {
			    	Set_left_pwm(0);
   		      Set_right_pwm(0);
				    change_flag1++;
			    }
}

void track4(void)
{
	  if(D1==1 && D2==1 && D3==1 && D4==0 && D5==0 && D6==1 && D7==1 && D8==1)     //11100111
	{
//		sensor_bias = 0;
		Set_right_pwm(4500); Set_left_pwm(4500); 
	}
	
	 else if(D1==1 && D2==1 && D3==0 && D4==0 && D5==1 && D6==1 && D7==1 && D8==1) //11001111
	{
//		sensor_bias = -15;
//    Set_right_pwm(5250); Set_left_pwm(3300); 
		Set_right_pwm(6300); Set_left_pwm(2100); 
	}
   else if(D1==1 && D2==0 && D3==0 && D4==1 && D5==1 && D6==1 && D7==1 && D8==1)  //10011111
	{
//		sensor_bias = -25;
		Set_right_pwm(7050); Set_left_pwm(1500); 
	}
	 else if(D1==0 && D2==0 && D3==1 && D4==1 && D5==1 && D6==1 && D7==1 && D8==1)  //00111111
	{
//		sensor_bias = -35; 
			if(yaw_angle_int>-120&&yaw_angle_int<-90)
		{
			Set_right_pwm(1050);  Set_left_pwm(8100);  
		}
		 else
		 { Set_right_pwm(8100);  Set_left_pwm(1050); } 
		             //5000               800
	}
	
	 else if(D1==0 && D2==1 && D3==1 && D4==1 && D5==1 && D6==1 && D7==1 && D8==1)  //01111111
	{
			if(yaw_angle_int>-120&&yaw_angle_int<-90)
		{
			Set_right_pwm(300);  Set_left_pwm(9300);  
		}
		 else
		 { Set_right_pwm(9300);  Set_left_pwm(300); } 
		              //5500              500
	 }
	 // danglu
	 	else if(D1==1 && D2==1 && D3==1 && D4==0 && D5==1 && D6==1 && D7==1 && D8==1) //11101111
	{
//		sensor_bias = -5;
		Set_right_pwm(5250); Set_left_pwm(3750); 
	}
   else if(D1==1 && D2==1 && D3==0 && D4==1 && D5==1 && D6==1 && D7==1 && D8==1)   //11011111
	{
//		sensor_bias = -20;
		Set_right_pwm(6750); Set_left_pwm(1500); 
	}
	 else if(D1==1 && D2==0 && D3==1 && D4==1 && D5==1 && D6==1 && D7==1 && D8==1)   //10111111
	{
     	 if(yaw_angle_int>-120&&yaw_angle_int<-90)
		{
			Set_right_pwm(300);  Set_left_pwm(9300);  
		}
		 else
		 { Set_right_pwm(9300);  Set_left_pwm(300); } 
	}
	 else if(D1==1 && D2==1 && D3==1 && D4==1 && D5==0 && D6==1 && D7==1 && D8==1)  //11110111
	{    
//		sensor_bias = 10;
		 Set_right_pwm(2250); Set_left_pwm(6750);  
	}
   else if(D1==1 && D2==1 && D3==1 && D4==1 && D5==1 && D6==0 && D7==1 && D8==1)   //11111011
	{
//		sensor_bias = 20;
		Set_right_pwm(1200); Set_left_pwm(7800);  
	}
	 else if(D1==1 && D2==1 && D3==1 && D4==1 && D5==1 && D6==1 && D7==0 && D8==1)   //11111101
	{
//		sensor_bias = 30;
			if(yaw_angle_int>-60&&yaw_angle_int<-20)
		{
			Set_right_pwm(9300);  Set_left_pwm(300);  
		}
		 else
		 { Set_right_pwm(300);  Set_left_pwm(9300);} 
	}

	 else if(D1==1 && D2==1 && D3==1 && D4==1 && D5==0 && D6==0 && D7==1 && D8==1)  //11110011
	{
//		sensor_bias = 15;
		Set_right_pwm(1800); Set_left_pwm(7200);  
	}
	 else if(D1==1 && D2==1 && D3==1 && D4==1 && D5==1 && D6==0 && D7==0 && D8==1)  //11111001
	{
//		sensor_bias = 25;
		Set_right_pwm(1500); Set_left_pwm(7500);  
	}
	 else if(D1==1 && D2==1 && D3==1 && D4==1 && D5==1 && D6==1 && D7==0 && D8==0)  //11111100
	{
		if(yaw_angle_int>-60&&yaw_angle_int<-20)
		{
			Set_right_pwm(6750);  Set_left_pwm(3000);  
		}
//		sensor_bias = 35;
		 else
		 { Set_right_pwm(900);  Set_left_pwm(8700);} 
	}
	
	 else if(D1==1 && D2==1 && D3==1 && D4==1 && D5==1 && D6==1 && D7==1 && D8==0)  //11111110
	{
//		sensor_bias = 45;
			if(yaw_angle_int>-60&&yaw_angle_int<-20)
		{
			Set_right_pwm(6750);  Set_left_pwm(3000);  
		}
		 else
		 { Set_right_pwm(300);  Set_left_pwm(9300);} 
		  
	}
	
	  else if(D1==1 && D2==1 && D3==1 && D4==1 && D5==1 && D6==1 && D7==1 && D8==1)  //11111111
   {
		  last_statue=now_statue;
		  now_statue=0;
   }
	 
	   if(D1==0|D2==0|D3==0|D4==0|D5==0|D6==0|D7==0|D8==0)
	 {
		    last_statue=now_statue;
		    now_statue=1;
	 }
	        
	 		    if(now_statue!=last_statue) 
	       {
			     change_flag1++;
           beep();	
		     }
/****************************************************************************************
				                          ��һȦ
*****************************************************************************************/
			 if(change_flag1==0) {turn_pid(10,-42);}	 
			 if(change_flag1==2) {check(-138); turn_pid(10,-138);}	 
			 
/****************************************************************************************
				                          �ڶ�Ȧ
*****************************************************************************************/			 
			     if(change_flag1==4) {turn_pid(10,-46);}
			                                  //45 
			 	   if(change_flag1==6) //****** �׶�
					{
						check(-141);     
				    turn_pid(10,-141); //137 138
				  }
					
/****************************************************************************************
				                          ����Ȧ
*****************************************************************************************/	
		 if(change_flag1==8)  { turn_pid(10,-47);  }  //-45
		 if(change_flag1==10) { check(-142); turn_pid(10,-142);}
					
					
/****************************************************************************************
				                          ����Ȧ
*****************************************************************************************/						
         
           if(change_flag1==12)
			    {
          turn_pid(10,-50);  //-45
			    }
					 if(change_flag1==14)
					{
						check(-142); turn_pid(10,-142); 
					}
					
					  if(change_flag1==16)
			    {
			    	Set_left_pwm(0);
   		      Set_right_pwm(0);
				    change_flag1++;
			    }
}

