/******************************************************************************
** 工程：   	STM32-MPU9150驱动
** 作者：	    vino
** 修改日志：
2016-9-20
计算z分量

2017-2-12
整合防盗检测

********************************************************************************/
#include "vl53l0x.h"
#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "I2C.h"
#include "mpu9250.h"
#include "time.h"
#include "bsp_led.h"
#include "bicycle.h"

BICYCLE mBICYCLE;

uint8_t  Sec_Count;
uint16_t uSec_Count;

uint16_t u_Count_Lift;
uint16_t s_Count_Lift;

uint16_t u_Count_Shake;
uint16_t s_Count_Shake;

extern float Pitch,Roll;
extern float accel_x,accel_y,accel_z;

uint8_t new_stand = 0;
uint8_t new_fall = 0;
uint8_t new_lift = 0;
uint8_t new_shake = 0;

uint8_t alarm;

int main(void)
{ 
	float distance = 0;
	static uint8_t lift_shake_flag;													//抬起->放下->震动
	static uint8_t stand_shake_lift_flag;											//直立->震动->抬起
	static float dis_z;
	float lift_ignore;
	static uint8_t lift_stand_cnt;
	
	SystemInit();
	Shake_GPIO_Config();															//震动检测IO检测
	buzzer_init();																	//蜂鸣器IO初始化
	vl530l0_iic_init();																//激光测距iic初始化
	USARTx_Config();																//USART1初始化
	USART2_Config();																//USART2初始化
	I2C_Config();																	//MPU9150 I2C初始化//SDA-PC11   SCL-PC12	 
	init_mpu();																		//DMP初始化
	TIM2_Configuration();															//TIM2初始化
	get_accel_bias();																//获取Accel_ZOffset
	
	while(1)			
	{
		static uint8_t new_stand,new_fall,new_danger;
			
		if(mBICYCLE.bicycle_state == STATE_LIFT)
			lift_ignore = get_accel(); 														//获取当前 Pitch Roll accel_x accel_y accel_z 处于抬高状态 不再获取dis
		else
		{
			dis_z = get_accel();
		}
			
		distance = vl530l0_get_distance();							//获取当前dis---激光
	
		
		if(STATIC_ANGLE_DEMAND && STATIC_ACCEL_DEMAND)								//检测直立的条件
		{	

			if(mBICYCLE.bicycle_state == STATE_LIFT)
			{
				if(distance > 47)
				{
					uSec_Count = 0;														//定时器计数清零
					Sec_Count = 0;	
				}
			}
			
			if(new_stand == 0)														//若为初次数据对定时器计数清零	
			{						
				new_stand = 1;																				
				uSec_Count = 0;														//定时器计数清零
				Sec_Count = 0;	
			}		
					
			if(Sec_Count == 1)
			{
				if(mBICYCLE.bicycle_state != STATE_STAND_STATIC)
					mBICYCLE.last_state = mBICYCLE.bicycle_state;
				mBICYCLE.bicycle_state = STATE_STAND_STATIC;
				dis_z = 0;
			}
		}
		else
			new_stand = 0;															//清除判断直立初次数据的标志
		
		if(FALL_ANGLE_DEMAND && FALL_ACCEL_DEMAND)									//检测是否倒下
		{
			if(new_fall == 0)		
			{		
				new_fall = 1;		
				uSec_Count = 0;														//定时器计数清零
				Sec_Count = 0;		
			}		
			if(Sec_Count == 1)
			{
				if(mBICYCLE.bicycle_state != STATE_FALL)
					mBICYCLE.last_state = mBICYCLE.bicycle_state;
				mBICYCLE.bicycle_state = STATE_FALL;
				dis_z = 0;
			}
		}
		else		
			new_fall = 0;															//清除判断倒下初次数据的标志
		
		if((dis_z > STATIC_LIFT_HIGHT) || ((distance - STATIC_HIGHT) > STATIC_LIFT_HIGHT))			//检测是否被抬起
		{
			if((mBICYCLE.bicycle_state == STATE_STAND_STATIC) || (mBICYCLE.bicycle_state == STATE_SHAKE))
			{	
				mBICYCLE.last_state = mBICYCLE.bicycle_state;
				mBICYCLE.bicycle_state = STATE_LIFT;
			}
			
			if(mBICYCLE.bicycle_state == STATE_FALL)
			{
				if(dis_z > STATIC_LIFT_HIGHT)
				{
					mBICYCLE.last_state = mBICYCLE.bicycle_state;
					mBICYCLE.bicycle_state = STATE_LIFT;
				}
			}
		}
		if((mBICYCLE.bicycle_state == STATE_STAND_STATIC) || (mBICYCLE.bicycle_state == STATE_FALL))
			new_lift = 0;
		
		if(SHAKE_DEMAND && (dis_z < 10))											//检测是否发生震动
		{
			if(mBICYCLE.bicycle_state != STATE_SHAKE)
				mBICYCLE.last_state = mBICYCLE.bicycle_state;
			mBICYCLE.bicycle_state = STATE_SHAKE;
		}
		if((mBICYCLE.bicycle_state == STATE_STAND_STATIC) || (mBICYCLE.bicycle_state == STATE_FALL))
			new_shake = 0;
		
		switch(mBICYCLE.bicycle_state)
		{
			case STATE_STAND_STATIC:
				if(mBICYCLE.last_state == STATE_LIFT)
				{
					if(s_Count_Lift <= (LIFT_TIME + 1))
					{
						lift_stand_cnt++;
						//少于10s恢复到正常  记下次数  操作留空
						printf("dander lift < 10s to static\r\n");
					}
				}
				
				if(mBICYCLE.last_state == STATE_SHAKE)
				{
					if(lift_shake_flag == 0)														//此处是  从震动到正常的过程
					{
						if(s_Count_Shake <= 5)
						{
							//5s内从震动恢复到正常  最轻微级别的误报 不做记录			//包含倒下到直立的条件
							printf("dander shake < 5s to static\r\n");
						}
						
						if((s_Count_Shake > 5) && (s_Count_Shake <= 20))
						{
							//在5s-20s之间恢复到正常  记下次数  
							printf("dander shake 5s - 20s to static\r\n");
						}
					}
					
					if(lift_shake_flag == 1)														//抬高到正常  中间产生震动
					{
						if((s_Count_Lift + s_Count_Shake) <= (LIFT_TIME + 1))								//抬高->震动->静止 在7s内
						{
							//在抬高震动10s恢复到正常  记下次数  
							printf("dander lift&shake 10s to static\r\n");
						}
					}

				}
			break;
			
			case STATE_FALL:
				if(mBICYCLE.last_state == STATE_LIFT)
				{
					if(s_Count_Lift <= (LIFT_TIME + 1))
					{
						//少于10s恢复到倒下  记下次数  操作留空					//包含直立到倒下的条件
						printf("dander lift < 10s to fall\r\n");
					}
				}
				if(mBICYCLE.last_state == STATE_SHAKE)
				{
					if(lift_shake_flag == 0)
					{
						if(s_Count_Shake <= 5)
						{
							//5s内从震动恢复到倒下  最轻微级别的误报 不做记录
							printf("dander shake < 5s to fall\r\n");
						}
						
						if((s_Count_Shake > 5) && (s_Count_Shake <= 20))
						{
							//在5s-20s之间恢复倒下  记下次数  
							printf("dander shake 5s - 20s to fall\r\n");
						}
					}
					
					if(lift_shake_flag == 1)
					{
						if((s_Count_Lift + s_Count_Shake) <= (LIFT_TIME + 1))								//抬高->震动->倒下 在7s内
						{
							//在抬高震动10s恢复到倒下  记下次数  
							printf("dander lift&shake 10s to fall\r\n");
						}
					}
				}
			break;
			
			case STATE_LIFT:
				if(mBICYCLE.last_state == STATE_STAND_STATIC)
				{
					if(dis_z > 50 || (distance - STATIC_HIGHT) > 50)
					{
						//整车抬起超过50cm  直接报警
						printf("danger lift > 50cm\r\n");
					}
					
					if(((dis_z > 10) && (dis_z < 50)) || (((distance - STATIC_HIGHT) > 10) && (distance < 50)))
					{
						if(s_Count_Lift > LIFT_TIME)
						{
							//抬起超过10s  报警
							printf("dander lift > 10s \r\n");
						}
					}
				}
				
				if(mBICYCLE.last_state == STATE_FALL)
				{
					if(dis_z > 50)
					{
						//整车抬起超过50cm  直接报警
						printf("danger lift > 50cm\r\n");
					}
					
					if((dis_z > 10) && (dis_z < 50))
					{
						if(s_Count_Lift > LIFT_TIME)
						{
							//抬起超过10s  报警
							printf("dander lift > 10s \r\n");
						}
					}
				}
				
				if(mBICYCLE.last_state == STATE_SHAKE)
				{
					if(stand_shake_lift_flag == STATE_STAND_STATIC)
					{
						if(dis_z > 50 || distance > 50)
						{
							//整车抬起超过50cm  直接报警
							printf("danger lift > 50cm\r\n");
						}
						
						if(((dis_z > 10) && (dis_z < 50)) || ((distance > 10) && (distance < 50)))
						{
							if(s_Count_Lift > LIFT_TIME)
							{
								//抬起超过10s  报警
								printf("dander lift > 10s \r\n");
							}
						}
					}
					
					if(stand_shake_lift_flag == STATE_FALL)
					{
						if(dis_z > 50)
						{
							//整车抬起超过50cm  直接报警
							printf("danger lift > 50cm\r\n");
						}
						
						if((dis_z > 10) && (dis_z < 50))
						{
							if(s_Count_Lift > LIFT_TIME)
							{
								//抬起超过10s  报警
								printf("dander lift > 10s \r\n");
							}
						}
					}
					
				}
			break;
			
			case STATE_SHAKE:
				if(mBICYCLE.last_state == STATE_LIFT)							//对应的情况是举高->放下->震动				
				{
					lift_shake_flag = 1;
					if((s_Count_Shake + s_Count_Lift) > LIFT_TIME)
					{
						//抬高的时间大于10s  报警
						printf("dander lift > 10s\r\n");
					}
				}
				else
					lift_shake_flag = 0;
				
				if(mBICYCLE.last_state == STATE_STAND_STATIC)
					stand_shake_lift_flag = STATE_STAND_STATIC;
				if(mBICYCLE.last_state == STATE_FALL)
					stand_shake_lift_flag = STATE_FALL;
				
				if(s_Count_Shake > 20)
				{
					//持续时间大于20s  可以视为危险行为
					printf("dander shake > 20s\r\n");
				}
				
				
			break;
			
			default:
			
			break;
		} 

//		printf("dis_z %f dis %f\r\n",dis_z,distance);   
//	printf("pitch %f roll %f dis_z %f dis %f\r\n",Pitch,Roll,dis_z,distance);        
//	printf("accel_x %f accel_y %f accel_z %f\r\n",accel_x,accel_y,accel_z);
	printf("lift_t %d shake_t %d dis_z %f dis %f last state %d  now state %d\r\n",s_Count_Lift,s_Count_Shake,dis_z,lift_ignore,mBICYCLE.last_state,mBICYCLE.bicycle_state);
//    printf("accel_z = %f dis_z = %f\r\n",accel_z,dis_z);
	}
}
