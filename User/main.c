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

uint16_t u_Count_Danger;
uint16_t s_Count_Danger;

extern float Pitch,Roll;
extern float accel_x,accel_y,accel_z;

int main(void)
{ 
	float distance = 0;
	static float dis_z;
	
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
			
		dis_z = get_accel();																//获取当前 Pitch Roll accel_x accel_y accel_z dis
		distance = vl530l0_get_distance() - STATIC_HIGHT;													//获取当前dis---激光
//		printf("dis %f",dis_z);
//		if(! isShake())																		//检测是否发生震动
//		{		
//			Buzzer_ON;		
//		}		
//		else		
//		{		
//			Buzzer_OFF;		
//		}		
				
		if(STATIC_ANGLE_DEMAND && STATIC_ACCEL_DEMAND)										//检测直立的条件
		{
//			if(dis_z < 0)																	//限于精度问题 一旦检测到下降动作清零dis_z  避免干扰下面判断
//			{
//				dis_z = 0;
//			}
		
			if(new_stand == 0)																//若为初次数据对定时器计数清零
			{		
				new_stand = 1;																
				uSec_Count = 0;																//定时器计数清零
				Sec_Count = 0;		
			}		
					
			if(Sec_Count == 1)																
			{		
				if(mBICYCLE.bicycle_state == STATE_DANGER)									//自行车从危险动作恢复到静止
				{		
					if(s_Count_Danger <= 9)													//10s内从危险动作恢复到静止 可以视为误触
					{		
						mBICYCLE.bicycle_state = STATE_STAND_STATIC;
						new_danger = 0;	
						printf("maybe warming   s_count%d\r\n\r\n\r\n",s_Count_Danger);
					}		
					else		
					{		
						//超过10s恢复   操作留空		
					}		
				}		
				else		
					mBICYCLE.bicycle_state = STATE_STAND_STATIC;							//条件持续一秒  视为直立静止状态
			}		
		}		
		else		
			new_stand = 0;																	//清除判断直立初次数据的标志	
					                                                                                                   	 
							
		if(FALL_ANGLE_DEMAND && FALL_ACCEL_DEMAND)											//检测是否倒下
		{			
			if(new_fall == 0)		
			{		
				new_fall = 1;		
				uSec_Count = 0;																//定时器计数清零
				Sec_Count = 0;		
			}		
			if(Sec_Count == 1)
			{
				if(mBICYCLE.bicycle_state == STATE_DANGER)									//自行车从危险动作恢复到静止
				{		
					if(s_Count_Danger <= 9)													//10s内从危险动作恢复到静止 可以视为误触
					{		
						mBICYCLE.bicycle_state = STATE_FALL;
						new_danger = 0;	
						printf("maybe warming   s_count%d\r\n\r\n\r\n",s_Count_Danger);
					}		
					else		
					{		
						//超过10s恢复   操作留空		
					}		
				}		
				else
					mBICYCLE.bicycle_state = STATE_FALL;										//条件持续一秒  视为直立倒下状态	
			}
		}		
		else		
			new_fall = 0;																	//清除判断倒下初次数据的标志	
		
		if((dis_z > STATIC_LIFT_HIGHT) || (distance > STATIC_LIFT_HIGHT))					//检测是否被抬起
		{
			if(mBICYCLE.bicycle_state == STATE_STAND_STATIC)
			{
				mBICYCLE.bicycle_state = STATE_DANGER;
			}
			
			if(mBICYCLE.bicycle_state == STATE_FALL)
			{
				if(dis_z > STATIC_LIFT_HIGHT)
					mBICYCLE.bicycle_state = STATE_DANGER;
			}
		}
		else if(MOVE_DEMAND)																//检测是否发生异常移动
		{	
			mBICYCLE.bicycle_state = STATE_DANGER;	
		}	
		else	
		{
//			printf("ihdis\r\n\r\n\r\n");
//			new_danger = 0;																	////清除判断抬高初次数据的标志
		}
			
	
		if(mBICYCLE.bicycle_state == STATE_DANGER)											//检测到自行车触发危险行为 重置定时器
		{	
			if(new_danger == 0)	
			{	
				new_danger = 1;	
				u_Count_Danger = 0;															//定时器计数清零
				s_Count_Danger = 0;
			}
			
			if(s_Count_Danger >= 10)
			{
				//车在10s内持续发生异常动作 可以鉴定为被盗
				printf(" FBI wriming   s_count %d\r\n\r\n\r\n",s_Count_Danger);
			}
		}
		printf("state %d s_count %d\r\n",mBICYCLE.bicycle_state,s_Count_Danger);
//		printf("Pitch: %f  Roll %f  accel_x %f  accel_y %f accel_z %f\r\n",Pitch,Roll,accel_x,accel_y,accel_z);
//		printf("dis %f distance %f state %d s_count %d\r\n",dis_z,distance,mBICYCLE.bicycle_state,s_Count_Danger);
	}
}
