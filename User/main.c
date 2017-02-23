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
uint8_t s_Count_Lift;

uint16_t u_Count_Shake;
uint8_t s_Count_Shake;

extern float Pitch,Roll;
extern float accel_x,accel_y,accel_z;

extern uint8_t SendBuff[SENDBUFF_SIZE];

uint8_t new_stand = 0;
uint8_t new_fall = 0;
uint8_t new_lift = 0;
uint8_t new_shake = 0;

uint8_t alarm = 0;

int main(void)
{ 
	float distance = 0;
//	static uint8_t lift_shake_flag;													//抬起->放下->震动
	static uint8_t stand_shake_lift_flag;											//直立->震动->抬起
	static float dis_z;
	float lift_ignore;
	static uint8_t first_to_stand,first_to_fall,first_to_lift,first_to_shake;
	static uint8_t lift_stand_cnt,shake_stand_cnt,lift_fall_cnt,shake_fall_cnt;
	
	Shake_GPIO_Config();															//震动检测IO检测
	buzzer_init();																	//蜂鸣器IO初始化
	vl530l0_iic_init();																//激光测距iic初始化
	USART1_DMA_Config();															//USART1 DMA4 Init
	USARTx_Config();																//USART1初始化
	USART2_Config();																//USART2初始化
	I2C_Config();																	//MPU9150 I2C初始化//SDA-PC11   SCL-PC12	 
	init_mpu();																		//DMP初始化
	TIM2_Configuration();															//TIM2初始化
	get_accel_bias();																//获取Accel_ZOffset
	USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
	
	while(1)			
	{
			
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
				if((distance - STATIC_HIGHT) > 10)
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
				{
					mBICYCLE.last_state = mBICYCLE.bicycle_state;
					first_to_stand = 1;												//在首次从其他状态变成静止状态时  将该标志位置一
				}
					
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
				{
					mBICYCLE.last_state = mBICYCLE.bicycle_state;
					first_to_fall = 1;	
				}	
				mBICYCLE.bicycle_state = STATE_FALL;
				dis_z = 0;
			}
		}
		else		
			new_fall = 0;															//清除判断倒下初次数据的标志
		
//		if((dis_z > STATIC_LIFT_HIGHT) || ((distance - STATIC_HIGHT) > STATIC_LIFT_HIGHT))			//检测是否被抬起
//		{
//			if((mBICYCLE.bicycle_state == STATE_STAND_STATIC) || (mBICYCLE.bicycle_state == STATE_SHAKE))
//			{	
//				mBICYCLE.last_state = mBICYCLE.bicycle_state;
//				mBICYCLE.bicycle_state = STATE_LIFT;
//			}
//			
//			if(mBICYCLE.bicycle_state == STATE_FALL)
//			{
//				if(dis_z > STATIC_LIFT_HIGHT)
//				{
//					mBICYCLE.last_state = mBICYCLE.bicycle_state;
//					mBICYCLE.bicycle_state = STATE_LIFT;
//				}
//			}
//		}
		if((dis_z > STATIC_LIFT_HIGHT))			//检测是否被抬起
		{
			if(mBICYCLE.bicycle_state != STATE_LIFT)
			{
					mBICYCLE.last_state = mBICYCLE.bicycle_state;
					mBICYCLE.bicycle_state = STATE_LIFT;
					first_to_lift = 1;
			}
		}
		if((mBICYCLE.bicycle_state == STATE_STAND_STATIC) || (mBICYCLE.bicycle_state == STATE_FALL))
			new_lift = 0;
		
		if(SHAKE_DEMAND && (dis_z < 10))											//检测是否发生震动
		{
			if(mBICYCLE.bicycle_state != STATE_SHAKE)
			{
				mBICYCLE.last_state = mBICYCLE.bicycle_state;
				first_to_shake = 1;
			}
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
						if(first_to_stand)														//检测是否首次从抬高变成静止
						{
							printf("dander lift < 8s to static\r\n");
							first_to_stand = 0;
							lift_stand_cnt++;													//少于10s恢复到正常  记下次数 
						}
//						printf("dander lift < 10s to static\r\n");
					}
				}
				
				if(mBICYCLE.last_state == STATE_SHAKE)
				{
//					if(lift_shake_flag == 0)														//此处是  从震动到正常的过程
					{
						if(s_Count_Shake <= 5)
						{
							//5s内从震动恢复到正常  最轻微级别的误报 不做记录			//包含倒下到直立的条件
//							printf("dander shake < 5s to static\r\n");
						}
						
						if((s_Count_Shake > 5) && (s_Count_Shake <= SHAKE_TIME))
						{
							if(first_to_stand)														//检测是否首次从震动变成静止
							{
								first_to_stand = 0;
								shake_stand_cnt++;													//在5s-20s之间恢复到正常  记下次数   
								printf("dander shake 5s - 10s to static\r\n");
								printf("shake_stand_cnt %d\r\n\r\n",shake_stand_cnt);
							}
//							printf("dander shake 5s - 20s to static\r\n");
						}
					}
					
//					if(lift_shake_flag == 1)														//抬高到正常  中间产生震动
//					{
//						if((s_Count_Lift + s_Count_Shake) <= (LIFT_TIME + 1))								//抬高->震动->静止 在7s内
//						{
//							//在抬高震动10s恢复到正常  记下次数  
//							printf("dander lift&shake 10s to static\r\n");
//						}
//					}

				}
			break;
			
			case STATE_FALL:
				if(mBICYCLE.last_state == STATE_LIFT)
				{
					if(s_Count_Lift <= (LIFT_TIME + 1))
					{
						if(first_to_fall)														//检测是否首次从抬高变成倒下
						{
							printf("dander lift < 10s to fall\r\n");
							first_to_fall = 0;
							lift_fall_cnt++;													//少于10s恢复到倒下  记下次数  
						}
						
//						printf("dander lift < 10s to fall\r\n");
					}
				}
				if(mBICYCLE.last_state == STATE_SHAKE)
				{
//					if(lift_shake_flag == 0)
					{
						if(s_Count_Shake <= 5)
						{
							//5s内从震动恢复到倒下  最轻微级别的误报 不做记录						//直立到倒下
//							printf("dander shake < 5s to fall\r\n");
						}
						
						if((s_Count_Shake > 5) && (s_Count_Shake <= SHAKE_TIME))
						{
							if(first_to_fall)													//检测是否首次从震动变成倒下
							{
								first_to_fall = 0;
								shake_fall_cnt++;												//在5s-10s之间恢复倒下  记下次数    
								printf("shake_fall_cnt %d\r\n\r\n",shake_fall_cnt);
							}
//							printf("dander shake 5s - 20s to fall\r\n");
						}
					}
					
//					if(lift_shake_flag == 1)
//					{
//						if((s_Count_Lift + s_Count_Shake) <= (LIFT_TIME + 1))					//抬高->震动->倒下 在7s内
//						{
//							//在抬高震动10s恢复到倒下  记下次数  
//							printf("dander lift&shake 10s to fall\r\n");
//						}
//					}
				}
			break;
			
			case STATE_LIFT:
				if(mBICYCLE.last_state == STATE_STAND_STATIC)
				{
					if(dis_z > 50 || (distance - STATIC_HIGHT) > 50)
					{
//						printf("danger lift > 50cm\r\n");
						//整车抬起超过50cm  直接报警	
						if(first_to_lift)
						{
							first_to_lift = 0;
							alarm = 1;
							printf("danger lift > 50cm 1\r\n");
						}
						while(alarm)
						{
							Buzzer_ON;
							delay_ms(1000);
							Buzzer_OFF;
							delay_ms(1000);
						}
					}
					
					if(((dis_z > 10) && (dis_z < 50)) || (((distance - STATIC_HIGHT) > 10) && ((distance - STATIC_HIGHT) < 50)))
					{
						if(s_Count_Lift > LIFT_TIME)
						{
							//抬起超过10s  报警
//							printf("dander lift > 10s \r\n");
							if(first_to_lift)
							{
								first_to_lift = 0;
								alarm = 1;
								printf("dander lift > 10s 1\r\n");
							}
							while(alarm)
							{
								Buzzer_ON;
								delay_ms(1000);
								Buzzer_OFF;
								delay_ms(1000);
							}
						}
					}
				}
				
				if(mBICYCLE.last_state == STATE_FALL)
				{
					if(dis_z > 50)
					{
						//整车抬起超过50cm  直接报警
//						printf("danger lift > 50cm\r\n");
						if(first_to_lift)
						{
							first_to_lift = 0;
							alarm = 1;
							printf("danger lift > 50cm 2\r\n");
						}
						while(alarm)
						{
							Buzzer_ON;
							delay_ms(1000);
							Buzzer_OFF;
							delay_ms(1000);
						}
					}
					
					if((dis_z > 15) && (dis_z < 50))
					{
						if(s_Count_Lift > LIFT_TIME)
						{
							//抬起超过10s  报警
//							printf("dander lift > 10s \r\n");
							if(first_to_lift)
							{
								first_to_lift = 0;
								alarm = 1;
								printf("dander lift > 10s 2\r\n");
							}
							while(alarm)
							{
								Buzzer_ON;
								delay_ms(1000);
								Buzzer_OFF;
								delay_ms(1000);
							}
						}
					}
				}
				
				if(mBICYCLE.last_state == STATE_SHAKE)
				{
					if(stand_shake_lift_flag == STATE_STAND_STATIC)
					{
						if(dis_z > 50 || (distance - STATIC_HIGHT) > 50)
						{
							//整车抬起超过50cm  直接报警
//							printf("danger lift > 50cm\r\n");
							if(first_to_lift)
							{
								first_to_lift = 0;
								alarm = 1;
								printf("danger lift > 50cm 3\r\n");
							}
							while(alarm)
							{
								Buzzer_ON;
								delay_ms(1000);
								Buzzer_OFF;
								delay_ms(1000);
							}
						}
						
						if(((dis_z > 10) && (dis_z < 50)) || (((distance - STATIC_HIGHT) > 10) && ((distance - STATIC_HIGHT) < 50)))
						{
//							if(s_Count_Shake > 3)												//如果抬高前的震动超过3s  则认为该震动不是由抬高引起的 应该计算进危险时间		
//								s_Count_Lift += s_Count_Shake;
							
							if(s_Count_Lift > LIFT_TIME)
							{
								//抬起超过10s  报警
//								printf("dander lift > 10s \r\n");
								if(first_to_lift)
								{
									first_to_lift = 0;
									alarm = 1;
									printf("dander lift > 10s 3\r\n");
								}
								while(alarm)
								{
									Buzzer_ON;
									delay_ms(1000);
									Buzzer_OFF;
									delay_ms(1000);
								}
							}
						}
					}
					
					if(stand_shake_lift_flag == STATE_FALL)
					{
						if(dis_z > 50)
						{
							//整车抬起超过50cm  直接报警
//							printf("danger lift > 50cm\r\n");
							if(first_to_lift)
							{
								first_to_lift = 0;
								alarm = 1;
							}
							while(alarm)
							{
								Buzzer_ON;
								delay_ms(1000);
								Buzzer_OFF;
								delay_ms(1000);
							}
						}
						
						if((dis_z > 15) && (dis_z < 50))
						{
							if(s_Count_Lift > LIFT_TIME)
							{
								//抬起超过10s  报警
//								printf("dander lift > 10s \r\n");
								if(first_to_lift)
								{
									first_to_lift = 0;
									alarm = 1;
								}
								while(alarm)
								{
									Buzzer_ON;
									delay_ms(1000);
									Buzzer_OFF;
									delay_ms(1000);
								}
							}
						}
					}
				}
			break;
			
			case STATE_SHAKE:
//				if(mBICYCLE.last_state == STATE_LIFT)							//对应的情况是举高->放下->震动				
//				{
//					lift_shake_flag = 1;
//					if((s_Count_Shake + s_Count_Lift) > LIFT_TIME)
//					{
//						//抬高的时间大于10s  报警
//						printf("dander lift > 10s\r\n");
//					}
//				}
//				else
//					lift_shake_flag = 0;
				
				if(mBICYCLE.last_state == STATE_STAND_STATIC)
					stand_shake_lift_flag = STATE_STAND_STATIC;
				if(mBICYCLE.last_state == STATE_FALL)
					stand_shake_lift_flag = STATE_FALL;
				
				if(s_Count_Shake > SHAKE_TIME)
				{
					//持续时间大于10s  可以视为危险行为
					if(first_to_shake)
					{
						first_to_shake = 0;
						alarm = 1;
						printf("dander shake > 10s\r\n");
					}
					while(alarm)
					{
						Buzzer_ON;
						delay_ms(1000);
						Buzzer_OFF;
						delay_ms(1000);
					}
				}
				
			break;
			
			default:
			
			break;
		} 

		SendBuff[3] = s_Count_Lift/10 + '0';
		SendBuff[4] = s_Count_Lift%10 + '0';
		SendBuff[9] = s_Count_Shake/10 + '0';
		SendBuff[10] = s_Count_Shake%10 + '0';
		SendBuff[14] = (uint16_t)dis_z/10 + '0';
		SendBuff[15] = (uint16_t)dis_z%10 + '0';
		SendBuff[19] = mBICYCLE.bicycle_state + '0';
		reInitDMA4();
		
//		printf("dis_z %f \r\n",dis_z);   
//	printf("pitch %f roll %f dis_z %f dis %f\r\n",Pitch,Roll,dis_z,distance);        
//	printf("accel_x %f accel_y %f accel_z %f dis  %f\r\n",accel_x,accel_y,accel_z,dis_z);
//	printf("l_t %d s_t %d d_z %f  %f state %d first_to_lift %d\r\n",s_Count_Lift,s_Count_Shake,dis_z,distance,mBICYCLE.bicycle_state,first_to_lift);
//    printf("accel_z = %f dis_z = %f\r\n",accel_z,dis_z);
	}
}
