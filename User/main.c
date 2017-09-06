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
#include "stopwatch.h"
#include "lock.h"


float stopwatchTime;                     //码表的计数

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
uint8_t speed_send = 0;

extern uint8_t SendSpeed[SENDBUFF_SIZE];
extern uint8_t SendData[SENDDATA_SIZE];
extern uint8_t mode_select;

int main(void)
{ 
	float speed;
//	float distance = 0;
	static uint8_t stand_shake_lift_flag;											//直立->震动->抬起
	static float dis_z;
	float lift_ignore;
	static uint8_t first_to_stand,first_to_fall,first_to_lift,first_to_shake;
	static uint8_t lift_stand_cnt,shake_stand_cnt,lift_fall_cnt,shake_fall_cnt;
	static uint8_t isBroken = 0;
	static uint8_t firstToMain = 0;
	
//	StepFreq_Config();																//步频检测红外初始化
//	ZigbeeGpioInit();																//zigbee 点对点模式初始化 
	Shake_GPIO_Config();															//撬锁检测IO初始化
	buzzer_init();																	//蜂鸣器IO初始化
 	vl530l0_iic_init();																//激光测距iic初始化
//	USART1_DMA_Config();															//USART1 DMA4 Init
//	USARTx_Config();																//USART1初始化
	USART2_Config();																//USART2初始化
 	I2C_Config();																	//MPU9150 I2C初始化//SDA-PC11   SCL-PC12	 
 	init_mpu();																		//DMP初始化
	TIM2_Configuration();															//TIM2初始化

//	TIM3_Int_Init(49,7199);															//TIM3初始化10Khz的计数频率，计数到50为5ms 
//	StopwatchIO_Init();																//码表io口初始化	
	LOCK_Init();																	//初始化与LOCK连接的硬件接口
	
	get_accel_bias();																//获取Accel_ZOffset
//	USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
	
//	UnLock();
	Lock();
	
	while(1)			
	{
				
		if(mode_select == ALARM_MODE)
		{

		if(mBICYCLE.bicycle_state == STATE_LIFT)
			lift_ignore = get_accel(); 												//获取当前 Pitch Roll accel_x accel_y accel_z 处于抬高状态 不再获取dis
		else
		{
			dis_z = get_accel();
			printf("dis: %f",dis_z);
		}

		if(STATIC_ANGLE_DEMAND && STATIC_ACCEL_DEMAND)								//检测直立的条件
		{	
			
			if(new_stand == 0)														//若为初次数据对定时器计数清零	
			{						
				new_stand = 1;																				
				uSec_Count = 0;														//定时器计数清零
				Sec_Count = 0;	
			}		
					
			if(Sec_Count == 1)
			{
				if(firstToMain == 0)
				{
					SendWarning(STATE_STATIC_NORMAL,0);								//直立 无危险
					firstToMain = 1;
				}
				if(mBICYCLE.bicycle_state != STATE_STAND_STATIC)
				{
					mBICYCLE.last_state = mBICYCLE.bicycle_state;
					first_to_stand = 1;												//在首次从其他状态变成静止状态时  将该标志位置一
					SendWarning(STATE_STATIC_NORMAL,0);								//直立 无危险
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
					SendWarning(STATE_FALDOWN,0);									//倒下 无危险
				}	
				mBICYCLE.bicycle_state = STATE_FALL;
				dis_z = 0;
			}
		}
		else		
			new_fall = 0;															//清除判断倒下初次数据的标志
		
		if((dis_z > STATIC_LIFT_HIGHT))												//检测是否被抬起
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
						if(first_to_stand)												//检测是否首次从抬高变成静止
						{
							first_to_stand = 0;
						}
					}
				}
				
				if(mBICYCLE.last_state == STATE_SHAKE)
				{
	
					if(s_Count_Shake <= 5)
					{
						//5s内从震动恢复到正常  最轻微级别的误报 不做记录					//包含倒下到直立的条件
//						printf("dander shake < 5s to static\r\n");
					}
					
					if((s_Count_Shake > 5) && (s_Count_Shake <= SHAKE_TIME))
					{
						if(first_to_stand)												//检测是否首次从震动变成静止
						{
							first_to_stand = 0;
						}
					}
				}
			break;
			
			case STATE_FALL:
				if(mBICYCLE.last_state == STATE_LIFT)
				{
					if(s_Count_Lift <= (LIFT_TIME + 1))
					{
						if(first_to_fall)												//检测是否首次从抬高变成倒下
						{
//							printf("dander lift < 10s to fall\r\n");
							first_to_fall = 0;
//							lift_fall_cnt++;											//少于10s恢复到倒下  记下次数  
						}
					}
				}
				if(mBICYCLE.last_state == STATE_SHAKE)
				{
					if(s_Count_Shake <= 5)
					{
						//5s内从震动恢复到倒下  最轻微级别的误报 不做记录					//直立到倒下
//						printf("dander shake < 5s to fall\r\n");
					}
					
					if((s_Count_Shake > 5) && (s_Count_Shake <= SHAKE_TIME))
					{
						if(first_to_fall)												//检测是否首次从震动变成倒下
						{
							first_to_fall = 0;
//							shake_fall_cnt++;											//在5s-10s之间恢复倒下  记下次数    
//							printf("shake_fall_cnt %d\r\n\r\n",shake_fall_cnt);
						}
					}
				}
			break;
			
			case STATE_LIFT:
				if(mBICYCLE.last_state == STATE_STAND_STATIC)
				{
					if(dis_z > 50)
					{
						//整车抬起超过50cm  直接报警	
						if(first_to_lift)
						{
							first_to_lift = 0;
							alarm = 1;
							SendWarning(STATE_LIFT_BEYOND_HEIGHT,ALARM);				//当触发被盗行为 向摄像头模块发送指令
//							printf("danger lift > 50cm \r\n");
						}
						while(alarm)
						{
							Buzzer_ON;
							delay_ms(1000);
							Buzzer_OFF;
							delay_ms(1000);
						}
					}
					
					if(((dis_z > 10) && (dis_z < 50)))// || (((distance - STATIC_HIGHT) > 10) && ((distance - STATIC_HIGHT) < 50)))
					{
						if(s_Count_Lift > LIFT_TIME)
						{
							//抬起超过8s  报警
							if(first_to_lift)
							{
								first_to_lift = 0;
								alarm = 1;
								SendWarning(STATE_LIFT_BEYOND_TIME,ALARM);				//当触发被盗行为 向摄像头模块发送指令
//								printf("dander lift > 8s\r\n");
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
						if(first_to_lift)
						{
							first_to_lift = 0;
							alarm = 1;
							SendWarning(STATE_LIFT_BEYOND_HEIGHT,ALARM);				//当触发被盗行为 向摄像头模块发送指令
//							printf("danger lift > 50cm \r\n");
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
							//抬起超过8s  报警
							if(first_to_lift)
							{
								first_to_lift = 0;
								alarm = 1;
								SendWarning(STATE_LIFT_BEYOND_TIME,ALARM);				//当触发被盗行为 向摄像头模块发送指令
//								printf("dander lift > 8s \r\n");
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
						if(dis_z > 50)
						{
							//整车抬起超过50cm  直接报警
							if(first_to_lift)
							{
								first_to_lift = 0;
								alarm = 1;
								SendWarning(STATE_LIFT_BEYOND_HEIGHT,ALARM);			//当触发被盗行为 向摄像头模块发送指令
//								printf("danger lift > 50cm\r\n");
							}
							while(alarm)
							{
								Buzzer_ON;
								delay_ms(1000);
								Buzzer_OFF;
								delay_ms(1000);
							}
						}
						
						if(((dis_z > 10) && (dis_z < 50)))// || (((distance - STATIC_HIGHT) > 10) && ((distance - STATIC_HIGHT) < 50)))
						{						
							if(s_Count_Lift > LIFT_TIME)
							{
								//抬起超过8s  报警
								if(first_to_lift)
								{
									first_to_lift = 0;
									alarm = 1;
									SendWarning(STATE_LIFT_BEYOND_TIME,ALARM);				//当触发被盗行为 向摄像头模块发送指令
//									printf("dander lift > 8s\r\n");
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
							if(first_to_lift)
							{
								first_to_lift = 0;
								alarm = 1;
//								printf("danger lift > 50cm\r\n");
								SendWarning(STATE_LIFT_BEYOND_HEIGHT,ALARM);				//当触发被盗行为 向摄像头模块发送指令
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
								//抬起超过8s  报警
								if(first_to_lift)
								{
									first_to_lift = 0;
									alarm = 1;
//									printf("dander lift > 8s\r\n");
									SendWarning(STATE_LIFT_BEYOND_TIME,ALARM);				//当触发被盗行为 向摄像头模块发送指令
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
						SendWarning(STATE_SHAKE_BEYOND_TIME,ALARM);							//当触发被盗行为 向摄像头模块发送指令
//						printf("dander shake > 10s\r\n");
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
	}
		
	
	}
}
