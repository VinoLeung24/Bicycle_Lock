/******************************************************************************
** ���̣�   	STM32-MPU9150����
** ���ߣ�	    vino
** �޸���־��
2016-9-20
����z����

2017-2-12
���Ϸ������

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


float stopwatchTime;                     //���ļ���

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
	static uint8_t stand_shake_lift_flag;											//ֱ��->��->̧��
	static float dis_z;
	float lift_ignore;
	static uint8_t first_to_stand,first_to_fall,first_to_lift,first_to_shake;
	static uint8_t lift_stand_cnt,shake_stand_cnt,lift_fall_cnt,shake_fall_cnt;
	static uint8_t isBroken = 0;
	static uint8_t firstToMain = 0;
	
//	StepFreq_Config();																//��Ƶ�������ʼ��
//	ZigbeeGpioInit();																//zigbee ��Ե�ģʽ��ʼ�� 
	Shake_GPIO_Config();															//�������IO��ʼ��
	buzzer_init();																	//������IO��ʼ��
 	vl530l0_iic_init();																//������iic��ʼ��
//	USART1_DMA_Config();															//USART1 DMA4 Init
//	USARTx_Config();																//USART1��ʼ��
	USART2_Config();																//USART2��ʼ��
 	I2C_Config();																	//MPU9150 I2C��ʼ��//SDA-PC11   SCL-PC12	 
 	init_mpu();																		//DMP��ʼ��
	TIM2_Configuration();															//TIM2��ʼ��

//	TIM3_Int_Init(49,7199);															//TIM3��ʼ��10Khz�ļ���Ƶ�ʣ�������50Ϊ5ms 
//	StopwatchIO_Init();																//���io�ڳ�ʼ��	
	LOCK_Init();																	//��ʼ����LOCK���ӵ�Ӳ���ӿ�
	
	get_accel_bias();																//��ȡAccel_ZOffset
//	USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
	
//	UnLock();
	Lock();
	
	while(1)			
	{
				
		if(mode_select == ALARM_MODE)
		{

		if(mBICYCLE.bicycle_state == STATE_LIFT)
			lift_ignore = get_accel(); 												//��ȡ��ǰ Pitch Roll accel_x accel_y accel_z ����̧��״̬ ���ٻ�ȡdis
		else
		{
			dis_z = get_accel();
			printf("dis: %f",dis_z);
		}

		if(STATIC_ANGLE_DEMAND && STATIC_ACCEL_DEMAND)								//���ֱ��������
		{	
			
			if(new_stand == 0)														//��Ϊ�������ݶԶ�ʱ����������	
			{						
				new_stand = 1;																				
				uSec_Count = 0;														//��ʱ����������
				Sec_Count = 0;	
			}		
					
			if(Sec_Count == 1)
			{
				if(firstToMain == 0)
				{
					SendWarning(STATE_STATIC_NORMAL,0);								//ֱ�� ��Σ��
					firstToMain = 1;
				}
				if(mBICYCLE.bicycle_state != STATE_STAND_STATIC)
				{
					mBICYCLE.last_state = mBICYCLE.bicycle_state;
					first_to_stand = 1;												//���״δ�����״̬��ɾ�ֹ״̬ʱ  ���ñ�־λ��һ
					SendWarning(STATE_STATIC_NORMAL,0);								//ֱ�� ��Σ��
				}
					
				mBICYCLE.bicycle_state = STATE_STAND_STATIC;
				dis_z = 0;
			}
		}
		else
			new_stand = 0;															//����ж�ֱ���������ݵı�־
		
		if(FALL_ANGLE_DEMAND && FALL_ACCEL_DEMAND)									//����Ƿ���
		{
			if(new_fall == 0)		
			{		
				new_fall = 1;		
				uSec_Count = 0;														//��ʱ����������
				Sec_Count = 0;		
			}		
			if(Sec_Count == 1)
			{
				if(mBICYCLE.bicycle_state != STATE_FALL)
				{
					mBICYCLE.last_state = mBICYCLE.bicycle_state;
					first_to_fall = 1;	
					SendWarning(STATE_FALDOWN,0);									//���� ��Σ��
				}	
				mBICYCLE.bicycle_state = STATE_FALL;
				dis_z = 0;
			}
		}
		else		
			new_fall = 0;															//����жϵ��³������ݵı�־
		
		if((dis_z > STATIC_LIFT_HIGHT))												//����Ƿ�̧��
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
		
		if(SHAKE_DEMAND && (dis_z < 10))											//����Ƿ�����
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
						if(first_to_stand)												//����Ƿ��״δ�̧�߱�ɾ�ֹ
						{
							first_to_stand = 0;
						}
					}
				}
				
				if(mBICYCLE.last_state == STATE_SHAKE)
				{
	
					if(s_Count_Shake <= 5)
					{
						//5s�ڴ��𶯻ָ�������  ����΢������� ������¼					//�������µ�ֱ��������
//						printf("dander shake < 5s to static\r\n");
					}
					
					if((s_Count_Shake > 5) && (s_Count_Shake <= SHAKE_TIME))
					{
						if(first_to_stand)												//����Ƿ��״δ��𶯱�ɾ�ֹ
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
						if(first_to_fall)												//����Ƿ��״δ�̧�߱�ɵ���
						{
//							printf("dander lift < 10s to fall\r\n");
							first_to_fall = 0;
//							lift_fall_cnt++;											//����10s�ָ�������  ���´���  
						}
					}
				}
				if(mBICYCLE.last_state == STATE_SHAKE)
				{
					if(s_Count_Shake <= 5)
					{
						//5s�ڴ��𶯻ָ�������  ����΢������� ������¼					//ֱ��������
//						printf("dander shake < 5s to fall\r\n");
					}
					
					if((s_Count_Shake > 5) && (s_Count_Shake <= SHAKE_TIME))
					{
						if(first_to_fall)												//����Ƿ��״δ��𶯱�ɵ���
						{
							first_to_fall = 0;
//							shake_fall_cnt++;											//��5s-10s֮��ָ�����  ���´���    
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
						//����̧�𳬹�50cm  ֱ�ӱ���	
						if(first_to_lift)
						{
							first_to_lift = 0;
							alarm = 1;
							SendWarning(STATE_LIFT_BEYOND_HEIGHT,ALARM);				//������������Ϊ ������ͷģ�鷢��ָ��
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
							//̧�𳬹�8s  ����
							if(first_to_lift)
							{
								first_to_lift = 0;
								alarm = 1;
								SendWarning(STATE_LIFT_BEYOND_TIME,ALARM);				//������������Ϊ ������ͷģ�鷢��ָ��
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
						//����̧�𳬹�50cm  ֱ�ӱ���
						if(first_to_lift)
						{
							first_to_lift = 0;
							alarm = 1;
							SendWarning(STATE_LIFT_BEYOND_HEIGHT,ALARM);				//������������Ϊ ������ͷģ�鷢��ָ��
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
							//̧�𳬹�8s  ����
							if(first_to_lift)
							{
								first_to_lift = 0;
								alarm = 1;
								SendWarning(STATE_LIFT_BEYOND_TIME,ALARM);				//������������Ϊ ������ͷģ�鷢��ָ��
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
							//����̧�𳬹�50cm  ֱ�ӱ���
							if(first_to_lift)
							{
								first_to_lift = 0;
								alarm = 1;
								SendWarning(STATE_LIFT_BEYOND_HEIGHT,ALARM);			//������������Ϊ ������ͷģ�鷢��ָ��
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
								//̧�𳬹�8s  ����
								if(first_to_lift)
								{
									first_to_lift = 0;
									alarm = 1;
									SendWarning(STATE_LIFT_BEYOND_TIME,ALARM);				//������������Ϊ ������ͷģ�鷢��ָ��
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
							//����̧�𳬹�50cm  ֱ�ӱ���
							if(first_to_lift)
							{
								first_to_lift = 0;
								alarm = 1;
//								printf("danger lift > 50cm\r\n");
								SendWarning(STATE_LIFT_BEYOND_HEIGHT,ALARM);				//������������Ϊ ������ͷģ�鷢��ָ��
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
								//̧�𳬹�8s  ����
								if(first_to_lift)
								{
									first_to_lift = 0;
									alarm = 1;
//									printf("dander lift > 8s\r\n");
									SendWarning(STATE_LIFT_BEYOND_TIME,ALARM);				//������������Ϊ ������ͷģ�鷢��ָ��
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
					//����ʱ�����10s  ������ΪΣ����Ϊ
					if(first_to_shake)
					{
						first_to_shake = 0;
						alarm = 1;
						SendWarning(STATE_SHAKE_BEYOND_TIME,ALARM);							//������������Ϊ ������ͷģ�鷢��ָ��
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
