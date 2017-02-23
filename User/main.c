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
//	static uint8_t lift_shake_flag;													//̧��->����->��
	static uint8_t stand_shake_lift_flag;											//ֱ��->��->̧��
	static float dis_z;
	float lift_ignore;
	static uint8_t first_to_stand,first_to_fall,first_to_lift,first_to_shake;
	static uint8_t lift_stand_cnt,shake_stand_cnt,lift_fall_cnt,shake_fall_cnt;
	
	Shake_GPIO_Config();															//�𶯼��IO���
	buzzer_init();																	//������IO��ʼ��
	vl530l0_iic_init();																//������iic��ʼ��
	USART1_DMA_Config();															//USART1 DMA4 Init
	USARTx_Config();																//USART1��ʼ��
	USART2_Config();																//USART2��ʼ��
	I2C_Config();																	//MPU9150 I2C��ʼ��//SDA-PC11   SCL-PC12	 
	init_mpu();																		//DMP��ʼ��
	TIM2_Configuration();															//TIM2��ʼ��
	get_accel_bias();																//��ȡAccel_ZOffset
	USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
	
	while(1)			
	{
			
		if(mBICYCLE.bicycle_state == STATE_LIFT)
			lift_ignore = get_accel(); 														//��ȡ��ǰ Pitch Roll accel_x accel_y accel_z ����̧��״̬ ���ٻ�ȡdis
		else
		{
			dis_z = get_accel();
		}
			
		distance = vl530l0_get_distance();							//��ȡ��ǰdis---����
	
		
		if(STATIC_ANGLE_DEMAND && STATIC_ACCEL_DEMAND)								//���ֱ��������
		{	

			if(mBICYCLE.bicycle_state == STATE_LIFT)
			{
				if((distance - STATIC_HIGHT) > 10)
				{
					uSec_Count = 0;														//��ʱ����������
					Sec_Count = 0;	
				}
			}
			
			if(new_stand == 0)														//��Ϊ�������ݶԶ�ʱ����������	
			{						
				new_stand = 1;																				
				uSec_Count = 0;														//��ʱ����������
				Sec_Count = 0;	
			}		
					
			if(Sec_Count == 1)
			{
				if(mBICYCLE.bicycle_state != STATE_STAND_STATIC)
				{
					mBICYCLE.last_state = mBICYCLE.bicycle_state;
					first_to_stand = 1;												//���״δ�����״̬��ɾ�ֹ״̬ʱ  ���ñ�־λ��һ
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
				}	
				mBICYCLE.bicycle_state = STATE_FALL;
				dis_z = 0;
			}
		}
		else		
			new_fall = 0;															//����жϵ��³������ݵı�־
		
//		if((dis_z > STATIC_LIFT_HIGHT) || ((distance - STATIC_HIGHT) > STATIC_LIFT_HIGHT))			//����Ƿ�̧��
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
		if((dis_z > STATIC_LIFT_HIGHT))			//����Ƿ�̧��
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
						if(first_to_stand)														//����Ƿ��״δ�̧�߱�ɾ�ֹ
						{
							printf("dander lift < 8s to static\r\n");
							first_to_stand = 0;
							lift_stand_cnt++;													//����10s�ָ�������  ���´��� 
						}
//						printf("dander lift < 10s to static\r\n");
					}
				}
				
				if(mBICYCLE.last_state == STATE_SHAKE)
				{
//					if(lift_shake_flag == 0)														//�˴���  ���𶯵������Ĺ���
					{
						if(s_Count_Shake <= 5)
						{
							//5s�ڴ��𶯻ָ�������  ����΢������� ������¼			//�������µ�ֱ��������
//							printf("dander shake < 5s to static\r\n");
						}
						
						if((s_Count_Shake > 5) && (s_Count_Shake <= SHAKE_TIME))
						{
							if(first_to_stand)														//����Ƿ��״δ��𶯱�ɾ�ֹ
							{
								first_to_stand = 0;
								shake_stand_cnt++;													//��5s-20s֮��ָ�������  ���´���   
								printf("dander shake 5s - 10s to static\r\n");
								printf("shake_stand_cnt %d\r\n\r\n",shake_stand_cnt);
							}
//							printf("dander shake 5s - 20s to static\r\n");
						}
					}
					
//					if(lift_shake_flag == 1)														//̧�ߵ�����  �м������
//					{
//						if((s_Count_Lift + s_Count_Shake) <= (LIFT_TIME + 1))								//̧��->��->��ֹ ��7s��
//						{
//							//��̧����10s�ָ�������  ���´���  
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
						if(first_to_fall)														//����Ƿ��״δ�̧�߱�ɵ���
						{
							printf("dander lift < 10s to fall\r\n");
							first_to_fall = 0;
							lift_fall_cnt++;													//����10s�ָ�������  ���´���  
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
							//5s�ڴ��𶯻ָ�������  ����΢������� ������¼						//ֱ��������
//							printf("dander shake < 5s to fall\r\n");
						}
						
						if((s_Count_Shake > 5) && (s_Count_Shake <= SHAKE_TIME))
						{
							if(first_to_fall)													//����Ƿ��״δ��𶯱�ɵ���
							{
								first_to_fall = 0;
								shake_fall_cnt++;												//��5s-10s֮��ָ�����  ���´���    
								printf("shake_fall_cnt %d\r\n\r\n",shake_fall_cnt);
							}
//							printf("dander shake 5s - 20s to fall\r\n");
						}
					}
					
//					if(lift_shake_flag == 1)
//					{
//						if((s_Count_Lift + s_Count_Shake) <= (LIFT_TIME + 1))					//̧��->��->���� ��7s��
//						{
//							//��̧����10s�ָ�������  ���´���  
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
						//����̧�𳬹�50cm  ֱ�ӱ���	
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
							//̧�𳬹�10s  ����
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
						//����̧�𳬹�50cm  ֱ�ӱ���
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
							//̧�𳬹�10s  ����
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
							//����̧�𳬹�50cm  ֱ�ӱ���
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
//							if(s_Count_Shake > 3)												//���̧��ǰ���𶯳���3s  ����Ϊ���𶯲�����̧������� Ӧ�ü����Σ��ʱ��		
//								s_Count_Lift += s_Count_Shake;
							
							if(s_Count_Lift > LIFT_TIME)
							{
								//̧�𳬹�10s  ����
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
							//����̧�𳬹�50cm  ֱ�ӱ���
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
								//̧�𳬹�10s  ����
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
//				if(mBICYCLE.last_state == STATE_LIFT)							//��Ӧ������Ǿٸ�->����->��				
//				{
//					lift_shake_flag = 1;
//					if((s_Count_Shake + s_Count_Lift) > LIFT_TIME)
//					{
//						//̧�ߵ�ʱ�����10s  ����
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
					//����ʱ�����10s  ������ΪΣ����Ϊ
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
