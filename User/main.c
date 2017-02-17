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

uint16_t u_Count_Danger;
uint16_t s_Count_Danger;

extern float Pitch,Roll;
extern float accel_x,accel_y,accel_z;

int main(void)
{ 
	float distance = 0;
	static float dis_z;
	
	SystemInit();
	Shake_GPIO_Config();															//�𶯼��IO���
	buzzer_init();																	//������IO��ʼ��
	vl530l0_iic_init();																//������iic��ʼ��
	USARTx_Config();																//USART1��ʼ��
	USART2_Config();																//USART2��ʼ��
	I2C_Config();																	//MPU9150 I2C��ʼ��//SDA-PC11   SCL-PC12	 
	init_mpu();																		//DMP��ʼ��
	TIM2_Configuration();															//TIM2��ʼ��
	get_accel_bias();																//��ȡAccel_ZOffset
								
	while(1)			
	{
		static uint8_t new_stand,new_fall,new_danger;
			
		dis_z = get_accel();																//��ȡ��ǰ Pitch Roll accel_x accel_y accel_z dis
		distance = vl530l0_get_distance() - STATIC_HIGHT;													//��ȡ��ǰdis---����
//		printf("dis %f",dis_z);
//		if(! isShake())																		//����Ƿ�����
//		{		
//			Buzzer_ON;		
//		}		
//		else		
//		{		
//			Buzzer_OFF;		
//		}		
				
		if(STATIC_ANGLE_DEMAND && STATIC_ACCEL_DEMAND)										//���ֱ��������
		{
//			if(dis_z < 0)																	//���ھ������� һ����⵽�½���������dis_z  ������������ж�
//			{
//				dis_z = 0;
//			}
		
			if(new_stand == 0)																//��Ϊ�������ݶԶ�ʱ����������
			{		
				new_stand = 1;																
				uSec_Count = 0;																//��ʱ����������
				Sec_Count = 0;		
			}		
					
			if(Sec_Count == 1)																
			{		
				if(mBICYCLE.bicycle_state == STATE_DANGER)									//���г���Σ�ն����ָ�����ֹ
				{		
					if(s_Count_Danger <= 9)													//10s�ڴ�Σ�ն����ָ�����ֹ ������Ϊ��
					{		
						mBICYCLE.bicycle_state = STATE_STAND_STATIC;
						new_danger = 0;	
						printf("maybe warming   s_count%d\r\n\r\n\r\n",s_Count_Danger);
					}		
					else		
					{		
						//����10s�ָ�   ��������		
					}		
				}		
				else		
					mBICYCLE.bicycle_state = STATE_STAND_STATIC;							//��������һ��  ��Ϊֱ����ֹ״̬
			}		
		}		
		else		
			new_stand = 0;																	//����ж�ֱ���������ݵı�־	
					                                                                                                   	 
							
		if(FALL_ANGLE_DEMAND && FALL_ACCEL_DEMAND)											//����Ƿ���
		{			
			if(new_fall == 0)		
			{		
				new_fall = 1;		
				uSec_Count = 0;																//��ʱ����������
				Sec_Count = 0;		
			}		
			if(Sec_Count == 1)
			{
				if(mBICYCLE.bicycle_state == STATE_DANGER)									//���г���Σ�ն����ָ�����ֹ
				{		
					if(s_Count_Danger <= 9)													//10s�ڴ�Σ�ն����ָ�����ֹ ������Ϊ��
					{		
						mBICYCLE.bicycle_state = STATE_FALL;
						new_danger = 0;	
						printf("maybe warming   s_count%d\r\n\r\n\r\n",s_Count_Danger);
					}		
					else		
					{		
						//����10s�ָ�   ��������		
					}		
				}		
				else
					mBICYCLE.bicycle_state = STATE_FALL;										//��������һ��  ��Ϊֱ������״̬	
			}
		}		
		else		
			new_fall = 0;																	//����жϵ��³������ݵı�־	
		
		if((dis_z > STATIC_LIFT_HIGHT) || (distance > STATIC_LIFT_HIGHT))					//����Ƿ�̧��
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
		else if(MOVE_DEMAND)																//����Ƿ����쳣�ƶ�
		{	
			mBICYCLE.bicycle_state = STATE_DANGER;	
		}	
		else	
		{
//			printf("ihdis\r\n\r\n\r\n");
//			new_danger = 0;																	////����ж�̧�߳������ݵı�־
		}
			
	
		if(mBICYCLE.bicycle_state == STATE_DANGER)											//��⵽���г�����Σ����Ϊ ���ö�ʱ��
		{	
			if(new_danger == 0)	
			{	
				new_danger = 1;	
				u_Count_Danger = 0;															//��ʱ����������
				s_Count_Danger = 0;
			}
			
			if(s_Count_Danger >= 10)
			{
				//����10s�ڳ��������쳣���� ���Լ���Ϊ����
				printf(" FBI wriming   s_count %d\r\n\r\n\r\n",s_Count_Danger);
			}
		}
		printf("state %d s_count %d\r\n",mBICYCLE.bicycle_state,s_Count_Danger);
//		printf("Pitch: %f  Roll %f  accel_x %f  accel_y %f accel_z %f\r\n",Pitch,Roll,accel_x,accel_y,accel_z);
//		printf("dis %f distance %f state %d s_count %d\r\n",dis_z,distance,mBICYCLE.bicycle_state,s_Count_Danger);
	}
}
