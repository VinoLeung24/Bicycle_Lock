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
	static uint8_t lift_shake_flag;													//̧��->����->��
	static uint8_t stand_shake_lift_flag;											//ֱ��->��->̧��
	static float dis_z;
	float lift_ignore;
	static uint8_t lift_stand_cnt;
	
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
				if(distance > 47)
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
					mBICYCLE.last_state = mBICYCLE.bicycle_state;
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
					mBICYCLE.last_state = mBICYCLE.bicycle_state;
				mBICYCLE.bicycle_state = STATE_FALL;
				dis_z = 0;
			}
		}
		else		
			new_fall = 0;															//����жϵ��³������ݵı�־
		
		if((dis_z > STATIC_LIFT_HIGHT) || ((distance - STATIC_HIGHT) > STATIC_LIFT_HIGHT))			//����Ƿ�̧��
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
		
		if(SHAKE_DEMAND && (dis_z < 10))											//����Ƿ�����
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
						//����10s�ָ�������  ���´���  ��������
						printf("dander lift < 10s to static\r\n");
					}
				}
				
				if(mBICYCLE.last_state == STATE_SHAKE)
				{
					if(lift_shake_flag == 0)														//�˴���  ���𶯵������Ĺ���
					{
						if(s_Count_Shake <= 5)
						{
							//5s�ڴ��𶯻ָ�������  ����΢������� ������¼			//�������µ�ֱ��������
							printf("dander shake < 5s to static\r\n");
						}
						
						if((s_Count_Shake > 5) && (s_Count_Shake <= 20))
						{
							//��5s-20s֮��ָ�������  ���´���  
							printf("dander shake 5s - 20s to static\r\n");
						}
					}
					
					if(lift_shake_flag == 1)														//̧�ߵ�����  �м������
					{
						if((s_Count_Lift + s_Count_Shake) <= (LIFT_TIME + 1))								//̧��->��->��ֹ ��7s��
						{
							//��̧����10s�ָ�������  ���´���  
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
						//����10s�ָ�������  ���´���  ��������					//����ֱ�������µ�����
						printf("dander lift < 10s to fall\r\n");
					}
				}
				if(mBICYCLE.last_state == STATE_SHAKE)
				{
					if(lift_shake_flag == 0)
					{
						if(s_Count_Shake <= 5)
						{
							//5s�ڴ��𶯻ָ�������  ����΢������� ������¼
							printf("dander shake < 5s to fall\r\n");
						}
						
						if((s_Count_Shake > 5) && (s_Count_Shake <= 20))
						{
							//��5s-20s֮��ָ�����  ���´���  
							printf("dander shake 5s - 20s to fall\r\n");
						}
					}
					
					if(lift_shake_flag == 1)
					{
						if((s_Count_Lift + s_Count_Shake) <= (LIFT_TIME + 1))								//̧��->��->���� ��7s��
						{
							//��̧����10s�ָ�������  ���´���  
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
						//����̧�𳬹�50cm  ֱ�ӱ���
						printf("danger lift > 50cm\r\n");
					}
					
					if(((dis_z > 10) && (dis_z < 50)) || (((distance - STATIC_HIGHT) > 10) && (distance < 50)))
					{
						if(s_Count_Lift > LIFT_TIME)
						{
							//̧�𳬹�10s  ����
							printf("dander lift > 10s \r\n");
						}
					}
				}
				
				if(mBICYCLE.last_state == STATE_FALL)
				{
					if(dis_z > 50)
					{
						//����̧�𳬹�50cm  ֱ�ӱ���
						printf("danger lift > 50cm\r\n");
					}
					
					if((dis_z > 10) && (dis_z < 50))
					{
						if(s_Count_Lift > LIFT_TIME)
						{
							//̧�𳬹�10s  ����
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
							//����̧�𳬹�50cm  ֱ�ӱ���
							printf("danger lift > 50cm\r\n");
						}
						
						if(((dis_z > 10) && (dis_z < 50)) || ((distance > 10) && (distance < 50)))
						{
							if(s_Count_Lift > LIFT_TIME)
							{
								//̧�𳬹�10s  ����
								printf("dander lift > 10s \r\n");
							}
						}
					}
					
					if(stand_shake_lift_flag == STATE_FALL)
					{
						if(dis_z > 50)
						{
							//����̧�𳬹�50cm  ֱ�ӱ���
							printf("danger lift > 50cm\r\n");
						}
						
						if((dis_z > 10) && (dis_z < 50))
						{
							if(s_Count_Lift > LIFT_TIME)
							{
								//̧�𳬹�10s  ����
								printf("dander lift > 10s \r\n");
							}
						}
					}
					
				}
			break;
			
			case STATE_SHAKE:
				if(mBICYCLE.last_state == STATE_LIFT)							//��Ӧ������Ǿٸ�->����->��				
				{
					lift_shake_flag = 1;
					if((s_Count_Shake + s_Count_Lift) > LIFT_TIME)
					{
						//̧�ߵ�ʱ�����10s  ����
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
					//����ʱ�����20s  ������ΪΣ����Ϊ
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
