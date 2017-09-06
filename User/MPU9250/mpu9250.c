#include "mpu9250.h"
#include "inv_mpu.h"
#include "usart.h"
#include "delay.h"
#include "math.h"
#include "inv_mpu_dmp_motion_driver.h"

uint8_t Time_Stamp;

float Accel_XOffset = 0;
float Accel_YOffset = 0;
float Accel_ZOffset = 0;

int Accel_ZErr = 1416;

float Pitch,Roll;
float accel_x,accel_y,accel_z;

#define ACCEL_WINDOW_H	400
#define ACCEL_WINDOW_L	-400
#define ACC_FILTER_COUNT	16
int16_t	acc_xyz_data[3][ACC_FILTER_COUNT] = {0};
int16_t	acc_data_index = 0;
/* These next two functions converts the orientation matrix (see
 * gyro_orientation) to a scalar representation for use by the DMP.
 * NOTE: These functions are borrowed from Invensense's MPL.
 */
static  unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;
}

static  unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx)
{
    unsigned short scalar;

    /*
       XYZ  010_001_000 Identity Matrix
       XZY  001_010_000
       YXZ  010_000_001
       YZX  000_010_001
       ZXY  001_000_010
       ZYX  000_001_010
     */

    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;


    return scalar;
}

void init_mpu(void)
{
	uint8_t result = 0;
	do
	{
		result = mpu_init();//��ʼ��
	}while(result);
	
//	if(result)
//	{
//		printf("mpu init failed...\n");
//		delay_ms(1000);
//	}
	if(!result)   //����0�����ʼ���ɹ�
    {   
        printf("mpu initialization complete......\n");
        
        //mpu_set_sensor
        if(!mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL))
        {
            printf("mpu_set_sensor complete ......\n");
        }
        else
        {
            printf("mpu_set_sensor come across error ......\n");
			while(1);
        }
        
        //mpu_configure_fifo
        if(!mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL))
        {
            printf("mpu_configure_fifo complete ......\n");
        }
        else
        {
            printf("mpu_configure_fifo come across error ......\n");
			while(1);
        }
        
		//���ü��ٶȼƲ�����Χ��+-4G
		if((mpu_set_accel_fsr(2)) == 0)
		{
			printf("mpu_set_accel_fsr complete ......\r\n");
		}
		else
		{
			printf("mpu_set_accel_fsr error ......\r\n");
			while(1);
		}
		
        //mpu_set_sample_rate
        if(!mpu_set_sample_rate(1000))//DEFAULT_MPU_HZ
        {
            printf("mpu_set_sample_rate complete ......\n");
        }
        else
        {
            printf("mpu_set_sample_rate error ......\n");
			while(1);
        }
        
        //dmp_load_motion_driver_firmvare
        if(!dmp_load_motion_driver_firmware())
        {
            printf("dmp_load_motion_driver_firmware complete ......\n");
        }
        else
        {
			printf("dmp_load_motion_driver_firmware come across error ......\n");
			while(1);
        }
        
        //dmp_set_orientation
        if(!dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation)))
        {
            printf("dmp_set_orientation complete ......\n");
        }
        else
        {
            printf("dmp_set_orientation come across error ......\n");
			while(1);
        }
        
        //dmp_enable_feature
        if(!dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
            DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
            DMP_FEATURE_GYRO_CAL))
        {
			printf("dmp_enable_feature complete ......\n");
        }
        else
        {
			printf("dmp_enable_feature come across error ......\n");
			while(1);
        }
        
        //dmp_set_fifo_rate
        if(!dmp_set_fifo_rate(DEFAULT_MPU_HZ))
        {
            printf("dmp_set_fifo_rate complete ......\n");
        }
        else
        {
            printf("dmp_set_fifo_rate come across error ......\n");
			while(1);
        }
        
        if(!mpu_set_dmp_state(1))
        {
            printf("mpu_set_dmp_state complete ......\n");
        }
        else
        {
            printf("mpu_set_dmp_state come across error ......\n");
			while(1);
        }
        
    }
}

/* ��ԭʼ���ݼ��ٶ�ֵ�����˲� */
void acc_filter(int16_t accel[3],int16_t acc_ave[3])
{
	int i,j;
	int32_t	acc_data_sum[3] = {0};
	
	//�Ƚ���һ�λ�е�����˲�
	for(i = 0; i < 3; i++)
	{
		if(accel[i] < ACCEL_WINDOW_H && accel[i] > ACCEL_WINDOW_L)
			accel[i] = 0;
	}
	
	//��i��ļ��ٶȱ�����acc_data_index����
	for(i=0;i<3;i++)
	{
		acc_xyz_data[i][acc_data_index] = accel[i];
	}
	
	//acc_data_indexѭ����1
	acc_data_index++;
	
	if(acc_data_index == ACC_FILTER_COUNT)
	{
		acc_data_index = 0;
	}
	
	//�����
	for(i=0;i<3;i++)
	{
		for(j=0;j<ACC_FILTER_COUNT;j++)
		{
			acc_data_sum[i] +=  acc_xyz_data[i][j];
		}
		acc_ave[i] = acc_data_sum[i]/ACC_FILTER_COUNT;
	}
	
	//�ٶ�acc_ave����һ�λ�е�����˲�
	for(i=0;i<3;i++)
	{
		if(acc_ave[i] < ACCEL_WINDOW_H && acc_ave[i] > ACCEL_WINDOW_L)
			acc_ave[i] = 0;
	}
}

void get_accel_bias(void)
{
	float accel_res[3];
	short filter_accel[3];
	uint16_t i;
	static uint16_t count;
	short gyro[3], accel[3], sensors;
	float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;
	long quat[4];
	unsigned long sensor_timestamp;
	unsigned char more;
//	float Err = 0;
	
	for(i = 0; i < 10000; i++)				//��ȡ���ϵ粻ƽ�ȵ�����
	{
		dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more);	 	

		/*��Ԫ������̬*/	
		if (sensors & INV_WXYZ_QUAT )    
		{    
			q0 = quat[0] / q30;    
			q1 = quat[1] / q30;    
			q2 = quat[2] / q30;    
			q3 = quat[3] / q30;    
		
			acc_filter(accel,filter_accel);
		
			accel_res[0] = (q0*q0 + q1*q1 - q2*q2 - q3*q3) * filter_accel[0] + (2*q1*q2-2*q0*q3) * filter_accel[1] + (2*q0*q2+2*q1*q3) * filter_accel[2];	
			accel_res[1] = (2*q0*q3+2*q1*q2) * filter_accel[0] + (q0*q0 - q1*q1 + q2*q2 - q3*q3) * filter_accel[1] + (-2*q0*q1+2*q2*q3) * filter_accel[2];	
			accel_res[2] = (2*q1*q3-2*q0*q2) * filter_accel[0] + (2*q0*q1+2*q2*q3) * filter_accel[1] + (q0*q0 - q1*q1 - q2*q2 + q3*q3) * filter_accel[2];	
		}	
	}
	
	for(i = 0; i < 10000; i++)				//�õ���ǰ�ļ��ٶȻ�׼
	{
		dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more);	 	

		/*��Ԫ������̬*/	
		if (sensors & INV_WXYZ_QUAT )    
		{    
			q0 = quat[0] / q30;    
			q1 = quat[1] / q30;    
			q2 = quat[2] / q30;    
			q3 = quat[3] / q30;    
		
			acc_filter(accel,filter_accel);
		
			accel_res[0] = (q0*q0 + q1*q1 - q2*q2 - q3*q3) * filter_accel[0] + (2*q1*q2-2*q0*q3) * filter_accel[1] + (2*q0*q2+2*q1*q3) * filter_accel[2];	
			accel_res[1] = (2*q0*q3+2*q1*q2) * filter_accel[0] + (q0*q0 - q1*q1 + q2*q2 - q3*q3) * filter_accel[1] + (-2*q0*q1+2*q2*q3) * filter_accel[2];	
			accel_res[2] = (2*q1*q3-2*q0*q2) * filter_accel[0] + (2*q0*q1+2*q2*q3) * filter_accel[1] + (q0*q0 - q1*q1 - q2*q2 + q3*q3) * filter_accel[2];	

			Accel_XOffset += ((float)accel_res[0] / 16384);	
			Accel_YOffset += ((float)accel_res[1] / 16384);	
			Accel_ZOffset += ((float)accel_res[2] / 16384);
			
			count++;
		}	
	}
	Accel_XOffset /= count;
	Accel_YOffset /= count;
	Accel_ZOffset /= count;
	
		
	Accel_ZErr = (sqrt(1-(Accel_XOffset*Accel_XOffset)-(Accel_YOffset*Accel_YOffset)) - Accel_ZOffset)*16384;
	if(Accel_ZErr < 0)
		Accel_ZErr *= -1;

}

float get_accel(void)	
{	
	static uint8_t fifter_flag,offset_count,wave_state,valley_flag,isDown;
	float accel_res[3];
	short filter_accel[3];
	static float offset_update_z; 
	short gyro[3], accel[3], sensors;
	float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;
	long quat[4];	
	unsigned long sensor_timestamp;
	unsigned char more;
	static float speed,last_accel_z,dis,valley_dis,valley_min,down_peak;
	float dis_accel;
	float dis_z = 0;

	Time_Stamp = 0;
	
	do
	{
		dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more);
	}while (!(sensors & INV_WXYZ_QUAT ));
	
	if (sensors & INV_WXYZ_QUAT )    
	{ 
		q0 = quat[0] / q30;    
		q1 = quat[1] / q30;    
		q2 = quat[2] / q30;    
		q3 = quat[3] / q30;    
        
    	Pitch  = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3 + Pitch_error; // pitch    
    	Roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3 + Roll_error; // roll    

		acc_filter(accel,filter_accel);
		
//		accel_x0 =  ((float)filter_accel[0] / 16384);
//		accel_y0 =  ((float)filter_accel[1] / 16384);
//		accel_z0 =  ((float)filter_accel[2] / 16384);
//		
//		Accel_ZErr = (sqrt(1-(accel_x0*accel_x0)-(accel_y0*accel_y0)) - accel_z0)*16384;
//		if(Accel_ZErr < 0)
//			Accel_ZErr *= -1;
		
		accel_res[0] = (q0*q0 + q1*q1 - q2*q2 - q3*q3) * filter_accel[0] + (2*q1*q2-2*q0*q3) * filter_accel[1] + (2*q0*q2+2*q1*q3) * filter_accel[2];	
		accel_res[1] = (2*q0*q3+2*q1*q2) * filter_accel[0] + (q0*q0 - q1*q1 + q2*q2 - q3*q3) * filter_accel[1] + (-2*q0*q1+2*q2*q3) * filter_accel[2];	
		accel_res[2] = (2*q1*q3-2*q0*q2) * filter_accel[0] + (2*q0*q1+2*q2*q3) * filter_accel[1] + (q0*q0 - q1*q1 - q2*q2 + q3*q3) * (filter_accel[2] + Accel_ZErr);
		 
		accel_x = (accel_res[0] / 16384) * 9.8 + 9.8;	
		accel_y = (accel_res[1] / 16384) * 9.8 + 9.8;	
		accel_z = (accel_res[2] / 16384) * 9.8;		
	
//		printf("%f  %f  %f   %d   %d   %d  %f  %f  %f \r\n",Accel_XOffset,Accel_YOffset,Accel_ZOffset,filter_accel[0],filter_accel[1],filter_accel[2],accel_x,accel_y,accel_z);
//		printf("%f  %f  %f \r\n",accel_x,accel_y,accel_z);
//		printf("add 1,0,%d",(int)(accel_z*10));											//��������ʾ����  �ٶ�  λ��
//		Usart_SendByte(USART1, 0xFF);
//		Usart_SendByte(USART1, 0xFF);
//		Usart_SendByte(USART1, 0xFF);
//		
//		printf("t2.txt=\"%f\"",Accel_ZOffset);
//		Usart_SendByte(USART1, 0xFF);
//		Usart_SendByte(USART1, 0xFF);
//		Usart_SendByte(USART1, 0xFF);												
		
		if(last_accel_z == 0.0) 
		{
			last_accel_z = Accel_ZOffset;
		}
	
		if((accel_z > 9.65) && (accel_z < 9.85))										//����accel_offset
		{			
			
			if((accel_z > (last_accel_z-0.05)) && (accel_z < (last_accel_z+0.05)))		//ÿ��accel����ֵ����һ����ֵ����0.05������һ  �����������ﵽʮ��  ����ƽ��ֵΪ�µ�accel_offset
			{
				offset_count++;
				offset_update_z += accel_z;
			}
			else																		//�������㾲ֹ����  ����/offset����
			{
				offset_count = 0;
				offset_update_z = 0;
			}
			
			if(wave_state == STATE_UP_LOW)												//����⵽������������   ��״̬2�ص�ˮƽ
			{
				
				if(fifter_flag == STATE_UP_LOW)											//�ж��Ƿ���������
				{
					wave_state = STATE_STATIC;
					fifter_flag = 0;
					
//					printf("t3.txt=\"%f\"",dis);
//					Usart_SendByte(USART1, 0xFF);
//					Usart_SendByte(USART1, 0xFF);
//					Usart_SendByte(USART1, 0xFF);
					dis_z = dis;
//					printf("dis = %f",dis_z);
					speed = 0;
					dis = 0;
				}			
			}
			
			if(wave_state == STATE_DOWN_HIGHT)											//����⵽�½���������   ��״̬3�ص�ˮƽ
			{
			
				if(isDown == 0)																	
				{
					if(valley_min < 8)
						dis = valley_dis;											//�˶���λ��Ϊ�ڲ���ʱ��¼��λ��
					valley_flag = 0;
				}
				else
				{
					isDown = 0;														//����ж��Ƿ�Ϊ�½��˶��ı�־
				}
				
				if(fifter_flag == STATE_DOWN_HIGHT)											//�ж��Ƿ���������
				{
					wave_state = STATE_STATIC;
					
					fifter_flag = 0;
					
//					printf("t3.txt=\"%f\"",dis);
//					Usart_SendByte(USART1, 0xFF);
//					Usart_SendByte(USART1, 0xFF);
//					Usart_SendByte(USART1, 0xFF);
					dis_z = dis;
//					printf("dis = %f",dis_z);
					speed = 0;
					dis = 0;
				}
			}
			
			if(offset_count == 10)
			{
				speed = 0.0;
				Accel_ZOffset = offset_update_z/10;
				offset_count = 0;
				offset_update_z = 0;

				if(wave_state == STATE_UP_HIGHT)										//�����ɲ���
				{
					wave_state = STATE_STATIC;
					speed = 0;
					dis = 0;
				}

				if(wave_state == STATE_DOWN_LOW)										//���½��˶����°벿����ֱ�ӻص�ˮƽ(��ͣ) ���߸�����΢�����˶�
				{
					wave_state = STATE_STATIC;
					if(valley_min < 8)												//��ͣ���ٶȽϴ� �˴���8Ϊ׼
					{
						dis = valley_dis;												//�˶���λ��Ϊ�ڲ���ʱ��¼��λ��
						
						if(fifter_flag == STATE_DOWN_LOW)
						{
							fifter_flag = 0; 
					
//							printf("t3.txt=\"%f\"",dis);
//							Usart_SendByte(USART1, 0xFF);
//							Usart_SendByte(USART1, 0xFF);
//							Usart_SendByte(USART1, 0xFF);
							dis_z = dis;
//							printf("dis = %f",dis_z);
						}
					}
					
					valley_flag = 0;													//�������»ص�ˮƽʱ �ñ�־����  �ȴ���һ���״γ����½���ǰ�°벿�ֲ�����һ
					speed = 0;
					dis = 0;	
				}
			}
		}
		
		if(accel_z < 9.65 || accel_z > 9.85)											//���accel_z�����������ٶȵġ�0.2   �����Ϊ��������(���忪ʼ�˶�)
		{																
			if(wave_state == STATE_STATIC)
			{
				if(accel_z > 10)// && speed > 0)
				{
					wave_state = STATE_UP_HIGHT;
				}
				if(accel_z < 9.5)// && speed < 0)
				{
					wave_state = STATE_DOWN_LOW;
				}
			}
			
			if(wave_state == STATE_UP_HIGHT)
			{
				if(accel_z > 10)														//�ж�up_hightʱ  ��ֵ �Ƿ�ﵽ�������εķ�ֵ
					fifter_flag = STATE_UP_HIGHT;	
				
				if(accel_z < Accel_ZOffset)// && speed > 0)
				{
					if(fifter_flag == STATE_UP_HIGHT)
						wave_state = STATE_UP_LOW;
					if(fifter_flag == STATE_STATIC)
					{
						wave_state = STATE_DOWN_LOW;
						speed = 0;
						dis = 0;
					}
				}
			}
			
			if(wave_state == STATE_UP_LOW)
			{
				if(speed < 0) speed = 0;
				
				if(accel_z < 9.5)														//�ж�up_lowʱ  ��ֵ �Ƿ�ﵽ�������εĹ�ֵ
					fifter_flag = STATE_UP_LOW;
			}
			
			if(wave_state == STATE_DOWN_LOW)
			{
				if(accel_z > Accel_ZOffset)// && speed < 0)
				{
					down_peak = (((Accel_ZOffset-valley_min)/2)+Accel_ZOffset);
					if(fifter_flag == STATE_DOWN_LOW)
						wave_state = STATE_DOWN_HIGHT;
					if(fifter_flag == STATE_STATIC)
					{
						wave_state = STATE_DOWN_LOW;
						speed = 0;
						dis = 0;
					}
				}
				
				if(valley_flag == 0)													//
				{		
					valley_min = 9.8;		
					valley_flag = 1;		
				}
				
				if(valley_flag == 1)													//��һ�θ�min��ֵ����Ϊ1
				{		
					if(accel_z < valley_min)												
					{		
						valley_min = accel_z;											//��¼���ٶ���Сֵ
						valley_dis = dis;												//���µ�ǰdisΪvalley_dis
					}
				}
				
				if(accel_z < 9.5)														//�ж�down_lowʱ  ��ֵ �Ƿ�ﵽ�������εĹ�ֵ
					fifter_flag = STATE_DOWN_LOW;
			}
			
			if(wave_state == STATE_DOWN_HIGHT)
			{
				if(speed > 0) 
					speed = 0;
				
				if(accel_z > down_peak)													//�ж��Ƿ�Ϊ�������½��˶�								
					isDown = 1;		
				
				if(accel_z > 10)														//�ж�down_hightʱ  ��ֵ �Ƿ�ﵽ�������εķ�ֵ
					fifter_flag = STATE_DOWN_HIGHT;
			}
			
			dis_accel = ((accel_z + last_accel_z) / 2) - Accel_ZOffset;	

			speed += dis_accel * Time_Stamp * 0.0001;									//m/s
			dis += speed * Time_Stamp * 0.01;											//cm
		}
		
		last_accel_z = accel_z;
//		printf("dis %f",dis_z);
		return dis_z;
		
	}
}	

//void get_accel_xyz(void)	
//{	
//	
//	float accel_res[3];
//	short filter_accel[3];
//	float accel_x,accel_y,accel_z;
//	short gyro[3], accel[3], sensors;	float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;
//	long quat[4];	
//	unsigned long sensor_timestamp;
//	unsigned char more;
//	
//	Time_Stamp = 0;
//	QUAT:
//	dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more);	 	

//	/*��Ԫ������̬*/
//	if (!(sensors & INV_WXYZ_QUAT ))
//		goto QUAT;
//	if (sensors & INV_WXYZ_QUAT )    
//	{ 
//		q0 = quat[0] / q30;    
//		q1 = quat[1] / q30;    
//		q2 = quat[2] / q30;    
//		q3 = quat[3] / q30;    
//        
//    	Pitch  = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3 + Pitch_error; // pitch    
//    	Roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3 + Roll_error; // roll    
//		
//		acc_filter(accel,filter_accel);
//		
//		accel_res[0] = (q0*q0 + q1*q1 - q2*q2 - q3*q3) * filter_accel[0] + (2*q1*q2-2*q0*q3) * filter_accel[1] + (2*q0*q2+2*q1*q3) * filter_accel[2];	
//		accel_res[1] = (2*q0*q3+2*q1*q2) * filter_accel[0] + (q0*q0 - q1*q1 + q2*q2 - q3*q3) * filter_accel[1] + (-2*q0*q1+2*q2*q3) * filter_accel[2];	
//		accel_res[2] = (2*q1*q3-2*q0*q2) * filter_accel[0] + (2*q0*q1+2*q2*q3) * filter_accel[1] + (q0*q0 - q1*q1 - q2*q2 + q3*q3) * filter_accel[2];
//	
//		accel_x = ((accel_res[0] / 16384) * 9.8)+9.8;	
//		accel_y = ((accel_res[1] / 16384) * 9.8)+9.8;	
//		accel_z = (accel_res[2] / 16384) * 9.8;		
//	
//		printf("add 1,0,%d",(int)(accel_x*10));											//��������ʾ����  �ٶ�  λ�� 
//		Usart_SendByte(USART1, 0xFF);
//		Usart_SendByte(USART1, 0xFF);
//		Usart_SendByte(USART1, 0xFF);

//		printf("t2.txt=\"%f\"",Accel_XOffset);
//		Usart_SendByte(USART1, 0xFF);
//		Usart_SendByte(USART1, 0xFF);
//		Usart_SendByte(USART1, 0xFF);											
//		
//		get_dis_z(accel_z); 								
//	}
//}	

//void get_dis_z(float accel_z)
//{		
//	static uint8_t isDown;
//	static uint8_t wave_state;															
//	static uint8_t fifter_flag;
//	static uint8_t valley_flag;
//	static uint8_t offset_count;
//	static float dis;
//	static float speed;
//	static float dis_accel;
//	static float down_peak;
//	static float valley_dis;
//	static float valley_min;
//	static float last_accel_z;
//	static float offset_update_z; 

//	if(last_accel_z == 0.0) 
//	{
//		last_accel_z = Accel_ZOffset;
//	}
//	
//	if((accel_z > 9.65) && (accel_z < 9.85))										//����accel_offset
//	{			
//		
//		if((accel_z > (last_accel_z-0.05)) && (accel_z < (last_accel_z+0.05)))		//ÿ��accel����ֵ����һ����ֵ����0.05������һ  �����������ﵽʮ��  ����ƽ��ֵΪ�µ�accel_offset
//		{
//			offset_count++;
//			offset_update_z += accel_z;
//		}
//		else																		//�������㾲ֹ����  ����/offset����
//		{
//			offset_count = 0;
//			offset_update_z = 0;
//		}
//		
//		if(wave_state == STATE_UP_LOW)												//����⵽������������   ��״̬2�ص�ˮƽ
//		{
//			
//			if(fifter_flag == STATE_UP_LOW)											//�ж��Ƿ���������
//			{
//				wave_state = STATE_STATIC;
//				fifter_flag = 0;
//				
//				printf("t3.txt=\"%f\"",dis);
//				Usart_SendByte(USART1, 0xFF);
//				Usart_SendByte(USART1, 0xFF);
//				Usart_SendByte(USART1, 0xFF);
//				
//				speed = 0;
//				dis = 0;
//			}			
//		}
//		
//		if(wave_state == STATE_DOWN_HIGHT)											//����⵽�½���������   ��״̬3�ص�ˮƽ
//		{
//		
//			if(isDown == 0)																	
//			{
//				if(valley_min < 8)
//					dis = valley_dis;												//�˶���λ��Ϊ�ڲ���ʱ��¼��λ��
//				valley_flag = 0;
//			}
//			else
//			{
//				isDown = 0;															//����ж��Ƿ�Ϊ�½��˶��ı�־
//			}
//			
//			if(fifter_flag == STATE_DOWN_HIGHT)										//�ж��Ƿ���������
//			{
//				wave_state = STATE_STATIC;
//				
//				fifter_flag = 0;
//				
//				printf("t3.txt=\"%f\"",dis);
//				Usart_SendByte(USART1, 0xFF);
//				Usart_SendByte(USART1, 0xFF);
//				Usart_SendByte(USART1, 0xFF);
//				
//				speed = 0;
//				dis = 0;
//			}
//		}
//		
//		if(offset_count == 10)
//		{
//			speed = 0.0;
//			Accel_ZOffset = offset_update_z/10;
//			offset_count = 0;
//			offset_update_z = 0;

//			if(wave_state == STATE_UP_HIGHT)										//�����ɲ���
//			{
//				wave_state = STATE_STATIC;
//				speed = 0;
//				dis = 0;
//			}

//			if(wave_state == STATE_DOWN_LOW)										//���½��˶����°벿����ֱ�ӻص�ˮƽ(��ͣ) ���߸�����΢�����˶�
//			{
//				wave_state = STATE_STATIC;
//				if(valley_min < 8)													//��ͣ���ٶȽϴ� �˴���8Ϊ׼
//				{
//					dis = valley_dis;												//�˶���λ��Ϊ�ڲ���ʱ��¼��λ��
//					
//					if(fifter_flag == STATE_DOWN_LOW)
//					{
//						fifter_flag = 0; 
//				
//						printf("t3.txt=\"%f\"",dis);
//						Usart_SendByte(USART1, 0xFF);
//						Usart_SendByte(USART1, 0xFF);
//						Usart_SendByte(USART1, 0xFF);
//					}
//				}
//				
//				valley_flag = 0;													//�������»ص�ˮƽʱ �ñ�־����  �ȴ���һ���״γ����½���ǰ�°벿�ֲ�����һ
//				speed = 0;
//				dis = 0;	
//			}
//		}
//	}
//	
//	if(accel_z < 9.65 || accel_z > 9.85)											//���accel_z�����������ٶȵġ�0.2   �����Ϊ��������(���忪ʼ�˶�)
//	{																
//		if(wave_state == STATE_STATIC)
//		{
//			if(accel_z > 10)// && speed > 0)
//			{
//				wave_state = STATE_UP_HIGHT;
//			}
//			if(accel_z < 9.5)// && speed < 0)
//			{
//				wave_state = STATE_DOWN_LOW;
//			}
//		}
//		
//		if(wave_state == STATE_UP_HIGHT)
//		{
//			if(accel_z > 10)														//�ж�up_hightʱ  ��ֵ �Ƿ�ﵽ�������εķ�ֵ
//				fifter_flag = STATE_UP_HIGHT;	
//			
//			if(accel_z < Accel_ZOffset)// && speed > 0)
//			{
//				if(fifter_flag == STATE_UP_HIGHT)
//					wave_state = STATE_UP_LOW;
//				if(fifter_flag == STATE_STATIC)
//				{
//					wave_state = STATE_DOWN_LOW;
//					speed = 0;
//					dis = 0;
//				}
//			}
//		}
//		
//		if(wave_state == STATE_UP_LOW)
//		{
//			if(speed < 0) speed = 0;
//			
//			if(accel_z < 9.5)														//�ж�up_lowʱ  ��ֵ �Ƿ�ﵽ�������εĹ�ֵ
//				fifter_flag = STATE_UP_LOW;
//		}
//		
//		if(wave_state == STATE_DOWN_LOW)
//		{
//			if(accel_z > Accel_ZOffset)// && speed < 0)
//			{
//				down_peak = (((Accel_ZOffset-valley_min)/2)+Accel_ZOffset);
//				if(fifter_flag == STATE_DOWN_LOW)
//					wave_state = STATE_DOWN_HIGHT;
//				if(fifter_flag == STATE_STATIC)
//				{
//					wave_state = STATE_DOWN_LOW;
//					speed = 0;
//					dis = 0;
//				}
//			}
//			
//			if(valley_flag == 0)													
//			{		
//				valley_min = 9.8;		
//				valley_flag = 1;		
//			}
//			
//			if(valley_flag == 1)													//��һ�θ�min��ֵ����Ϊ1
//			{		
//				if(accel_z < valley_min)												
//				{		
//					valley_min = accel_z;											//��¼���ٶ���Сֵ
//					valley_dis = dis;												//���µ�ǰdisΪvalley_dis
//				}
//			}
//			
//			if(accel_z < 9.5)														//�ж�down_lowʱ  ��ֵ �Ƿ�ﵽ�������εĹ�ֵ
//				fifter_flag = STATE_DOWN_LOW;
//		}
//		
//		if(wave_state == STATE_DOWN_HIGHT)
//		{
//			if(speed > 0) 
//				speed = 0;
//			
//			if(accel_z > down_peak)													//�ж��Ƿ�Ϊ�������½��˶�								
//				isDown = 1;		
//			
//			if(accel_z > 10)														//�ж�down_hightʱ  ��ֵ �Ƿ�ﵽ�������εķ�ֵ
//				fifter_flag = STATE_DOWN_HIGHT;
//		}
//		
//		dis_accel = ((accel_z + last_accel_z) / 2) - Accel_ZOffset;	

//		speed += dis_accel * Time_Stamp * 0.0001;									//m/s
//		dis += speed * Time_Stamp * 0.01;											//cm
//	}
//	
//	last_accel_z = accel_z;
//	Usart_SendByte(USART2,wave_state+'0');
//}

//void get_dis_x(float accel_x)
//{		
//	static uint8_t wave_state;															
//	static uint8_t fifter_flag;
//	static uint8_t offset_count;
//	static float dis;
//	static float speed;
//	static float dis_accel;
//	static float last_accel_x;
//	static float offset_update_x; 

//	if(last_accel_x == 0.0)
//	{
//		last_accel_x = Accel_XOffset;
//	}
//	
//	if((accel_x > 9.70) && (accel_x < 9.90))										//����accel_offset
//	{			
//		
//		if((accel_x > (last_accel_x-0.05)) && (accel_x < (last_accel_x+0.05)))		//ÿ��accel����ֵ����һ����ֵ����0.05������һ  �����������ﵽʮ��  ����ƽ��ֵΪ�µ�accel_offset
//		{
//			offset_count++;
//			offset_update_x += accel_x;
//		}
//		else																		//�������㾲ֹ����  ����/offset����
//		{
//			offset_count = 0;
//			offset_update_x = 0;
//		}
//		
//		if(wave_state == STATE_RIGHT_LOW)												//����⵽������������   ��״̬2�ص�ˮƽ
//		{
//			if(fifter_flag == STATE_RIGHT_LOW)											//�ж��Ƿ���������
//			{
//				wave_state = STATE_STATIC_X;
//				
//				fifter_flag = STATE_STATIC_X;
//				
//				printf("t3.txt=\"%f\"",dis);
//				Usart_SendByte(USART1, 0xFF);
//				Usart_SendByte(USART1, 0xFF);
//				Usart_SendByte(USART1, 0xFF);
//				
//				speed = 0;
//				dis = 0;	
//			}
//		}
//		
//		if(wave_state == STATE_LEFT_HIGHT)											//����⵽�½���������   ��״̬3�ص�ˮƽ
//		{
//			if(fifter_flag == STATE_LEFT_HIGHT)										//�ж��Ƿ���������
//			{
//				wave_state = STATE_STATIC_X;
//				
//				fifter_flag = STATE_STATIC_X;
//				
//				printf("t3.txt=\"%f\"",dis);
//				Usart_SendByte(USART1, 0xFF);
//				Usart_SendByte(USART1, 0xFF);
//				Usart_SendByte(USART1, 0xFF);
//				
//				speed = 0;
//				dis = 0;
//			}
//		}
//		
//		if(offset_count == 10)
//		{
//			speed = 0.0;
//			Accel_XOffset = offset_update_x/10;
//			offset_count = 0;
//			offset_update_x = 0;

//			wave_state = STATE_STATIC_X;
//			
//			if(wave_state == STATE_RIGHT_HIGHT)										//�����ɲ���
//			{
//				speed = 0;
//				dis = 0;
//			}

//			if(wave_state == STATE_LEFT_LOW)										//�����ɲ���
//			{
//				speed = 0;
//				dis = 0;	
//			}
//		}
//	}
//	
//	if(accel_x < 9.70 || accel_x > 9.90)											//���accel_x�����������ٶȵġ�0.2   �����Ϊ��������(���忪ʼ�˶�)
//	{																
//		if(wave_state == STATE_STATIC_X)
//		{
//			if(accel_x > 10.5)// && speed > 0)
//			{
//				wave_state = STATE_RIGHT_HIGHT;
//			}
//			if(accel_x < 9.55)// && speed < 0)
//			{
//				wave_state = STATE_LEFT_LOW;
//			}
//		}
//		
//		if(wave_state == STATE_RIGHT_HIGHT)
//		{
//			if(accel_x > 10.5)														//�ж�right_hightʱ  ��ֵ �Ƿ�ﵽ�������εķ�ֵ
//				fifter_flag = STATE_RIGHT_HIGHT;	
//			
//			if(accel_x < Accel_XOffset)// && speed > 0)
//			{
//				if(fifter_flag == STATE_RIGHT_HIGHT)
//					wave_state = STATE_RIGHT_LOW;
//				if(fifter_flag == STATE_STATIC_X)
//				{
//					wave_state = STATE_LEFT_LOW;
//					speed = 0;
//					dis = 0;
//				}
//			}
//		}
//		
//		if(wave_state == STATE_RIGHT_LOW)
//		{
//			if(speed < 0) speed = 0;
//			
//			if(accel_x < 9.55)														//�ж�right_lowʱ  ��ֵ �Ƿ�ﵽ�������εĹ�ֵ
//				fifter_flag = STATE_RIGHT_LOW;
//		}
//		
//		if(wave_state == STATE_LEFT_LOW)
//		{
//			if(accel_x > Accel_XOffset)// && speed < 0)
//			{
//				if(fifter_flag == STATE_LEFT_LOW)
//					wave_state = STATE_LEFT_HIGHT;
//				if(fifter_flag == STATE_STATIC_X)
//				{
//					wave_state = STATE_LEFT_LOW;
//					speed = 0;
//					dis = 0;
//				}
//			}

//			if(accel_x < 9.55)														//�ж�down_lowʱ  ��ֵ �Ƿ�ﵽ�������εĹ�ֵ
//				fifter_flag = STATE_LEFT_LOW;
//		}
//		
//		if(wave_state == STATE_LEFT_HIGHT)
//		{
//			if(speed > 0) 
//				speed = 0;

//			if(accel_x > 10.5)														//�ж�down_hightʱ  ��ֵ �Ƿ�ﵽ�������εķ�ֵ
//				fifter_flag = STATE_LEFT_HIGHT;
//		}
//		
//		dis_accel = ((accel_x + last_accel_x) / 2) - Accel_XOffset;	

//		speed += dis_accel * Time_Stamp * 0.0001;									//m/s
//		dis += speed * Time_Stamp * 0.01;											//cm
//	}
//	
//	last_accel_x = accel_x;
//	Usart_SendByte(USART2,wave_state+'0');
//}
