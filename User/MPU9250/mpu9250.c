#include "mpu9250.h"
#include "inv_mpu.h"
#include "usart.h"
#include "delay.h"
#include "math.h"
#include "inv_mpu_dmp_motion_driver.h"

uint8_t Time_Stamp;
float Accel_XOffset;
float Accel_YOffset;
float Accel_ZOffset = 9.8;
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
//	MPU_INIT:
	result = mpu_init();//��ʼ��
	if(result)
	{
		printf("mpu init failed...\n");
		delay_ms(1000);
//		goto MPU_INIT;
	}
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

			Accel_XOffset += (accel_res[0] / 16384) * 9.8;	
			Accel_YOffset += (accel_res[1] / 16384) * 9.8;	
			Accel_ZOffset += (accel_res[2] / 16384) * 9.8;

			count++;
//			printf("%f\r\n",(accel_res[2] / 16384) * 9.8);
		}	
	}
	Accel_XOffset /= count;
	Accel_YOffset /= count;
	Accel_ZOffset /= count;
	
//	Accel_ZOffset = (float)((uint16_t)(Accel_ZOffset*100))/100;
	
}

void get_accel_1(void)	
{	
	static uint8_t new_data,count;
	float accel_res[3];
	short filter_accel[3];
	float accel_x,accel_y,accel_z;
	short gyro[3], accel[3], sensors;
	float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;
	long quat[4];
	float Pitch,Roll,Yaw;	
	unsigned long sensor_timestamp;
	unsigned char more;
	static float speed,last_accel_z,last_speed,dis_accel,dis;
	static uint8_t wave_state;

	
	Time_Stamp = 0;
	QUAT:
	dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more);	 	

	/*��Ԫ������̬*/
	if (!(sensors & INV_WXYZ_QUAT )) 
		goto QUAT;
	if (sensors & INV_WXYZ_QUAT )    
	{ 
		q0 = quat[0] / q30;    
		q1 = quat[1] / q30;    
		q2 = quat[2] / q30;    
		q3 = quat[3] / q30;    
        
//    	Pitch  = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3 + Pitch_error; // pitch    
//    	Roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3 + Roll_error; // roll    
		
		acc_filter(accel,filter_accel);
		
		accel_res[0] = (q0*q0 + q1*q1 - q2*q2 - q3*q3) * filter_accel[0] + (2*q1*q2-2*q0*q3) * filter_accel[1] + (2*q0*q2+2*q1*q3) * filter_accel[2];	
		accel_res[1] = (2*q0*q3+2*q1*q2) * filter_accel[0] + (q0*q0 - q1*q1 + q2*q2 - q3*q3) * filter_accel[1] + (-2*q0*q1+2*q2*q3) * filter_accel[2];	
		accel_res[2] = (2*q1*q3-2*q0*q2) * filter_accel[0] + (2*q0*q1+2*q2*q3) * filter_accel[1] + (q0*q0 - q1*q1 - q2*q2 + q3*q3) * filter_accel[2];	

//		accel_res[0] = (q0*q0 + q1*q1 - q2*q2 - q3*q3) * accel[0] + (2*q1*q2-2*q0*q3) * accel[1] + (2*q0*q2+2*q1*q3) * accel[2];	
//		accel_res[1] = (2*q0*q3+2*q1*q2) * accel[0] + (q0*q0 - q1*q1 + q2*q2 - q3*q3) * accel[1] + (-2*q0*q1+2*q2*q3) * accel[2];	
//		accel_res[2] = (2*q1*q3-2*q0*q2) * accel[0] + (2*q0*q1+2*q2*q3) * accel[1] + (q0*q0 - q1*q1 - q2*q2 + q3*q3) * accel[2];
		
		accel_x = ((float)accel_res[0] / 16384) * 9.8;	
		accel_y = ((float)accel_res[1] / 16384) * 9.8;	
		accel_z = ((float)accel_res[2] / 16384) * 9.8;	

	
		printf("add 1,0,%d",(int)(accel_z*10));				//��������ʾ����  �ٶ�  λ��
		Usart_SendByte(USART1, 0xFF);
		Usart_SendByte(USART1, 0xFF);
		Usart_SendByte(USART1, 0xFF);
		
		printf("t2.txt=\"%f\"",accel_z);
		Usart_SendByte(USART1, 0xFF);
		Usart_SendByte(USART1, 0xFF);
		Usart_SendByte(USART1, 0xFF);
		
		printf("t3.txt=\"%f\"",dis);
		Usart_SendByte(USART1, 0xFF);
		Usart_SendByte(USART1, 0xFF);
		Usart_SendByte(USART1, 0xFF);
		
		if(last_accel_z == 0.0) 
		{
			last_accel_z = Accel_ZOffset;
		}
		
		if(accel_z < 9.4 || accel_z > 10.2)			
		{
			new_data = 1;													//���accel_z�����������ٶȵġ�0.2   �����Ϊ��������(���忪ʼ�˶�)
			count = 0;
			if(wave_state == STATE_STATIC)
			{
				if(accel_z > 9.8 && speed > 0)
				{
					wave_state = STATE_UP_HIGHT;
				}
				if(accel_z < 9.8 && speed < 0)
				{
					wave_state = STATE_DOWN_LOW;
				}
			}
			if(wave_state == STATE_UP_HIGHT)
			{
				if(accel_z < 9.8 && speed > 0)
				{
					wave_state = STATE_UP_LOW;
				}
			}
			if(wave_state == STATE_DOWN_LOW)
			{
				if(accel_z > 9.8 && speed < 0)
				{
					wave_state = STATE_DOWN_HIGHT;
				}
			}
		}
		if(accel_z > 9.4 && accel_z < 10.2)	
		{
			if(wave_state == STATE_UP_LOW || wave_state == STATE_DOWN_HIGHT)
				wave_state = STATE_STATIC;
			if(accel_z - last_accel_z > -0.05 && accel_z - last_accel_z < 0.05)
			{
				if(count == 10 && new_data == 1)													//���������⵽���ٶ���g��0.2��Χ   �����Ϊ��ǰ���˶�
				{
					if(new_data)
					{
//						printf("\r\naccel %f speed %f dis:%f \r\n",accel_z,speed,dis);
					}
//					Accel_ZOffset = accel_z;														//������ʮ�μ�⵽accel_z�仯С��0.1   �����Ϊ��ǰ���˶�  �ѵ�ǰ��accel_z����accelZOFFSET
					new_data = 0;
					speed = 0.0;
				}
				if(count < 10)
				count++;
			}
		}
		
		if(new_data)
		{
			dis_accel = ((accel_z + last_accel_z) / 2) - Accel_ZOffset;
			
			if(wave_state == STATE_UP_LOW)
			{
				if(speed < 0) speed = 0;
			}
			if(wave_state == STATE_DOWN_HIGHT)
			{
				if(speed > 0) speed = 0;
			}
			
			speed += dis_accel * Time_Stamp * 0.0001;					//m/s
			dis += speed * Time_Stamp * 0.01;							//cm
	
		}
		last_accel_z = accel_z;
	}
	
}	

void get_accel(void)	
{	
	static uint8_t new_data,fifter_flag,offset_count,wave_state,valley_flag,isDown;
	float accel_res[3];
	short filter_accel[3];
	float accel_x,accel_y,accel_z;
	static float offset_update_z; 
	short gyro[3], accel[3], sensors;
	float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;
	long quat[4];
	float Pitch,Roll,Yaw;	
	unsigned long sensor_timestamp;
	unsigned char more;
	static float speed,last_accel_z,last_speed,dis_accel,dis,valley_dis,valley_min,down_peak;
	
	Time_Stamp = 0;
	QUAT:
	dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more);	 	

	/*��Ԫ������̬*/
	if (!(sensors & INV_WXYZ_QUAT )) 
		goto QUAT;
	if (sensors & INV_WXYZ_QUAT )    
	{ 
		q0 = quat[0] / q30;    
		q1 = quat[1] / q30;    
		q2 = quat[2] / q30;    
		q3 = quat[3] / q30;    
        
//    	Pitch  = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3 + Pitch_error; // pitch    
//    	Roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3 + Roll_error; // roll    
		
		acc_filter(accel,filter_accel);
		
		accel_res[0] = (q0*q0 + q1*q1 - q2*q2 - q3*q3) * filter_accel[0] + (2*q1*q2-2*q0*q3) * filter_accel[1] + (2*q0*q2+2*q1*q3) * filter_accel[2];	
		accel_res[1] = (2*q0*q3+2*q1*q2) * filter_accel[0] + (q0*q0 - q1*q1 + q2*q2 - q3*q3) * filter_accel[1] + (-2*q0*q1+2*q2*q3) * filter_accel[2];	
		accel_res[2] = (2*q1*q3-2*q0*q2) * filter_accel[0] + (2*q0*q1+2*q2*q3) * filter_accel[1] + (q0*q0 - q1*q1 - q2*q2 + q3*q3) * filter_accel[2];	
		
		accel_x = (accel_res[0] / 16384) * 9.8;	
		accel_y = (accel_res[1] / 16384) * 9.8;	
		accel_z = (accel_res[2] / 16384) * 9.8;		
	
		printf("add 1,0,%d",(int)(accel_z*10));				//��������ʾ����  �ٶ�  λ��
		Usart_SendByte(USART1, 0xFF);
		Usart_SendByte(USART1, 0xFF);
		Usart_SendByte(USART1, 0xFF);

		
		printf("t2.txt=\"%f\"",speed);
		Usart_SendByte(USART1, 0xFF);
		Usart_SendByte(USART1, 0xFF);
		Usart_SendByte(USART1, 0xFF);
		
//		printf("t3.txt=\"%f\"",dis);
//		Usart_SendByte(USART1, 0xFF);
//		Usart_SendByte(USART1, 0xFF);
//		Usart_SendByte(USART1, 0xFF);

//		printf("Accel_ZOffset %f  accel %f  speed %f dis%f  state %d\r\n",Accel_ZOffset,accel_z,speed,dis,wave_state);
		
//		D_accel_z = accel_z - last_accel_z;															
		
		if(last_accel_z == 0.0) 
		{
			last_accel_z = Accel_ZOffset;
		}
	
		if(accel_z > 9.65 && accel_z < 9.85)														//����accel_offset
		{
			
//			if(wave_state == STATE_UP_LOW || wave_state == STATE_DOWN_HIGHT)
//				wave_state = STATE_STATIC;
			
			if(wave_state == STATE_UP_LOW)
				wave_state = STATE_STATIC;
			
			if((accel_z > (last_accel_z-0.05)) && (accel_z < (last_accel_z+0.05)))					//ÿ��accel����ֵ����һ����ֵ����0.05������һ  �����������ﵽʮ��  ����ƽ��ֵΪ�µ�accel_offset
			{
				offset_count++;
				offset_update_z += accel_z;
			}
			else																					//�������㾲ֹ����  ����/offset����
			{
				offset_count = 0;
				offset_update_z = 0;
			}
			
			if(offset_count == 10)
			{
				Accel_ZOffset = offset_update_z/10;
				offset_count = 0;
				offset_update_z = 0;
				
				if(wave_state == STATE_UP_HIGHT)
					wave_state = STATE_STATIC;
				
				if(wave_state == STATE_DOWN_LOW)							//���½��˶����°벿����ֱ�ӻص�ˮƽ(��ͣ) ���߸�����΢�����˶�
				{
					wave_state = STATE_STATIC;
					dis = valley_dis;																//�˶���λ��Ϊ�ڲ���ʱ��¼��λ��
					valley_flag = 0;
				}
				
				if(wave_state == STATE_DOWN_HIGHT)
				{
					wave_state = STATE_STATIC;
				
					if(isDown == 0)																	//
					{
						wave_state = STATE_STATIC;
						dis = valley_dis;															//�˶���λ��Ϊ�ڲ���ʱ��¼��λ��
						valley_flag = 0;
					}
					else
					{
						isDown = 0;																	//����ж��Ƿ�Ϊ�½��˶��ı�־
					}
				}
				
				
				if(new_data == 1) 																	//�˴���������  (ǰ�Ŵ�������δ���)
				{
					new_data = 0;
					speed = 0.0;
//					down_hight_count = 0;															//����½��˶����ϰ벿�������ݼ�����־
					
					if(fifter_flag == 0)															//��ʱ��disΪ����/������Ķ���
						dis = 0;
					if(fifter_flag == 1)
						fifter_flag = 0; 
					
					printf("t3.txt=\"%f\"",dis);
					Usart_SendByte(USART1, 0xFF);
					Usart_SendByte(USART1, 0xFF);
					Usart_SendByte(USART1, 0xFF);
					dis = 0.0;
				}
			}
		}
		
		if(accel_z < 9.65 || accel_z > 9.85)			
		{
			new_data = 1;													//���accel_z�����������ٶȵġ�0.2   �����Ϊ��������(���忪ʼ�˶�)
			if(wave_state == STATE_STATIC)
			{
				if(accel_z > Accel_ZOffset && speed > 0)
				{
					wave_state = STATE_UP_HIGHT;
				}
				if(accel_z < Accel_ZOffset && speed < 0)
				{
					wave_state = STATE_DOWN_LOW;
				}
			}
			if(wave_state == STATE_UP_HIGHT)
			{
				if(accel_z < Accel_ZOffset && speed > 0)
				{
					wave_state = STATE_UP_LOW;
				}
			}
			if(wave_state == STATE_DOWN_LOW)
			{
				if(accel_z > Accel_ZOffset && speed < 0)
				{
					down_peak = (((Accel_ZOffset-valley_min)/2)+Accel_ZOffset);
					wave_state = STATE_DOWN_HIGHT;
				}
			}
		}

		if(new_data)
		{
			dis_accel = ((accel_z + last_accel_z) / 2) - Accel_ZOffset;	
			
			if(accel_z > 10 || accel_z < 9.5)
				fifter_flag = 1;												//�˴�Ϊ��¼���εķ�ֵ   Ŀ�� �˳�С��  ������
			
			if(wave_state == STATE_UP_LOW)
			{
				if(speed < 0) speed = 0;
			}
			
//			if(wave_state == STATE_DOWN_LOW)									//��6050�����½��˶�ʱ  ���ܻ����ֱ��ײ������(����)������ͣ�����
//			{																	//���½����̵��°벿���μ�¼����ͣ���ٽ��
//				if(accel_z < last_accel_z)										//�ж����°벿�������Ƿ��ڼ��ٶȼ��ٲ���
//					valley_flag = 1;
//				else															//��������°벿��������ٶ����ӵĲ���
//				{
//					if(valley_flag == 1)										//��ǰһ�̴����°벿���εļ��ٶȼ��ٲ���
//					{																
//						valley_dis = dis;										//�õ�Ϊ����  ��¼��ǰ��dis
//						valley_flag = 0;										//����½����β��ּ��ٶȼ��ٵı�־
//					}
//				}

//			}
			
			if(wave_state == STATE_DOWN_LOW)									//��6050�����½��˶�ʱ  ���ܻ����ֱ��ײ������(����)������ͣ�����
			{																	//���½����̵��°벿���μ�¼����ͣ���ٽ��
				if(valley_flag == 0)											//
				{
					valley_min = 9.8;
					valley_flag = 1;
				}
				if(valley_flag == 1)											//��һ�θ�min��ֵ����Ϊ1
				{
					if(accel_z < valley_min)										
					{
						valley_min = accel_z;									//��¼���ٶ���Сֵ
						valley_dis = dis;										//���µ�ǰdisΪvalley_dis
					}
				}
				
			}
			
			if(wave_state == STATE_DOWN_HIGHT)
			{
				if(speed > 0) 
					speed = 0;
				
				if(accel_z > down_peak)											//�ж��Ƿ�Ϊ�������½��˶�								
					isDown = 1;
			}
			
			speed += dis_accel * Time_Stamp * 0.0001;							//m/s
			dis += speed * Time_Stamp * 0.01;									//cm
	
		}
		last_accel_z = accel_z;
//		printf("state %d",wave_state);
	}

	
}	
