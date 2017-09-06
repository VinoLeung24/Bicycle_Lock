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
		result = mpu_init();//初始化
	}while(result);
	
//	if(result)
//	{
//		printf("mpu init failed...\n");
//		delay_ms(1000);
//	}
	if(!result)   //返回0代表初始化成功
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
        
		//设置加速度计测量范围：+-4G
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

/* 对原始数据加速度值进行滤波 */
void acc_filter(int16_t accel[3],int16_t acc_ave[3])
{
	int i,j;
	int32_t	acc_data_sum[3] = {0};
	
	//先进行一次机械窗口滤波
	for(i = 0; i < 3; i++)
	{
		if(accel[i] < ACCEL_WINDOW_H && accel[i] > ACCEL_WINDOW_L)
			accel[i] = 0;
	}
	
	//将i轴的加速度保存在acc_data_index列中
	for(i=0;i<3;i++)
	{
		acc_xyz_data[i][acc_data_index] = accel[i];
	}
	
	//acc_data_index循环加1
	acc_data_index++;
	
	if(acc_data_index == ACC_FILTER_COUNT)
	{
		acc_data_index = 0;
	}
	
	//行求和
	for(i=0;i<3;i++)
	{
		for(j=0;j<ACC_FILTER_COUNT;j++)
		{
			acc_data_sum[i] +=  acc_xyz_data[i][j];
		}
		acc_ave[i] = acc_data_sum[i]/ACC_FILTER_COUNT;
	}
	
	//再对acc_ave进行一次机械窗口滤波
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
	
	for(i = 0; i < 10000; i++)				//抽取刚上电不平稳的数据
	{
		dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more);	 	

		/*四元数解姿态*/	
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
	
	for(i = 0; i < 10000; i++)				//得到当前的加速度基准
	{
		dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more);	 	

		/*四元数解姿态*/	
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
//		printf("add 1,0,%d",(int)(accel_z*10));											//串口屏显示波形  速度  位移
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
	
		if((accel_z > 9.65) && (accel_z < 9.85))										//更新accel_offset
		{			
			
			if((accel_z > (last_accel_z-0.05)) && (accel_z < (last_accel_z+0.05)))		//每当accel的数值和上一个数值相差±0.05计数加一  当计数连续达到十次  更新平均值为新的accel_offset
			{
				offset_count++;
				offset_update_z += accel_z;
			}
			else																		//当不满足静止条件  计数/offset清零
			{
				offset_count = 0;
				offset_update_z = 0;
			}
			
			if(wave_state == STATE_UP_LOW)												//当检测到上升动作结束   在状态2回到水平
			{
				
				if(fifter_flag == STATE_UP_LOW)											//判断是否完整波形
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
			
			if(wave_state == STATE_DOWN_HIGHT)											//当检测到下降动作结束   在状态3回到水平
			{
			
				if(isDown == 0)																	
				{
					if(valley_min < 8)
						dis = valley_dis;											//运动的位移为在波谷时记录的位移
					valley_flag = 0;
				}
				else
				{
					isDown = 0;														//清除判断是否为下降运动的标志
				}
				
				if(fifter_flag == STATE_DOWN_HIGHT)											//判断是否完整波形
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

				if(wave_state == STATE_UP_HIGHT)										//不构成波形
				{
					wave_state = STATE_STATIC;
					speed = 0;
					dis = 0;
				}

				if(wave_state == STATE_DOWN_LOW)										//从下降运动的下半部波形直接回到水平(骤停) 或者附带轻微反向运动
				{
					wave_state = STATE_STATIC;
					if(valley_min < 8)												//骤停加速度较大 此处以8为准
					{
						dis = valley_dis;												//运动的位移为在波谷时记录的位移
						
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
					
					valley_flag = 0;													//波形重新回到水平时 该标志清零  等待下一次首次出现下降的前下半部分波形置一
					speed = 0;
					dis = 0;	
				}
			}
		}
		
		if(accel_z < 9.65 || accel_z > 9.85)											//如果accel_z超出重力加速度的±0.2   则可认为有新数据(物体开始运动)
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
				if(accel_z > 10)														//判断up_hight时  峰值 是否达到正常波形的峰值
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
				
				if(accel_z < 9.5)														//判断up_low时  峰值 是否达到正常波形的谷值
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
				
				if(valley_flag == 1)													//第一次给min赋值后置为1
				{		
					if(accel_z < valley_min)												
					{		
						valley_min = accel_z;											//记录加速度最小值
						valley_dis = dis;												//更新当前dis为valley_dis
					}
				}
				
				if(accel_z < 9.5)														//判断down_low时  峰值 是否达到正常波形的谷值
					fifter_flag = STATE_DOWN_LOW;
			}
			
			if(wave_state == STATE_DOWN_HIGHT)
			{
				if(speed > 0) 
					speed = 0;
				
				if(accel_z > down_peak)													//判断是否为正常的下降运动								
					isDown = 1;		
				
				if(accel_z > 10)														//判断down_hight时  峰值 是否达到正常波形的峰值
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

//	/*四元数解姿态*/
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
//		printf("add 1,0,%d",(int)(accel_x*10));											//串口屏显示波形  速度  位移 
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
//	if((accel_z > 9.65) && (accel_z < 9.85))										//更新accel_offset
//	{			
//		
//		if((accel_z > (last_accel_z-0.05)) && (accel_z < (last_accel_z+0.05)))		//每当accel的数值和上一个数值相差±0.05计数加一  当计数连续达到十次  更新平均值为新的accel_offset
//		{
//			offset_count++;
//			offset_update_z += accel_z;
//		}
//		else																		//当不满足静止条件  计数/offset清零
//		{
//			offset_count = 0;
//			offset_update_z = 0;
//		}
//		
//		if(wave_state == STATE_UP_LOW)												//当检测到上升动作结束   在状态2回到水平
//		{
//			
//			if(fifter_flag == STATE_UP_LOW)											//判断是否完整波形
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
//		if(wave_state == STATE_DOWN_HIGHT)											//当检测到下降动作结束   在状态3回到水平
//		{
//		
//			if(isDown == 0)																	
//			{
//				if(valley_min < 8)
//					dis = valley_dis;												//运动的位移为在波谷时记录的位移
//				valley_flag = 0;
//			}
//			else
//			{
//				isDown = 0;															//清除判断是否为下降运动的标志
//			}
//			
//			if(fifter_flag == STATE_DOWN_HIGHT)										//判断是否完整波形
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

//			if(wave_state == STATE_UP_HIGHT)										//不构成波形
//			{
//				wave_state = STATE_STATIC;
//				speed = 0;
//				dis = 0;
//			}

//			if(wave_state == STATE_DOWN_LOW)										//从下降运动的下半部波形直接回到水平(骤停) 或者附带轻微反向运动
//			{
//				wave_state = STATE_STATIC;
//				if(valley_min < 8)													//骤停加速度较大 此处以8为准
//				{
//					dis = valley_dis;												//运动的位移为在波谷时记录的位移
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
//				valley_flag = 0;													//波形重新回到水平时 该标志清零  等待下一次首次出现下降的前下半部分波形置一
//				speed = 0;
//				dis = 0;	
//			}
//		}
//	}
//	
//	if(accel_z < 9.65 || accel_z > 9.85)											//如果accel_z超出重力加速度的±0.2   则可认为有新数据(物体开始运动)
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
//			if(accel_z > 10)														//判断up_hight时  峰值 是否达到正常波形的峰值
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
//			if(accel_z < 9.5)														//判断up_low时  峰值 是否达到正常波形的谷值
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
//			if(valley_flag == 1)													//第一次给min赋值后置为1
//			{		
//				if(accel_z < valley_min)												
//				{		
//					valley_min = accel_z;											//记录加速度最小值
//					valley_dis = dis;												//更新当前dis为valley_dis
//				}
//			}
//			
//			if(accel_z < 9.5)														//判断down_low时  峰值 是否达到正常波形的谷值
//				fifter_flag = STATE_DOWN_LOW;
//		}
//		
//		if(wave_state == STATE_DOWN_HIGHT)
//		{
//			if(speed > 0) 
//				speed = 0;
//			
//			if(accel_z > down_peak)													//判断是否为正常的下降运动								
//				isDown = 1;		
//			
//			if(accel_z > 10)														//判断down_hight时  峰值 是否达到正常波形的峰值
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
//	if((accel_x > 9.70) && (accel_x < 9.90))										//更新accel_offset
//	{			
//		
//		if((accel_x > (last_accel_x-0.05)) && (accel_x < (last_accel_x+0.05)))		//每当accel的数值和上一个数值相差±0.05计数加一  当计数连续达到十次  更新平均值为新的accel_offset
//		{
//			offset_count++;
//			offset_update_x += accel_x;
//		}
//		else																		//当不满足静止条件  计数/offset清零
//		{
//			offset_count = 0;
//			offset_update_x = 0;
//		}
//		
//		if(wave_state == STATE_RIGHT_LOW)												//当检测到上升动作结束   在状态2回到水平
//		{
//			if(fifter_flag == STATE_RIGHT_LOW)											//判断是否完整波形
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
//		if(wave_state == STATE_LEFT_HIGHT)											//当检测到下降动作结束   在状态3回到水平
//		{
//			if(fifter_flag == STATE_LEFT_HIGHT)										//判断是否完整波形
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
//			if(wave_state == STATE_RIGHT_HIGHT)										//不构成波形
//			{
//				speed = 0;
//				dis = 0;
//			}

//			if(wave_state == STATE_LEFT_LOW)										//不构成波形
//			{
//				speed = 0;
//				dis = 0;	
//			}
//		}
//	}
//	
//	if(accel_x < 9.70 || accel_x > 9.90)											//如果accel_x超出重力加速度的±0.2   则可认为有新数据(物体开始运动)
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
//			if(accel_x > 10.5)														//判断right_hight时  峰值 是否达到正常波形的峰值
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
//			if(accel_x < 9.55)														//判断right_low时  峰值 是否达到正常波形的谷值
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

//			if(accel_x < 9.55)														//判断down_low时  峰值 是否达到正常波形的谷值
//				fifter_flag = STATE_LEFT_LOW;
//		}
//		
//		if(wave_state == STATE_LEFT_HIGHT)
//		{
//			if(speed > 0) 
//				speed = 0;

//			if(accel_x > 10.5)														//判断down_hight时  峰值 是否达到正常波形的峰值
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
