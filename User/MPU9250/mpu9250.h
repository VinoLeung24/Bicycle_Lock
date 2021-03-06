#ifndef __MPU9250_H__
#define __MPU9250_H__

#include "stm32f10x.h"

#define q30  1073741824.0f

//������
#define  Pitch_error  1.0
#define  Roll_error   -2.0
#define  Yaw_error    0.0

#define PRINT_ACCEL     (0x01)
#define PRINT_GYRO      (0x02)
#define PRINT_QUAT      (0x04)

#define ACCEL_ON        (0x01)
#define GYRO_ON         (0x02)

#define MOTION          (0)
#define NO_MOTION       (1)


#define FLASH_SIZE      (512)
#define FLASH_MEM_START ((void*)0x1800)

#define DEFAULT_MPU_HZ  (200)

static signed char gyro_orientation[9] = {1, 0, 0,
                                           0,1, 0,
                                           0, 0, 1};

enum WAVE_Z
{
	STATE_STATIC,
	STATE_UP_HIGHT,
	STATE_UP_LOW,
	STATE_DOWN_HIGHT,
	STATE_DOWN_LOW,
};

enum WAVE_X
{
	STATE_STATIC_X,
	STATE_RIGHT_HIGHT,
	STATE_RIGHT_LOW,
	STATE_LEFT_HIGHT,
	STATE_LEFT_LOW,
};

void init_mpu(void);
float get_accel(void);	
void get_accel_bias(void);
void get_accel_xyz(void);
void get_dis_z(float accel_z);
void get_dis_x(float accel_x);
#endif
