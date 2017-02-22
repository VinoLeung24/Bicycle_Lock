#ifndef __BICYCLE_H__
#define __BICYCLE_H__

#include "stm32f10x.h"
#include "stdint.h"

#define STATIC_HIGHT  	   37																			//�������г���ֹʱ��������߶�  ��λcm
																										
#define STATIC_LIFT_HIGHT  10																			//�������г���̧����ٽ�߶�  ��λcm
#define LIFT_TIME          10
																										
#define FALL_RIGHT_ANGLE  -60																			//�������ұߵ���ʱ���ٽ�Ƕ�
#define FALL_LEFT_ANGLE    60																			//��������ߵ���ʱ���ٽ�Ƕ�

#define STATIC_ANGLE_DEMAND		(((Pitch > -30) && (Pitch < 30)))										//���徲ֹʱ�ĽǶ�����
#define STATIC_ACCEL_DEMAND		(((accel_x > 9.55) && (accel_x < 10.05)) && ((accel_y > 9.65) && (accel_y < 10.05)) && ((accel_z > 9.55) && (accel_z < 10.05)))

#define FALL_ANGLE_DEMAND		( Pitch < FALL_RIGHT_ANGLE || Pitch > FALL_LEFT_ANGLE)					//���嵹��ʱ�ĽǶ�����
#define FALL_ACCEL_DEMAND		(((accel_x > 9.55) && (accel_x < 10.05)) && ((accel_y > 9.55) && (accel_y < 10.05)) && ((accel_z > 9.55) && (accel_z < 10.05)))

#define SHAKE_DEMAND 			(((accel_x < 9.55) || (accel_x > 10.05)) || ((accel_y < 9.55) || (accel_y > 10.05)))

typedef struct 
{
	enum STATE
	{
		STATE_STAND_STATIC,
		STATE_FALL,
		STATE_LIFT,
		STATE_SHAKE,
	};

	uint8_t bicycle_state;
	uint8_t last_state;
	
}BICYCLE;

#endif
