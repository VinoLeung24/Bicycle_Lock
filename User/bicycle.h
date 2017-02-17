#ifndef __BICYCLE_H__
#define __BICYCLE_H__

#include "stm32f10x.h"
#include "stdint.h"

#define STATIC_HIGHT  35																				//�������г���ֹʱ��������߶�  ��λcm
																										
#define STATIC_LIFT_HIGHT  10																			//�������г���̧����ٽ�߶�  ��λcm
																										
#define FALL_RIGHT_ANGLE  65																			//�������ұߵ���ʱ���ٽ�Ƕ�
#define FALL_LEFT_ANGLE   -50																			//��������ߵ���ʱ���ٽ�Ƕ�
																										
#define LIFT_BEHIND_ANGLE   8																			//�������̧��ʱ���ٽ�Ƕ�

#define STATIC_ANGLE_DEMAND		(((Pitch > -25) && (Pitch < -15)))										//���徲ֹʱ�ĽǶ�����
#define STATIC_ACCEL_DEMAND		(((accel_x > 9.55) && (accel_x < 10.05)) && ((accel_y > 9.55) && (accel_y < 10.05)) && ((accel_z > 9.55) && (accel_z < 10.05)))

#define FALL_ANGLE_DEMAND		( Pitch > FALL_RIGHT_ANGLE || Pitch < FALL_LEFT_ANGLE)					//���嵹��ʱ�ĽǶ�����
#define FALL_ACCEL_DEMAND		(((accel_x > 9.55) && (accel_x < 10.05)) && ((accel_y > 9.55) && (accel_y < 10.05)) && ((accel_z > 9.55) && (accel_z < 10.05)))

#define MOVE_DEMAND 			(((accel_x < 9.55) || (accel_x > 10.05)) || ((accel_y < 9.55) || (accel_y > 10.05)) || ((accel_z < 9.55) || (accel_z > 10.05)))
#define STATIC_STAND 0
typedef struct 
{
	enum STATE
	{
		STATE_STAND_STATIC,
		STATE_FALL,
		STATE_DANGER,
	};

	uint8_t bicycle_state;
	
}BICYCLE;

#endif
