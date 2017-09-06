#ifndef __BICYCLE_H__
#define __BICYCLE_H__

#include "stm32f10x.h"
#include "stdint.h"

#define NORMAL_MODE			0
#define ALARM_MODE			1

#define noALARM				0
#define ALARM				1

#define STATIC_HIGHT  	   37																			//�������г���ֹʱ��������߶�  ��λcm
																										
#define STATIC_LIFT_HIGHT  10																			//�������г���̧����ٽ�߶�  ��λcm
#define LIFT_TIME           8

#define SHAKE_TIME		   10

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

typedef struct 
{
	enum state{
        STATE_START_MODE,
		STATE_FREEDOM_MODE,
		STATE_SPORT_MODE,
		STATE_SPORT_MODE_WARM1,
		STATE_SPORT_MODE_WARM2,
		STATE_SPORT_MODE_WARM3,
		STATE_SPORT_MODE_LEVEL,
		STATE_SPORT_MODE_CLIMB,
		STATE_LESSON_MODE,
		STATE_LESSON_MODE_WARM1,
		STATE_LESSON_MODE_WARM2,
		STATE_LESSON_MODE_WARM3,
		STATE_LESSON_MODE_LEVEL,
		STATE_LESSON_MODE_CLIMB,
        STATE_OUTDOOR_MODE,
		STATE_USER_REMENBER_DATA,
		STATE_USER_CHECK,
		STATE_CALIBRATION,
	};
	uint8_t outer_status;
}MODE;

enum STATE_CHECK
{
	NONE,
	CMD,
	DATA,
	
};

enum BICYCLE_STATE
{
	STATE_NONE,
	STATE_STATIC_NORMAL,
	STATE_LIFT_TO_STATIC,		//���С��Χ�ƶ� 	--��Ⱦ���
	STATE_SHAKE_TO_STATIC,		//���������   	--��Ⱦ���
	STATE_FALDOWN,				//���г�����			--����
	STATE_LIFT_BEYOND_HEIGHT,	//��̧�߳���50cm 	--���ؾ���
	STATE_LIFT_BEYOND_TIME,		//��ʱ�䴦��̧��״̬	--���ؾ���
	STATE_SHAKE_BEYOND_TIME,	//��ʱ�䴦����״̬	--���ؾ���
	STATE_DESTORY_LOCK,			//����
};

#endif
