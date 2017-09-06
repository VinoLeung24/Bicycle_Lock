#ifndef __BICYCLE_H__
#define __BICYCLE_H__

#include "stm32f10x.h"
#include "stdint.h"

#define NORMAL_MODE			0
#define ALARM_MODE			1

#define noALARM				0
#define ALARM				1

#define STATIC_HIGHT  	   37																			//定义自行车静止时激光所测高度  单位cm
																										
#define STATIC_LIFT_HIGHT  10																			//定义自行车被抬起的临界高度  单位cm
#define LIFT_TIME           8

#define SHAKE_TIME		   10

#define FALL_RIGHT_ANGLE  -60																			//定义向右边倒下时的临界角度
#define FALL_LEFT_ANGLE    60																			//定义向左边倒下时的临界角度

#define STATIC_ANGLE_DEMAND		(((Pitch > -30) && (Pitch < 30)))										//定义静止时的角度条件
#define STATIC_ACCEL_DEMAND		(((accel_x > 9.55) && (accel_x < 10.05)) && ((accel_y > 9.65) && (accel_y < 10.05)) && ((accel_z > 9.55) && (accel_z < 10.05)))

#define FALL_ANGLE_DEMAND		( Pitch < FALL_RIGHT_ANGLE || Pitch > FALL_LEFT_ANGLE)					//定义倒下时的角度条件
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
	STATE_LIFT_TO_STATIC,		//多次小范围移动 	--轻度警告
	STATE_SHAKE_TO_STATIC,		//持续多次震动   	--轻度警告
	STATE_FALDOWN,				//自行车倒下			--提醒
	STATE_LIFT_BEYOND_HEIGHT,	//被抬高超过50cm 	--严重警告
	STATE_LIFT_BEYOND_TIME,		//长时间处于抬高状态	--严重警告
	STATE_SHAKE_BEYOND_TIME,	//长时间处于震动状态	--严重警告
	STATE_DESTORY_LOCK,			//撬锁
};

#endif
