#ifndef __BICYCLE_H__
#define __BICYCLE_H__

#include "stm32f10x.h"
#include "stdint.h"

#define STATIC_HIGHT  	   37																			//定义自行车静止时激光所测高度  单位cm
																										
#define STATIC_LIFT_HIGHT  10																			//定义自行车被抬起的临界高度  单位cm
#define LIFT_TIME          10
																										
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

#endif
