#ifndef __TIME_H__
#define __TIME_H__

#include "sys.h"


void SYSTICK_INIT(void);
void TIM2_Configuration(void);
void TIM3_Int_Init(u16 arr,u16 psc);

#endif

