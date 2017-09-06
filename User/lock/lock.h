#ifndef __LOCK_H
#define __LOCK_H	 
#include "sys.h"

void lockLineA_H(void);
void lockLineA_L(void);
void lockLineB_H(void);
void lockLineB_L(void);
void Lock(void);
void UnLock(void);
void LOCK_Init(void);
		 				    
#endif
