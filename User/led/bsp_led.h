#ifndef __LED_H
#define	__LED_H


#include "stm32f10x.h"


/* 直接操作寄存器的方法控制IO */
#define	digitalHi(p,i)			{p->BSRR=i;}			//设置为高电平		
#define digitalLo(p,i)			{p->BRR=i;}				//输出低电平
#define digitalToggle(p,i)		{p->ODR ^=i;}			//输出反转状态

#define Buzzer_OFF	GPIOB->BRR  |= 0X4000;
#define Buzzer_ON 	GPIOB->BSRR |= 0X4000;

void LED_GPIO_Config(void);
void buzzer_init(void);
void Shake_GPIO_Config(void);
uint8_t isShake(void);

#endif /* __LED_H */
