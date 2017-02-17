#ifndef __LED_H
#define	__LED_H


#include "stm32f10x.h"


/* ֱ�Ӳ����Ĵ����ķ�������IO */
#define	digitalHi(p,i)			{p->BSRR=i;}			//����Ϊ�ߵ�ƽ		
#define digitalLo(p,i)			{p->BRR=i;}				//����͵�ƽ
#define digitalToggle(p,i)		{p->ODR ^=i;}			//�����ת״̬

#define Buzzer_OFF	GPIOB->BRR  |= 0X4000;
#define Buzzer_ON 	GPIOB->BSRR |= 0X4000;

void LED_GPIO_Config(void);
void buzzer_init(void);
void Shake_GPIO_Config(void);
uint8_t isShake(void);

#endif /* __LED_H */
