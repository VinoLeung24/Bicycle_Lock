#include "delay.h"


/************************************************
 ������:     delay_us
 ��������:	  ��ʱn΢��
 �β�:		  ��ʱʱ�� 
 ���ز���:	  ��
************************************************/
void delay_us(uint32_t nus)
{
	uint32_t temp;
	SysTick->LOAD=nus*(SystemCoreClock/8000000);
	SysTick->VAL=0x00;
	SysTick->CTRL=0x01;
	do
	{
		temp=SysTick->CTRL;
	}
	while((temp&0x01)&&!(temp&(1<<16)));
	SysTick->CTRL=0x00;
	SysTick->VAL=0x00;
}

/************************************************
 ������:     delay_ms
 ��������:	  ��ʱn����,ϵͳ��ʱΪ24λ �������ʱΪ 1.8s
 �β�:		  ��ʱʱ�� 
 ���ز���:	  ��
************************************************/
void delay_ms(uint16_t nms)
{
	uint32_t temp;
	SysTick->LOAD=nms*(SystemCoreClock/8000);
	SysTick->VAL=0X00;
	SysTick->CTRL=0X01;
	do
	{
	 temp=SysTick->CTRL;
	}
	while((temp&0x01)&&!(temp&(1<<16)));
	SysTick->CTRL=0X00;
	SysTick->VAL=0x00;
}


void mget_ms(unsigned long *time)
{

}




































