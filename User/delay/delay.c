#include "delay.h"


/************************************************
 函数名:     delay_us
 函数功能:	  延时n微秒
 形参:		  延时时间 
 返回参数:	  无
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
 函数名:     delay_ms
 函数功能:	  延时n毫秒,系统定时为24位 ，最大延时为 1.8s
 形参:		  延时时间 
 返回参数:	  无
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




































