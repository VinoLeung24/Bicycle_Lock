/******************************************************************************
** 工程：   	STM32-MPU9150驱动
** 作者：	    vino
** 修改日志：
2016-9-20
计算z分量



********************************************************************************/


#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "I2C.h"
#include "mpu9250.h"
#include "time.h"

void led(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
}

void delay_1us(void)
{
	uint16_t i;
	i = 3;
	while(i--);
}

void delay_2us(void)
{
	uint16_t i;
	i = 14;
	while(i--);
}

void delay_1us1(u16 time) 
{ 
	u16 i=0;      
	while(time--)    
	{        
		i=2;         
		while(i--);         
	} 
}

int main(void)
{ 
//	delay_init();	    						//延时函数初始化	
	USARTx_Config();	
	I2C_Config();									//I2C初始化//SDA-PC11   SCL-PC12	 
	init_mpu();
//	delay_ms(2000);
	TIM2_Configuration();
	get_accel_bias();

	while(1)
	{
		
		get_accel();

	}
}
