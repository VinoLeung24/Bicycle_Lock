#include "lock.h"
#include "delay.h"
#include "bicycle.h"
#include "usart.h"

uint8_t mode_select = 0;

//LOCK的IO口初始化
void LOCK_Init(void)
{
 
	GPIO_InitTypeDef  GPIO_InitStructure;
 	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	 //使能PB,PE端口时钟
	
	
	//PB0
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;				 //LED0-->PB.0 端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
	GPIO_Init(GPIOB, &GPIO_InitStructure);					 //根据设定参数初始化GPIOB.0
	GPIO_ResetBits(GPIOB,GPIO_Pin_0);						 //PB.0 输出低
 
	//PB1
 	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;				 //LED0-->PB.1 端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
	GPIO_Init(GPIOB, &GPIO_InitStructure);					 //根据设定参数初始化GPIOB.1
	GPIO_ResetBits(GPIOB,GPIO_Pin_1);						 //PB.1 输出低
 
	//PB10
 	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;				 //LED0-->PB.10 端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
	GPIO_Init(GPIOB, &GPIO_InitStructure);					 //根据设定参数初始化GPIOB.10
	GPIO_ResetBits(GPIOB,GPIO_Pin_10);						 //PB.10 输出低
 
	//PB11
 	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;				 //LED0-->PB.11 端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
	GPIO_Init(GPIOB, &GPIO_InitStructure);					 //根据设定参数初始化GPIOB.11
	GPIO_ResetBits(GPIOB,GPIO_Pin_11);						 //PB.5 输出低
 
}


void lockLineA_H(void)
{
	GPIO_SetBits(GPIOB,GPIO_Pin_1);
	GPIO_SetBits(GPIOB,GPIO_Pin_10);
}
void lockLineA_L(void)
{
	GPIO_ResetBits(GPIOB,GPIO_Pin_1);
	GPIO_ResetBits(GPIOB,GPIO_Pin_10);
}



void lockLineB_H(void)
{
	GPIO_SetBits(GPIOB,GPIO_Pin_0);
	GPIO_SetBits(GPIOB,GPIO_Pin_11);
}
void lockLineB_L(void)
{
	GPIO_ResetBits(GPIOB,GPIO_Pin_0);
	GPIO_ResetBits(GPIOB,GPIO_Pin_11);
}

//关锁函数
void Lock(void)
{
  //电机上锁动作
	lockLineA_H();
	lockLineB_L();
	
	delay_ms(1000);
	delay_ms(1000);
	delay_ms(1000);
   
	//电机不动
	lockLineA_L();
	lockLineB_L();
	
	
	mode_select = ALARM_MODE;
	Usart_SendByte(USART2,'l');

}

//开锁函数
void UnLock(void)
{
	//电机开锁动作
	lockLineA_L();
	lockLineB_H();

	mode_select = NORMAL_MODE;
	Usart_SendByte(USART2,'u');
	
}
