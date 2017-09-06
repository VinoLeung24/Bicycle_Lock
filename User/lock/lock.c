#include "lock.h"
#include "delay.h"
#include "bicycle.h"
#include "usart.h"

uint8_t mode_select = 0;

//LOCK��IO�ڳ�ʼ��
void LOCK_Init(void)
{
 
	GPIO_InitTypeDef  GPIO_InitStructure;
 	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	 //ʹ��PB,PE�˿�ʱ��
	
	
	//PB0
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;				 //LED0-->PB.0 �˿�����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
	GPIO_Init(GPIOB, &GPIO_InitStructure);					 //�����趨������ʼ��GPIOB.0
	GPIO_ResetBits(GPIOB,GPIO_Pin_0);						 //PB.0 �����
 
	//PB1
 	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;				 //LED0-->PB.1 �˿�����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
	GPIO_Init(GPIOB, &GPIO_InitStructure);					 //�����趨������ʼ��GPIOB.1
	GPIO_ResetBits(GPIOB,GPIO_Pin_1);						 //PB.1 �����
 
	//PB10
 	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;				 //LED0-->PB.10 �˿�����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
	GPIO_Init(GPIOB, &GPIO_InitStructure);					 //�����趨������ʼ��GPIOB.10
	GPIO_ResetBits(GPIOB,GPIO_Pin_10);						 //PB.10 �����
 
	//PB11
 	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;				 //LED0-->PB.11 �˿�����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
	GPIO_Init(GPIOB, &GPIO_InitStructure);					 //�����趨������ʼ��GPIOB.11
	GPIO_ResetBits(GPIOB,GPIO_Pin_11);						 //PB.5 �����
 
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

//��������
void Lock(void)
{
  //�����������
	lockLineA_H();
	lockLineB_L();
	
	delay_ms(1000);
	delay_ms(1000);
	delay_ms(1000);
   
	//�������
	lockLineA_L();
	lockLineB_L();
	
	
	mode_select = ALARM_MODE;
	Usart_SendByte(USART2,'l');

}

//��������
void UnLock(void)
{
	//�����������
	lockLineA_L();
	lockLineB_H();

	mode_select = NORMAL_MODE;
	Usart_SendByte(USART2,'u');
	
}
