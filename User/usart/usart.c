/**
  ******************************************************************************
  * @file    bsp_usart1.c
  * @author  
  * @version 
  * @date    2013-xx-xx
  * @brief   usart
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */
  
#include "usart.h"
#include "bicycle.h"

uint8_t SendSpeed[SENDBUFF_SIZE]={0xAA,0x00,0x01,0x55,'s','1',DATA,0,0};
uint8_t SendFreq[FREQ_SIZE]={0xAA,0x00,0x01,0x55,'s','1',CMD,0,};
uint8_t SendAlarm[ALARM_SIZE]={0xAA,0x00,0x02,0x55,'{',0,',',0,'}'};


void USART1_DMA_Config(void)
{
		DMA_InitTypeDef DMA_InitStructure;
	
		/*开启DMA时钟*/  
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

		/*设置DMA源：串口数据寄存器地址*/
//		DMA_InitStructure.DMA_PeripheralBaseAddr = USART1_DR_Base;	  
		DMA_InitStructure.DMA_PeripheralBaseAddr = ( u32 ) ( & ( USART1->DR ) );	

		/*内存地址(要传输的变量的指针)*/
		DMA_InitStructure.DMA_MemoryBaseAddr = (u32)SendSpeed;

		/*方向：从内存到外设*/		
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;	

		/*传输大小DMA_BufferSize=SENDBUFF_SIZE*/	
		DMA_InitStructure.DMA_BufferSize = SENDBUFF_SIZE;

		/*外设地址不增*/	    
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; 

		/*内存地址自增*/
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;	

		/*外设数据单位*/	
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;

		/*内存数据单位 8bit*/
		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;	 

		/*DMA模式：不断循环*/
		DMA_InitStructure.DMA_Mode = DMA_Mode_Normal ;
//		DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;	 

		/*优先级：中*/	
		DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;  

		/*禁止内存到内存的传输	*/
		DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

		/*配置DMA1的4通道*/		   
		DMA_Init(DMA1_Channel4, &DMA_InitStructure); 	   
		
		/*使能DMA*/
		DMA_Cmd (DMA1_Channel4,DISABLE);					
		//DMA_ITConfig(DMA1_Channel4,DMA_IT_TC,ENABLE);  //配置DMA发送完成后产生中断
}

void NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure; 
	/* Configure the NVIC Preemption Priority Bits */  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	
	/* Enable the USARTy Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;	 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void NVIC_Configuration_2(void)
{
	NVIC_InitTypeDef NVIC_InitStructure; 
	/* Configure the NVIC Preemption Priority Bits */  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	
	/* Enable the USARTy Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;	 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

 /**
  * @brief  USART1 GPIO 配置,工作模式配置。115200 8-N-1
  * @param  无
  * @retval 无
  */
void USARTx_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	/* config USART1 clock */
	macUSART_APBxClock_FUN(macUSART_CLK, ENABLE);
	macUSART_GPIO_APBxClock_FUN(macUSART_GPIO_CLK, ENABLE);
	
	/* USART1 GPIO config */
	/* Configure USART1 Tx (PA.09) as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin =  macUSART_TX_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(macUSART_TX_PORT, &GPIO_InitStructure);    
	/* Configure USART1 Rx (PA.10) as input floating */
	GPIO_InitStructure.GPIO_Pin = macUSART_RX_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(macUSART_RX_PORT, &GPIO_InitStructure);
	
	/* USART1 mode config */
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(macUSARTx, &USART_InitStructure);
	
	/* 使能串口1接收中断 */
	NVIC_Configuration();
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	
	USART_Cmd(macUSARTx, ENABLE);
}

void USART2_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	/* config USART2 clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_APB2PeriphClockCmd((RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO) , ENABLE);

	/* USART2 GPIO config */
	 /* Configure USART2 Tx (PA.02) as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	    
	/* Configure USART2 Rx (PA.03) as input floating */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	  
	/* USART2 mode config */
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(USART2, &USART_InitStructure); 
	
	/* 使能串口1接收中断 */
	NVIC_Configuration_2();
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	
	USART_Cmd(USART2, ENABLE);
	USART_ClearFlag(USART2, USART_FLAG_TC);
}


/// 重定向c库函数printf到USART1
int fputc(int ch, FILE *f)
{
		/* 发送一个字节数据到USART1 */
		USART_SendData(USART1, (uint8_t) ch);
		
		/* 等待发送完毕 */
		while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);		
	
		return (ch);
}

/// 重定向c库函数scanf到USART1
int fgetc(FILE *f)
{
		/* 等待串口1输入数据 */
		while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET);

		return (int)USART_ReceiveData(USART1);
}

void Usart_SendByte( USART_TypeDef * pUSARTx, uint8_t ch)
{
	/* 发送一个字节数据到USART1 */
	USART_SendData(pUSARTx,ch);
		
	/* 等待发送完毕 */
	while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);	
}

void Usart_SendStr( USART_TypeDef * pUSARTx, uint8_t *str )
{
	unsigned int k=0;
    do 
    {
        Usart_SendByte( pUSARTx, *(str + k) );
        k++;
    }while(*(str + k) != '\0');
}

/*****************  指定长度的发送字符串 **********************/
void Usart_SendStr_length( USART_TypeDef * pUSARTx, uint8_t *str,uint32_t strlen )
{
	unsigned int k=0;
    do 
    {
        Usart_SendByte( pUSARTx, *(str + k) );
        k++;
    } while(k < strlen);
}

void reSendSpeed(void)
{
	DMA1_Channel4->CCR&=~(1<<0);  				//关闭DMA传输 
	DMA1_Channel4->CMAR = (u32)SendSpeed;		//要传输的数据
	DMA1_Channel4->CNDTR=SENDBUFF_SIZE;  		//DMA1,传输数据量
	DMA1_Channel4->CCR|=1<<0;  					//开启DMA传输
}

//void SendWarning(uint8_t state, uint8_t cmd)
//{
//	SendAlarm[5] = state + '0';
//	SendAlarm[7] = cmd + '0';
//	DMA1_Channel4->CCR&=~(1<<0);  				//关闭DMA传输 
//	DMA1_Channel4->CMAR=(u32)SendAlarm;		//要传输的数据
//	DMA1_Channel4->CNDTR=ALARM_SIZE;  		//DMA1,传输数据量
//	DMA1_Channel4->CCR|=1<<0;  					//开启DMA传输
//}

void SendWarning(uint8_t state, uint8_t cmd)
{
//	Usart_SendByte(USART2,0xAA);
//	Usart_SendByte(USART2,0x00);
//	Usart_SendByte(USART2,0x02);
//	Usart_SendByte(USART2,0x55);
	Usart_SendByte(USART2,'{');
	Usart_SendByte(USART2,state + '0');
	Usart_SendByte(USART2,',');
	Usart_SendByte(USART2,cmd + '0');
	Usart_SendByte(USART2,'}');
}

void reSendFreq(void)
{
	DMA1_Channel4->CCR&=~(1<<0);  				//关闭DMA传输 
	DMA1_Channel4->CMAR = (u32)SendFreq;		//要传输的数据
	DMA1_Channel4->CNDTR=FREQ_SIZE;  			//DMA1,传输数据量
	DMA1_Channel4->CCR|=1<<0;  					//开启DMA传输
}



/*********************************************END OF FILE**********************/
