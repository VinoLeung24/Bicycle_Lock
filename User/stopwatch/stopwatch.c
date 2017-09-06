#include "stopwatch.h"

void StopwatchIO_Init(void)
{
 GPIO_InitTypeDef  GPIO_InitStructure;
 	
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);//使能PA端口时钟
	
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;				 //PA15端口配置
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 		 //带上拉输入
 GPIO_Init(GPIOA, &GPIO_InitStructure);					 //根据设定参数初始化GPIOB.5
 GPIO_SetBits(GPIOA,GPIO_Pin_15);						 //PA.15 输出高
 
}



float StopwatchGetVelocity(void)
{
	   static float  velocity = 0;
	   static int InputDownFlag = 1;
	   static float last_stopwatchTime = 0;
	   float OneRoundTime = 0;
	
		 OneRoundTime = stopwatchTime -last_stopwatchTime;
	 
		 if(OneRoundTime > 800)
		 {
			 velocity = 0;
		 }

	   if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_15) == 1)
		 {
				InputDownFlag = 1;
     }
		 if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_15) == 0)
		 {
         
			   if(InputDownFlag == 1)
				 {
					 InputDownFlag = 0;
					 
					 {
							velocity = 2.05/(OneRoundTime*5/1000);
					 }
					 
					 last_stopwatchTime = stopwatchTime;
					 //printf("OneRoundTime = %f\r\n",OneRoundTime);
				 }
		 }
		 

		 
		 return velocity;
}
