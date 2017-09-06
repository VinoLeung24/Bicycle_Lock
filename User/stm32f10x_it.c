/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTI
  
  AL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "usart.h"
#include "bicycle.h"
#include "bsp_led.h"
#include "lock.h"

extern BICYCLE mBICYCLE;

extern uint8_t  Time_Stamp;
extern uint16_t  uSec_Count;
extern uint8_t Sec_Count;

extern uint16_t u_Count_Lift;
extern uint8_t s_Count_Lift;

extern uint16_t u_Count_Shake;
extern uint8_t s_Count_Shake;

extern uint8_t new_lift;
extern uint8_t new_shake;

/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
	
}

extern uint8_t alarm;
//extern uint8_t speed_send;
//uint8_t freq_send = 0;
//extern uint8_t SendSpeed[SENDBUFF_SIZE];

void USART2_IRQHandler(void)
{
	uint8_t data_rx;

	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{
		data_rx = USART_ReceiveData(USART2);
		if(data_rx == 'c')											//清除情报标志
		{
			alarm = 0;
			s_Count_Shake = 0;
			s_Count_Lift = 0;
			Buzzer_OFF;
		}
		
		if(data_rx == 'o')											//清除情报标志
		{
			UnLock();
			
		}
	
		if(data_rx == 'u')											//清除情报标志
		{
			Lock();
		}
	}
}


//void USART1_IRQHandler(void)
//{
//	static uint8_t model_flag;
//	uint8_t data_rx;

//	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
//	{
//		data_rx = USART_ReceiveData(USART1);

//		if(model_flag)
//		{
//			model_flag = 0;
//			switch(data_rx)
//			{
//				case '0':													//返回主界面 初始化  关闭led指示灯  关闭tim3 (心率)															
//					freq_send = 0;
//					speed_send = 0;
//					SendSpeed[7] = 0;										//速度清零
//				    SendSpeed[8] = 0;
//				break;
//				
//				case '1':													//自由模式 打开tim3(心率)	//计时清零	
//					freq_send = 1;
//					speed_send = 1;
//				break;
//				
//				case '2':													//自由模式  (平路模式) 打开tim3(心率)	
//					freq_send = 1;
//					speed_send = 1;
//				break;
//				
//				case '3':													//自由模式 (爬坡模式) 打开tim3(心率)	
//					freq_send = 1;
//					speed_send = 1;
//				break;
//				
//				case '4':													//自由模式 (热身模式) 关闭tim3(心率)
//					freq_send = 0;
//					speed_send = 0;
//					SendSpeed[7] = 0;										//速度清零
//				    SendSpeed[8] = 0;
//				break;
//				
//				default:
//				break;
//			}
//		}
//		
//		if(data_rx == 'm')
//			model_flag = 1;
//		
//		
//	} 
//}

void TIM2_IRQHandler(void)
{
	if ( TIM_GetITStatus(TIM2 , TIM_IT_Update) != RESET ) 
	{
		Time_Stamp++;
		
		uSec_Count++;
		if(uSec_Count == 10000)
		{
			Sec_Count++;
			uSec_Count = 0;
		}
		
		if(mBICYCLE.bicycle_state == STATE_LIFT)
		{
			if(new_lift == 0)
			{
				new_lift = 1;
				s_Count_Lift = 0;
				u_Count_Lift = 0;
			}
			
			if(alarm == 0)
			{
				u_Count_Lift++;
				if(u_Count_Lift == 10000)
				{
					s_Count_Lift++;
					u_Count_Lift = 0;
				}
			}
			
		}
		
		if(mBICYCLE.bicycle_state == STATE_SHAKE)
		{
			if(new_shake == 0)
			{
				new_shake = 1;
				s_Count_Shake = 0;
				u_Count_Shake = 0;
			}
			
			if(alarm == 0)
			{
				u_Count_Shake++;
				if(u_Count_Shake == 10000)
				{
					s_Count_Shake++;
					u_Count_Shake = 0;
				}
			}
		}

		
		TIM_ClearITPendingBit(TIM2 , TIM_FLAG_Update);  		 
	}		 	
}

//extern float stopwatchTime;                              //主要用于码表的计时
////定时器3中断服务程序                                
//void TIM3_IRQHandler(void)   //TIM3中断
//{
//	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)  //检查TIM3更新中断发生与否
//	{
//		TIM_ClearITPendingBit(TIM3, TIM_IT_Update  );  //清除TIMx更新中断标志 
//		
//		stopwatchTime = stopwatchTime + 1;
//		
//	}
//}

//extern uint8_t SendFreq[FREQ_SIZE];

//void EXTI15_10_IRQHandler(void)
//{
//	static uint8_t freq = 0;
//	if(EXTI_GetITStatus(EXTI_Line12) != RESET) //确保是否产生了EXTI Line中断
//	{
//		if(freq_send)
//		{
//			SendFreq[7] = 'l';
//			if(freq == 1)
//				reSendFreq();
//			freq = 0;
//		}
//		
//		EXTI_ClearITPendingBit(EXTI_Line12);     //清除中断标志位
//	} 
//	
//	if(EXTI_GetITStatus(EXTI_Line13) != RESET) //确保是否产生了EXTI Line中断
//	{
//		if(freq_send)
//		{
//			SendFreq[7] = 'r';
//			if(freq == 0)
//				reSendFreq();
//			freq = 1;
//		}
//		
//		EXTI_ClearITPendingBit(EXTI_Line13);     //清除中断标志位
//	} 
//}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
