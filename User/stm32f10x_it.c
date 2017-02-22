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

extern BICYCLE mBICYCLE;

extern uint8_t  Time_Stamp;
extern uint16_t  uSec_Count;
extern uint8_t Sec_Count;

extern uint16_t u_Count_Lift;
extern uint16_t s_Count_Lift;

extern uint16_t u_Count_Shake;
extern uint16_t s_Count_Shake;

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

void USART1_IRQHandler(void)
{
	char ch;
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{ 	
		ch	= USART_ReceiveData(USART1);
	  	if(ch == 'C')
			alarm = 0;
	} 
}

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
			
				u_Count_Lift++;
				if(u_Count_Lift == 10000)
				{
					s_Count_Lift++;
					u_Count_Lift = 0;
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
			
			u_Count_Shake++;
			if(u_Count_Shake == 10000)
			{
				s_Count_Shake++;
				u_Count_Shake = 0;
			}
		}

		
		TIM_ClearITPendingBit(TIM2 , TIM_FLAG_Update);  		 
	}		 	
}

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
