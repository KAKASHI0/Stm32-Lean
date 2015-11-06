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
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"

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
extern int16_t count;
uint8_t Flag=0;
s16 encoder_num_1,encoder_num_old_1,speed_1;
s16 encoder_num_2,encoder_num_old_2,speed_2; 

void TIM4_IRQHandler(void)		    //5ms÷–∂œ“ª¥Œ
{	
	
}

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
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
	while(1);
}

void WWDG_IRQHandler(void)
{
while(1);
}
void PVD_IRQHandler(void)
{
while(1);
}
void TAMPER_IRQHandler(void)
{
while(1);
}

void FLASH_IRQHandler(void)
{
while(1);
}
void RCC_IRQHandler(void)
{
while(1);
}
void DMA1_Channel1_IRQHandler(void)
{
while(1);
}
void DMA1_Channel2_IRQHandler(void)
{
while(1);
}
void DMA1_Channel3_IRQHandler(void)
{
while(1);
}

 void DMA1_Channel4_IRQHandler(void)
 {

 }

void DMA1_Channel5_IRQHandler(void)
{
while(1);
}

void DMA1_Channel6_IRQHandler(void)
{
while(1);
}

void DMA1_Channel7_IRQHandler(void)
{
while(1);
}

//void USART1_IRQHandler(void)
//{
//while(1);
//}





// ADC1_2_IRQHandler
// USB_HP_CAN1_TX_IRQHandler
// USB_LP_CAN1_RX0_IRQHandler
// CAN1_RX1_IRQHandler
// CAN1_SCE_IRQHandler
// EXTI9_5_IRQHandler
// TIM1_BRK_IRQHandler
// TIM1_UP_IRQHandler
// TIM1_TRG_COM_IRQHandler
// TIM1_CC_IRQHandler
// TIM2_IRQHandler
// TIM3_IRQHandler
// TIM4_IRQHandler
// I2C1_EV_IRQHandler
// I2C1_ER_IRQHandler
// I2C2_EV_IRQHandler
// // I2C2_ER_IRQHandler
// SPI1_IRQHandler
// SPI2_IRQHandler



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
