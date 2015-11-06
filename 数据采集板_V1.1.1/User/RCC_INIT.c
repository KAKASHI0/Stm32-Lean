#include "stm32f10x.h"
#include "rcc_config.h"

/*******************************************************************************
* Function Name  : RCC_Configuration
* Description    : Configures the different system clocks.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
ErrorStatus HSEStartUpStatus;

void RCC_Configuration(void)
{
	//---------------------串口功能配置---------------------
	//打开串口对应的外设时钟  
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2 , ENABLE); 
	//串口发DMA配置  
	//启动DMA时钟
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	

  // Enable AFIO clock 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

#ifdef	OpenGPIOA
  //GPIOA Periph clock enable
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
		#endif
#ifdef OpenGPIOB
  // GPIOB Periph clock enable 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
		#endif
#ifdef	OpenGPIOC
  // GPIOC Periph clock enable 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
	#endif
#ifdef	OpenGPIOD
  // GPI0D Periph clock enable 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD,ENABLE);
	#endif
#ifdef	OpenGPIOE
  // GPI0E Periph clock enable 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE,ENABLE);
	#endif
#ifdef	OpenTIM1
  // Enable TIM1 clock
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	#endif
#ifdef	OpenTIM2_3_4_5_6_7//(TIM2,TIM3,TIM4)
  // EnableTIM2  TIM3  TIM4 clocks 
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4|
												  RCC_APB1Periph_TIM5|RCC_APB1Periph_TIM6 | RCC_APB1Periph_TIM7|, ENABLE);
	#endif
#ifndef OpenTIM8
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
	#endif
#ifdef	OpenUSART1	
  //Enable USART1 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	#endif
#ifdef	OpenUSART2
  // Enable USART2 
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
	#endif
#ifdef	OpenUSART3
  //Enable USART3 
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
	#endif
#ifdef	OpenI2C1
  //Enable  I2C1 
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,ENABLE);
	#endif
#ifdef	OpenADC1
  //Enable  ADC1 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);
	#endif
#ifdef	OpenADC2
  //Enable ADC2
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2,ENABLE);
	#endif
#ifdef	OpenDAC
  // DAC Periph clock enable 
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);
	#endif
}


/******************************************************
						初始化时
******************************************************/
void Init_SysTick(void)
{
  if(SysTick_Config(SystemCoreClock / 1000)) //3.5库中，SystemFrequency 被 SystemCoreClock取代 
  while(1);
}

/******************************************************
stm32f10x_it.c中的void SysTick_Handler(void)被被改为
******************************************************/
// extern __IO uint32_t TimingDelay;
// void SysTick_Handler(void)
// {
//   if (TimingDelay != 0x00)
//   { 
//     TimingDelay--;
//   }
// }

/*****************************************************
						延时1ms函数
*****************************************************/
__IO uint32_t TimingDelay;
void delay_ms(__IO uint32_t nTime)
{
  TimingDelay = nTime;
  while(TimingDelay != 0);
}

