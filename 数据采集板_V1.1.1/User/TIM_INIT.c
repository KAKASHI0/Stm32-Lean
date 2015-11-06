#include "stm32f10x.h"
#include "TIM_config.h"

#define OpenTIM4
#define OpenTIM2

/**************************************************
PWM_DUOJI 
TIM8_CH2N
GPIOB_Pin_0
**************************************************/
#ifdef	OpenTIM4
	void TIM4_Configration(void)
	{
			GPIO_InitTypeDef GPIO_InitStructure;
			TIM_TimeBaseInitTypeDef	TIM_InitStructure;
			TIM_OCInitTypeDef	TIM_OCInitStructure;
			
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
		
			GPIO_InitStructure.GPIO_Pin		=	GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;
			GPIO_InitStructure.GPIO_Mode	=	GPIO_Mode_AF_PP;
			GPIO_InitStructure.GPIO_Speed	=	GPIO_Speed_2MHz;
			GPIO_Init(GPIOB,&GPIO_InitStructure);
		
			TIM_InitStructure.TIM_ClockDivision	=	TIM_CKD_DIV1;
			TIM_InitStructure.TIM_CounterMode	=	TIM_CounterMode_Up;
			TIM_InitStructure.TIM_Period	=	20000;							//20ms =50MHz
			TIM_InitStructure.TIM_Prescaler	=	7;							//TIM4->APB1,24MHz?,NOW IS 1us 
			TIM_InitStructure.TIM_RepetitionCounter	=	0x00;
			TIM_TimeBaseInit(TIM4,&TIM_InitStructure);
		
			TIM_OCInitStructure.TIM_OCMode	=	TIM_OCMode_PWM1;
			TIM_OCInitStructure.TIM_Pulse	=	20000;					//PB6->BHI1,AHI1
			TIM_OCInitStructure.TIM_OCPolarity	=	TIM_OCPolarity_High;
			TIM_OCInitStructure.TIM_OCNPolarity	=	TIM_OCNPolarity_High;
			TIM_OCInitStructure.TIM_OCIdleState	=	TIM_OCIdleState_Set;
			TIM_OCInitStructure.TIM_OCNIdleState	=	TIM_OCNIdleState_Reset;
			TIM_OCInitStructure.TIM_OutputState	=	TIM_OutputState_Enable;	
			TIM_OCInitStructure.TIM_OutputNState	=	TIM_OutputNState_Enable;
			TIM_OC1Init(TIM4,&TIM_OCInitStructure);
			
			TIM_OCInitStructure.TIM_Pulse	=	0;						//PB7->BLI1,ALI1
			TIM_OC2Init(TIM4,&TIM_OCInitStructure);
			
			TIM_OCInitStructure.TIM_Pulse	=	20000;						//PB8->BHI2,AHI2
			TIM_OC3Init(TIM4,&TIM_OCInitStructure);
			
			TIM_OCInitStructure.TIM_Pulse	=	0;						//PB9->BLI2,ALI2
			TIM_OC4Init(TIM4,&TIM_OCInitStructure);
			
			TIM_OC1PreloadConfig(TIM4,TIM_OCPreload_Enable);
			TIM_OC2PreloadConfig(TIM4,TIM_OCPreload_Enable);
			TIM_OC3PreloadConfig(TIM4,TIM_OCPreload_Enable);
			TIM_OC4PreloadConfig(TIM4,TIM_OCPreload_Enable);
			TIM_ARRPreloadConfig(TIM4,ENABLE);
				
			TIM_Cmd(TIM4,ENABLE);	
			TIM_CtrlPWMOutputs(TIM4,ENABLE);
	}
	#endif
	
#ifdef	OpenTIM2
	
void	TIM2_Configuration(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); 		
	
  TIM_TimeBaseStructure.TIM_Period = 50000-1;       
  TIM_TimeBaseStructure.TIM_Prescaler = 12-1; 
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;	
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
			
  TIM_Cmd(TIM2, ENABLE);
  TIM_ITConfig(TIM2,TIM_IT_Update, ENABLE);
	
}
	
	
	#endif
