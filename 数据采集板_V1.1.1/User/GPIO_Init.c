#include "stm32f10x.h"
#include "gpio_config.h"

#define OpenLED
//#define	OpenSHACHE

void GPIO_Configration(void)
{
	
	GPIO_InitTypeDef GPIO_InitStructure;
	/**************************************************************************
	GPIOA0->D9_left   GPIOA1->D9_right
	GPIOA7->D10_left	GPIOA8->D10_right
	
	**************************************************************************/

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
	
	//GPIO_InitStructure.GPIO_Pin		=	GPIO_Pin_1|GPIO_Pin_9|GPIO_Pin_10;
	//GPIO_InitStructure.GPIO_Mode	=	GPIO_Mode_Out_PP;
	//GPIO_InitStructure.GPIO_Speed	= GPIO_Speed_50MHz;
	
	//GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin		=	GPIO_Pin_2;//USART2_TX
	GPIO_InitStructure.GPIO_Mode	=	GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed	= GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin		=	GPIO_Pin_3;//USART2_Rx
	GPIO_InitStructure.GPIO_Mode	=	GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed	= GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin		=	GPIO_Pin_9;//USART1_TX
	GPIO_InitStructure.GPIO_Mode	=	GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed	= GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin		=	GPIO_Pin_10;//USART1_RX
	GPIO_InitStructure.GPIO_Mode	=	GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed	= GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin		=	GPIO_Pin_10;//USART3_TX
	GPIO_InitStructure.GPIO_Mode	=	GPIO_Mode_AF_PP;// 推完输出
	GPIO_InitStructure.GPIO_Speed	= GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin		=	GPIO_Pin_11;//USART3_RX
	GPIO_InitStructure.GPIO_Mode	=	GPIO_Mode_IN_FLOATING;//浮空输入
	GPIO_InitStructure.GPIO_Speed	= GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin		=	GPIO_Pin_2|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode	=	GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	GPIO_ResetBits(GPIOB,GPIO_Pin_12);
	GPIO_SetBits(GPIOB,GPIO_Pin_12);
	GPIO_ResetBits(GPIOB,GPIO_Pin_12);
	GPIO_ResetBits(GPIOB,GPIO_Pin_13);
	
	GPIO_InitStructure.GPIO_Pin		=	GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode	=	GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed	= GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	
	/**************************************************************************
	GPIOA9->brake   GPIOA10->brake
	
	**************************************************************************/
	#ifdef OpenSHACHE
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin		=	GPIO_Pin_9|GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode	=	GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed	= GPIO_Speed_50MHz;
	
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	#endif
}
