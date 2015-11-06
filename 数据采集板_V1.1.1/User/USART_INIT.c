#include "stm32f10x.h"
#include "usart_config.h"
#include "stdio.h"

//#define OpenUSART3
//#define OpenUSART2

/******************************************
USART3_TX->PB10
USART3_RX->PB11
******************************************/
#ifdef OpenUSART3
		void USART3_Configration(void)
		{
			GPIO_InitTypeDef 		GPIO_InitStructure;
			USART_InitTypeDef 	USART_InitStructure;
			
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
			
			GPIO_InitStructure.GPIO_Pin		=	GPIO_Pin_10;
			GPIO_InitStructure.GPIO_Mode	=	GPIO_Mode_AF_PP;					//推挽输出
			GPIO_InitStructure.GPIO_Speed	=	GPIO_Speed_50MHz;
			GPIO_Init(GPIOB,&GPIO_InitStructure);
			
			GPIO_InitStructure.GPIO_Pin		=	GPIO_Pin_11;
			GPIO_InitStructure.GPIO_Mode	=	GPIO_Mode_IN_FLOATING;		//浮空输入
			GPIO_Init(GPIOB,&GPIO_InitStructure);
				
			USART_InitStructure.USART_BaudRate	=	256000;
			USART_InitStructure.USART_Mode	=	USART_Mode_Tx|USART_Mode_Rx;
			USART_InitStructure.USART_WordLength	=	USART_WordLength_8b;
			USART_InitStructure.USART_StopBits	=	USART_StopBits_1;
			USART_InitStructure.USART_Parity	=	USART_Parity_No;
			USART_InitStructure.USART_HardwareFlowControl	=	USART_HardwareFlowControl_None;
			
			USART_Init(USART3,&USART_InitStructure);
			USART_ITConfig(USART3,USART_IT_RXNE,ENABLE);				//开中断
			USART_Cmd(USART3, ENABLE); 
		}
		/*********************************************************
				配置USART3接收中断
// 		**********************************************************/
// 		void USART3_IRQHandler(void)
// 		{

// 			 if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
// 			 {
// 					/* Read one byte from the receive data register */
// 					RxBuffer[RxCounter++] = USART_ReceiveData(USART3);
// 					if(RxCounter == NbrOfDataToRead)
// 					{
// 						/* Disable the USART Receive interrupt */
// 						USART_ITConfig(USART3, USART_IT_RXNE, DISABLE);
// 					}
// 			 }
// 			 
// 			 if(USART_GetITStatus(USART3, USART_IT_TXE) != RESET)
// 			 {  
// 					/* Write one byte to the transmit data register */
// 						USART_SendData(USART3, TxBuffer[TxCounter++]);
// 					if(TxCounter == NbrOfDataToTransfer)
// 					{
// 						/* Disable the USART1 Transmit interrupt */
// 						USART_ITConfig(USART3, USART_IT_TXE, DISABLE);
// 					}
// 			 }
// 			 
// 		}
		#endif

/******************************************
USART2_TX->PA2
USART2_RX->PA3
******************************************/		
#ifdef OpenUSART2
		void USART2_Configration(void)
		{
			GPIO_InitTypeDef 		GPIO_InitStructure;
			USART_InitTypeDef 	USART_InitStructure;
			
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
			
			GPIO_InitStructure.GPIO_Pin		=	GPIO_Pin_2;
			GPIO_InitStructure.GPIO_Mode	=	GPIO_Mode_AF_PP;		//推挽输出
			GPIO_InitStructure.GPIO_Speed	=	GPIO_Speed_50MHz;
			GPIO_Init(GPIOA,&GPIO_InitStructure);
			
			GPIO_InitStructure.GPIO_Pin		=	GPIO_Pin_3;
			GPIO_InitStructure.GPIO_Mode	=	GPIO_Mode_IN_FLOATING;	//浮空输入
			GPIO_Init(GPIOA,&GPIO_InitStructure);
			
			
			USART_InitStructure.USART_BaudRate	=	115200;
			USART_InitStructure.USART_Mode	=	USART_Mode_Tx|USART_Mode_Rx;
			USART_InitStructure.USART_WordLength	=	USART_WordLength_8b;
			USART_InitStructure.USART_StopBits	=	USART_StopBits_1;
			USART_InitStructure.USART_Parity	=	USART_Parity_No;
			USART_InitStructure.USART_HardwareFlowControl	=	USART_HardwareFlowControl_None;
			
			USART_Init(USART2,&USART_InitStructure);
			USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);			//开中断
			USART_Cmd(USART2, ENABLE); 
			
		}
		#endif
		
		
		

