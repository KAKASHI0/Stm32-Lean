#include "stm32f10x.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_rcc.h"
#include "misc.h"
#include "stm32f10x_rcc.h"
#include "Sensor.h"

uint8_t Uart_Rx1[UART_RX_LEN] = {0};
uint8_t Uart_Rx2[UART_RX_LEN] = {0};
uint8_t Uart_Rx3[UART_RX_LEN] = {0};
uint8_t Uart_Rx4[UART_RX_LEN] = {0};
uint8_t Uart_Rx5[UART_RX_LEN] = {0};
uint8_t Uart_Send_Buffer[] = {0};


void Sensor1_6_Configration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	//---------------------串口功能配置---------------------
	//打开串口对应的外设时钟  
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2 , ENABLE); 
	//串口发DMA配置  
	//启动DMA时钟
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	//DMA发送中断设置
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	//DMA1通道4配置
	DMA_DeInit(DMA1_Channel4);
	//外设地址
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&USART1->DR);
	//内存地址
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)Uart_Send_Buffer;
	//dma传输方向单向
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	//设置DMA在传输时缓冲区的长度
	DMA_InitStructure.DMA_BufferSize = 100;
	//设置DMA的外设递增模式，一个外设
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	//设置DMA的内存递增模式
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	//外设数据字长
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	//内存数据字长
	DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;
	//设置DMA的传输模式
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	//设置DMA的优先级别
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	//设置DMA的2个memory中的变量互相访问
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel4,&DMA_InitStructure);
	DMA_ITConfig(DMA1_Channel4,DMA_IT_TC,ENABLE);
	
	//使能通道4
	//DMA_Cmd(DMA1_Channel4, ENABLE);

	//串口收DMA配置  
	//启动DMA时钟
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	//DMA1通道5配置
	DMA_DeInit(DMA1_Channel5);
	//外设地址
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&USART1->DR);
	//内存地址
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)Uart_Rx2;
	//dma传输方向单向
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	//设置DMA在传输时缓冲区的长度
	DMA_InitStructure.DMA_BufferSize = UART_RX_LEN;
	//设置DMA的外设递增模式，一个外设
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	//设置DMA的内存递增模式
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	//外设数据字长
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	//内存数据字长
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	//设置DMA的传输模式
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	//设置DMA的优先级别
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	//设置DMA的2个memory中的变量互相访问
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel5,&DMA_InitStructure);

	//使能通道5
	DMA_Cmd(DMA1_Channel5,ENABLE);
	
	  
    //初始化参数  
    //USART_InitStructure.USART_BaudRate = DEFAULT_BAUD;  
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;  
    USART_InitStructure.USART_StopBits = USART_StopBits_1;  
    USART_InitStructure.USART_Parity = USART_Parity_No;  
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;    
    USART_InitStructure.USART_BaudRate = DEFAULT_BAUD; 
	//初始化串口 
    USART_Init(USART1,&USART_InitStructure);  
    //TXE发送中断,TC传输完成中断,RXNE接收中断,PE奇偶错误中断,可以是多个   
    //USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);
	
	//中断配置
	USART_ITConfig(USART1,USART_IT_TC,DISABLE);
	USART_ITConfig(USART1,USART_IT_RXNE,DISABLE);
	USART_ITConfig(USART1,USART_IT_IDLE,ENABLE);  

	//配置UART1中断  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;               //通道设置为串口1中断  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;       //中断占先等级0  
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;              //中断响应优先级0  
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                 //打开中断  
    NVIC_Init(&NVIC_InitStructure);   
        
	//采用DMA方式发送
	USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);
	//采用DMA方式接收
	USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);
    //启动串口  
    USART_Cmd(USART1, ENABLE); 
}


