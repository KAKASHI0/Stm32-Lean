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
	//---------------------���ڹ�������---------------------
	//�򿪴��ڶ�Ӧ������ʱ��  
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2 , ENABLE); 
	//���ڷ�DMA����  
	//����DMAʱ��
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	//DMA�����ж�����
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	//DMA1ͨ��4����
	DMA_DeInit(DMA1_Channel4);
	//�����ַ
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&USART1->DR);
	//�ڴ��ַ
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)Uart_Send_Buffer;
	//dma���䷽����
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	//����DMA�ڴ���ʱ�������ĳ���
	DMA_InitStructure.DMA_BufferSize = 100;
	//����DMA���������ģʽ��һ������
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	//����DMA���ڴ����ģʽ
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	//���������ֳ�
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	//�ڴ������ֳ�
	DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;
	//����DMA�Ĵ���ģʽ
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	//����DMA�����ȼ���
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	//����DMA��2��memory�еı����������
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel4,&DMA_InitStructure);
	DMA_ITConfig(DMA1_Channel4,DMA_IT_TC,ENABLE);
	
	//ʹ��ͨ��4
	//DMA_Cmd(DMA1_Channel4, ENABLE);

	//������DMA����  
	//����DMAʱ��
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	//DMA1ͨ��5����
	DMA_DeInit(DMA1_Channel5);
	//�����ַ
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&USART1->DR);
	//�ڴ��ַ
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)Uart_Rx2;
	//dma���䷽����
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	//����DMA�ڴ���ʱ�������ĳ���
	DMA_InitStructure.DMA_BufferSize = UART_RX_LEN;
	//����DMA���������ģʽ��һ������
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	//����DMA���ڴ����ģʽ
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	//���������ֳ�
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	//�ڴ������ֳ�
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	//����DMA�Ĵ���ģʽ
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	//����DMA�����ȼ���
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	//����DMA��2��memory�еı����������
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel5,&DMA_InitStructure);

	//ʹ��ͨ��5
	DMA_Cmd(DMA1_Channel5,ENABLE);
	
	  
    //��ʼ������  
    //USART_InitStructure.USART_BaudRate = DEFAULT_BAUD;  
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;  
    USART_InitStructure.USART_StopBits = USART_StopBits_1;  
    USART_InitStructure.USART_Parity = USART_Parity_No;  
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;    
    USART_InitStructure.USART_BaudRate = DEFAULT_BAUD; 
	//��ʼ������ 
    USART_Init(USART1,&USART_InitStructure);  
    //TXE�����ж�,TC��������ж�,RXNE�����ж�,PE��ż�����ж�,�����Ƕ��   
    //USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);
	
	//�ж�����
	USART_ITConfig(USART1,USART_IT_TC,DISABLE);
	USART_ITConfig(USART1,USART_IT_RXNE,DISABLE);
	USART_ITConfig(USART1,USART_IT_IDLE,ENABLE);  

	//����UART1�ж�  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;               //ͨ������Ϊ����1�ж�  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;       //�ж�ռ�ȵȼ�0  
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;              //�ж���Ӧ���ȼ�0  
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                 //���ж�  
    NVIC_Init(&NVIC_InitStructure);   
        
	//����DMA��ʽ����
	USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);
	//����DMA��ʽ����
	USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);
    //��������  
    USART_Cmd(USART1, ENABLE); 
}


