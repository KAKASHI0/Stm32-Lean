#include 	"stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_adc.h"
#include "RTL.h"
#include 	"all_init_heard.h"
#include	"all_task_heard.h"

/************macro definition*****************/
#define SENSOR_BAUD 512000
#define WIFI_BAUD 921600
//串口接收缓存
#define UART_RX_LEN		30

#define PACKAGE_SIZE	24 //包头*4 数据*16 crc*4 
#define SEND_BUFFER_SIZE 1000
#define FIFO_SIZE 10000

#define ADC1_DR_Address    ((u32)0x4001244C)
vu16 ADC_ConvertedValue;

typedef struct _FIFO_QUEUE_STRU 
{ 
  uint16_t front; 
  uint16_t rear; 
  uint16_t count; 
  uint8_t data[FIFO_SIZE]; 
}FIFO_QUEUE_STRU;



/************global veriable*****************/
//双缓冲
uint8_t USART1_DMA_RX_BUF[UART_RX_LEN] = {0};//DMA缓存区
uint8_t USART2_DMA_RX_BUF[UART_RX_LEN] = {0};
uint8_t USART3_DMA_RX_BUF[UART_RX_LEN] = {0};
uint8_t UART4_DMA_RX_BUF[UART_RX_LEN] = {0};
uint8_t UART5_DMA_RX_BUF[UART_RX_LEN] = {0};

/*数据缓存区 在串口接收完一帧后产生IDLE中断,
将DMA_BUF的数据复制到RX_DATA中后,清空DMA1_BUF用于接收新数据*/
uint8_t USART1_RX_DATA[UART_RX_LEN] = {0};
uint8_t USART2_RX_DATA[UART_RX_LEN] = {0};
uint8_t USART3_RX_DATA[UART_RX_LEN] = {0};
uint8_t UART4_RX_DATA[UART_RX_LEN] = {0};
uint8_t UART5_RX_DATA[UART_RX_LEN] = {0};

uint8_t Uart_Send_Buffer[SEND_BUFFER_SIZE] = {0};
FIFO_QUEUE_STRU TX_FIFO;

BOOL DMA_Statue=0;//DMA状态 0=停止发送 1=正在发送

/***********task ID***********************/
OS_TID IMU_DATA_TRANS_1_9_ID;
OS_TID IMU_DATA_TRANS_10_18_ID;
OS_TID IMU_DATA_TRANS_13_18_ID;

/*************function**************/

void GPIO_PinReverse(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)//IO取反
{
  /* Check the parameters */
  assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
  assert_param(IS_GPIO_PIN(GPIO_Pin));
  
  GPIOx->ODR ^=  GPIO_Pin;
}

void Peripherals_On()
{
	GPIO_SetBits(GPIOA,GPIO_Pin_1);//IMU1-9
	GPIO_SetBits(GPIOB,GPIO_Pin_2);//IMU10-18
	GPIO_SetBits(GPIOB,GPIO_Pin_13);//WIFI
	GPIO_ResetBits(GPIOB,GPIO_Pin_12);//LED
}

void Peripherals_Off()
{
	GPIO_ResetBits(GPIOA,GPIO_Pin_1);//IMU1-9
	GPIO_ResetBits(GPIOB,GPIO_Pin_2);//IMU10-18
	GPIO_ResetBits(GPIOB,GPIO_Pin_13);//WIFI
}

void ADC_Configuration(void)
{
  ADC_InitTypeDef ADC_InitStructure;
  DMA_InitTypeDef DMA_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

  /* Configure PA.06 (ADC Channel6), PA.07 (ADC Channel7) as analog input -------------------------*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOA, &GPIO_InitStructure);   
   
  /* DMA channel1 configuration ----------------------------------------------*/
  DMA_DeInit(DMA1_Channel1);
  DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;
  DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&ADC_ConvertedValue;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = 1;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);
  
  /* Enable DMA1 channel1 */
  DMA_Cmd(DMA1_Channel1, ENABLE);
    
  /* ADC1 configuration ------------------------------------------------------*/
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfChannel = 1;
  ADC_Init(ADC1, &ADC_InitStructure);

  /* ADC1 regular channel6 configuration */ 
  ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 1, ADC_SampleTime_239Cycles5);

  /* Enable ADC1 DMA */
  ADC_DMACmd(ADC1, ENABLE);
  
  /* Enable ADC1 */
  ADC_Cmd(ADC1, ENABLE);

  /* Enable ADC1 reset calibaration register */   
  ADC_ResetCalibration(ADC1);
  /* Check the end of ADC1 reset calibration register */
  while(ADC_GetResetCalibrationStatus(ADC1));

  /* Start ADC1 calibaration */
  ADC_StartCalibration(ADC1);
  /* Check the end of ADC1 calibration */
  while(ADC_GetCalibrationStatus(ADC1));
     
  /* Start ADC1 Software Conversion */ 
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

void RCC_Configuration(void)
{
	// Enable CRC clock
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_CRC,ENABLE);
	// Enable DMA1 clock 
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	// Enable DMA1 clock 
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);
	
  // Enable AFIO clock 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  //GPIOA Periph clock enable
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
  // GPIOB Periph clock enable 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
  // GPIOC Periph clock enable 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
	
  // Enable TIM2 clock
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
  // Enable USART2 
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
  //Enable USART3 
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
	//Enable UART4 
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);
	//Enable UART5 
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5,ENABLE);
	
  //Enable  ADC1 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);
}

void NVIC_Configuration()
{
	NVIC_InitTypeDef NVIC_InitStructure;
	/*---------------- Configure and enable TIM2 interrupt ---------------------------*/
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	//配置UART1中断  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;               //通道设置为串口1中断  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;       //中断占先等级0  
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;              //中断响应优先级0  
	NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;                 //暂时不接受数据 屏蔽中断  
	NVIC_Init(&NVIC_InitStructure);   
	
		
	//配置UART2中断  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;               //通道设置为串口2中断  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;       //中断占先等级0  
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;              //中断响应优先级0  
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                 //打开中断  
	NVIC_Init(&NVIC_InitStructure);   
	
	//配置UART3中断  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;               //通道设置为串口3中断  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;       //中断占先等级0  
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;              //中断响应优先级0  
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                 //打开中断  
	NVIC_Init(&NVIC_InitStructure);   

	//DMA发送中断设置
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}


void DMA_Configration()
{
	DMA_InitTypeDef DMA_InitStructure;
	
	/**********************USART1 DMA COnfiguration*************/
	//DMA发送数据
	//DMA1通道4配置
	DMA_DeInit(DMA1_Channel4);
	//外设地址
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&USART1->DR);
	//内存地址
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)Uart_Send_Buffer;
	//dma传输方向单向
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	//设置DMA在传输时缓冲区的长度
	DMA_InitStructure.DMA_BufferSize = SEND_BUFFER_SIZE;
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
	DMA_Cmd(DMA1_Channel4, ENABLE);

	//串口收DMA配置  
	//DMA1通道5配置
	DMA_DeInit(DMA1_Channel5);
	//外设地址
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&USART1->DR);
	//内存地址
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)USART1_DMA_RX_BUF;
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
	//DMA_Cmd(DMA1_Channel5,ENABLE);
	/**********************USART2 DMA COnfiguration*************/
	//串口2收DMA配置  
	//DMA1通道6配置
	DMA_DeInit(DMA1_Channel6);
	//外设地址
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&USART2->DR);
	//内存地址
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)USART2_DMA_RX_BUF;
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
	DMA_Init(DMA1_Channel6,&DMA_InitStructure);

	//使能通道6
	DMA_Cmd(DMA1_Channel6,ENABLE);	
	
	/**********************USART3 DMA COnfiguration*************/
	//串口3收DMA配置  
	//DMA1通道3配置
	DMA_DeInit(DMA1_Channel3);
	//外设地址
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&USART3->DR);
	//内存地址
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)USART3_DMA_RX_BUF;
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
	DMA_Init(DMA1_Channel3,&DMA_InitStructure);

	//使能通道3
	DMA_Cmd(DMA1_Channel3,ENABLE);	
	
// 	/**********************UART4 DMA COnfiguration*************/
//选型失误 stm32f103xb和stm32f103x8系列芯片不带DMA2 现改为usart2和usart3承担18个传感器通讯任务
// 	//串口4收DMA配置  
// 	//DMA2通道3配置
// 	DMA_DeInit(DMA2_Channel3);
// 	//外设地址
// 	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&UART4->DR);
// 	//内存地址
// 	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)UART4_DMA_RX_BUF;
// 	//dma传输方向单向
// 	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
// 	//设置DMA在传输时缓冲区的长度
// 	DMA_InitStructure.DMA_BufferSize = UART_RX_LEN;
// 	//设置DMA的外设递增模式，一个外设
// 	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
// 	//设置DMA的内存递增模式
// 	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
// 	//外设数据字长
// 	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
// 	//内存数据字长
// 	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
// 	//设置DMA的传输模式
// 	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
// 	//设置DMA的优先级别
// 	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
// 	//设置DMA的2个memory中的变量互相访问
// 	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
// 	DMA_Init(DMA2_Channel3,&DMA_InitStructure);

// 	//使能通道3
// 	DMA_Cmd(DMA2_Channel3,ENABLE);	
	
}
void USART1_Configration()
{
// 	USART_InitTypeDef USART_InitStructure;
//  //初始化参数  
//     //USART_InitStructure.USART_BaudRate = SENSOR_BAUD;  
//     USART_InitStructure.USART_WordLength = USART_WordLength_8b;  
//     USART_InitStructure.USART_StopBits = USART_StopBits_1;  
//     USART_InitStructure.USART_Parity = USART_Parity_No;  
//     USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  
//     USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;    
//     USART_InitStructure.USART_BaudRate = WIFI_BAUD; 
// 	//初始化串口 
//     USART_Init(USART2,&USART_InitStructure);  
//     //TXE发送中断,TC传输完成中断,RXNE接收中断,PE奇偶错误中断,可以是多个   
//     //USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);
// 	
// 	//中断配置
// 	USART_ITConfig(USART1,USART_IT_RXNE,DISABLE);
// 	
// 	//采用DMA方式发送
// 	USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);
// 	//采用DMA方式接收
// 	USART_DMACmd(USART1,USART_DMAReq_Rx,DISABLE);
//     //启动串口  
//   USART_Cmd(USART1, ENABLE); 
	NVIC_InitTypeDef NVIC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	//---------------------??????---------------------
//???????????  
RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 , ENABLE);   
//??DMA??
RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
//DMA??????
NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQn;
NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
NVIC_Init(&NVIC_InitStructure);

//DMA1??4??
DMA_DeInit(DMA1_Channel4);
//????
DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&USART1->DR);
//????
DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)Uart_Send_Buffer;
//dma??????
DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
//??DMA??????????
DMA_InitStructure.DMA_BufferSize = 100;
//??DMA???????,????
DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//??DMA???????
DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
//??????
DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
//??????
DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;
//??DMA?????
DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
//??DMA?????
DMA_InitStructure.DMA_Priority = DMA_Priority_High;
//??DMA?2?memory????????
DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
DMA_Init(DMA1_Channel4,&DMA_InitStructure);
DMA_ITConfig(DMA1_Channel4,DMA_IT_TC,ENABLE);

//????4
//DMA_Cmd(DMA1_Channel4, ENABLE);

  
//?????  
//USART_InitStructure.USART_BaudRate = DEFAULT_BAUD;  
USART_InitStructure.USART_WordLength = USART_WordLength_8b;  
USART_InitStructure.USART_StopBits = USART_StopBits_1;  
USART_InitStructure.USART_Parity = USART_Parity_No;  
USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_RTS_CTS;  
USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;    
USART_InitStructure.USART_BaudRate = WIFI_BAUD; 
//????? 
USART_Init(USART1,&USART_InitStructure);  
//TXE????,TC??????,RXNE????,PE??????,?????   
USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);  

//??UART1??  
NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);
NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;               //???????1??  
NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;       //??????0  
NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;              //???????0  
NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;                 //????  
NVIC_Init(&NVIC_InitStructure);                                 //???  

//??DMA????
USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);
//????  
USART_Cmd(USART1, ENABLE);   

//??IO???      
RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);    
//??1??????    
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_12;                       //??9  
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;                //??GPIO????  
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;                 //??????  
GPIO_Init(GPIOA, &GPIO_InitStructure);                          //TX???  
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;                      //??10  
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;                //??GPIO????  
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;           //????  
GPIO_Init(GPIOA, &GPIO_InitStructure);                          //RX???                                                      
AFIO->MAPR = (AFIO->MAPR|AFIO_MAPR_CAN_REMAP_0|AFIO_MAPR_CAN_REMAP_1)&AFIO_MAPR_CAN_REMAP_0;
//??IO???      
RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);   
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;                       //??9  
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;                //??GPIO????  
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;                 //??????  

}
void USART2_Configration()
{
	USART_InitTypeDef USART_InitStructure;
    //初始化参数  
    //USART_InitStructure.USART_BaudRate = SENSOR_BAUD;  
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;  
    USART_InitStructure.USART_StopBits = USART_StopBits_1;  
    USART_InitStructure.USART_Parity = USART_Parity_No;  
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;    
    USART_InitStructure.USART_BaudRate = SENSOR_BAUD; 
	//初始化串口 
    USART_Init(USART2,&USART_InitStructure);  
    //TXE发送中断,TC传输完成中断,RXNE接收中断,PE奇偶错误中断,可以是多个   
    //USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);
	
	//中断配置
	USART_ITConfig(USART2,USART_IT_TC,DISABLE);
	USART_ITConfig(USART2,USART_IT_RXNE,DISABLE);
	USART_ITConfig(USART2,USART_IT_IDLE,ENABLE);  
	
	//不采用DMA方式发送
	USART_DMACmd(USART2,USART_DMAReq_Tx,DISABLE);
	//采用DMA方式接收
	USART_DMACmd(USART2,USART_DMAReq_Rx,ENABLE);
    //启动串口  
  USART_Cmd(USART2, ENABLE); 
}
void USART3_Configration()
{
	USART_InitTypeDef USART_InitStructure;
    //初始化参数  
    //USART_InitStructure.USART_BaudRate = SENSOR_BAUD;  
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;  
    USART_InitStructure.USART_StopBits = USART_StopBits_1;  
    USART_InitStructure.USART_Parity = USART_Parity_No;  
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;    
    USART_InitStructure.USART_BaudRate = SENSOR_BAUD; 
	//初始化串口 
    USART_Init(USART3,&USART_InitStructure);  
    //TXE发送中断,TC传输完成中断,RXNE接收中断,PE奇偶错误中断,可以是多个   
    //USART_ITConfig(USART3,USART_IT_RXNE,ENABLE);
	
	//中断配置
	USART_ITConfig(USART3,USART_IT_TC,DISABLE);
	USART_ITConfig(USART3,USART_IT_RXNE,DISABLE);
	USART_ITConfig(USART3,USART_IT_IDLE,ENABLE);  
	
	//不采用DMA方式发送
	USART_DMACmd(USART3,USART_DMAReq_Tx,DISABLE);
	//采用DMA方式接收
	USART_DMACmd(USART3,USART_DMAReq_Rx,ENABLE);
    //启动串口  
  USART_Cmd(USART3, ENABLE); 
}
void UART4_Configration()
{
	USART_InitTypeDef USART_InitStructure;
    //初始化参数  
    //USART_InitStructure.USART_BaudRate = SENSOR_BAUD;  
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;  
    USART_InitStructure.USART_StopBits = USART_StopBits_1;  
    USART_InitStructure.USART_Parity = USART_Parity_No;  
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;    
    USART_InitStructure.USART_BaudRate = SENSOR_BAUD; 
	//初始化串口 
    USART_Init(UART4,&USART_InitStructure);  
    //TXE发送中断,TC传输完成中断,RXNE接收中断,PE奇偶错误中断,可以是多个   
    //USART_ITConfig(UART4,USART_IT_RXNE,ENABLE);
	
	//中断配置
	USART_ITConfig(UART4,USART_IT_TC,DISABLE);
	USART_ITConfig(UART4,USART_IT_RXNE,DISABLE);
	USART_ITConfig(UART4,USART_IT_IDLE,ENABLE);  
	
	//不采用DMA方式发送
	USART_DMACmd(UART4,USART_DMAReq_Tx,DISABLE);
	//采用DMA方式接收
	USART_DMACmd(UART4,USART_DMAReq_Rx,ENABLE);
    //启动串口  
  USART_Cmd(UART4, ENABLE); 
}
void UART5_Configration()
{
	//uart5 不支持DMA通道
}
uint8_t testlist[10];

ErrorStatus Write_TX_FIFO(uint8_t *pdata,uint8_t length)
{
	uint8_t i;
  if(FIFO_SIZE - TX_FIFO.count >= length) 
  { 
		__disable_irq();
		for (i=0; i<length; i++)
		{
			TX_FIFO.data[TX_FIFO.rear] = *(pdata+i); 
			TX_FIFO.rear = (TX_FIFO.rear + 1) % FIFO_SIZE; 
			TX_FIFO.count = TX_FIFO.count + 1; 
			if (TX_FIFO.count == FIFO_SIZE)
			{				
				TX_FIFO.rear = TX_FIFO.front;
			}
		}
		__enable_irq();
    return SUCCESS; 
  } 
	else
	  return ERROR;
}

ErrorStatus Read_TX_FIFO(uint8_t *data)
{	
  if((TX_FIFO.front == TX_FIFO.rear) && (TX_FIFO.count == 0)) 
  {
    return ERROR; 
  }
	else 
  {
		__disable_irq();
    *data = TX_FIFO.data[TX_FIFO.front]; 
    TX_FIFO.front = (TX_FIFO.front + 1) % FIFO_SIZE; 
    TX_FIFO.count = TX_FIFO.count - 1; 
		__enable_irq();
    return SUCCESS; 
  }
}

void Send_Data(void)//数据发送开关 状态机
{
	uint8_t count;
	if(DMA_Statue==0)//DMA停止发送
	{
		for(count=0;count<SEND_BUFFER_SIZE;count++)
		{
			if(Read_TX_FIFO(&Uart_Send_Buffer[count])==ERROR)
				break;//FIFO为空 提前退出
		}
		if(count>0)
		{
			__disable_irq();
			DMA_SetCurrDataCounter(DMA1_Channel4,count);
			DMA_Cmd(DMA1_Channel4,ENABLE);
			DMA_Statue = 1;
			__enable_irq();
		}
		else
		{
			DMA_Statue = 0;
		}
			
	}
	
}
uint32_t crc_error_count=0;
uint8_t crc_error_list[1000];
uint8_t crc_mutex;
void IMU_Get_DATA2(USART_TypeDef* USARTx,uint8_t imu_id)//从一个传感器获取数据，校验后写入到发送fifo，不可重入
{
	USART_SendData(USARTx,imu_id);
	if(os_evt_wait_or(0x02,2)!=OS_R_TMO)
	{
		Write_TX_FIFO(USART2_RX_DATA,USART2_RX_DATA[2]+2);//数据长度加上包头2字节
		Send_Data();
		
// 		crcarray[0] = USART2_RX_DATA[0]<<24|USART2_RX_DATA[1]<<16|USART2_RX_DATA[2]<<8|USART2_RX_DATA[3];
// 		crcarray[1] = USART2_RX_DATA[4]<<24|USART2_RX_DATA[5]<<16|USART2_RX_DATA[6]<<8|USART2_RX_DATA[7];
// 		crcarray[2] = USART2_RX_DATA[8]<<24|USART2_RX_DATA[9]<<16|USART2_RX_DATA[10]<<8|USART2_RX_DATA[11];
// 		crcarray[3] = USART2_RX_DATA[12]<<24|USART2_RX_DATA[13]<<16|USART2_RX_DATA[14]<<8|USART2_RX_DATA[15];
// 		crcarray[4] = USART2_RX_DATA[16]<<24|USART2_RX_DATA[17]<<16|USART2_RX_DATA[18]<<8|USART2_RX_DATA[19];
// 		crcresult = USART2_RX_DATA[20]<<24|USART2_RX_DATA[21]<<16|USART2_RX_DATA[22]<<8|USART2_RX_DATA[23];
// 		if(crc_mutex==0)
// 		{
// 			crc_mutex =1;
// 			CRC_ResetDR();
// 			if(crcresult == CRC_CalcBlockCRC(crcarray,5))//crc校验位以32位数据长度进行
// 			{
// 				Write_TX_FIFO(USART2_RX_DATA,PACKAGE_SIZE);
// 				Send_Data();
// 			}
// 			else
// 			{
// 				crc_error_count++;
// 				if(crc_error_count<1000)
// 					crc_error_list[crc_error_count]= USART2_RX_DATA[3];
// 				__NOP();//CRC校验失败后重发机制
// 			}
// 			crc_mutex=0;
// 		}
// 		else
// 		{
// 			__nop();
// 		}
	}
}

void IMU_Get_DATA3(USART_TypeDef* USARTx,uint8_t imu_id)//从一个传感器获取数据，校验后写入到发送fifo，不可重入
{
	USART_SendData(USARTx,imu_id);
	if(os_evt_wait_or(0x02,2)!=OS_R_TMO)
	{
		
		Write_TX_FIFO(USART3_RX_DATA,USART3_RX_DATA[2]+2);//数据长度加上包头2字节
		//Write_TX_FIFO(USART3_RX_DATA,USART3_RX_DATA[2]+2);//数据长度加上包头2字节
		Send_Data();
		
// 		放弃物理层CRC校验
// 		crcarray[0] = USART3_RX_DATA[0]<<24|USART3_RX_DATA[1]<<16|USART3_RX_DATA[2]<<8|USART3_RX_DATA[3];
// 		crcarray[1] = USART3_RX_DATA[4]<<24|USART3_RX_DATA[5]<<16|USART3_RX_DATA[6]<<8|USART3_RX_DATA[7];
// 		crcarray[2] = USART3_RX_DATA[8]<<24|USART3_RX_DATA[9]<<16|USART3_RX_DATA[10]<<8|USART3_RX_DATA[11];
// 		crcarray[3] = USART3_RX_DATA[12]<<24|USART3_RX_DATA[13]<<16|USART3_RX_DATA[14]<<8|USART3_RX_DATA[15];
// 		crcarray[4] = USART3_RX_DATA[16]<<24|USART3_RX_DATA[17]<<16|USART3_RX_DATA[18]<<8|USART3_RX_DATA[19];
// 		crcresult = USART3_RX_DATA[20]<<24|USART3_RX_DATA[21]<<16|USART3_RX_DATA[22]<<8|USART3_RX_DATA[23];
// 		
// 		if(crc_mutex==0)
// 		{
// 			crc_mutex =1;
// 			CRC_ResetDR();
// 			if(crcresult == CRC_CalcBlockCRC(crcarray,5))//crc校验位以32位数据长度进行
// 			{
// 				Write_TX_FIFO(USART3_RX_DATA,PACKAGE_SIZE);
// 				Send_Data();
// 			}
// 			else
// 			{
// 				crc_error_count++;
// 				if(crc_error_count<1000)
// 					crc_error_list[crc_error_count] = USART2_RX_DATA[3];
// 				__NOP();//CRC校验失败后重发机制
// 			}
// 			crc_mutex =0;
// 		}
// 		else
// 		{
// 			__nop();
// 		}
	}
}

__task void IMU_DATA_TRANS_1_9 (void) 
{
	IMU_DATA_TRANS_1_9_ID = os_tsk_self();
	while(1)
	{
		os_evt_wait_or(0x01,0xffff);
		//GPIO_SetBits(GPIOA,GPIO_Pin_3);
// 		os_evt_wait_or(0xff,1);//任务与系统时钟同步
		IMU_Get_DATA2(USART2,0x01);
 		IMU_Get_DATA2(USART2,0x02);
 		IMU_Get_DATA2(USART2,0x03);
 		IMU_Get_DATA2(USART2,0x04);
 		IMU_Get_DATA2(USART2,0x05);
 		IMU_Get_DATA2(USART2,0x06);
 		IMU_Get_DATA2(USART2,0x07);
 		IMU_Get_DATA2(USART2,0x08);
 		IMU_Get_DATA2(USART2,0x09);
		__NOP();
		//os_dly_wait(10);
	}
}

__task void IMU_DATA_TRANS_10_18 (void) 
{
	IMU_DATA_TRANS_10_18_ID = os_tsk_self();
	while(1)
	{
		os_evt_wait_or(0x01,0xffff);
		
		IMU_Get_DATA3(USART3,0x0a);
 		IMU_Get_DATA3(USART3,0x0b);
 		IMU_Get_DATA3(USART3,0x0c);
 		IMU_Get_DATA3(USART3,0x0d);
 		IMU_Get_DATA3(USART3,0x0e);
		IMU_Get_DATA3(USART3,0x0f);
 		IMU_Get_DATA3(USART3,0x10);
 		IMU_Get_DATA3(USART3,0x11);
 		IMU_Get_DATA3(USART3,0x12);
		__NOP();//os_dly_wait(10);
	}
}

// __task void IMU_DATA_TRANS_13_18 (void) 
// {
// 	IMU_DATA_TRANS_13_18_ID = os_tsk_self();
// 	while(1)
// 	{
// 		os_evt_wait_or(0x01,0xffff);
// 		IMU_Get_DATA(USART3,0x01);
//  		IMU_Get_DATA(USART3,0x02);
// 		IMU_Get_DATA(USART3,0x03);
//  		IMU_Get_DATA(USART3,0x04);
//  		IMU_Get_DATA(USART3,0x05);
//  		IMU_Get_DATA(USART3,0x06);
// // 		IMU_Get_DATA(UART4,0x01);//看来还是要分开来几个任务
// // 		IMU_Get_DATA(UART4,0x02);
// // 		IMU_Get_DATA(UART4,0x03);
// // 		IMU_Get_DATA(UART4,0x04);
// // 		IMU_Get_DATA(UART4,0x05);
// // 		IMU_Get_DATA(UART4,0x06);
// 		__NOP();//os_dly_wait(10);
// 		GPIO_ResetBits(GPIOA,GPIO_Pin_3);
// 	}
// }

__task void BATTERY_CHECK(void)
{
	float AD_value,Battery_value;
	uint8_t i=0;
	while(1)
	{
		os_dly_wait(500);//100ms检查一次电量
		//USART_SendData(USART1,0xaa);
		AD_value = ADC_ConvertedValue;
		Battery_value = (AD_value/4096)*3.3*2;
		if(Battery_value>3.68)
		{
			Peripherals_On();
			GPIO_ResetBits(GPIOB,GPIO_Pin_12);
		}
		else if(Battery_value>3.5&&Battery_value<3.68)
		{
			i++;
			if(i>5)
			{
				i=0;
				GPIO_PinReverse(GPIOB,GPIO_Pin_12);//LED慢闪
			}
		}
		else if(Battery_value<3.5)
		{
			Peripherals_Off();
			GPIO_PinReverse(GPIOB,GPIO_Pin_12);//LED快闪
		}
	}
}

__task void Init_task (void)  
{
	__disable_irq();
	
	RCC_Configuration();
	GPIO_Configration();
	TIM2_Configuration();
	NVIC_Configuration();
	DMA_Configration();
	USART1_Configration();
	USART2_Configration();
	USART3_Configration();
	ADC_Configuration();
	//UART4_Configration();
	//UART5_Configration();
	DMA_Cmd(DMA1_Channel4,ENABLE);
	Peripherals_On();

	
	__enable_irq();
	
	/*----------------------create and initial Tasks---------------------*/
	tsk_lock();

	os_tsk_create(IMU_DATA_TRANS_1_9 , 90);
	os_tsk_create(IMU_DATA_TRANS_10_18 , 95);
	os_tsk_create(BATTERY_CHECK , 100);
	//os_tsk_create(IMU_DATA_TRANS_13_18 , 100);

	tsk_unlock();
	os_tsk_delete_self();
}	

int main(void)
{
	os_sys_init(Init_task);
	while(1);
}


 
/**/

void TIM2_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
		isr_evt_set(0x01,IMU_DATA_TRANS_1_9_ID);
		isr_evt_set(0x01,IMU_DATA_TRANS_10_18_ID);
		//isr_evt_set(0x01,IMU_DATA_TRANS_13_18_ID);
		GPIO_SetBits(GPIOA,GPIO_Pin_2);
		GPIO_ResetBits(GPIOA,GPIO_Pin_2);
	}
}

//串口2接收中断   
void USART2_IRQHandler(void)                               
{   
	uint32_t temp = 0;
	uint16_t i = 0;
	
	if(USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)
    {
    	//USART_ClearFlag(USART1,USART_IT_IDLE);
    	temp = USART2->SR;
    	temp = USART2->DR; //清USART_IT_IDLE标志
    	DMA_Cmd(DMA1_Channel6,DISABLE);

		temp = UART_RX_LEN - DMA_GetCurrDataCounter(DMA1_Channel6);
		for (i = 0;i < temp;i++)
		{
			USART2_RX_DATA[i] = USART2_DMA_RX_BUF[i];
		}
		if(USART2_RX_DATA[0]==0xa5&&USART2_RX_DATA[1]==0x5a&&USART2_RX_DATA[2]==i-2&&USART2_RX_DATA[i-1]==0xaa)
		{
			__NOP();
			isr_evt_set(0x02,IMU_DATA_TRANS_1_9_ID);
		}
		else
		{
			__NOP();
		}
			//设置传输数据长度
		DMA_SetCurrDataCounter(DMA1_Channel6,UART_RX_LEN);
    	//打开DMA
		DMA_Cmd(DMA1_Channel6,ENABLE);
    } 
	
	__nop(); 
} 

//串口3接收中断   
void USART3_IRQHandler(void)                               
{   
	uint32_t temp = 0;
	uint16_t i = 0;
	
	if(USART_GetITStatus(USART3, USART_IT_IDLE) != RESET)
	{
		//USART_ClearFlag(USART3,USART_IT_IDLE);
		temp = USART3->SR;
		temp = USART3->DR; //清USART_IT_IDLE标志
		DMA_Cmd(DMA1_Channel3,DISABLE);

		temp = UART_RX_LEN - DMA_GetCurrDataCounter(DMA1_Channel3);
		for (i = 0;i < temp;i++)
		{
			USART3_RX_DATA[i] = USART3_DMA_RX_BUF[i];
		}
		
		if(USART3_RX_DATA[0]==0xa5&&USART3_RX_DATA[1]==0x5a&&USART3_RX_DATA[2]==i-2&&USART3_RX_DATA[i-1]==0xaa)//包头2字节不计入长度
		{
			isr_evt_set(0x02,IMU_DATA_TRANS_10_18_ID);
		}
		else
		{
			__NOP();
		}
		//设置传输数据长度
		DMA_SetCurrDataCounter(DMA1_Channel3,UART_RX_LEN);
			//打开DMA
		DMA_Cmd(DMA1_Channel3,ENABLE);
	} 
	__nop(); 
} 

// //串口4接收中断   
// void USART4_IRQHandler(void)                               
// {   
// 	uint32_t temp = 0;
// 	uint16_t i = 0;
// 	
// 	if(USART_GetITStatus(UART4, USART_IT_IDLE) != RESET)
//     {
//     	//USART_ClearFlag(USART4,USART_IT_IDLE);
//     	temp = UART4->SR;
//     	temp = UART4->DR; //清USART_IT_IDLE标志
//     	DMA_Cmd(DMA2_Channel3,DISABLE);

// 		temp = UART_RX_LEN - DMA_GetCurrDataCounter(DMA2_Channel3);
// 		for (i = 0;i < temp;i++)
// 		{
// 			USART1_RX_DATA[i] = USART2_DMA_RX_BUF[i];
// 		}

// 		//设置传输数据长度
// 		DMA_SetCurrDataCounter(DMA2_Channel3,UART_RX_LEN);
//     	//打开DMA
// 		DMA_Cmd(DMA2_Channel3,ENABLE);
//     } 
// 	
// 	__nop(); 
// } 

//串口1DMA方式发送中断
void DMA1_Channel4_IRQHandler(void)
{
	uint16_t count;
	//清除标志位
  	DMA_ClearFlag(DMA1_FLAG_TC4);
	//DMA_ClearITPendingBit(DMA1_FLAG_TC4);
  	//DMA1->IFCR |= DMA1_FLAG_TC4;
	//关闭DMA
	
	DMA_Cmd(DMA1_Channel4,DISABLE);
	for(count=0;count<SEND_BUFFER_SIZE;count++)
	{
		if(Read_TX_FIFO(&Uart_Send_Buffer[count])==ERROR)
			break;//FIFO为空 提前退出
	}
	if(count>0)
	{
		__disable_irq();
		DMA_SetCurrDataCounter(DMA1_Channel4,count);
		DMA_Cmd(DMA1_Channel4,ENABLE);
		DMA_Statue = 1;
		__enable_irq();
 	}
	else
	{
		DMA_Statue = 0;
	}
		
	//DMA1_Channel4->CCR &= ~(1<<0);
	//允许再次发送
	//Flag_Uart_Send = 0;
}
