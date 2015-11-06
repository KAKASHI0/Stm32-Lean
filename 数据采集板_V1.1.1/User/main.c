#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_adc.h"
#include "RTL.h"
#include "all_init_heard.h"
#include "all_task_heard.h"

/************macro definition*****************/
#define SENSOR_BAUD 512000
//#define WIFI_BAUD 921600
#define WIFI_BAUD 115200
//���ڽ��ջ���
#define UART_RX_LEN		30

#define PACKAGE_SIZE	24 //��ͷ*4 ����*16 crc*4 
#define SEND_BUFFER_SIZE 1000
#define FIFO_SIZE 10000
//#define SEND_BUFFER_SIZE 10

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
//˫����
uint8_t USART1_DMA_RX_BUF[UART_RX_LEN] = {0};//DMA������
uint8_t USART2_DMA_RX_BUF[UART_RX_LEN] = {0};
uint8_t USART3_DMA_RX_BUF[UART_RX_LEN] = {0};
uint8_t UART4_DMA_RX_BUF[UART_RX_LEN] = {0};
uint8_t UART5_DMA_RX_BUF[UART_RX_LEN] = {0};

/*���ݻ����� �ڴ��ڽ�����һ֡�����IDLE�ж�,
��DMA_BUF�����ݸ��Ƶ�RX_DATA�к�,���DMA1_BUF���ڽ���������*/
uint8_t USART1_RX_DATA[UART_RX_LEN] = {0};
uint8_t USART2_RX_DATA[UART_RX_LEN] = {0};
uint8_t USART3_RX_DATA[UART_RX_LEN] = {0};
uint8_t UART4_RX_DATA[UART_RX_LEN] = {0};
uint8_t UART5_RX_DATA[UART_RX_LEN] = {0};

uint8_t Uart_Send_Buffer[SEND_BUFFER_SIZE] = {0};
uint8_t Uart_Receive_Buffer[SEND_BUFFER_SIZE] = {0};

FIFO_QUEUE_STRU TX_FIFO;
FIFO_QUEUE_STRU RX_FIFO;

BOOL DMA_Statue=0;//DMA״̬ 0=ֹͣ���� 1=���ڷ���

/***********task ID***********************/
OS_TID IMU_DATA_TRANS_1_9_ID;
OS_TID IMU_DATA_TRANS_10_18_ID;


/*************function**************/

void GPIO_PinReverse(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)//IOȡ��
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

void RCC_Configuration(void)//����STM32��ʱ��
{
	// Enable CRC clock
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_CRC,ENABLE);
	
	// Enable DMA1 clock 
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	// Enable DMA2 clock 
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);
	
  // Enable AFIO clock 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  
  // Enable TIM2 clock ����ʱ��2�ж�
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	
	// Enable USART1�� 
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

void NVIC_Configuration()//�ж����ú���
{
	NVIC_InitTypeDef NVIC_InitStructure;
	/*---------------- Configure and enable TIM2 interrupt ---------------------------*/
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	
	
	//UART1 �ж�����
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;               //ͨ������Ϊ����1�ж�  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;       //�ж�ռ�ȵȼ�2  
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;              //�ж���Ӧ���ȼ�1  
	//NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;                 //��ʱ���������� �����ж� 
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                  //���ж�   	
	NVIC_Init(&NVIC_InitStructure);   
	
	//����UART2�ж�  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;               //ͨ������Ϊ����2�ж�  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;       //�ж�ռ�ȵȼ�0  
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;              //�ж���Ӧ���ȼ�0  
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                 //���ж�  
	NVIC_Init(&NVIC_InitStructure);   
	
	//����UART3�ж�  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;               //ͨ������Ϊ����3�ж�  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;       //�ж�ռ�ȵȼ�0  
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;              //�ж���Ӧ���ȼ�0  
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                 //���ж�  
	NVIC_Init(&NVIC_InitStructure);   

	//DMA�����ж�����
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
	
	//DMA����
	DMA_Cmd(DMA1_Channel4,DISABLE);
	//�ָ�ȱʡֵ
	DMA_DeInit(DMA1_Channel4);
	//�����ַ
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&USART1->DR);
	//�ڴ��ַ
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)Uart_Send_Buffer;
	//dma���䷽����
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	//����DMA�ڴ���ʱ�������ĳ���
	DMA_InitStructure.DMA_BufferSize = SEND_BUFFER_SIZE;
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
	DMA_ITConfig(DMA1_Channel4,DMA_IT_TC,ENABLE);//��������DMAͨ���ж�
	
	//ʹ��ͨ��4
	DMA_Cmd(DMA1_Channel4, ENABLE);//�ر�DMA����ͨ��

	//����1����DMA����  
	
	//DMA1ͨ��5����
	DMA_Cmd(DMA1_Channel5,DISABLE);
	DMA_DeInit(DMA1_Channel5);
	//�����ַ
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&USART1->DR);
	//�ڴ��ַ
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)USART1_DMA_RX_BUF;
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
	DMA_Cmd(DMA1_Channel5,ENABLE);



	/**********************USART2 DMA COnfiguration*************/
	//����2��DMA����  
	//DMA1ͨ��6����
	DMA_DeInit(DMA1_Channel6);
	//�����ַ
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&USART2->DR);
	//�ڴ��ַ
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)USART2_DMA_RX_BUF;
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
	DMA_Init(DMA1_Channel6,&DMA_InitStructure);

	//ʹ��ͨ��6
	DMA_Cmd(DMA1_Channel6,ENABLE);	
	
	/**********************USART3 DMA COnfiguration*************/
	//����3��DMA����  
	//DMA1ͨ��3����
	DMA_DeInit(DMA1_Channel3);
	//�����ַ
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&USART3->DR);
	//�ڴ��ַ
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)USART3_DMA_RX_BUF;
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
	DMA_Init(DMA1_Channel3,&DMA_InitStructure);

	//ʹ��ͨ��3
	DMA_Cmd(DMA1_Channel3,ENABLE);		
	
}




/**************************************************************************************/

void USART1_Configration()
{
	
	USART_InitTypeDef USART_InitStructure;//����USART
	//ʱ������ ��RCC���ú�����
	//GPIO ������ GPIO_Init.c������

	//���ô���
	/*USART_InitStructure.USART_WordLength = USART_WordLength_8b;  
	USART_InitStructure.USART_StopBits = USART_StopBits_1;  
	USART_InitStructure.USART_Parity = USART_Parity_No;  
	//USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_RTS_CTS;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;    
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;    //�շ�ģʽ
	USART_InitStructure.USART_BaudRate = WIFI_BAUD;
	USART_Init(USART1,&USART_InitStructure);  
	
	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);  //ʹ�ܴ���1�����ж�
	USART_ITConfig(USART1,USART_IT_TC,DISABLE);   //0ʹ�ܷ����жϣ�
	USART_ITConfig(USART1,USART_IT_IDLE,ENABLE);	 //ʹ�ܿ����ж�
	
	
	USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);
	USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);
	USART_Cmd(USART1, ENABLE);   //��������*/

	/*********************************************************************/
	
    //��ʼ������  
    USART_InitStructure.USART_BaudRate = WIFI_BAUD;  
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;  
    USART_InitStructure.USART_StopBits = USART_StopBits_1;  
    USART_InitStructure.USART_Parity = USART_Parity_No;  
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;    
	//��ʼ������ 
    USART_Init(USART1,&USART_InitStructure);  
    //TXE�����ж�,TC��������ж�,RXNE�����ж�,PE��ż�����ж�,�����Ƕ��   
    //USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);
	
	//�ж�����
	USART_ITConfig(USART1,USART_IT_TC,ENABLE);
	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);
	USART_ITConfig(USART1,USART_IT_IDLE,ENABLE);  
	
	//������DMA��ʽ����
	USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);
	//����DMA��ʽ����
	USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);
    //��������  
  USART_Cmd(USART1, ENABLE); 
	
	
}

/**************************************************************************************/

void USART2_Configration()
{
	USART_InitTypeDef USART_InitStructure;
    //��ʼ������  
    USART_InitStructure.USART_BaudRate = SENSOR_BAUD;  
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;  
    USART_InitStructure.USART_StopBits = USART_StopBits_1;  
    USART_InitStructure.USART_Parity = USART_Parity_No;  
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;    
	//��ʼ������ 
    USART_Init(USART2,&USART_InitStructure);  
    //TXE�����ж�,TC��������ж�,RXNE�����ж�,PE��ż�����ж�,�����Ƕ��   
    //USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);
	
	//�ж�����
	USART_ITConfig(USART2,USART_IT_TC,DISABLE);
	USART_ITConfig(USART2,USART_IT_RXNE,DISABLE);
	USART_ITConfig(USART2,USART_IT_IDLE,ENABLE);  
	
	//������DMA��ʽ����
	//USART_DMACmd(USART2,USART_DMAReq_Tx,DISABLE);
	//����DMA��ʽ����
	//USART_DMACmd(USART2,USART_DMAReq_Rx,ENABLE);
    //��������  
  //USART_Cmd(USART2, ENABLE); 
}
void USART3_Configration()
{
	USART_InitTypeDef USART_InitStructure;
    //��ʼ������  
    //USART_InitStructure.USART_BaudRate = SENSOR_BAUD;  
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;  
    USART_InitStructure.USART_StopBits = USART_StopBits_1;  
    USART_InitStructure.USART_Parity = USART_Parity_No;  
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;    
    USART_InitStructure.USART_BaudRate = SENSOR_BAUD; 
	//��ʼ������ 
    USART_Init(USART3,&USART_InitStructure);  
    //TXE�����ж�,TC��������ж�,RXNE�����ж�,PE��ż�����ж�,�����Ƕ��   
    //USART_ITConfig(USART3,USART_IT_RXNE,ENABLE);
	
	//�ж�����
	USART_ITConfig(USART3,USART_IT_TC,DISABLE);
	USART_ITConfig(USART3,USART_IT_RXNE,DISABLE);
	USART_ITConfig(USART3,USART_IT_IDLE,ENABLE);  
	
	//������DMA��ʽ����
	//USART_DMACmd(USART3,USART_DMAReq_Tx,DISABLE);
	//����DMA��ʽ����
	//USART_DMACmd(USART3,USART_DMAReq_Rx,ENABLE);
    //��������  
  //USART_Cmd(USART3, ENABLE); 
}


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


ErrorStatus Write_RX_FIFO(uint8_t *pdata,uint8_t length)
{
	uint8_t i;
  if(FIFO_SIZE - RX_FIFO.count >= length) 
  { 
		__disable_irq();
		for (i=0; i<length; i++)
		{
			RX_FIFO.data[RX_FIFO.rear] = *(pdata+i); 
			RX_FIFO.rear = (RX_FIFO.rear + 1) % FIFO_SIZE; 
			RX_FIFO.count = RX_FIFO.count + 1; 
			if (RX_FIFO.count == FIFO_SIZE)
			{				
				RX_FIFO.rear = RX_FIFO.front;
			}
		}
		__enable_irq();
    return SUCCESS; 
  } 
	else
	  return ERROR;
}

ErrorStatus Read_RX_FIFO(uint8_t *data)
{	
  if((RX_FIFO.front == RX_FIFO.rear) && (RX_FIFO.count == 0)) 
  {
    return ERROR; 
  }
	else 
  {
		__disable_irq();
    *data = RX_FIFO.data[TX_FIFO.front]; 
    RX_FIFO.front = (RX_FIFO.front + 1) % FIFO_SIZE; 
    RX_FIFO.count = RX_FIFO.count - 1; 
		__enable_irq();
    return SUCCESS; 
  }
}

//���ݷ��Ϳ��� ״̬��
void Send_Data(void)
{
	uint8_t count;
	if(DMA_Statue==0)//DMAֹͣ����
	{
		for(count=0;count<SEND_BUFFER_SIZE;count++)
		{
			if(Read_TX_FIFO(&Uart_Send_Buffer[count])==ERROR)
				break;//FIFOΪ�� ��ǰ�˳�
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


//��ȡ���ӵ�����2�ϵĴ�����������
//��һ����������ȡ���ݣ�У���д�뵽����fifo����������
void IMU_Get_DATA2(USART_TypeDef* USARTx,uint8_t imu_id)
{
	USART_SendData(USARTx,imu_id);
	if(os_evt_wait_or(0x02,2)!=OS_R_TMO)
	{
		Write_TX_FIFO(USART2_RX_DATA,USART2_RX_DATA[2]+2);//���ݳ��ȼ��ϰ�ͷ2�ֽ�
		Send_Data();
		
	}
}

//��һ����������ȡ���ݣ�У���д�뵽����fifo����������
void IMU_Get_DATA3(USART_TypeDef* USARTx,uint8_t imu_id)
{
	USART_SendData(USARTx,imu_id);
	if(os_evt_wait_or(0x02,2)!=OS_R_TMO)
	{
		
		Write_TX_FIFO(USART3_RX_DATA,USART3_RX_DATA[2]+2);//���ݳ��ȼ��ϰ�ͷ2�ֽ�
		//Write_TX_FIFO(USART3_RX_DATA,USART3_RX_DATA[2]+2);//���ݳ��ȼ��ϰ�ͷ2�ֽ�
		Send_Data();
	}
}

//ɨ��1-9оƬ
__task void IMU_DATA_TRANS_1_9 (void) //#define __task  __declspec(noreturn) �ӿ��ٶ�
{
	IMU_DATA_TRANS_1_9_ID = os_tsk_self();
	while(1)
	{
		os_evt_wait_or(0x01,0xffff);
		//GPIO_SetBits(GPIOA,GPIO_Pin_3);
// 		os_evt_wait_or(0xff,1);//������ϵͳʱ��ͬ��
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

//ɨ��10-18оƬ
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

__task void BATTERY_CHECK(void)
{
	float AD_value,Battery_value;
	uint8_t i=0;
	while(1)
	{
		os_dly_wait(500);//100ms���һ�ε���
		//USART_SendData(USART1,0xaa);
		AD_value = ADC_ConvertedValue;
		Battery_value = (AD_value/4096)*3.3*2;
		//if(Battery_value>3.68)
		//{
		//	Peripherals_On();
		//	GPIO_ResetBits(GPIOB,GPIO_Pin_12);
		//}
		//else if(Battery_value>3.5&&Battery_value<3.68)
		{
			i++;
			if(i>5)
			{
				i=0;
				GPIO_PinReverse(GPIOB,GPIO_Pin_12);//LED����
			}
		}
		//else if(Battery_value<3.5)
		if(Battery_value<3.5)
		{
			Peripherals_Off();
			GPIO_PinReverse(GPIOB,GPIO_Pin_12);//LED����
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
	
	Peripherals_On();

	
	__enable_irq();
	
	/*----------------------create and initial Tasks---------------------*/
	tsk_lock();

	os_tsk_create(IMU_DATA_TRANS_1_9 , 90);//�������� �涨���ȼ� 0-255 Ĭ��1 Խ�����ȼ�Խ��
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
//��ʱɨ��
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


//����1�����ж�
void USART1_IRQHandler(void)
{
	uint32_t temp = 0;
	uint16_t i = 0;
	if(USART_GetITStatus(USART1, USART_IT_TC) != RESET)
	{
		USART_ClearITPendingBit(USART1,USART_IT_TC);//����жϱ�־
	}
	if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)
    {
    	//USART_ClearFlag(USART1,USART_IT_IDLE);
    	temp = USART1->SR;
    	temp = USART1->DR; //��USART_IT_IDLE��־
    	DMA_Cmd(DMA1_Channel5,DISABLE);//�ر�DMA ��ֹ����

		temp = UART_RX_LEN - DMA_GetCurrDataCounter(DMA1_Channel5);
		for (i = 0;i < temp;i++)
		{
			USART1_RX_DATA[i] = USART1_DMA_RX_BUF[i];
		}
		Write_TX_FIFO(USART1_RX_DATA,temp-1);//���ݳ��ȼ��ϰ�ͷ2�ֽ�
		Send_Data();
			//__NOP();
			//���ô������ݳ���
		DMA_SetCurrDataCounter(DMA1_Channel5,UART_RX_LEN);
    	//��DMA
		DMA_Cmd(DMA1_Channel5,ENABLE);
    } 
	__nop(); 
}


//����2�����ж�   
void USART2_IRQHandler(void)                               
{   
	uint32_t temp = 0;
	uint16_t i = 0;
	
	if(USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)
    {
    	//USART_ClearFlag(USART1,USART_IT_IDLE);
    	temp = USART2->SR;
    	temp = USART2->DR; //��USART_IT_IDLE��־
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
			//���ô������ݳ���
		DMA_SetCurrDataCounter(DMA1_Channel6,UART_RX_LEN);
    	//��DMA
		DMA_Cmd(DMA1_Channel6,ENABLE);
    } 
	
	__nop(); 
} 

//����3�����ж�   
void USART3_IRQHandler(void)                               
{   
	uint32_t temp = 0;
	uint16_t i = 0;
	
	if(USART_GetITStatus(USART3, USART_IT_IDLE) != RESET)
	{
		//USART_ClearFlag(USART3,USART_IT_IDLE);
		temp = USART3->SR;
		temp = USART3->DR; //��USART_IT_IDLE��־
		DMA_Cmd(DMA1_Channel3,DISABLE);

		temp = UART_RX_LEN - DMA_GetCurrDataCounter(DMA1_Channel3);
		for (i = 0;i < temp;i++)
		{
			USART3_RX_DATA[i] = USART3_DMA_RX_BUF[i];
		}
		
		if(USART3_RX_DATA[0]==0xa5&&USART3_RX_DATA[1]==0x5a&&USART3_RX_DATA[2]==i-2&&USART3_RX_DATA[i-1]==0xaa)//��ͷ2�ֽڲ����볤��
		{
			isr_evt_set(0x02,IMU_DATA_TRANS_10_18_ID);
		}
		else
		{
			__NOP();
		}
		//���ô������ݳ���
		DMA_SetCurrDataCounter(DMA1_Channel3,UART_RX_LEN);
			//��DMA
		DMA_Cmd(DMA1_Channel3,ENABLE);
	} 
	__nop(); 
} 



//����1DMA��ʽ�����ж�
void DMA1_Channel4_IRQHandler(void)
{
	uint16_t count;
	//�����־λ
  	DMA_ClearFlag(DMA1_FLAG_TC4);
	//DMA_ClearITPendingBit(DMA1_FLAG_TC4);
  	//DMA1->IFCR |= DMA1_FLAG_TC4;
	//�ر�DMA
	
	DMA_Cmd(DMA1_Channel4,DISABLE);
	for(count=0;count<SEND_BUFFER_SIZE;count++)
	{
		if(Read_TX_FIFO(&Uart_Send_Buffer[count])==ERROR)//������FIFO�е����ݶ���Uart_Send_Buffer�з���
			break;//FIFOΪ�� ��ǰ�˳�
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
	//�����ٴη���
	//Flag_Uart_Send = 0;
}


