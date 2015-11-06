#include "stm32f10x.h"
#include "ADC_config.h"

#define OpenADC1

/*********************************************************
ADC12_IN4->PA4
ADC12_IN5->PA5
ADC12_IN6->PA6

**********************************************************/

#ifdef	OpenADC1
		
	void ADC1_Configration(void)
	{
		GPIO_InitTypeDef GPIO_InitStructure;
		ADC_InitTypeDef	ADC_InitStructure;
		DMA_InitTypeDef DMA_InitStructure;
		
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO ,ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

		GPIO_InitStructure.GPIO_Pin	=	GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6;
		GPIO_InitStructure.GPIO_Mode	=	GPIO_Mode_AIN;									//ģ������
		GPIO_InitStructure.GPIO_Speed	=	GPIO_Speed_50MHz;
		GPIO_Init(GPIOA,&GPIO_InitStructure);
		
		ADC_DeInit(ADC1);
		ADC_InitStructure.ADC_ContinuousConvMode	=	ENABLE;											//����������ת��ģʽ
		ADC_InitStructure.ADC_Mode	=	ADC_Mode_Independent;											//�����ڶ���ģʽ
		ADC_InitStructure.ADC_ExternalTrigConv	=	ADC_ExternalTrigConv_None;		//�ⲿ����ת���ر�
		ADC_InitStructure.ADC_ScanConvMode	=	ENABLE;														//ģ��ת��������ɨ��ģʽ
		ADC_InitStructure.ADC_DataAlign	=	ADC_DataAlign_Right;									//ADC�����Ҷ���
		ADC_InitStructure.ADC_NbrOfChannel	=	N;																//˳����е�ADCת����ͨ����
		ADC_Init(ADC1,&ADC_InitStructure);
		
		//ADC1,ADCͨ��X���������˳��ֵY������ʱ��1.5������
		ADC_RegularChannelConfig(ADC1,ADC_Channel_4,1,ADC_SampleTime_239Cycles5);			//Channel_4
		ADC_RegularChannelConfig(ADC1,ADC_Channel_5,2,ADC_SampleTime_239Cycles5);			//Channel_5
		ADC_RegularChannelConfig(ADC1,ADC_Channel_6,3,ADC_SampleTime_239Cycles5);			//Channel_6
		
		ADC_DMACmd(ADC1,ENABLE);
		ADC_Cmd(ADC1,ENABLE);
		
		/*Enable the ADC1 reset calibration register*/
		ADC_ResetCalibration(ADC1);
		/*Check the end of ADC1 reset calibration register*/
		while(ADC_GetResetCalibrationStatus(ADC1)){};
		
		/*Start the ADC1 calibration*/
		ADC_StartCalibration(ADC1);
		/*Check the end of ADC1 calibration*/
		while(ADC_GetCalibrationStatus(ADC1)){};
		
		DMA_DeInit(DMA1_Channel1);
		DMA_InitStructure.DMA_BufferSize	=	N*M;								//DMAͨ��DMA�����С
		DMA_InitStructure.DMA_DIR	=	DMA_DIR_PeripheralSRC;		//�ڴ���Ϊ���ݴ���Ŀ�ĵ�
		DMA_InitStructure.DMA_M2M	=	DMA_M2M_Disable;					//ʧ���ڴ浽�ڴ�
		DMA_InitStructure.DMA_MemoryBaseAddr	=	(u32)&AD_Value;		//DMA���ڴ����ַ��AD_ValueΪ������ݵ�����
		DMA_InitStructure.DMA_MemoryDataSize	=	DMA_MemoryDataSize_HalfWord;		//���ݿ��Ϊ16λ
		DMA_InitStructure.DMA_MemoryInc	=	DMA_MemoryInc_Enable;		//�ڴ��ַ�Ĵ�������
		DMA_InitStructure.DMA_Mode	=	DMA_Mode_Circular;					//������ѭ������ģʽ
		DMA_InitStructure.DMA_PeripheralBaseAddr	=	(u32)&ADC1->DR;	//DMA����ADC����ַ
		DMA_InitStructure.DMA_PeripheralDataSize	=	DMA_PeripheralDataSize_HalfWord;	//���ݿ��Ϊ16λ
		DMA_InitStructure.DMA_PeripheralInc	=	DMA_PeripheralInc_Disable;				//�����ַ�Ĵ�������
		DMA_InitStructure.DMA_Priority	=	DMA_Priority_High;								//DMAͨ��1ӵ�и����ȼ�
			
		DMA_Init(DMA1_Channel1,&DMA_InitStructure);
		
		ADC_SoftwareStartConvCmd(ADC1, ENABLE);		//start ADC1 software conversion
		DMA_Cmd(DMA1_Channel1,ENABLE);												//����DMA1_Channel1
		
	}
	#endif

	
