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
		GPIO_InitStructure.GPIO_Mode	=	GPIO_Mode_AIN;									//模拟输入
		GPIO_InitStructure.GPIO_Speed	=	GPIO_Speed_50MHz;
		GPIO_Init(GPIOA,&GPIO_InitStructure);
		
		ADC_DeInit(ADC1);
		ADC_InitStructure.ADC_ContinuousConvMode	=	ENABLE;											//工作在连续转换模式
		ADC_InitStructure.ADC_Mode	=	ADC_Mode_Independent;											//工作在独立模式
		ADC_InitStructure.ADC_ExternalTrigConv	=	ADC_ExternalTrigConv_None;		//外部触发转换关闭
		ADC_InitStructure.ADC_ScanConvMode	=	ENABLE;														//模数转换工作在扫描模式
		ADC_InitStructure.ADC_DataAlign	=	ADC_DataAlign_Right;									//ADC数据右对齐
		ADC_InitStructure.ADC_NbrOfChannel	=	N;																//顺序进行的ADC转换的通道数
		ADC_Init(ADC1,&ADC_InitStructure);
		
		//ADC1,ADC通道X，规则采样顺序值Y，采样时间1.5个周期
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
		DMA_InitStructure.DMA_BufferSize	=	N*M;								//DMA通道DMA缓存大小
		DMA_InitStructure.DMA_DIR	=	DMA_DIR_PeripheralSRC;		//内存作为数据传输目的地
		DMA_InitStructure.DMA_M2M	=	DMA_M2M_Disable;					//失能内存到内存
		DMA_InitStructure.DMA_MemoryBaseAddr	=	(u32)&AD_Value;		//DMA的内存基地址。AD_Value为存放数据的数组
		DMA_InitStructure.DMA_MemoryDataSize	=	DMA_MemoryDataSize_HalfWord;		//数据宽度为16位
		DMA_InitStructure.DMA_MemoryInc	=	DMA_MemoryInc_Enable;		//内存地址寄存器递增
		DMA_InitStructure.DMA_Mode	=	DMA_Mode_Circular;					//工作在循环缓存模式
		DMA_InitStructure.DMA_PeripheralBaseAddr	=	(u32)&ADC1->DR;	//DMA外设ADC基地址
		DMA_InitStructure.DMA_PeripheralDataSize	=	DMA_PeripheralDataSize_HalfWord;	//数据宽度为16位
		DMA_InitStructure.DMA_PeripheralInc	=	DMA_PeripheralInc_Disable;				//外设地址寄存器不变
		DMA_InitStructure.DMA_Priority	=	DMA_Priority_High;								//DMA通道1拥有高优先级
			
		DMA_Init(DMA1_Channel1,&DMA_InitStructure);
		
		ADC_SoftwareStartConvCmd(ADC1, ENABLE);		//start ADC1 software conversion
		DMA_Cmd(DMA1_Channel1,ENABLE);												//启动DMA1_Channel1
		
	}
	#endif

	
