/**
  ******************************************************************************
  * @file    RC_Channel_Init.c
  * @author  HR_Z
  * @version V1.0
  * @date    15-November-2013
  * @brief   This file provides all the RC channel init functions.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, IROBOT SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2013 Irobot</center></h2>
  ******************************************************************************/

#include "stm32f10x_rc.h"

/* --------------------------------- RC Hardware ----------------------------------- */
RC_HardWare_Struct RC_Hardware[RC_USED_NUM]=
{	
#ifdef RC1
			{1,GPIOC,GPIO_Pin_0,TIM2,GPIO_PortSourceGPIOC,GPIO_PinSource0,EXTI_Line0,EXTI0_IRQn},			//RC1
#endif
#ifdef RC2
			{2,GPIOC,GPIO_Pin_1,TIM2,GPIO_PortSourceGPIOC,GPIO_PinSource1,EXTI_Line1,EXTI1_IRQn},			//RC2
#endif
#ifdef RC3
			{3,GPIOC,GPIO_Pin_2,TIM2,GPIO_PortSourceGPIOC,GPIO_PinSource2,EXTI_Line2,EXTI2_IRQn},			//RC3
#endif
#ifdef RC4
			{4,GPIOC,GPIO_Pin_3,TIM2,GPIO_PortSourceGPIOC,GPIO_PinSource3,EXTI_Line3,EXTI3_IRQn},			//RC4
#endif
#ifdef RC5
			{5,GPIOC,GPIO_Pin_4,TIM2,GPIO_PortSourceGPIOC,GPIO_PinSource4,EXTI_Line4,EXTI4_IRQn},			//RC5
#endif
#ifdef RC6
			{6,GPIOC,GPIO_Pin_5,TIM2,GPIO_PortSourceGPIOC,GPIO_PinSource5,EXTI_Line5,EXTI9_5_IRQn},		//RC6
#endif
#ifdef RC7
			{7,GPIOA,GPIO_Pin_6,TIM2,GPIO_PortSourceGPIOA,GPIO_PinSource6,EXTI_Line6,EXTI9_5_IRQn},		//RC7
#endif
#ifdef RC8
			{8,GPIOA,GPIO_Pin_7,TIM2,GPIO_PortSourceGPIOA,GPIO_PinSource7,EXTI_Line7,EXTI9_5_IRQn},		//RC8	
#endif
#ifdef RC9
			{9,GPIOA,GPIO_Pin_8,TIM2,GPIO_PortSourceGPIOA,GPIO_PinSource8,EXTI_Line8,EXTI9_5_IRQn},		//RC9
#endif
	};

/* --------------------------- PWM_BASE --------------------------- */
#ifdef RC1
		RC_TypeDef_Struct  RC1_Registers;
#endif
#ifdef RC2
		RC_TypeDef_Struct  RC2_Registers;
#endif
#ifdef RC3
		RC_TypeDef_Struct  RC3_Registers;
#endif
#ifdef RC4
		RC_TypeDef_Struct  RC4_Registers;
#endif
#ifdef RC5
		RC_TypeDef_Struct  RC5_Registers;
#endif
#ifdef RC6
		RC_TypeDef_Struct  RC6_Registers;
#endif
#ifdef RC7
		RC_TypeDef_Struct  RC7_Registers;
#endif
#ifdef RC8
		RC_TypeDef_Struct  RC8_Registers;
#endif
#ifdef RC9
		RC_TypeDef_Struct  RC9_Registers;
#endif	

	/**
  * @brief  Initializes the RCx  according to the specified
  *   parameters in the RC_InitStruct.
  * @param RCx: where x can be  1-9  to select the RC.
  * @param RC_InitStruct : pointer to a RC_InitTypeDef
  *   structure which will be initialized.
  * @retval : None
  */
void RC_Init(RC_TypeDef_Struct* RCx, RC_InitTypeDef_Struct*  RC_InitStruct)
{
		GPIO_InitTypeDef 					GPIO_InitStructure;
		TIM_TimeBaseInitTypeDef  	TIM_TimeBaseStructure;
		EXTI_InitTypeDef   				EXTI_InitStructure;
		NVIC_InitTypeDef   				NVIC_InitStructure;
	
/* RCx_Registers->RCx_Hardware Configuration  */
#ifdef RC1
	if(RCx==RC1)
		{
	    RC1_Registers.RCx_Hardware = &RC_Hardware[0];
		}
#endif
#ifdef RC2
	if(RCx==RC2)
		{
	    RC2_Registers.RCx_Hardware = &RC_Hardware[1];
		}
#endif
#ifdef RC3
	if(RCx==RC3)
		{
	    RC3_Registers.RCx_Hardware = &RC_Hardware[2];
		}
#endif
#ifdef RC4
	if(RCx==RC4)
		{
	    RC4_Registers.RCx_Hardware = &RC_Hardware[3];
		}
#endif
#ifdef RC5
	if(RCx==RC5)
		{
	    RC5_Registers.RCx_Hardware = &RC_Hardware[4];
		}
#endif
#ifdef RC6
	if(RCx==RC6)
		{
	    RC6_Registers.RCx_Hardware = &RC_Hardware[5];
		}
#endif
#ifdef RC7
	if(RCx==RC7)
		{
	    RC7_Registers.RCx_Hardware = &RC_Hardware[6];
		}
#endif
#ifdef RC8
	if(RCx==RC8)
		{
	    RC8_Registers.RCx_Hardware = &RC_Hardware[7];
		}
#endif
#ifdef RC9
	if(RCx==RC9)
		{
	    RC9_Registers.RCx_Hardware = &RC_Hardware[8];
		}
#endif
	/*RCx_Hardware configration*/
			
				 /* RC GPIO Configuration */
					GPIO_StructInit(&GPIO_InitStructure);
					GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
					GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
					GPIO_InitStructure.GPIO_Pin = RCx->RCx_Hardware->GPIO_Pin;
					GPIO_Init(RCx->RCx_Hardware->GPIOx, &GPIO_InitStructure);
		
					/* RC TIM Configuration */
					TIM_TimeBaseStructure.TIM_Period = 0xffff;
					TIM_TimeBaseStructure.TIM_ClockDivision = 0;		
					TIM_TimeBaseStructure.TIM_Prescaler = TIM_PRESCALE_7;
					TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
					TIM_TimeBaseInit(RCx->RCx_Hardware->TIMx, &TIM_TimeBaseStructure); 
					TIM_Cmd(RCx->RCx_Hardware->TIMx,ENABLE);	
		
					/* RC EXIT Configuration */
					EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
					EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
					EXTI_InitStructure.EXTI_LineCmd = ENABLE;
					GPIO_EXTILineConfig(RCx->RCx_Hardware->EXTI_GPIO_PortSource, RCx->RCx_Hardware->EXTI_GPIO_PinSource);
					EXTI_InitStructure.EXTI_Line =	RCx->RCx_Hardware->EXTI_Line;
					EXTI_Init(&EXTI_InitStructure);
					
					/* RC NVIC Configuration */
					NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = RC_GPIO_NVIC_PREEMPTIONPRIORITY_DEFAULT;
					NVIC_InitStructure.NVIC_IRQChannelSubPriority = RC_GPIO_NVIC_SUBPRIORITY_DEFAULT;
					NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
					NVIC_InitStructure.NVIC_IRQChannel = RCx->RCx_Hardware->NVIC_IRQChannel;
					NVIC_Init(&NVIC_InitStructure);
					
					
					RCx->mode = RC_InitStruct->mode;
					RCx->enableFlag = RC_InitStruct->enableFlag;
					RCx->prec =	RC_InitStruct->precision;
					
					RCx->freshFlag = FRESHED;
					RCx->value.ivalue = 0;
					RCx->value.bvalue = 0;
					RCx->value.tvalue = T_MIDDLE;
					RCx->orgvalue_StartTime = 0;
					RCx->orgvalue = MID_POSITION_VALUE_DEFAULT;
					RCx->midPositionValue = MID_POSITION_VALUE_DEFAULT;

}

/**
  * @brief  Fills each RC_InitStruct member with its default value.
  * @param RC_InitStruct : pointer to a RC_InitTypeDef
  *   structure which will be initialized.
  * @retval : None
  */
void RC_StructInit(RC_InitTypeDef_Struct* RC_InitStruct)
{	
/* Set the default configuration */
	RC_InitStruct->enableFlag = DISABLE;
	RC_InitStruct->mode = ANALOG_MODE;
	RC_InitStruct->precision = MIDDLEPRECISION;
}

/**
  * @brief  Eables the specified RC peripheral.
  * @param RCx: where x can be 1-9 to select the RCx peripheral.
  * @retval : None
  */
void RC_On(RC_TypeDef_Struct* RCx)
{
		
		RCx->enableFlag = ENABLE;   //enable  irq
}

/**
  * @brief  Disables the specified RC peripheral.
  * @param RCx: where x can be 1-9 to select the RCx peripheral.
  * @retval : None
  */
void RC_Off(RC_TypeDef_Struct* RCx)
{
		RCx->enableFlag = DISABLE;  //disable irq
		
}
/*Clear the FreshFlag*/
//void RC_ClearFlag(RC_TypeDef_Struct* RCx)
//{
//		RCx->freshFlag = NOT_FRESHED;
//}


/**
  * @brief  Get the Value of the RCx.
  * @param RCx: where x can be 1-9 to select the RCx peripheral.
  * @retval : Value_Type_Struct
		only when thr RCx have fresh the freshFlag,can the function to get
	  the value of the RCx according RCx mode.
  */

//ErrorStatus


ErrorStatus RC_GetOriginalValue(RC_TypeDef_Struct* RCx, RC_OrgValue_Type_Struct* porgvalue)
{
		//parameters are valid ?
	
	porgvalue->freshFlag = RCx->RCx_Register.freshFlag;
	//porgvalue->orgvalue  = RCx->RCx_Register.endtime_pulse - RCx->RCx_Register.starttime_pulse;??
	
}

ErrorStatus RC_GetValue(RC_TypeDef_Struct* RCx, RC_Value_Type_Struct* rcvalue)
{
	
	//parameters are valid ?
	
	if(RCx->freshFlag == FRESHED)
	{
		
			/*RCx's mode is Analog 
			*In this mode ,the retval is int
			*retval: HIGHPRECISION:  -9990 to +9990
			*			   MIDDLEPRECISION: -4995 to  +4995
			*        LOWPRECISION:    -1980 to +1980
		  */
			if(RCx->mode == ANALOG_MODE)
			{
					/*RCx's Precision is High */
					if(RCx->prec == HIGHPRECISION)
					{
						__disable_irq();
							RCx->value.ivalue = (RCx->orgvalue - MID_POSITION_VALUE_DEFAULT)*22.2;	//MAX RC_VALUE:1950,1950-1500*22.2=9990; 
																																											//MIN RC_VALUE:1050,1050-1500*22.2=-9990;
						__enable_irq();																														//SO : -9990~+9990
								return RCx->value;
					}
					/*RCx's Precision is Middle */
					if(RCx->prec == MIDDLEPRECISION)
					{
						__disable_irq();
						RCx->value.ivalue = (RCx->orgvalue - MID_POSITION_VALUE_DEFAULT)*11.1;		//MAX RC_VALUE:1950,1950-1500*11.1=4995;
																																											//MIN RC_VALUE:1050,1050-1500*11.1=-4995;
						__enable_irq();																														//SO : -4995~+4995
						return RCx->value;
					}
					/*RCx's Precision is Low */
					if(RCx->prec == LOWPRECISION)
					{
						__disable_irq();
						RCx->value.ivalue = (RCx->orgvalue - MID_POSITION_VALUE_DEFAULT)*4.4;			//MAX RC_VALUE:1950,1950-1500*4.4=1980;
																																											//MIN RC_VALUE:1050,1050-1500*4.4=-1980;
						__enable_irq();																														//SO:-1980~+1980
						
						/*When the function has got the new value,refresh the freshFlag*/
			     RCx->freshFlag = NOT_FRESHED;
						return RCx->value;
					}			
			}
			
			/*RCx's mode is Binary 
			*In this mode ,the retval is boolean
			*if the orgvalue bigger than 1500  return 1 else return 0
			*retval:0 or 1
			*/
			else if(RCx->mode == BINARY_MODE)
			{
					__disable_irq();
						if(RCx->orgvalue > 1500)
								RCx->value.bvalue = 1;
						else 
								RCx->value.bvalue = 0;
						__enable_irq();
						
					/*When the function has got the new value,refresh the freshFlag*/
					RCx->freshFlag = NOT_FRESHED;
						return RCx->value;
			}
			
			/*RCx's mode is Ternary 
			*In this mode , the retval have three  kinds
			*if the orgvalue bigger than 1700  return T_HIGH,if between 1500 to 1700 return T_MIDDLE, else return 0
			*retval: T_LOW , T_MIDDLE  or T_HIGH
			*/
			else if(RCx->mode == TERNARY_MODE)
			{
					__disable_irq();
						if(RCx->orgvalue > 1700)
								RCx->value.tvalue = T_HIGH;
						else if (RCx->orgvalue > 1300& RCx->orgvalue < 1700)
								RCx->value.tvalue = T_MIDDLE;
						else if(RCx->orgvalue < 1300)
								RCx->value.tvalue = T_LOW;
						__enable_irq();
						
						/*When the function has got the new value,refresh the freshFlag*/
						RCx->freshFlag = NOT_FRESHED;
						return RCx->value;
			}
		}
		return RCx->value;
}

///*Get the Origional value of RCx*/
//uint16_t RC_GetOrgValue(RC_TypeDef_Struct* RCx)
//{
//	return RCx->orgvalue;
//}

/*Get the FreshFlag of RCx*/
RC_FreshStatus_Enum RC_GetFlagStatus(RC_TypeDef_Struct* RCx)
{
		return RCx->freshFlag;
}


/*To catch the RCx signal*/
/**
  * @brief  Get the real signal of the RCx peripherals.
  * @param RCx: where x can be 1-9 to select the RCx peripheral.
  * @retval : None
		used the TIMER to count the time value of the RC siganl 
    the min scale in 1us 
  */
void CatchExternalRC(RC_TypeDef_Struct* RCx)
{
		int16_t 	orvalue;
	
	
		if(EXTI_GetITStatus(RCx->RCx_Hardware->EXTI_Line) != RESET)
		{
				  EXTI_ClearITPendingBit(RCx->RCx_Hardware->EXTI_Line); 
					if(RCx->RCx_Hardware->GPIOx->IDR & RCx->RCx_Hardware->GPIO_Pin)			//wait the rising edge
					 { 
						 __disable_irq();
						 RCx->orgvalue_StartTime = TIM_GetCounter(RCx->RCx_Hardware->TIMx);
						 __enable_irq();
					 }
					 else														//wati the falling edge
							{		
								__disable_irq();
								orvalue = TIM_GetCounter(RCx->RCx_Hardware->TIMx) - RCx->orgvalue_StartTime;
								RCx->orgvalue = orvalue;
								RCx->freshFlag = FRESHED ;
								__enable_irq();
							}			 
		}	
}


/**
  * @brief  Get the real signal of the RCx peripherals.
  * @param RCx: where x can be 1-9 to select the RCx peripheral.
  * @retval : None
		used the TIMER to count the time value of the RC siganl 
    the min scale in 1us 
  */
void RC1_CatchExternalIntTime(void)
{
		if(RC1->RCx_Hardware->GPIOx->IDR & RC1->RCx_Hardware->GPIO_Pin)			//wait the rising edge
		{
				RC1->orgvalue_StartTime = TIM_GetCounter(RC1->RCx_Hardware->TIMx);
				RC1->freshFlag = FRESHING ;	
		}
		else if( RC1->freshFlag == FRESHING )													//wati the falling edge
		{		
			
				RC1->orgvalue_EndTime = TIM_GetCounter(RCx->RCx_Hardware->TIMx);
				RC1->freshFlag = FRESHED ;		 
		}	
		else
		{
				RC1->freshFlag = NOTFRESHED ;
		}
}


/**
  * @brief  Initializes the RCx_IRQHandler according to the specified parameters
  *         in the RC_HareWare_Struct.
  * @param  RC_EXITx_IRQHandler: where x can be 1, 2,3,4 or 9_5 to select the EXIT_Handler.
  * @param  
  * @retval None
  */

void RC_EXTI0_IRQHandler(void)		//RC1
{
 #ifdef RC1
    if ((RC1->RCx_Hardware->NVIC_IRQChannel == EXTI0_IRQn)&&
			  (EXTI_GetITStatus(RC1->RCx_Hardware->EXTI_Line) != RESET))
		{
        __disable_irq();			
        RC1_CatchExternalIntTime();
				EXTI_ClearITPendingBit(RC1->RCx_Hardware->EXTI_Line); 			
				__enable_irq();			
		}	
#endif
		
#ifdef RC2
    if (RC2->RCx_Hardware->NVIC_IRQChannel == EXTI0_IRQn)	  
				CatchExternalRC(RC2);
#endif	

#ifdef RC3
    if (RC3->RCx_Hardware->NVIC_IRQChannel == EXTI0_IRQn)	  
				CatchExternalRC(RC3);
#endif
		
#ifdef RC4
    if (RC4->RCx_Hardware->NVIC_IRQChannel == EXTI0_IRQn)	  
				CatchExternalRC(RC4);
#endif				
	
#ifdef RC5
    if (RC5->RCx_Hardware->NVIC_IRQChannel == EXTI0_IRQn)	  
				CatchExternalRC(RC5);
#endif
		
#ifdef RC6
    if (RC6->RCx_Hardware->NVIC_IRQChannel == EXTI0_IRQn)	  
				CatchExternalRC(RC6);
#endif	

#ifdef RC7
    if (RC7->RCx_Hardware->NVIC_IRQChannel == EXTI0_IRQn)	  
				CatchExternalRC(RC7);
#endif
		
#ifdef RC8
    if (RC8->RCx_Hardware->NVIC_IRQChannel == EXTI0_IRQn)	  
				CatchExternalRC(RC8);
#endif	

#ifdef RC9
    if (RC9->RCx_Hardware->NVIC_IRQChannel == EXTI0_IRQn)	  
				CatchExternalRC(RC9);
#endif				
}

void RC_EXTI1_IRQHandler(void)		//RC1
{
#ifdef RC1
    if (RC1->RCx_Hardware->NVIC_IRQChannel == EXTI1_IRQn)	  
				CatchExternalRC(RC1);
#endif
		
#ifdef RC2
    if (RC2->RCx_Hardware->NVIC_IRQChannel == EXTI1_IRQn)	  
				CatchExternalRC(RC2);
#endif	

#ifdef RC3
    if (RC3->RCx_Hardware->NVIC_IRQChannel == EXTI1_IRQn)	  
				CatchExternalRC(RC3);
#endif
		
#ifdef RC4
    if (RC4->RCx_Hardware->NVIC_IRQChannel == EXTI1_IRQn)	  
				CatchExternalRC(RC4);
#endif				
	
#ifdef RC5
    if (RC5->RCx_Hardware->NVIC_IRQChannel == EXTI1_IRQn)	  
				CatchExternalRC(RC5);
#endif
		
#ifdef RC6
    if (RC6->RCx_Hardware->NVIC_IRQChannel == EXTI1_IRQn)	  
				CatchExternalRC(RC6);
#endif	

#ifdef RC7
    if (RC7->RCx_Hardware->NVIC_IRQChannel == EXTI1_IRQn)	  
				CatchExternalRC(RC7);
#endif
		
#ifdef RC8
    if (RC8->RCx_Hardware->NVIC_IRQChannel == EXTI1_IRQn)	  
				CatchExternalRC(RC8);
#endif	

#ifdef RC9
    if (RC9->RCx_Hardware->NVIC_IRQChannel == EXTI1_IRQn)	  
				CatchExternalRC(RC9);
#endif			
}

void RC_EXTI2_IRQHandler(void)		//RC1
{
#ifdef RC1
    if (RC1->RCx_Hardware->NVIC_IRQChannel == EXTI2_IRQn)	  
				CatchExternalRC(RC1);
#endif
		
#ifdef RC2
    if (RC2->RCx_Hardware->NVIC_IRQChannel == EXTI2_IRQn)	  
				CatchExternalRC(RC2);
#endif	

#ifdef RC3
    if (RC3->RCx_Hardware->NVIC_IRQChannel == EXTI2_IRQn)	  
				CatchExternalRC(RC3);
#endif
		
#ifdef RC4
    if (RC4->RCx_Hardware->NVIC_IRQChannel == EXTI2_IRQn)	  
				CatchExternalRC(RC4);
#endif				
	
#ifdef RC5
    if (RC5->RCx_Hardware->NVIC_IRQChannel == EXTI2_IRQn)	  
				CatchExternalRC(RC5);
#endif
		
#ifdef RC6
    if (RC6->RCx_Hardware->NVIC_IRQChannel == EXTI2_IRQn)	  
				CatchExternalRC(RC6);
#endif	

#ifdef RC7
    if (RC7->RCx_Hardware->NVIC_IRQChannel == EXTI2_IRQn)	  
				CatchExternalRC(RC7);
#endif
		
#ifdef RC8
    if (RC8->RCx_Hardware->NVIC_IRQChannel == EXTI2_IRQn)	  
				CatchExternalRC(RC8);
#endif	

#ifdef RC9
    if (RC9->RCx_Hardware->NVIC_IRQChannel == EXTI2_IRQn)	  
				 CatchExternalRC(RC9);
#endif			
}

void RC_EXTI3_IRQHandler(void)		//RC1
{
#ifdef RC1
    if (RC1->RCx_Hardware->NVIC_IRQChannel == EXTI3_IRQn)	  
				CatchExternalRC(RC1);
#endif
		
#ifdef RC2
    if (RC2->RCx_Hardware->NVIC_IRQChannel == EXTI3_IRQn)	  
				CatchExternalRC(RC2);
#endif	

#ifdef RC3
    if (RC3->RCx_Hardware->NVIC_IRQChannel == EXTI3_IRQn)	  
				CatchExternalRC(RC3);
#endif
		
#ifdef RC4
    if (RC4->RCx_Hardware->NVIC_IRQChannel == EXTI3_IRQn)	  
				CatchExternalRC(RC4);
#endif				
	
#ifdef RC5
    if (RC5->RCx_Hardware->NVIC_IRQChannel == EXTI3_IRQn)	  
				CatchExternalRC(RC5);
#endif
		
#ifdef RC6
    if (RC6->RCx_Hardware->NVIC_IRQChannel == EXTI3_IRQn)	  
				CatchExternalRC(RC6);
#endif	

#ifdef RC7
    if (RC7->RCx_Hardware->NVIC_IRQChannel == EXTI3_IRQn)	  
				CatchExternalRC(RC7);
#endif
		
#ifdef RC8
    if (RC8->RCx_Hardware->NVIC_IRQChannel == EXTI3_IRQn)	  
				CatchExternalRC(RC8);
#endif	

#ifdef RC9
    if (RC9->RCx_Hardware->NVIC_IRQChannel == EXTI3_IRQn)	  
				CatchExternalRC(RC9);
#endif			
}

void RC_EXTI4_IRQHandler(void)		//RC1
{
#ifdef RC1
    if (RC1->RCx_Hardware->NVIC_IRQChannel == EXTI4_IRQn)	  
				CatchExternalRC(RC1);
#endif
		
#ifdef RC2
    if (RC2->RCx_Hardware->NVIC_IRQChannel == EXTI4_IRQn)	  
				CatchExternalRC(RC2);
#endif	

#ifdef RC3
    if (RC3->RCx_Hardware->NVIC_IRQChannel == EXTI4_IRQn)	  
				CatchExternalRC(RC3);
#endif
		
#ifdef RC4
    if (RC4->RCx_Hardware->NVIC_IRQChannel == EXTI4_IRQn)	  
				CatchExternalRC(RC4);
#endif				
	
#ifdef RC5
    if (RC5->RCx_Hardware->NVIC_IRQChannel == EXTI4_IRQn)	  
				CatchExternalRC(RC5);
#endif
		
#ifdef RC6
    if (RC6->RCx_Hardware->NVIC_IRQChannel == EXTI4_IRQn)	  
				CatchExternalRC(RC6);
#endif	

#ifdef RC7
    if (RC7->RCx_Hardware->NVIC_IRQChannel == EXTI4_IRQn)	  
				CatchExternalRC(RC7);
#endif
		
#ifdef RC8
    if (RC8->RCx_Hardware->NVIC_IRQChannel == EXTI4_IRQn)	  
				CatchExternalRC(RC8);
#endif	

#ifdef RC9
    if (RC9->RCx_Hardware->NVIC_IRQChannel == EXTI4_IRQn)	  
				CatchExternalRC(RC9);
#endif			
}

void RC_EXTI9_5_IRQHandler(void)		//RC1
{
#ifdef RC1
		if (RC1->RCx_Hardware->NVIC_IRQChannel == EXTI9_5_IRQn)
				CatchExternalRC(RC1);
#endif
					
#ifdef RC2
		if (RC2->RCx_Hardware->NVIC_IRQChannel == EXTI9_5_IRQn)	  
				CatchExternalRC(RC2);
#endif	

#ifdef RC3
		if (RC3->RCx_Hardware->NVIC_IRQChannel == EXTI9_5_IRQn)	  
				CatchExternalRC(RC3);
#endif

#ifdef RC4
		if (RC4->RCx_Hardware->NVIC_IRQChannel == EXTI9_5_IRQn)	  
				CatchExternalRC(RC4);
#endif				

#ifdef RC5
		if (RC5->RCx_Hardware->NVIC_IRQChannel == EXTI9_5_IRQn)	  
				CatchExternalRC(RC5);
#endif

#ifdef RC6
		if (RC6->RCx_Hardware->NVIC_IRQChannel == EXTI9_5_IRQn)	  
				CatchExternalRC(RC6);
#endif	

#ifdef RC7
		if (RC7->RCx_Hardware->NVIC_IRQChannel == EXTI9_5_IRQn)	  
				CatchExternalRC(RC7);
#endif
					
#ifdef RC8
		if (RC8->RCx_Hardware->NVIC_IRQChannel == EXTI9_5_IRQn)	  
				CatchExternalRC(RC8);
#endif	

#ifdef RC9
		if (RC9->RCx_Hardware->NVIC_IRQChannel == EXTI9_5_IRQn)	  
				CatchExternalRC(RC9);
#endif		
								
}



