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
#ifndef __stm32f10x_rc_H
#define __stm32f10x_rc_H

/* Includes ********************************************************************/

#include <stm32f10x.h>
#include <math.h>

#include "stdbool.h"


/**/


/** 
  * @brief  Configuration FreshStatus enumeration  
  */
typedef enum __RC_FreshStatus_Enum
{
	NOT_FRESHED	 = -1,
	FRESHIING    =  0,
	FRESHED			 =  1 	
}  RC_FreshStatus_Enum;

/** 
  * @brief  Configuration ModeType enumeration  
  */
typedef enum  __RC_ModeType_Enum
{
		ANALOG_MODE		= 1,
		BINARY_MODE		= 2,	
		TERNARY_MODE	= 3
} RC_ModeType_Enum;

/** 
  * @brief  Configuration BinaryType enumeration  
  */
typedef enum  __RC_BinaryType_Enum
{
		B_LOW		    = 0,
		B_HIGH	    = 1
} RC_BinaryType_Enum;

/** 
  * @brief  Configuration TernaryType enumeration  
  */
typedef enum  __RC_TernaryType_Enum
{
		T_LOW		    = -1,
		T_MIDDLE		= 0,	
		T_HIGH	    = 1
} RC_TernaryType_Enum;

/** 
  * @brief  Configuration PrecisionType enumeration  
  */
typedef enum  __RC_PrecisionType_Enum
{ 	
		HIGHPRECISION 			= 10000,
    MIDDLEPRECISION 		= 2000,
		LOWPRECISION        = 200
} RC_PrecisionType_Enum;

/** 
  * @brief  RC Hardware structure definition   
  */
typedef struct __RC_HardWare_Struct
{
	  uint8_t           RC_id;
		GPIO_TypeDef*   	GPIOx;			
		uint16_t 					GPIO_Pin;
		TIM_TypeDef * 		TIMx;
		uint8_t  					EXTI_GPIO_PortSource;
		uint8_t  					EXTI_GPIO_PinSource;
		uint32_t 					EXTI_Line;
		IRQn_Type 				NVIC_IRQChannel;
	
}	 RC_HardWare_Struct;															

/** 
  * @brief  RC Init structure definition   
  */
typedef struct  __RC_InitTypeDef_Struct	
{
	  uint8_t									 RC_Id;
  	FunctionalState     		 enableFlag;           
		RC_ModeType_Enum    	   mode;                    // #define RC_MODETYPE_DEFAULT          ANALOG_MODE	
		RC_PrecisionType_Enum    precision;
	  
}	RC_InitTypeDef_Struct;


typedef struct  __RC_Analog_Value_Struct
{
    RC_PrecisionType_Enum	   prec;
    float                    normvalue;     //-1.0f~+1.0f
} RC_Analog_Value_Struct;


/** 
  * @brief  RC value_Type structure definition   
  */
typedef struct  __RC_Value_Type_Struct
{
	      RC_FreshStatus_Enum 		 freshFlag;
	      RC_ModeType_Enum     		 mode;
	      union{
						bool                    bvalue;
						RC_TernaryType_Enum     tvalue;
					  RC_Analog_Value_Struct  avalue;
				}value;
} RC_Value_Type_Struct;


typedef struct  __RC_OrgValue_Type_Struct
{
		RC_FreshStatus_Enum 		 freshFlag;
		uint16_t                 orgvalue;
} RC_OrgValue_Type_Struct;


typedef struct  __RC_Registers_Type_Struct
{
		FunctionalState			     enableFlag;            	// #define RC_ENABLEFLAG_DEFAULT        DISABLE
		RC_FreshStatus_Enum      freshFlag;         	 		// #define RC_FRESHSTATUS_DEFAULT       NOT_FRESHED	
	  RC_ModeType_Enum     		 mode;
    RC_PrecisionType_Enum	   prec;
		uint16_t                 midPositionValue;
	  uint16_t               	 starttime_pulse;
		uint16_t 								 endtime_pulse;	
} RC_Registers_Type_Struct;

/** 
  * @brief  RC   
  */
typedef struct  __RC_TypeDef_Struct	
{
	  RC_Registers_Type_Struct  RCx_Register;
		RC_HardWare_Struct*      RCx_Hardware;
}	RC_TypeDef_Struct;


#define RC_MAX_NUM   9
#if(RC_MAX_NUM>9)
		#ERROR RC_MAX_NUM isnot allowed >= 9 !!!
#endif


#define RC_USED_NUM   9
#if(RC_USED_NUM>RC_MAX_NUM)
		#ERROR RC_USED_NUM > RC_MAX_NUM isnot permitted !!!
#endif


#if(RC_USED_NUM>=1)
		extern RC_TypeDef_Struct  RC1_Registers;
		#define RC1  (&RC1_Registers)
#endif
#if(RC_USED_NUM>=2)
		extern RC_TypeDef_Struct  RC2_Registers;
		#define RC2  (&RC2_Registers)
#endif
#if(RC_USED_NUM>=3)
		extern RC_TypeDef_Struct  RC3_Registers;
		#define RC3  (&RC3_Registers)
#endif
#if(RC_USED_NUM>=4)
		extern RC_TypeDef_Struct  RC4_Registers;
		#define RC4  (&RC4_Registers)
#endif
#if(RC_USED_NUM>=5)
		extern RC_TypeDef_Struct  RC5_Registers;
		#define RC5  (&RC5_Registers)
#endif
#if(RC_USED_NUM>=6)
		extern RC_TypeDef_Struct  RC6_Registers;
		#define RC6  (&RC6_Registers)
#endif
#if(RC_USED_NUM>=7)
		extern RC_TypeDef_Struct  RC7_Registers;
		#define RC7  (&RC7_Registers)
#endif
#if(RC_USED_NUM>=8)
		extern RC_TypeDef_Struct  RC8_Registers;
		#define RC8  (&RC8_Registers)
#endif
#if(RC_USED_NUM>=9)
		extern RC_TypeDef_Struct  RC9_Registers;
		#define RC9  (&RC9_Registers)
#endif


 #define MID_POSITION_VALUE_DEFAULT  1500


#define TIM_PRESCALE_0								0
#define TIM_PRESCALE_7								7
#define TIM_PRESCALE_47								47

#define RC_GPIO_NVIC_PREEMPTIONPRIORITY_DEFAULT				0x0F
#define RC_GPIO_NVIC_SUBPRIORITY_DEFAULT							0x0F


/*************************************************************
RC->outer function

**************************************************************/

void RC_StructInit(RC_InitTypeDef_Struct* RC_InitStruct);
void RC_Init(RC_TypeDef_Struct* RCx ,RC_InitTypeDef_Struct*  RC_InitStruct); 
		
void RC_On(RC_TypeDef_Struct* RCx);
void RC_Off(RC_TypeDef_Struct* RCx);
void RC_ClearFlag(RC_TypeDef_Struct* RCx);
ErrorStatus RC_GetValue(RC_TypeDef_Struct* RCx, RC_Value_Type_Struct* rcvalue);
uint16_t RC_GetOrgValue(RC_TypeDef_Struct* RCx);
RC_FreshStatus_Enum RC_GetFlagStatus(RC_TypeDef_Struct* RCx);

void RC_EXTI0_IRQHandler(void);		//RC1
void RC_EXTI1_IRQHandler(void);		//RC2
void RC_EXTI2_IRQHandler(void);		//RC3
void RC_EXTI3_IRQHandler(void);		//RC4
void RC_EXTI4_IRQHandler(void);		//RC5
void RC_EXTI9_5_IRQHandler(void);		//RC6¡¢RC7¡¢RC8¡¢RC9


#endif

