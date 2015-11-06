#include 	"stm32f10x.h"
#include 	"all_init_heard.h"
#include	"all_task_heard.h"

void RC_Configration()
{
		RC_InitTypeDef_Struct  RC_InitStructure;
		
		RC_StructInit(&RC_InitStructure);
		
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	
		RC_InitStructure.mode = ANALOG_MODE;
		RC_InitStructure.precision = LOWPRECISION;
		RC_InitStructure.enableFlag = ENABLE;
		
		RC_Init(RC1,&RC_InitStructure);
		RC_Init(RC2,&RC_InitStructure);
		RC_Init(RC3,&RC_InitStructure);
		RC_Init(RC4,&RC_InitStructure);
		RC_Init(RC6,&RC_InitStructure);
		RC_Init(RC7,&RC_InitStructure);
		RC_Init(RC8,&RC_InitStructure);
		RC_Init(RC9,&RC_InitStructure);
	
		RC_InitStructure.mode = BINARY_MODE;
		RC_InitStructure.precision = HIGHPRECISION;
		RC_InitStructure.enableFlag = ENABLE;
		
		RC_Init(RC5,&RC_InitStructure);
		

	
}
