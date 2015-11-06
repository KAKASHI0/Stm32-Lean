#include "stm32f10x.h"
#include 	"all_init_heard.h"
#include	"all_task_heard.h"

__task void RC_RECEIVE_PEOCESS(void)
{
		RC_Value_Type_Struct Accelerator;
		RC_Value_Type_Struct Gear;
	  RC_Value_Type_Struct Direction;
  	Value_Type_Struct  v8,v7,v6,v4,v2;
	
	RC_OrgValue_Type_Struct   rcorgvalue;
	
	
//	RC_GetOriginalValue(RC1, &rcorgvalue);
	
	for(;;)
	{
			os_evt_wait_or (RC_RECEIVE_Event_Flag, RC_RECEIVE_Event_TimeOut);
				/*-----------------BACKWARD-----------------*/
		Gear = RC_GetValue(RC5);
		v8 = RC_GetValue(RC8);
		//v7 = RC_GetValue(RC7);
		//v6 = RC_GetValue(RC6);
		//v4 = RC_GetValue(RC4);
		//v2 = RC_GetValue(RC2);
		Accelerator = RC_GetValue(RC3);
		Direction = RC_GetValue(RC1);
		if( Gear.bvalue == 0)
		{
				//Servo
					TIM_SetCompare1(TIM1, servo_zero+Direction.ivalue);	
				//MOTOR
					TIM_SetCompare2(TIM4, (Accelerator.ivalue - 1106)*24);
					TIM_SetCompare4(TIM4, 0);
				  //printf("%d,%d,%d\r\n",Accelerator.ivalue,RC3->orgvalue,Gear.bvalue);
		}
			else  /*---------------------FORWARD---------------*/
			{
				//Servo
				TIM_SetCompare1(TIM1, servo_zero + Direction.ivalue);
					//MOTOR
				//	if(Accelerator.ivalue <= Max_rcvalute && Accelerator.ivalue >= Min_rcvalute)
					//{
						TIM_SetCompare2(TIM4, 0);
						TIM_SetCompare4(TIM4, (Accelerator.ivalue - 1106)*24);
						printf("%d,%d\r\n",RC8->orgvalue ,Gear.bvalue);
					//}
			}//printf("%d,%d,%d\r\n",Accelerator.ivalue,RC3->orgvalue,Gear.bvalue);
	}
}


void EXTI0_IRQHandler(void)		//RC1
{
	RC_EXTI0_IRQHandler();
	//To do:  add other exti0_irq service rountine
	//EXTI0_IRQHandler_Hook();
	//isr_evt_set (RC_RECEIVE_Event_Flag, RC_RECEIVE_PEOCESS_ID);
}

void EXTI1_IRQHandler(void)			//RC2
{	
	RC_EXTI1_IRQHandler();
	//To do:  add other exti1_irq service rountine
	//EXTI0_IRQHandler_Hook();
	//isr_evt_set (RC_RECEIVE_Event_Flag, RC_RECEIVE_PEOCESS_ID);
}	
	
void EXTI2_IRQHandler(void)			//RC3	
{
		RC_EXTI2_IRQHandler();
	//To do:  add other exti2_irq service rountine
	//EXTI0_IRQHandler_Hook();
	//isr_evt_set (RC_RECEIVE_Event_Flag, RC_RECEIVE_PEOCESS_ID);
}

void EXTI3_IRQHandler(void)			//RC4	
{
		RC_EXTI3_IRQHandler();
	//To do:  add other exti3_irq service rountine
	//EXTI0_IRQHandler_Hook();
	//isr_evt_set (RC_RECEIVE_Event_Flag, RC_RECEIVE_PEOCESS_ID);
}

void EXTI4_IRQHandler(void)			//RC5	
{
		RC_EXTI4_IRQHandler();
	//To do:  add other exti4_irq service rountine
	//EXTI0_IRQHandler_Hook();
	//isr_evt_set (RC_RECEIVE_Event_Flag, RC_RECEIVE_PEOCESS_ID);
}

void EXTI9_5_IRQHandler(void)			//RC6~9
{	
	RC_EXTI9_5_IRQHandler();
	//To do:  add other exti9_5_irq service rountine
	//EXTI0_IRQHandler_Hook();
	//isr_evt_set (RC_RECEIVE_Event_Flag, RC_RECEIVE_PEOCESS_ID);

}




