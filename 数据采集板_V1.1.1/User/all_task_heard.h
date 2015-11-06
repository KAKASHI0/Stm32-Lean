#ifndef 	__ALL_TASK_HEARD_H
#define		__ALL_TASK_HEARD_H

#include "RTl.h"

extern OS_TID ADC_Transform_ID;
extern OS_TID RC_RECEIVE_PEOCESS_ID;


__task void Peripherals_Init_task (void) ;
__task void ADC_Transform_Task(void) ;
__task void RC_RECEIVE_PEOCESS(void);
 

#define 	RC_RECEIVE_Event_Flag						0x0001
#define		RC_RECEIVE_Event_TimeOut				0xFFFF
 #define 	ADC_Transform_Task_Event_Flag		0x0010
	
	
 #define		RC_RECEIVE_Priority         5
 #define	 ADC_Transform_Task_Priority  1

#endif
