#ifndef 	__RC_RECEIVE_H
#define 	__RC_RECEIVE_H

#define Min_rcvalute		1000
#define Max_rcvalute		2000
#define Direct_value		1800
		

#define Min_Threshold		900
#define Max_Threshold		2100
		
#define servo_zero      1500	

void EXTI0_IRQHandler(void);
void EXTI1_IRQHandler(void);
void EXTI2_IRQHandler(void);
void EXTI3_IRQHandler(void);
void EXTI4_IRQHandler(void);
void EXTI9_5_IRQHandler(void);


#endif

