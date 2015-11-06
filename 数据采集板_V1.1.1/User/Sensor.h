#ifndef __USART_H
#define	__USART_H

#include <stm32f10x.h>

#define DEFAULT_BAUD 256000
//´®¿Ú½ÓÊÕ»º´æ
#define UART_RX_LEN		128
extern uint8_t Uart_Rx1[UART_RX_LEN];
extern uint8_t Uart_Rx2[UART_RX_LEN];
extern uint8_t Uart_Rx3[UART_RX_LEN];
extern uint8_t Uart_Rx4[UART_RX_LEN];
extern uint8_t Uart_Rx5[UART_RX_LEN];

void Sensor1_6_Configration(void);
void Sensor7_12_Configuration(void);
void Sensor13_18_Configuration(void);

#endif
