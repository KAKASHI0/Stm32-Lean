#ifndef __UARTS_H
#define __UARTS_H

#include <stdio.h>
#include "stm32f10x.h"

void Initial_UART1(u32 baudrate);
void UART1_Put_Char(unsigned char DataToSend);
u8 UART1_Get_Char(void);
void UART1_Put_String(unsigned char *Str);
void UART1_Putc_Hex(uint8_t b);
void UART1_Putw_Hex(uint16_t w);
void UART1_Putdw_Hex(uint32_t dw);
void UART1_Putw_Dec(uint32_t w);
void UART1_Putint_Dec(int16_t in);
void UART1_Putintp_Dec(int16_t in);
void UART1_ReportIMU(int16_t yaw,int16_t pitch,int16_t roll
,int16_t alt,int16_t tempr,int16_t press,int16_t IMUpersec);
void UART1_ReportMotion(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz,
					int16_t hx,int16_t hy,int16_t hz);

void UART1_ReportChipID(int16_t ChipID);
void UART1_Reportuint16(int16_t avalue, uint32_t* asum);
void UART1_Reportint16(int16_t avalue, uint32_t* asum);
void UART1_Reportfloat(float avalue, uint32_t* asum);
void UART1_ReportQuat(float q0, float q1, float q2, float q3);
void UART1_ReportYawPitchRoll(int16_t yaw, int16_t pitch, int16_t roll);
void UART1_ReportMotionAndGrav(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy , int16_t gz, int16_t hx, int16_t hy, int16_t hz, float* grav);
void UART1_ReportAccOffset(int16_t offax, int16_t offay, int16_t offaz);
void UART1_ReportComOffset(int16_t offhx, int16_t offhy, int16_t offhz);
void UART1_ReportData(char* dataptr);
#endif

//------------------End of File----------------------------

