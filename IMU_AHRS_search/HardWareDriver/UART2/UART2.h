#ifndef __UART2_H
#define __UART2_H

#include <stdio.h>
#include "stm32f10x.h"

#define Gyro_init  0xE0
#define Hmc_SetOffset  0xE1
#define High_init  0xE2
#define Acc_SetOffset  0xE3
#define Acc_SetEextremeValue 0xE4
#define SetUploadSpeed 0xE5
#define GetChipID 0xE6
#define SetReportType 0xE7


void Initial_UART2(u32 baudrate);
void UART2_Put_Char(unsigned char DataToSend);
u8 UART2_Get_Char(void);
void UART2_Put_String(unsigned char *Str);
void UART2_Putc_Hex(uint8_t b);
void UART2_Putw_Hex(uint16_t w);
void UART2_Putdw_Hex(uint32_t dw);
void UART2_Putw_Dec(uint32_t w);
void UART2_Putint_Dec(int16_t in);
void UART2_Putintp_Dec(int16_t in);
void UART2_ReportIMU(int16_t yaw,int16_t pitch,int16_t roll
,int16_t alt,int16_t tempr,int16_t press,int16_t IMUpersec);
void UART2_ReportMotion(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz,
					int16_t hx,int16_t hy,int16_t hz);
unsigned char UART2_CommandRoute(char* chache);
unsigned char UART2_CommandRoutedata(int16_t index);

void UART2_GetAccOffset(int16_t* ax, int16_t* ay, int16_t* az);
void UART2_GetHmcOffset(int16_t* hx, int16_t* hy, int16_t* hz);

void UART2_ReportChipID(int16_t ChipID);
void UART2_Reportuint16(int16_t avalue, uint32_t* asum);
void UART2_Reportint16(int16_t avalue, uint32_t* asum);
void UART2_Reportfloat(float avalue, uint32_t* asum);
void UART2_ReportQuat(float q0, float q1, float q2, float q3);
void UART2_ReportYawPitchRoll(int16_t yaw, int16_t pitch, int16_t roll);
void UART2_ReportMotionAndGrav(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy , int16_t gz, int16_t hx, int16_t hy, int16_t hz, float* grav);
void UART2_ReportAccOffset(int16_t offax, int16_t offay, int16_t offaz);
void UART2_ReportComOffset(int16_t offhx, int16_t offhy, int16_t offhz);
void UART2_ReportData(char* dataptr);
void Int16ToChar(int16_t nvalue, char* dataptr);
void FloatToChar(float fvalue, char* dataptr);
void YawPitchRollToChar(int16_t yaw, int16_t pitch, int16_t roll, int16_t chipid, char* dataptr);
void MotionAndGravToChar(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy , int16_t gz, int16_t hx, int16_t hy, int16_t hz, float* grav, int16_t chipid, char* dataptr);
void QuatToChar(float* quat, int16_t chipid, char* dataptr);

float myabs(float value);

#endif

//------------------End of File----------------------------

