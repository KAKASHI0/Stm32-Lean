/* UART2.c file
编写者：lisn3188
网址：www.chiplab7.com
作者E-mail：lisn3188@163.com
编译环境：MDK-Lite  Version: 4.23
初版时间: 2012-04-25
测试： 本程序已在第七实验室的mini IMU上完成测试
功能：
UART1通信 的API
------------------------------------
 */

#include "UART2.h"

//uart reicer flag
#define b_uart_head  0x80
#define b_rx_over    0x40

// USART Receiver buffer
#define RX_BUFFER_SIZE 100

u8 U2TxBuffer[258];
u8 U2TxCounter=0;
u8 U2count=0; 

void U2NVIC_Configuration(void)
{
        NVIC_InitTypeDef NVIC_InitStructure; 
          /* Enable the USART1 Interrupt */
        NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 8;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);
}

/**************************实现函数********************************************
*函数原型:		void Initial_UART2(u32 baudrate)
*功　　能:		初始化STM32-SDK开发板上的RS232接口
输入参数：
		u32 baudrate   设置RS232串口的波特率
输出参数：没有	
*******************************************************************************/
void Initial_UART2(u32 baudrate)
{
 	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	/* 使能 UART2 模块的时钟  使能 UART2对应的引脚端口PA的时钟*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2 | RCC_APB2Periph_GPIOA, ENABLE);

  	 /* 配置UART2 的发送引脚
	 配置PA9 为复用输出  刷新频率50MHz
	  */
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_Init(GPIOA, &GPIO_InitStructure);    
  	/* 
	  配置UART2 的接收引脚
	  配置PA10为浮地输入 
	 */
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  	GPIO_Init(GPIOA, &GPIO_InitStructure);
	  
	/* 
	  UART2的配置:
	  1.波特率为调用程序指定的输入 baudrate;
	  2. 8位数据			  USART_WordLength_8b;
	  3.一个停止位			  USART_StopBits_1;
	  4. 无奇偶效验			  USART_Parity_No ;
	  5.不使用硬件流控制	  USART_HardwareFlowControl_None;
	  6.使能发送和接收功能	  USART_Mode_Rx | USART_Mode_Tx;
	 */
	USART_InitStructure.USART_BaudRate = baudrate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	//应用配置到UART2
	USART_Init(USART2, &USART_InitStructure); 
	USART_ITConfig(USART2, USART_IT_TXE, DISABLE);        
    USART_ClearFlag(USART2,USART_FLAG_TC);
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);	//使能接收中断
	//启动UART2
  	USART_Cmd(USART2, ENABLE);
	U2NVIC_Configuration();
}

/**************************实现函数********************************************
*函数原型:		void UART2_Put_Char(unsigned char DataToSend)
*功　　能:		RS232发送一个字节
输入参数：
		unsigned char DataToSend   要发送的字节数据
输出参数：没有	
*******************************************************************************/
void UART2_Put_Char(unsigned char DataToSend)
{
	//将要发送的字节写到UART2的发送缓冲区
	//USART_SendData(USART1, (unsigned char) DataToSend);
	//等待发送完成
  	//while (!(USART1->SR & USART_FLAG_TXE));

	U2TxBuffer[U2count++] = DataToSend;  
    USART_ITConfig(USART2, USART_IT_TXE, ENABLE);  
}

/**************************实现函数********************************************
*函数原型:		u8 UART2_Get_Char(void)
*功　　能:		RS232接收一个字节  一直等待，直到UART2接收到一个字节的数据。
输入参数：		 没有
输出参数：       UART2接收到的数据	
*******************************************************************************/
u8 UART2_Get_Char(void)
{
	while (!(USART2->SR & USART_FLAG_RXNE));
	return(USART_ReceiveData(USART2));
}

/**************************实现函数********************************************
*函数原型:		void UART2_Put_String(unsigned char *Str)
*功　　能:		RS232发送字符串
输入参数：
		unsigned char *Str   要发送的字符串
输出参数：没有	
*******************************************************************************/
void UART2_Put_String(unsigned char *Str)
{
	//判断Str指向的数据是否有效.
	while(*Str){
	//是否是回车字符 如果是,则发送相应的回车 0x0d 0x0a
	if(*Str=='\r')UART2_Put_Char(0x0d);
		else if(*Str=='\n')UART2_Put_Char(0x0a);
			else UART2_Put_Char(*Str);
	//等待发送完成.
  	//while (!(USART1->SR & USART_FLAG_TXE));
	//指针++ 指向下一个字节.
	Str++;
	}
/*
	//判断Str指向的数据是否有效.
	while(*Str){
	//是否是回车字符 如果是,则发送相应的回车 0x0d 0x0a
	if(*Str=='\r')USART_SendData(USART1, 0x0d);
		else if(*Str=='\n')USART_SendData(USART1, 0x0a);
			else USART_SendData(USART1, *Str);
	//等待发送完成.
  	while (!(USART1->SR & USART_FLAG_TXE));
	//指针++ 指向下一个字节.
	Str++;
	}		 */
}

/**************************实现函数********************************************
*函数原型:		void UART2_Putc_Hex(uint8_t b)
*功　　能:		RS232以十六进制ASCII码的方式发送一个字节数据
				先将目标字节数据高4位转成ASCCII ，发送，再将低4位转成ASCII发送
				如:0xF2 将发送 " F2 "
输入参数：
		uint8_t b   要发送的字节
输出参数：没有	
*******************************************************************************/
void UART2_Putc_Hex(uint8_t b)
{
      /* 判断目标字节的高4位是否小于10 */
    if((b >> 4) < 0x0a)
        UART2_Put_Char((b >> 4) + '0'); //小于10  ,则相应发送0-9的ASCII
    else
        UART2_Put_Char((b >> 4) - 0x0a + 'A'); //大于等于10 则相应发送 A-F

    /* 判断目标字节的低4位 是否小于10*/
    if((b & 0x0f) < 0x0a)
        UART2_Put_Char((b & 0x0f) + '0');//小于10  ,则相应发送0-9的ASCII
    else
        UART2_Put_Char((b & 0x0f) - 0x0a + 'A');//大于等于10 则相应发送 A-F
   UART2_Put_Char(' '); //发送一个空格,以区分开两个字节
}

/**************************实现函数********************************************
*函数原型:		void UART2_Putw_Hex(uint16_t w)
*功　　能:		RS232以十六进制ASCII码的方式发送一个字的数据.就是发送一个int
				如:0x3456 将发送 " 3456 "
输入参数：
		uint16_t w   要发送的字
输出参数：没有	
*******************************************************************************/
void UART2_Putw_Hex(uint16_t w)
{
	//发送高8位数据,当成一个字节发送
    UART2_Putc_Hex((uint8_t) (w >> 8));
	//发送低8位数据,当成一个字节发送
    UART2_Putc_Hex((uint8_t) (w & 0xff));
}

/**************************实现函数********************************************
*函数原型:		void UART2_Putdw_Hex(uint32_t dw)
*功　　能:		RS232以十六进制ASCII码的方式发送32位的数据.
				如:0xF0123456 将发送 " F0123456 "
输入参数：
		uint32_t dw   要发送的32位数据值
输出参数：没有	
*******************************************************************************/
void UART2_Putdw_Hex(uint32_t dw)
{
    UART2_Putw_Hex((uint16_t) (dw >> 16));
    UART2_Putw_Hex((uint16_t) (dw & 0xffff));
}

/**************************实现函数********************************************
*函数原型:		void UART2_Putw_Dec(uint16_t w)
*功　　能:		RS232以十进制ASCII码的方式发送16位的数据.
				如:0x123 将发送它的十进制数据 " 291 "
输入参数：
		uint16_t w   要发送的16位数据值
输出参数：没有	
*******************************************************************************/
void UART2_Putw_Dec(uint32_t w)
{
    uint32_t num = 100000;
    uint8_t started = 0;

    while(num > 0)
    {
        uint8_t b = w / num;
        if(b > 0 || started || num == 1)
        {
            UART2_Put_Char('0' + b);
            started = 1;
        }
        w -= b * num;

        num /= 10;
    }
}

void UART2_Putint_Dec(int16_t in)
{
	if(in<0){
	in=-in;
	UART2_Put_Char('-');
	}
   UART2_Putw_Dec(in);
}

void UART2_Putintp_Dec(int16_t in)
{
	if(in<0){
	in=-in;
	UART2_Put_Char('-');
	}
   UART2_Putw_Dec(in/10);
   UART2_Put_Char('.');
   UART2_Putw_Dec(in%10);
}

void UART2_Reportuint16(int16_t avalue, uint32_t* asum)
{
    char ctemp;
	if(avalue<0)avalue=32768-avalue;
	ctemp=avalue>>8;
	UART2_Put_Char(ctemp);
	*asum = *asum + ctemp;
	ctemp=avalue;
	UART2_Put_Char(ctemp);
	*asum = *asum + ctemp;
}

void UART2_Reportint16(int16_t avalue, uint32_t* asum)
{
   	char ctemp;
	ctemp=avalue>>8;
	UART2_Put_Char(ctemp);
	*asum = *asum + ctemp;

	ctemp=avalue;							
	UART2_Put_Char(ctemp);
	*asum = *asum + ctemp;
}

void UART2_Reportfloat(float avalue, uint32_t* asum)
{
    uint32_t highaddr, lowaddr;
	int16_t	high, low;
	highaddr = (uint32_t)(&avalue);
	lowaddr = highaddr + 2;
    
	high = *((int16_t*)highaddr);
	UART2_Reportint16(high, asum);
	low = *((int16_t*)lowaddr);
	UART2_Reportint16(low, asum);

}

void UART2_ReportData(char* dataptr)
{
    int16_t i;
	int16_t nsize;
	uint32_t temp=0;
    if ((dataptr == 0) || dataptr[0] <= 0)
	    return;
    nsize = dataptr[0] + 2;
	UART2_Put_Char(0xa5);
	UART2_Put_Char(0x5a);
	UART2_Put_Char(nsize);
	temp += nsize;
    for (i=1; i<dataptr[0]; i++)
	{
	    UART2_Put_Char(dataptr[i]);
		temp += dataptr[i];
	}
	UART2_Put_Char(temp%256);
	UART2_Put_Char(0xaa);         
}

void Int16ToChar(int16_t nvalue, char* dataptr)
{
   	char ctemp;
	ctemp=nvalue>>8;
	dataptr[0] = ctemp;

	ctemp=nvalue;
	dataptr[1] = ctemp;  
}

void FloatToChar(float fvalue, char* dataptr)
{
    uint32_t highaddr, lowaddr;
	int16_t	high, low;
	highaddr = (uint32_t)(&fvalue);
	lowaddr = highaddr + 2;
    
	high = *((int16_t*)highaddr);
	Int16ToChar(high, dataptr);
	low = *((int16_t*)lowaddr);
	Int16ToChar(low, dataptr+2);
}

void YawPitchRollToChar(int16_t yaw, int16_t pitch, int16_t roll, int16_t chipid, char* dataptr)
{
    int16_t i = dataptr[0];
	dataptr[i++] = 0xA7;
	dataptr[i++] = yaw>>8;
	dataptr[i++] = yaw;
	dataptr[i++] = pitch>>8;
	dataptr[i++] = pitch;
	dataptr[i++] = roll>>8;
	dataptr[i++] = roll;
	dataptr[i++] = chipid>>8;
	dataptr[i++] = chipid;
	dataptr[0] = i;
    return;
}

void MotionAndGravToChar(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy , int16_t gz, int16_t hx, int16_t hy, int16_t hz, float* grav, int16_t chipid, char* dataptr)
{
    int16_t i = dataptr[0];
	dataptr[i++] = 0xA4;
	dataptr[i++] = ax>>8;
	dataptr[i++] = ax;
	dataptr[i++] = ay>>8;
	dataptr[i++] = ay;
	dataptr[i++] = az>>8;
	dataptr[i++] = az;
	dataptr[i++] = gx>>8;
	dataptr[i++] = gx;
	dataptr[i++] = gy>>8;
	dataptr[i++] = gy;
	dataptr[i++] = gz>>8;
	dataptr[i++] = gz;
	dataptr[i++] = hx>>8;
	dataptr[i++] = hx;
	dataptr[i++] = hy>>8;
	dataptr[i++] = hy;
	dataptr[i++] = hz>>8;
	dataptr[i++] = hz;

    FloatToChar(grav[0], dataptr + i);
	FloatToChar(grav[1], dataptr + i + 4);
	FloatToChar(grav[2], dataptr + i + 8);
	i+=12;

	dataptr[i++] = chipid>>8;
	dataptr[i++] = chipid;
	dataptr[0] = i;
    return;
}

void QuatToChar(float* quat, int16_t chipid, char* dataptr)
{
    int16_t i = dataptr[0];
	dataptr[i++] = 0xA3;

    FloatToChar(quat[0], dataptr + i);
	FloatToChar(quat[1], dataptr + i + 4);
	FloatToChar(quat[2], dataptr + i + 8);
	FloatToChar(quat[2], dataptr + i + 12);
	i+=16;

	dataptr[i++] = chipid>>8;
	dataptr[i++] = chipid;
	dataptr[0] = i;
    return;
}

void UART2_ReportChipID(int16_t ChipID)
{
 	uint32_t temp=0xa8 + 6;
	UART2_Put_Char(0xa5);
	UART2_Put_Char(0x5a);
	UART2_Put_Char(6);
	UART2_Put_Char(0xa8);
	UART2_Reportint16(ChipID, &temp);
	UART2_Put_Char(temp%256);
	UART2_Put_Char(0xaa);      
}

void UART2_ReportQuat(float q0, float q1, float q2, float q3)
{
 	uint32_t temp=0xA3 + 20;
	UART2_Put_Char(0xa5);
	UART2_Put_Char(0x5a);
	UART2_Put_Char(16+4);
	UART2_Put_Char(0xA3);
	UART2_Reportfloat(q0, &temp);
	UART2_Reportfloat(q1, &temp);
	UART2_Reportfloat(q2, &temp);
	UART2_Reportfloat(q3, &temp);
	UART2_Put_Char(temp%256);
	UART2_Put_Char(0xaa);
}

void UART2_ReportYawPitchRoll(int16_t yaw, int16_t pitch, int16_t roll)
{
 	uint32_t temp=0xA7 + 6 + 4;
	UART2_Put_Char(0xa5);
	UART2_Put_Char(0x5a);
	UART2_Put_Char(6+4);
	UART2_Put_Char(0xA7);
	UART2_Reportint16(yaw, &temp);
	UART2_Reportint16(pitch, &temp);
	UART2_Reportint16(roll, &temp);
	UART2_Put_Char(temp%256);
	UART2_Put_Char(0xaa);
}

void UART2_ReportMotionAndGrav(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy , int16_t gz, int16_t hx, int16_t hy, int16_t hz, float* grav)
{
 	uint32_t temp=0xA4 + 34;
	UART2_Put_Char(0xa5);
	UART2_Put_Char(0x5a);
	UART2_Put_Char(30+4);
	UART2_Put_Char(0xA4);
	UART2_Reportint16(ax, &temp);
	UART2_Reportint16(ay, &temp);
	UART2_Reportint16(az, &temp);
	UART2_Reportint16(gx, &temp);
	UART2_Reportint16(gy, &temp);
	UART2_Reportint16(gz, &temp);
	UART2_Reportint16(hx, &temp);
	UART2_Reportint16(hy, &temp);
	UART2_Reportint16(hz, &temp);

	UART2_Reportfloat(grav[0], &temp);
	UART2_Reportfloat(grav[1], &temp);
	UART2_Reportfloat(grav[2], &temp);
	UART2_Put_Char(temp%256);
	UART2_Put_Char(0xaa);
}

void UART2_ReportAccOffset(int16_t offax, int16_t offay, int16_t offaz)
{
 	uint32_t temp=0xa0 + 10;
	UART2_Put_Char(0xa5);
	UART2_Put_Char(0x5a);
	UART2_Put_Char(10);
	UART2_Put_Char(0xa0);
	UART2_Reportint16(offax, &temp);
	UART2_Reportint16(offay, &temp);
	UART2_Reportint16(offaz, &temp);
	UART2_Put_Char(temp%256);
	UART2_Put_Char(0xaa);    
}

void UART2_ReportComOffset(int16_t offhx, int16_t offhy, int16_t offhz)
{
 	uint32_t temp=0xa6 + 10;
	UART2_Put_Char(0xa5);
	UART2_Put_Char(0x5a);
	UART2_Put_Char(10);
	UART2_Put_Char(0xa6);
	UART2_Reportint16(offhx, &temp);
	UART2_Reportint16(offhy, &temp);
	UART2_Reportint16(offhz, &temp);
	UART2_Put_Char(temp%256);
	UART2_Put_Char(0xaa);  
}

void UART2_ReportIMU(int16_t yaw,int16_t pitch,int16_t roll
,int16_t alt,int16_t tempr,int16_t press,int16_t IMUpersec)
{
 	unsigned int temp=0xaF+2+2;
	char ctemp;
	UART2_Put_Char(0xa5);
	UART2_Put_Char(0x5a);
	UART2_Put_Char(14+4);
	UART2_Put_Char(0xA1);

	if(yaw<0)yaw=32768-yaw;
	ctemp=yaw>>8;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=yaw;							
	UART2_Put_Char(ctemp);
	temp+=ctemp;

	if(pitch<0)pitch=32768-pitch;
	ctemp=pitch>>8;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=pitch;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
								 
	if(roll<0)roll=32768-roll;
	ctemp=roll>>8;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=roll;
	UART2_Put_Char(ctemp);
	temp+=ctemp;

	if(alt<0)alt=32768-alt;
	ctemp=alt>>8;
	UART2_Put_Char(ctemp);
	temp+=ctemp;				
	ctemp=alt;
	UART2_Put_Char(ctemp);
	temp+=ctemp;

	if(tempr<0)tempr=32768-tempr;
	ctemp=tempr>>8;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=tempr;
	UART2_Put_Char(ctemp);	   
	temp+=ctemp;

	if(press<0)press=32768-press;
	ctemp=press>>8;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=press;
	UART2_Put_Char(ctemp);
	temp+=ctemp;

	ctemp=IMUpersec>>8;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=IMUpersec;
	UART2_Put_Char(ctemp);
	temp+=ctemp;

	UART2_Put_Char(temp%256);
	UART2_Put_Char(0xaa);
}/*			   				
void UART2_ReportIMU(int16_t yaw,int16_t pitch,int16_t roll
,int16_t alt,int16_t tempr,int16_t press,int16_t IMUpersec)
{
 	unsigned int temp=0xAB+12;
	char ctemp;
	UART2_Put_Char(0xa5);
	UART2_Put_Char(0x5a);
	UART2_Put_Char(12);
	UART2_Put_Char(0xAB);

	if(yaw<0)yaw=32768-yaw;
	ctemp=yaw>>8;
	UART2_Put_Char(ctemp);				  
	temp+=ctemp;
	ctemp=yaw;
	UART2_Put_Char(ctemp);
	temp+=ctemp;

	if(pitch<0)pitch=32768-pitch;
	ctemp=pitch>>8;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=pitch;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
										
	if(roll<0)roll=32768-roll;
	ctemp=roll>>8;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=roll;
	UART2_Put_Char(ctemp);
	temp+=ctemp;

	ctemp=IMUpersec>>8;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=IMUpersec;
	UART2_Put_Char(ctemp);
	temp+=ctemp;

	UART2_Put_Char(temp%256);		   
	UART2_Put_Char(0xaa);
}  */

void UART2_ReportMotion(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz,
					int16_t hx,int16_t hy,int16_t hz)
{
 	unsigned int temp=0xaF+9;
	char ctemp;
	UART2_Put_Char(0xa5);
	UART2_Put_Char(0x5a);
	UART2_Put_Char(14+8);
	UART2_Put_Char(0xA2);

	if(ax<0)ax=32768-ax;
	ctemp=ax>>8;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=ax;
	UART2_Put_Char(ctemp);
	temp+=ctemp;

	if(ay<0)ay=32768-ay;
	ctemp=ay>>8;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=ay;
	UART2_Put_Char(ctemp);
	temp+=ctemp;

	if(az<0)az=32768-az;
	ctemp=az>>8;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=az;
	UART2_Put_Char(ctemp);
	temp+=ctemp;

	if(gx<0)gx=32768-gx;
	ctemp=gx>>8;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=gx;
	UART2_Put_Char(ctemp);
	temp+=ctemp;

	if(gy<0)gy=32768-gy;
	ctemp=gy>>8;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=gy;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
//-------------------------
	if(gz<0)gz=32768-gz;
	ctemp=gz>>8;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=gz;
	UART2_Put_Char(ctemp);
	temp+=ctemp;

	if(hx<0)hx=32768-hx;
	ctemp=hx>>8;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=hx;
	UART2_Put_Char(ctemp);
	temp+=ctemp;

	if(hy<0)hy=32768-hy;
	ctemp=hy>>8;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=hy;
	UART2_Put_Char(ctemp);
	temp+=ctemp;

	if(hz<0)hz=32768-hz;
	ctemp=hz>>8;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=hz;
	UART2_Put_Char(ctemp);
	temp+=ctemp;

	UART2_Put_Char(temp%256);
	UART2_Put_Char(0xaa);
}

volatile unsigned char rx_buffer[RX_BUFFER_SIZE];
volatile unsigned char rx_wr_index;
volatile unsigned char RC_Flag;
//------------------------------------------------------
void USART2_IRQHandler(void)
{
  unsigned char data;
  if(USART_GetITStatus(USART2, USART_IT_TXE) != RESET)
  {   
    /* Write one byte to the transmit data register */
    USART_SendData(USART2, U2TxBuffer[U2TxCounter++]);                    

    /* Clear the USART1 transmit interrupt */
    USART_ClearITPendingBit(USART2, USART_IT_TXE); 

    if(U2TxCounter == U2count)
    {
      /* Disable the USART1 Transmit interrupt */
      USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
    }    
  }else if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
  {
  data=USART_ReceiveData(USART2);
  if(data==0xa5)
  { 
	RC_Flag|=b_uart_head;
    rx_buffer[rx_wr_index++]=data;
  }
  else if(data==0x5a)
       { if(RC_Flag&b_uart_head)
	     { rx_wr_index=0;
		   RC_Flag&=~b_rx_over;
         }
         else
		  rx_buffer[rx_wr_index++]=data;
         RC_Flag&=~b_uart_head;
       }
	   else
	   { rx_buffer[rx_wr_index++]=data;
		 RC_Flag&=~b_uart_head;
		 if(rx_wr_index==rx_buffer[0])
	     {  
			RC_Flag|=b_rx_over;
          }
	   }
  if(rx_wr_index==RX_BUFFER_SIZE)
  rx_wr_index--;
  /* Clear the USART1 transmit interrupt */
  USART_ClearITPendingBit(USART2, USART_IT_RXNE);
  }
}

/*
+------------------------------------------------------------------------------
| Function    : Sum_check(void)
+------------------------------------------------------------------------------
| Description : check
|
| Parameters  : 
| Returns     : 
|
+------------------------------------------------------------------------------
*/
unsigned char Sum_check(char* chache)
{ 
  unsigned char i;
  unsigned int checksum=0; 
  for(i=0;i<rx_buffer[0]-2;i++)
  {
     chache[i] = rx_buffer[i];
     checksum+=chache[i];
     
  }
  if((checksum%256)==rx_buffer[rx_buffer[0]-2])
   return(0x01); //Checksum successful
  else
   return(0x00); //Checksum error
}

unsigned char UART2_CommandRoute(char* chache)
{
 if(RC_Flag&b_rx_over){
		RC_Flag&=~b_rx_over;
		if(Sum_check(chache)){
		return rx_buffer[1];
		}
	}
return 0xff; //没有收到上位机的命令，或者是命令效验没有通过
}

unsigned char UART2_CommandRoutedata(int16_t index)
{
    return rx_buffer[index]; //没有收到上位机的命令，或者是命令效验没有通过
}

void UART2_GetAccOffset(int16_t* ax, int16_t* ay, int16_t* az)
{
    *ax = (rx_buffer[3]<<8) | rx_buffer[2];
	*ay = (rx_buffer[5]<<8) | rx_buffer[4];
	*az = (rx_buffer[7]<<8) | rx_buffer[6];
}

void UART2_GetHmcOffset(int16_t* hx, int16_t* hy, int16_t* hz)
{
    *hx = (rx_buffer[3]<<8) | rx_buffer[2];
	*hy = (rx_buffer[5]<<8) | rx_buffer[4];
	*hz = (rx_buffer[7]<<8) | rx_buffer[6];    
}

float myabs(float value)
{
    if (value > 0)
	    return value;
	else 
	    return -value;
}


//------------------End of File----------------------------
