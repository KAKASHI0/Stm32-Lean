/* UARTs.C file
STM32-SDK 开发板相关例程
编写者：lisn3188
网址：www.chiplab7.com
作者E-mail：lisn3188@163.com
编译环境：MDK-Lite  Version: 4.23
初版时间: 2012-02-28
测试： 本程序已在第七实验室的STM32-SDK上完成测试
功能：实现	STM32-SDK 开发板上的 UART1-RS232 接口操作

---------硬件上的引脚连接:----------
RS232接口：
RS232TXD  -->  PA9  (UART1-TXD)
RS232RXD  -->  PA10 (UART1-RXD)
------------------------------------
 */

#include "UART2.h"

u8 TxBuffer[258];
u8 TxCounter=0;
u8 count=0; 

void NVIC_Configuration(void)
{
        NVIC_InitTypeDef NVIC_InitStructure; 
          /* Enable the USART1 Interrupt */
        NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 7;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);
}

/**************************实现函数********************************************
*函数原型:		void Initial_UART1(u32 baudrate)
*功　　能:		初始化STM32-SDK开发板上的RS232接口
输入参数：
		u32 baudrate   设置RS232串口的波特率
输出参数：没有	
*******************************************************************************/
void Initial_UART1(u32 baudrate)
{

	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	USART_InitStructure.USART_BaudRate = baudrate;//500000;                 /*设置波特率为115200*/
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;  /*设置数据位为8位*/
    USART_InitStructure.USART_StopBits = USART_StopBits_1;       /*设置停止位为1位*/
    USART_InitStructure.USART_Parity = USART_Parity_No;          /*无奇偶校验*/    
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; /*没有硬件流控*/
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;      /*发送与接收*/
    /*完成串口COM1的时钟配置、GPIO配置，根据上述参数初始化并使能*/
	
	USART_Init(USART1, &USART_InitStructure);
	USART_Cmd(USART1, ENABLE);
	
	USART_ITConfig(USART1, USART_IT_TXE, ENABLE);//打开中断
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

  USART_ClearFlag(USART1, USART_FLAG_TC);       //清除中断标志
	USART_ClearFlag(USART1, USART_FLAG_TXE);      
	USART_ClearFlag(USART1, USART_FLAG_RXNE);
	
	
	NVIC_Configuration();
}



/**************************实现函数********************************************
*函数原型:		void UART1_Put_Char(unsigned char DataToSend)
*功　　能:		RS232发送一个字节
输入参数：
		unsigned char DataToSend   要发送的字节数据
输出参数：没有	
*******************************************************************************/
void UART1_Put_Char(unsigned char DataToSend)
{
	//将要发送的字节写到UART1的发送缓冲区
	//USART_SendData(USART1, (unsigned char) DataToSend);
	//等待发送完成
  	//while (!(USART1->SR & USART_FLAG_TXE));

	TxBuffer[count++] = DataToSend;  
    USART_ITConfig(USART1, USART_IT_TXE, ENABLE);  
}

/**************************实现函数********************************************
*函数原型:		u8 UART1_Get_Char(void)
*功　　能:		RS232接收一个字节  一直等待，直到UART1接收到一个字节的数据。
输入参数：		 没有
输出参数：       UART1接收到的数据	
*******************************************************************************/
u8 UART1_Get_Char(void)
{
	while (!(USART1->SR & USART_FLAG_RXNE));
	return(USART_ReceiveData(USART1));
}

/**************************实现函数********************************************
*函数原型:		void UART1_Put_String(unsigned char *Str)
*功　　能:		RS232发送字符串
输入参数：
		unsigned char *Str   要发送的字符串
输出参数：没有	
*******************************************************************************/
void UART1_Put_String(unsigned char *Str)
{
	//判断Str指向的数据是否有效.
	while(*Str){
	//是否是回车字符 如果是,则发送相应的回车 0x0d 0x0a
	if(*Str=='\r')UART1_Put_Char(0x0d);
		else if(*Str=='\n')UART1_Put_Char(0x0a);
			else UART1_Put_Char(*Str);
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
*函数原型:		void UART1_Putc_Hex(uint8_t b)
*功　　能:		RS232以十六进制ASCII码的方式发送一个字节数据
				先将目标字节数据高4位转成ASCCII ，发送，再将低4位转成ASCII发送
				如:0xF2 将发送 " F2 "
输入参数：
		uint8_t b   要发送的字节
输出参数：没有	
*******************************************************************************/
void UART1_Putc_Hex(uint8_t b)
{
      /* 判断目标字节的高4位是否小于10 */
    if((b >> 4) < 0x0a)
        UART1_Put_Char((b >> 4) + '0'); //小于10  ,则相应发送0-9的ASCII
    else
        UART1_Put_Char((b >> 4) - 0x0a + 'A'); //大于等于10 则相应发送 A-F

    /* 判断目标字节的低4位 是否小于10*/
    if((b & 0x0f) < 0x0a)
        UART1_Put_Char((b & 0x0f) + '0');//小于10  ,则相应发送0-9的ASCII
    else
        UART1_Put_Char((b & 0x0f) - 0x0a + 'A');//大于等于10 则相应发送 A-F
   UART1_Put_Char(' '); //发送一个空格,以区分开两个字节
}

/**************************实现函数********************************************
*函数原型:		void UART1_Putw_Hex(uint16_t w)
*功　　能:		RS232以十六进制ASCII码的方式发送一个字的数据.就是发送一个int
				如:0x3456 将发送 " 3456 "
输入参数：
		uint16_t w   要发送的字
输出参数：没有	
*******************************************************************************/
void UART1_Putw_Hex(uint16_t w)
{
	//发送高8位数据,当成一个字节发送
    UART1_Putc_Hex((uint8_t) (w >> 8));
	//发送低8位数据,当成一个字节发送
    UART1_Putc_Hex((uint8_t) (w & 0xff));
}

/**************************实现函数********************************************
*函数原型:		void UART1_Putdw_Hex(uint32_t dw)
*功　　能:		RS232以十六进制ASCII码的方式发送32位的数据.
				如:0xF0123456 将发送 " F0123456 "
输入参数：
		uint32_t dw   要发送的32位数据值
输出参数：没有	
*******************************************************************************/
void UART1_Putdw_Hex(uint32_t dw)
{
    UART1_Putw_Hex((uint16_t) (dw >> 16));
    UART1_Putw_Hex((uint16_t) (dw & 0xffff));
}

/**************************实现函数********************************************
*函数原型:		void UART1_Putw_Dec(uint16_t w)
*功　　能:		RS232以十进制ASCII码的方式发送16位的数据.
				如:0x123 将发送它的十进制数据 " 291 "
输入参数：
		uint16_t w   要发送的16位数据值
输出参数：没有	
*******************************************************************************/
void UART1_Putw_Dec(uint32_t w)
{
    uint32_t num = 100000;
    uint8_t started = 0;

    while(num > 0)
    {
        uint8_t b = w / num;
        if(b > 0 || started || num == 1)
        {
            UART1_Put_Char('0' + b);
            started = 1;
        }
        w -= b * num;

        num /= 10;
    }
}

void UART1_Putint_Dec(int16_t in)
{
	if(in<0){
	in=-in;
	UART1_Put_Char('-');
	}
   UART1_Putw_Dec(in);
}

void UART1_Putintp_Dec(int16_t in)
{
	if(in<0){
	in=-in;
	UART1_Put_Char('-');
	}
   UART1_Putw_Dec(in/10);
   UART1_Put_Char('.');
   UART1_Putw_Dec(in%10);
}

void UART1_Reportuint16(int16_t avalue, uint32_t* asum)
{
    char ctemp;
	if(avalue<0)avalue=32768-avalue;
	ctemp=avalue>>8;
	UART1_Put_Char(ctemp);
	*asum = *asum + ctemp;
	ctemp=avalue;
	UART1_Put_Char(ctemp);
	*asum = *asum + ctemp;
}

void UART1_Reportint16(int16_t avalue, uint32_t* asum)
{
   	char ctemp;
	ctemp=avalue>>8;
	UART1_Put_Char(ctemp);
	*asum = *asum + ctemp;

	ctemp=avalue;							
	UART1_Put_Char(ctemp);
	*asum = *asum + ctemp;
}

void UART1_Reportfloat(float avalue, uint32_t* asum)
{
    uint32_t highaddr, lowaddr;
	int16_t	high, low;
	highaddr = (uint32_t)(&avalue);
	lowaddr = highaddr + 2;
    
	high = *((int16_t*)highaddr);
	UART1_Reportint16(high, asum);
	low = *((int16_t*)lowaddr);
	UART1_Reportint16(low, asum);

}

void UART1_ReportChipID(int16_t ChipID)
{
 	uint32_t temp=0xa8 + 6;
	UART1_Put_Char(0xa5);
	UART1_Put_Char(0x5a);
	UART1_Put_Char(6);
	UART1_Put_Char(0xa8);
	UART1_Reportint16(ChipID, &temp);
	UART1_Put_Char(temp%256);
	UART1_Put_Char(0xaa); 
}

void UART1_ReportData(char* dataptr)
{
    int16_t i;
	int16_t nsize;
	uint32_t temp=0;
    if ((dataptr == 0) || dataptr[0] <= 0)
	    return;
    nsize = dataptr[0] + 2;
	UART1_Put_Char(0xa5);
	UART1_Put_Char(0x5a);
	UART1_Put_Char(nsize);
	temp += nsize;
    for (i=1; i<dataptr[0]; i++)
	{
	    UART1_Put_Char(dataptr[i]);
		temp += dataptr[i];
	}
	UART1_Put_Char(temp%256);
	UART1_Put_Char(0xaa);         
}

void UART1_ReportQuat(float q0, float q1, float q2, float q3)
{
 	uint32_t temp=0xA3 + 20;
	UART1_Put_Char(0xa5);
	UART1_Put_Char(0x5a);
	UART1_Put_Char(16+4);
	UART1_Put_Char(0xA3);
	UART1_Reportfloat(q0, &temp);
	UART1_Reportfloat(q1, &temp);
	UART1_Reportfloat(q2, &temp);
	UART1_Reportfloat(q3, &temp);
	UART1_Put_Char(temp%256);
	UART1_Put_Char(0xaa);
}

void UART1_ReportYawPitchRoll(int16_t yaw, int16_t pitch, int16_t roll)
{
 	uint32_t temp=0xA7 + 6 + 4;
	UART1_Put_Char(0xa5);
	UART1_Put_Char(0x5a);
	UART1_Put_Char(6+4);
	UART1_Put_Char(0xA7);
	UART1_Reportint16(yaw, &temp);
	UART1_Reportint16(pitch, &temp);
	UART1_Reportint16(roll, &temp);
	UART1_Put_Char(temp%256);
	UART1_Put_Char(0xaa);
}


void UART1_ReportMotionAndGrav(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy , int16_t gz, int16_t hx, int16_t hy, int16_t hz, float* grav)
{
 	uint32_t temp=0xA4 + 34;
	UART1_Put_Char(0xa5);
	UART1_Put_Char(0x5a);
	UART1_Put_Char(30+4);
	UART1_Put_Char(0xA4);
	UART1_Reportint16(ax, &temp);
	UART1_Reportint16(ay, &temp);
	UART1_Reportint16(az, &temp);
	UART1_Reportint16(gx, &temp);
	UART1_Reportint16(gy, &temp);
	UART1_Reportint16(gz, &temp);
	UART1_Reportint16(hx, &temp);
	UART1_Reportint16(hy, &temp);
	UART1_Reportint16(hz, &temp);

	UART1_Reportfloat(grav[0], &temp);
	UART1_Reportfloat(grav[1], &temp);
	UART1_Reportfloat(grav[2], &temp);
	UART1_Put_Char(temp%256);
	UART1_Put_Char(0xaa);
}

void UART1_ReportAccOffset(int16_t offax, int16_t offay, int16_t offaz)
{
 	uint32_t temp=0xa0 + 10;
	UART1_Put_Char(0xa5);
	UART1_Put_Char(0x5a);
	UART1_Put_Char(10);
	UART1_Put_Char(0xa0);
	UART1_Reportint16(offax, &temp);
	UART1_Reportint16(offay, &temp);
	UART1_Reportint16(offaz, &temp);
	UART1_Put_Char(temp%256);
	UART1_Put_Char(0xaa);    
}

void UART1_ReportComOffset(int16_t offhx, int16_t offhy, int16_t offhz)
{
 	uint32_t temp=0xa6 + 10;
	UART1_Put_Char(0xa5);
	UART1_Put_Char(0x5a);
	UART1_Put_Char(10);
	UART1_Put_Char(0xa6);
	UART1_Reportint16(offhx, &temp);
	UART1_Reportint16(offhy, &temp);
	UART1_Reportint16(offhz, &temp);
	UART1_Put_Char(temp%256);
	UART1_Put_Char(0xaa); 
}


void UART1_ReportIMU(int16_t yaw,int16_t pitch,int16_t roll
,int16_t alt,int16_t tempr,int16_t press,int16_t IMUpersec)
{
 	unsigned int temp=0xaF+2+2;
	char ctemp;
	
	UART1_Put_Char(0xa5);
	UART1_Put_Char(0x5a);
	UART1_Put_Char(14+4);
	UART1_Put_Char(0xA1);

	if(yaw<0)yaw=32768-yaw;
	ctemp=yaw>>8;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=yaw;
	UART1_Put_Char(ctemp);
	temp+=ctemp;

	if(pitch<0)pitch=32768-pitch;
	ctemp=pitch>>8;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=pitch;
	UART1_Put_Char(ctemp);
	temp+=ctemp;

	if(roll<0)roll=32768-roll;
	ctemp=roll>>8;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=roll;
	UART1_Put_Char(ctemp);
	temp+=ctemp;

	if(alt<0)alt=32768-alt;
	ctemp=alt>>8;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=alt;
	UART1_Put_Char(ctemp);
	temp+=ctemp;

	if(tempr<0)tempr=32768-tempr;
	ctemp=tempr>>8;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=tempr;
	UART1_Put_Char(ctemp);
	temp+=ctemp;

	if(press<0)press=32768-press;
	ctemp=press>>8;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=press;
	UART1_Put_Char(ctemp);
	temp+=ctemp;

	ctemp=IMUpersec>>8;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=IMUpersec;
	UART1_Put_Char(ctemp);
	temp+=ctemp;

	UART1_Put_Char(temp%256);
	UART1_Put_Char(0xaa);
}

void UART1_ReportMotion(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz,
					int16_t hx,int16_t hy,int16_t hz)
{
 	unsigned int temp=0xaF+9;
	char ctemp;
	UART1_Put_Char(0xa5);
	UART1_Put_Char(0x5a);
	UART1_Put_Char(14+8);
	UART1_Put_Char(0xA2);

	if(ax<0)ax=32768-ax;
	ctemp=ax>>8;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=ax;
	UART1_Put_Char(ctemp);
	temp+=ctemp;

	if(ay<0)ay=32768-ay;
	ctemp=ay>>8;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=ay;
	UART1_Put_Char(ctemp);
	temp+=ctemp;

	if(az<0)az=32768-az;
	ctemp=az>>8;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=az;
	UART1_Put_Char(ctemp);
	temp+=ctemp;

	if(gx<0)gx=32768-gx;
	ctemp=gx>>8;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=gx;
	UART1_Put_Char(ctemp);
	temp+=ctemp;

	if(gy<0)gy=32768-gy;
	ctemp=gy>>8;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=gy;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
//-------------------------
	if(gz<0)gz=32768-gz;
	ctemp=gz>>8;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=gz;
	UART1_Put_Char(ctemp);
	temp+=ctemp;

	if(hx<0)hx=32768-hx;
	ctemp=hx>>8;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=hx;
	UART1_Put_Char(ctemp);
	temp+=ctemp;

	if(hy<0)hy=32768-hy;
	ctemp=hy>>8;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=hy;
	UART1_Put_Char(ctemp);
	temp+=ctemp;

	if(hz<0)hz=32768-hz;
	ctemp=hz>>8;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=hz;
	UART1_Put_Char(ctemp);
	temp+=ctemp;

	UART1_Put_Char(temp%256);
	UART1_Put_Char(0xaa);
}
 
//------------------------------------------------------
// void USART1_IRQHandler(void)
// {
//   
//   if(USART_GetITStatus(USART1, USART_IT_TXE) != RESET)
//   {   
//     /* Write one byte to the transmit data register */
//     USART_SendData(USART1, TxBuffer[TxCounter++]);                    

//     /* Clear the USART1 transmit interrupt */
//     USART_ClearITPendingBit(USART1, USART_IT_TXE); 

//     if(TxCounter == count)
//     {
//       /* Disable the USART1 Transmit interrupt */
//       USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
//     }    
//   }

// }


//------------------End of File----------------------------
