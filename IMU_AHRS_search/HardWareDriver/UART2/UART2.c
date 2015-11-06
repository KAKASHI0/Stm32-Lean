/* UART2.c file
��д�ߣ�lisn3188
��ַ��www.chiplab7.com
����E-mail��lisn3188@163.com
���뻷����MDK-Lite  Version: 4.23
����ʱ��: 2012-04-25
���ԣ� ���������ڵ���ʵ���ҵ�mini IMU����ɲ���
���ܣ�
UART1ͨ�� ��API
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

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void Initial_UART2(u32 baudrate)
*��������:		��ʼ��STM32-SDK�������ϵ�RS232�ӿ�
���������
		u32 baudrate   ����RS232���ڵĲ�����
���������û��	
*******************************************************************************/
void Initial_UART2(u32 baudrate)
{
 	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	/* ʹ�� UART2 ģ���ʱ��  ʹ�� UART2��Ӧ�����Ŷ˿�PA��ʱ��*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2 | RCC_APB2Periph_GPIOA, ENABLE);

  	 /* ����UART2 �ķ�������
	 ����PA9 Ϊ�������  ˢ��Ƶ��50MHz
	  */
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_Init(GPIOA, &GPIO_InitStructure);    
  	/* 
	  ����UART2 �Ľ�������
	  ����PA10Ϊ�������� 
	 */
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  	GPIO_Init(GPIOA, &GPIO_InitStructure);
	  
	/* 
	  UART2������:
	  1.������Ϊ���ó���ָ�������� baudrate;
	  2. 8λ����			  USART_WordLength_8b;
	  3.һ��ֹͣλ			  USART_StopBits_1;
	  4. ����żЧ��			  USART_Parity_No ;
	  5.��ʹ��Ӳ��������	  USART_HardwareFlowControl_None;
	  6.ʹ�ܷ��ͺͽ��չ���	  USART_Mode_Rx | USART_Mode_Tx;
	 */
	USART_InitStructure.USART_BaudRate = baudrate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	//Ӧ�����õ�UART2
	USART_Init(USART2, &USART_InitStructure); 
	USART_ITConfig(USART2, USART_IT_TXE, DISABLE);        
    USART_ClearFlag(USART2,USART_FLAG_TC);
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);	//ʹ�ܽ����ж�
	//����UART2
  	USART_Cmd(USART2, ENABLE);
	U2NVIC_Configuration();
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void UART2_Put_Char(unsigned char DataToSend)
*��������:		RS232����һ���ֽ�
���������
		unsigned char DataToSend   Ҫ���͵��ֽ�����
���������û��	
*******************************************************************************/
void UART2_Put_Char(unsigned char DataToSend)
{
	//��Ҫ���͵��ֽ�д��UART2�ķ��ͻ�����
	//USART_SendData(USART1, (unsigned char) DataToSend);
	//�ȴ��������
  	//while (!(USART1->SR & USART_FLAG_TXE));

	U2TxBuffer[U2count++] = DataToSend;  
    USART_ITConfig(USART2, USART_IT_TXE, ENABLE);  
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		u8 UART2_Get_Char(void)
*��������:		RS232����һ���ֽ�  һֱ�ȴ���ֱ��UART2���յ�һ���ֽڵ����ݡ�
���������		 û��
���������       UART2���յ�������	
*******************************************************************************/
u8 UART2_Get_Char(void)
{
	while (!(USART2->SR & USART_FLAG_RXNE));
	return(USART_ReceiveData(USART2));
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void UART2_Put_String(unsigned char *Str)
*��������:		RS232�����ַ���
���������
		unsigned char *Str   Ҫ���͵��ַ���
���������û��	
*******************************************************************************/
void UART2_Put_String(unsigned char *Str)
{
	//�ж�Strָ��������Ƿ���Ч.
	while(*Str){
	//�Ƿ��ǻس��ַ� �����,������Ӧ�Ļس� 0x0d 0x0a
	if(*Str=='\r')UART2_Put_Char(0x0d);
		else if(*Str=='\n')UART2_Put_Char(0x0a);
			else UART2_Put_Char(*Str);
	//�ȴ��������.
  	//while (!(USART1->SR & USART_FLAG_TXE));
	//ָ��++ ָ����һ���ֽ�.
	Str++;
	}
/*
	//�ж�Strָ��������Ƿ���Ч.
	while(*Str){
	//�Ƿ��ǻس��ַ� �����,������Ӧ�Ļس� 0x0d 0x0a
	if(*Str=='\r')USART_SendData(USART1, 0x0d);
		else if(*Str=='\n')USART_SendData(USART1, 0x0a);
			else USART_SendData(USART1, *Str);
	//�ȴ��������.
  	while (!(USART1->SR & USART_FLAG_TXE));
	//ָ��++ ָ����һ���ֽ�.
	Str++;
	}		 */
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void UART2_Putc_Hex(uint8_t b)
*��������:		RS232��ʮ������ASCII��ķ�ʽ����һ���ֽ�����
				�Ƚ�Ŀ���ֽ����ݸ�4λת��ASCCII �����ͣ��ٽ���4λת��ASCII����
				��:0xF2 ������ " F2 "
���������
		uint8_t b   Ҫ���͵��ֽ�
���������û��	
*******************************************************************************/
void UART2_Putc_Hex(uint8_t b)
{
      /* �ж�Ŀ���ֽڵĸ�4λ�Ƿ�С��10 */
    if((b >> 4) < 0x0a)
        UART2_Put_Char((b >> 4) + '0'); //С��10  ,����Ӧ����0-9��ASCII
    else
        UART2_Put_Char((b >> 4) - 0x0a + 'A'); //���ڵ���10 ����Ӧ���� A-F

    /* �ж�Ŀ���ֽڵĵ�4λ �Ƿ�С��10*/
    if((b & 0x0f) < 0x0a)
        UART2_Put_Char((b & 0x0f) + '0');//С��10  ,����Ӧ����0-9��ASCII
    else
        UART2_Put_Char((b & 0x0f) - 0x0a + 'A');//���ڵ���10 ����Ӧ���� A-F
   UART2_Put_Char(' '); //����һ���ո�,�����ֿ������ֽ�
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void UART2_Putw_Hex(uint16_t w)
*��������:		RS232��ʮ������ASCII��ķ�ʽ����һ���ֵ�����.���Ƿ���һ��int
				��:0x3456 ������ " 3456 "
���������
		uint16_t w   Ҫ���͵���
���������û��	
*******************************************************************************/
void UART2_Putw_Hex(uint16_t w)
{
	//���͸�8λ����,����һ���ֽڷ���
    UART2_Putc_Hex((uint8_t) (w >> 8));
	//���͵�8λ����,����һ���ֽڷ���
    UART2_Putc_Hex((uint8_t) (w & 0xff));
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void UART2_Putdw_Hex(uint32_t dw)
*��������:		RS232��ʮ������ASCII��ķ�ʽ����32λ������.
				��:0xF0123456 ������ " F0123456 "
���������
		uint32_t dw   Ҫ���͵�32λ����ֵ
���������û��	
*******************************************************************************/
void UART2_Putdw_Hex(uint32_t dw)
{
    UART2_Putw_Hex((uint16_t) (dw >> 16));
    UART2_Putw_Hex((uint16_t) (dw & 0xffff));
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void UART2_Putw_Dec(uint16_t w)
*��������:		RS232��ʮ����ASCII��ķ�ʽ����16λ������.
				��:0x123 ����������ʮ�������� " 291 "
���������
		uint16_t w   Ҫ���͵�16λ����ֵ
���������û��	
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
return 0xff; //û���յ���λ�����������������Ч��û��ͨ��
}

unsigned char UART2_CommandRoutedata(int16_t index)
{
    return rx_buffer[index]; //û���յ���λ�����������������Ч��û��ͨ��
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
