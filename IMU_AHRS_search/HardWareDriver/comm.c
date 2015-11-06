#include "comm.h"
#include "IMU.h"

IMU_OUTPUT_REG IMU_OutputReg;
FIFO_QUEUE_STRU TX_FIFO;
volatile uint8_t weightlessness = 0;
void EulorToIMU_Reg(int16_t yaw, int16_t pitch, int16_t roll, int16_t acclen, int16_t chipid, float* pos)
{
    int16_t i = 1, j, ntemp;
	__disable_irq();//保证读取和写入的原子性
	//IMU_OutputReg.buffer[i++] = 0xA7;
	IMU_OutputReg.buffer[i++] = (weightlessness<<7)|(1<<5)|(DEVICE_ID);
	IMU_OutputReg.buffer[i++] = yaw>>8;
	IMU_OutputReg.buffer[i++] = yaw;
	IMU_OutputReg.buffer[i++] = pitch>>8;
	IMU_OutputReg.buffer[i++] = pitch;
	IMU_OutputReg.buffer[i++] = roll>>8;
	IMU_OutputReg.buffer[i++] = roll;
	//if ((DEVICE_ID == 13)||(DEVICE_ID == 10))
	//{
	    IMU_OutputReg.buffer[i++] = acclen>>8;
      IMU_OutputReg.buffer[i++] = acclen;
	if (pos != 0)
	{
		  for (j=0; j<3; j++)
		  {
				  ntemp = (int16_t)(pos[j]*10000);
				  IMU_OutputReg.buffer[i++] = ntemp>>8;
	        IMU_OutputReg.buffer[i++] = ntemp;
				  //npos = *((uint32_t*)(pos+j));
				  //IMU_OutputReg.buffer[i++] = npos;
				  //npos = npos>>8;
				  //IMU_OutputReg.buffer[i++] = npos;
				  //npos = npos>>8;
          //IMU_OutputReg.buffer[i++] = npos;
				  //npos = npos>>8;
				  //IMU_OutputReg.buffer[i++] = npos;
		  }
	}
	//}
	//IMU_OutputReg.buffer[i++] = chipid>>8;
	//IMU_OutputReg.buffer[i++] = chipid;
	IMU_OutputReg.buffer[0] = i;
	__enable_irq();
    return;
}

void QuatToIMU_Reg(float* quat, int16_t acclen, float* pos)
{
	  int16_t i = 1, j, ntemp;
	  __disable_irq();//保证读取和写入的原子性
	  IMU_OutputReg.buffer[i++] = (weightlessness<<7)|(2<<5)|(DEVICE_ID);
	  ntemp = (int16_t)(quat[0]*10000);
	  IMU_OutputReg.buffer[i++] = ntemp>>8;
	  IMU_OutputReg.buffer[i++] = ntemp;
	  
    ntemp = (int16_t)(quat[1]*10000);
	  IMU_OutputReg.buffer[i++] = ntemp>>8;
	  IMU_OutputReg.buffer[i++] = ntemp;	
	
	  ntemp = (int16_t)(quat[2]*10000);
	  IMU_OutputReg.buffer[i++] = ntemp>>8;
	  IMU_OutputReg.buffer[i++] = ntemp;
	
	  ntemp = (int16_t)(quat[3]*10000);
	  IMU_OutputReg.buffer[i++] = ntemp>>8;
	  IMU_OutputReg.buffer[i++] = ntemp;
	
		IMU_OutputReg.buffer[i++] = acclen>>8;
    IMU_OutputReg.buffer[i++] = acclen;
		if (pos != 0)
	  {
		  for (j=0; j<3; j++)
		  {
				  ntemp = (int16_t)(pos[j]*10000);
				  IMU_OutputReg.buffer[i++] = ntemp>>8;
	        IMU_OutputReg.buffer[i++] = ntemp;
		  }
	  }
	  IMU_OutputReg.buffer[0] = i;
		__enable_irq();
	  return;
}

void accgyrhmctoIMU_Reg(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz, int16_t hx, int16_t hy, int16_t hz, int16_t chipid)
{
    int16_t i = 1;
	__disable_irq();//保证读取和写入的原子性
    IMU_OutputReg.buffer[i++] = DEVICE_ID;
	if(ax<0)ax=32768-ax;
	IMU_OutputReg.buffer[i++] = ax>>8;//保存高八位
	IMU_OutputReg.buffer[i++] = ax; //保存第八位
	if(ay<0)ay=32768-ay;
	IMU_OutputReg.buffer[i++] = ay>>8;
	IMU_OutputReg.buffer[i++] = ay;
	if(az<0)az=32768-az;
	IMU_OutputReg.buffer[i++] = az>>8;
	IMU_OutputReg.buffer[i++] = az;

	if(gx<0)gx=32768-gx;
	IMU_OutputReg.buffer[i++] = gx>>8;
	IMU_OutputReg.buffer[i++] = gx;
	if(gy<0)gy=32768-gy;
	IMU_OutputReg.buffer[i++] = gy>>8;
	IMU_OutputReg.buffer[i++] = gy;
	if(gz<0)gz=32768-gz;
	IMU_OutputReg.buffer[i++] = gz>>8;
	IMU_OutputReg.buffer[i++] = gz;

	if(hx<0) hx=32768-hx;
	IMU_OutputReg.buffer[i++] = hx>>8;
	IMU_OutputReg.buffer[i++] = hx;
	if(hy<0) hy=32768-hy;
	IMU_OutputReg.buffer[i++] = hy>>8;
	IMU_OutputReg.buffer[i++] = hy;
	if(hz<0) hz=32768-hz;
	IMU_OutputReg.buffer[i++] = hz>>8;
	IMU_OutputReg.buffer[i++] = hz;
	IMU_OutputReg.buffer[i++] = chipid>>8;
	IMU_OutputReg.buffer[i++] = chipid;
	IMU_OutputReg.buffer[0] = i;
	__enable_irq();
    return;
}

uint8_t Write_FIFO(uint8_t *pdata,uint8_t length)
{
		uint8_t i;
	
  if(FIFO_SIZE - TX_FIFO.count >= length) 
  { 
		__disable_irq();
		for (i=0; i<length; i++)
		{
			TX_FIFO.data[TX_FIFO.rear] = *(pdata+i); 
			TX_FIFO.rear = (TX_FIFO.rear + 1) % FIFO_SIZE; 
			TX_FIFO.count = TX_FIFO.count + 1; 
			if (TX_FIFO.count == FIFO_SIZE)  TX_FIFO.rear = TX_FIFO.front;
		}
		__enable_irq();
    return SUCCESS; 
  }
	else
	
	  return ERROR; 
}


void Update_OutputIMU_Reg(int16_t yaw,int16_t pitch,int16_t roll,int16_t IMUpersec)//更新数据输出寄存器
{
    return;
	/*uint32_t crc_list[5];//crc计算数组用于向crc计算器传入数据.
	uint8_t count_byte =15;//不含帧头2字节 包长1 + ID1 + yaw2 +pitch2+ roll2+ HZ2+crc4 +包尾1
	uint8_t function_byte = 1<<5;//这里与原协议不同 高3位用来表示功能字节 低5位用来表示设备ID 最多可以有32个设备
	//帧头
	if(IMU_OutputReg.head == 0)//帧头固定,只需要算一次就可以了
	{
		IMU_OutputReg.head =(uint32_t)((uint32_t)FRAME_HEADER<<16)|((uint32_t)count_byte<<8)|function_byte|DEVICE_ID;
		crc_list[0] = (uint32_t)IMU_OutputReg.head;
	}
	if(IMU_OutputReg.rear == 0)
	{
		IMU_OutputReg.rear = FRAME_REAR;
	}
		
	
	//姿态角和CRC交验数据更新到输出寄存器
	__disable_irq();//保证读取和写入的原子性
	
	IMU_OutputReg.yaw = yaw;
	IMU_OutputReg.pitch = pitch;
	IMU_OutputReg.roll = roll;
	IMU_OutputReg.IMUpersec = IMUpersec;
	crc_list[1] = (uint32_t)(yaw<<16|pitch);
	crc_list[2] = (uint32_t)(roll<<16|IMUpersec);
	CRC_ResetDR();
	IMU_OutputReg.crcresult = CRC_CalcBlockCRC(crc_list,3);
	
	__enable_irq();	*/
	
}

void Add_Uint32_To_List(uint32_t date , uint8_t* list)
{
	list[0]= (date&0xff000000)>>24;
	list[1]= (date&0x00ff0000)>>16;
	list[2]= (date&0x0000ff00)>>8;
	list[3]= (date&0x000000ff);	
}

void Add_Uint16_To_List(uint16_t date , uint8_t* list)
{
	list[0]= (date&0xff00)>>8;
	list[1]= (date&0x00ff);
}

void Add_Uint8_To_List(uint8_t date , uint8_t* list)
{
	list[0]= (date);
}
/***********输出协议在这里修改**************/
uint8_t IRQ_Get_IMU_Data_Package(uint8_t* pdata)//在中断中调用该函数,将寄存器中的数据组织成数据包
{
	uint8_t i, j;
	uint32_t temp=0;
	uint8_t count = IMU_OutputReg.buffer[0] + 4; //加包头2个字节，校验1个字节，包尾1个字节
	pdata[0] = 0xa5;
	pdata[1] = 0x5a;
	pdata[2] = IMU_OutputReg.buffer[0]+2;//加校验1个字节，包尾1个字节
	temp += pdata[2];
	i = 3;

	for (j=1; j<IMU_OutputReg.buffer[0]; j++)
	{
	    pdata[i++] = IMU_OutputReg.buffer[j];
		temp += IMU_OutputReg.buffer[j];
	}
	pdata[i++] = temp%256;
	pdata[i++] = 0xaa;
	return count;
 

	/*uint8_t count=0;
	//包名
	Add_Uint32_To_List(IMU_OutputReg.head,&pdata[count]);
	count+=4;
	//四元数
	Add_Uint16_To_List(IMU_OutputReg.yaw ,&pdata[count]);
	count+=2;
	Add_Uint16_To_List(IMU_OutputReg.pitch ,&pdata[count]);
	count+=2;
	Add_Uint16_To_List(IMU_OutputReg.roll ,&pdata[count]);
	count+=2;
	Add_Uint16_To_List(IMU_OutputReg.IMUpersec ,&pdata[count]);
	count+=2;
	//crc校验
	Add_Uint32_To_List(IMU_OutputReg.crcresult,&pdata[count]);
	count+=4;
	Add_Uint8_To_List(IMU_OutputReg.rear,&pdata[count]);
	count+=1;
	return count;*/
}

ErrorStatus IRQ_Write_TX_FIFO(uint8_t *pdata,uint8_t length)
{
	uint8_t i;
  if(FIFO_SIZE - TX_FIFO.count >= length) 
  { 
		for (i=0; i<length; i++)
		{
			TX_FIFO.data[TX_FIFO.rear] = *(pdata+i); 
			TX_FIFO.rear = (TX_FIFO.rear + 1) % FIFO_SIZE; 
			TX_FIFO.count = TX_FIFO.count + 1; 
			if (TX_FIFO.count == FIFO_SIZE)  TX_FIFO.rear = TX_FIFO.front;
		}
    return SUCCESS; 
  } 
	else
	  return ERROR;
}

ErrorStatus IRQ_Read_TX_FIFO(uint8_t *data)
{	
  if((TX_FIFO.front == TX_FIFO.rear) && (TX_FIFO.count == 0)) 
  {
    return ERROR; 
  }
	else 
  {
    *data = TX_FIFO.data[TX_FIFO.front]; 
    TX_FIFO.front = (TX_FIFO.front + 1) % FIFO_SIZE; 
    TX_FIFO.count = TX_FIFO.count - 1; 
    return SUCCESS; 
  } 
}

void Start_Data_Auto_Send(void)
{
	USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
}

void End_Data_Auto_Send(void)
{
	USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
}

void USART1_IRQHandler(void)
{
	uint8_t length,tx_buffer[PACKAGE_MAX_SIZE];
	uint8_t tx_date;
	__disable_irq();
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)//收到数据
	{
		USART_ClearITPendingBit(USART1,USART_IT_RXNE); 
		if(USART_ReceiveData(USART1)==DEVICE_ID)
		{
			length = IRQ_Get_IMU_Data_Package(tx_buffer);
			if(length<PACKAGE_MAX_SIZE)
			{
				if(IRQ_Write_TX_FIFO(tx_buffer,length)==SUCCESS)
					Start_Data_Auto_Send();//打开发送中断，串口自动发码直到FIFO为空
			}
			else
			{
				__NOP();//数据包过长 无法写入
			}
		}
	}
	if(USART_GetITStatus(USART1, USART_IT_TXE) != RESET)//发送一个字节完成
	{
		USART_ClearITPendingBit(USART1,USART_IT_TXE); 
		if(IRQ_Read_TX_FIFO(&tx_date)==SUCCESS)
			USART_SendData(USART1,tx_date);
		else
			End_Data_Auto_Send();//FIFO为空结束自动发送		
	}
	__enable_irq();
}
