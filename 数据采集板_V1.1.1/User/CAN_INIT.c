#include 	"stm32f10x.h"
#include 	"can_config.h"

void CAN_Configration(void)
{
			GPIO_InitTypeDef 					GPIO_InitStructure;
	    CAN_InitTypeDef			 			CAN_InitStructure;
      CAN_FilterInitTypeDef 		CAN_FilterInitStructure;
	
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1,ENABLE);
			
			GPIO_InitStructure.GPIO_Pin		=	GPIO_Pin_12;
			GPIO_InitStructure.GPIO_Mode	=	GPIO_Mode_AF_PP;					//推挽输出
			GPIO_InitStructure.GPIO_Speed	=	GPIO_Speed_50MHz;
			GPIO_Init(GPIOA,&GPIO_InitStructure);
			
			GPIO_InitStructure.GPIO_Pin		=	GPIO_Pin_11;
			GPIO_InitStructure.GPIO_Mode	=	GPIO_Mode_IPU;		//上拉输入
			GPIO_Init(GPIOA,&GPIO_InitStructure);
	

			/* CAN register init */
       CAN_DeInit(CAN1);
			// CAN_StructInit(&CAN_InitStructure);
			/* CAN cell init */
			CAN_InitStructure.CAN_TTCM=DISABLE;									//禁止时间触发通信模式
			CAN_InitStructure.CAN_ABOM=DISABLE;									//软件对CAN_MCR寄存器的INRQ位进行置1，随后清0后
																												//到128次11位连续的隐形位，就退出离线状态

			CAN_InitStructure.CAN_AWUM=DISABLE;								//睡眠模式通过清除CAN_MCR寄存器的SLEEP位，有软件唤醒
			CAN_InitStructure.CAN_NART=ENABLE; 								//DISABLE；CAN报文只被发送1次，不管发送的结果如何（成功、出错或者仲裁丢失）
			CAN_InitStructure.CAN_RFLM=DISABLE;								//在接收溢出时FIFO未被锁定，当接收FIFO的报文未被读出，下一个收到的报文会覆盖

			CAN_InitStructure.CAN_TXFP=DISABLE;								//发送FIFO优先级由报文的标识符来决定
			// CAN_InitStructure.CAN_Mode=CAN_Mode_LoopBack;
			CAN_InitStructure.CAN_Mode=CAN_Mode_Normal; 			//CAN硬件工作在正常模式
			CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;						//重新同步跳跃宽度一个时间单位
			CAN_InitStructure.CAN_BS1=CAN_BS1_8tq;					//时间段1为8个时间单位
			CAN_InitStructure.CAN_BS2=CAN_BS2_7tq;					//时间段2为7个时间单位
			CAN_InitStructure.CAN_Prescaler = 9; 						//(pclk1/((1+8+7)*9)) = 36Mhz/16/9 = 250Kbits设定了一个时间单位的长度9
			CAN_Init(CAN1,&CAN_InitStructure);
			
			
			
			/*CAN filter init 过滤器初始化*/
			CAN_FilterInitStructure.CAN_FilterNumber=0;										//指定了待初始化的过滤器
			CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;//指定了过滤器将被初始化到的模式标识符屏蔽位模式

			CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;//给出了过滤器位宽1个32位过滤器
			CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;							//用来设定过滤器标识符（32位位宽时为其高段位，16位位宽时为第一个）

			CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;								//用来设定过滤器标识符（32位位宽时为其低段位，16位位宽时为第二个）

			CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;					// 用来设定过滤器屏蔽标识符或者过滤器标识符（32位位宽时为其高段位，16位位宽时为第一个）

			CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;						//  用来设定过滤器屏蔽标识符或者过滤器标识符（32位位宽时为其低段位，16位位宽时为第二个
			
			CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_FIFO0;		//设定了指向过滤器的FIFO0
			CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;					//使能过滤器
			CAN_FilterInit(&CAN_FilterInitStructure);
			
			/* CAN FIFO0 message pending interrupt enable */
			CAN_ITConfig(CAN1,CAN_IT_FMP0, ENABLE);												//使能指定的CAN中断
			
}

