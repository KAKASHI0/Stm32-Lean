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
			GPIO_InitStructure.GPIO_Mode	=	GPIO_Mode_AF_PP;					//�������
			GPIO_InitStructure.GPIO_Speed	=	GPIO_Speed_50MHz;
			GPIO_Init(GPIOA,&GPIO_InitStructure);
			
			GPIO_InitStructure.GPIO_Pin		=	GPIO_Pin_11;
			GPIO_InitStructure.GPIO_Mode	=	GPIO_Mode_IPU;		//��������
			GPIO_Init(GPIOA,&GPIO_InitStructure);
	

			/* CAN register init */
       CAN_DeInit(CAN1);
			// CAN_StructInit(&CAN_InitStructure);
			/* CAN cell init */
			CAN_InitStructure.CAN_TTCM=DISABLE;									//��ֹʱ�䴥��ͨ��ģʽ
			CAN_InitStructure.CAN_ABOM=DISABLE;									//�����CAN_MCR�Ĵ�����INRQλ������1�������0��
																												//��128��11λ����������λ�����˳�����״̬

			CAN_InitStructure.CAN_AWUM=DISABLE;								//˯��ģʽͨ�����CAN_MCR�Ĵ�����SLEEPλ�����������
			CAN_InitStructure.CAN_NART=ENABLE; 								//DISABLE��CAN����ֻ������1�Σ����ܷ��͵Ľ����Σ��ɹ�����������ٲö�ʧ��
			CAN_InitStructure.CAN_RFLM=DISABLE;								//�ڽ������ʱFIFOδ��������������FIFO�ı���δ����������һ���յ��ı��ĻḲ��

			CAN_InitStructure.CAN_TXFP=DISABLE;								//����FIFO���ȼ��ɱ��ĵı�ʶ��������
			// CAN_InitStructure.CAN_Mode=CAN_Mode_LoopBack;
			CAN_InitStructure.CAN_Mode=CAN_Mode_Normal; 			//CANӲ������������ģʽ
			CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;						//����ͬ����Ծ���һ��ʱ�䵥λ
			CAN_InitStructure.CAN_BS1=CAN_BS1_8tq;					//ʱ���1Ϊ8��ʱ�䵥λ
			CAN_InitStructure.CAN_BS2=CAN_BS2_7tq;					//ʱ���2Ϊ7��ʱ�䵥λ
			CAN_InitStructure.CAN_Prescaler = 9; 						//(pclk1/((1+8+7)*9)) = 36Mhz/16/9 = 250Kbits�趨��һ��ʱ�䵥λ�ĳ���9
			CAN_Init(CAN1,&CAN_InitStructure);
			
			
			
			/*CAN filter init ��������ʼ��*/
			CAN_FilterInitStructure.CAN_FilterNumber=0;										//ָ���˴���ʼ���Ĺ�����
			CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;//ָ���˹�����������ʼ������ģʽ��ʶ������λģʽ

			CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;//�����˹�����λ��1��32λ������
			CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;							//�����趨��������ʶ����32λλ��ʱΪ��߶�λ��16λλ��ʱΪ��һ����

			CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;								//�����趨��������ʶ����32λλ��ʱΪ��Ͷ�λ��16λλ��ʱΪ�ڶ�����

			CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;					// �����趨���������α�ʶ�����߹�������ʶ����32λλ��ʱΪ��߶�λ��16λλ��ʱΪ��һ����

			CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;						//  �����趨���������α�ʶ�����߹�������ʶ����32λλ��ʱΪ��Ͷ�λ��16λλ��ʱΪ�ڶ���
			
			CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_FIFO0;		//�趨��ָ���������FIFO0
			CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;					//ʹ�ܹ�����
			CAN_FilterInit(&CAN_FilterInitStructure);
			
			/* CAN FIFO0 message pending interrupt enable */
			CAN_ITConfig(CAN1,CAN_IT_FMP0, ENABLE);												//ʹ��ָ����CAN�ж�
			
}

