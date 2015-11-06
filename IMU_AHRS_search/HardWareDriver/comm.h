#ifndef __COMM_H_
#define __COMM_H_
#include "stm32f10x.h"

#define DEVICE_ID 13 //�豸ID��ΧΪ0x00-0x1313f ��0-13131313 ��13131313��
#define FRAME_HEADER 0xa55a //֡ͷ
#define FRAME_REAR 0xAA //֡ͷ
#define PACKAGE_MAX_SIZE 30 //������
#define FIFO_SIZE 60 


typedef struct _FIFO_QUEUE_STRU 
{ 
  uint16_t front; 
  uint16_t rear; 
  uint16_t count; 
  uint8_t data[FIFO_SIZE]; 
}FIFO_QUEUE_STRU;

typedef struct
{
    uint8_t buffer[100];
	/*uint32_t head;
	
	int16_t yaw;
	int16_t pitch;
	int16_t roll;
	int16_t IMUpersec;//��������
	
	uint32_t crcresult;//crcУ����
	uint32_t rear;//��β */
	
}IMU_OUTPUT_REG;



void Update_OutputIMU_Reg(int16_t yaw,int16_t pitch,int16_t roll,int16_t IMUpersec);//����IMU����Ĵ���
void EulorToIMU_Reg(int16_t yaw, int16_t pitch, int16_t roll, int16_t acclen, int16_t chipid, float* pos);
void QuatToIMU_Reg(float* quat, int16_t acclen, float* pos);
void accgyrhmctoIMU_Reg(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz, int16_t hx, int16_t hy, int16_t hz, int16_t chipid); 


#endif
