#ifndef __IMU_H
#define __IMU_H

#include "common.h"  //�������е����� ͷ�ļ�

#include <math.h>
#define M_PI  (float)3.1415926535


//Mini IMU AHRS �����API
void IMU_init(void); //��ʼ��
void IMU_getQuat(float* pquat, uint16_t* acclen, float* position);
void IMU_getYawPitchRollAndQuat(float * ypr, float * quat); //������̬
void IMU_getYPRAngle(float* yprangle, uint16_t* acclen, float* position);//��ȡ�Զ����yaw, pitch, roll�Ƕȣ�����Ԫ����Yaw, Pitch, Roll ��һ��
//void IMU_getQuatAndPos(float* quat, uint16_t* acclen, float* position);
uint32_t micros(void);	//��ȡϵͳ�ϵ���ʱ��  ��λ us 
void IMU_getgravity(/*float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, */float* grav);

#endif

//------------------End of File----------------------------
