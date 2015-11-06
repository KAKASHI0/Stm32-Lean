#ifndef __IMU_H
#define __IMU_H

#include "common.h"  //包含所有的驱动 头文件

#include <math.h>
#define M_PI  (float)3.1415926535


//Mini IMU AHRS 解算的API
void IMU_init(void); //初始化
void IMU_getQuat(float* pquat, uint16_t* acclen, float* position);
void IMU_getYawPitchRollAndQuat(float * ypr, float * quat); //更新姿态
void IMU_getYPRAngle(float* yprangle, uint16_t* acclen, float* position);//获取自定义的yaw, pitch, roll角度，跟四元数的Yaw, Pitch, Roll 不一样
//void IMU_getQuatAndPos(float* quat, uint16_t* acclen, float* position);
uint32_t micros(void);	//读取系统上电后的时间  单位 us 
void IMU_getgravity(/*float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, */float* grav);

#endif

//------------------End of File----------------------------
