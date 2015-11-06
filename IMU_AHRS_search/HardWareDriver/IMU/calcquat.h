#ifndef __algorithm_h__
#define __algorithm_h__

extern volatile float q0, q1, q2, q3;
extern void makeRotate(float* quat, float angle, char axistype);
extern void MultiQuat(float* quat1, float* quat2);
extern void EulorToQuat(float* quat, float yaw, float pitch, float roll);
extern void Vec3dXorVec3d(float*vec1, float* vec2, float* dest);
extern void QuatMultiVec3d(float* quat, float* v, float* dest);
extern void calcquatwithgam(float beta, float ONET, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
// Fast inverse square-root
/**************************实现函数********************************************
*函数原型:	   float invSqrt(float x)
*功　　能:	   快速计算 1/Sqrt(x) 	
输入参数： 要计算的值
输出参数： 结果
*******************************************************************************/
extern float invSqrt(float x);
#endif
