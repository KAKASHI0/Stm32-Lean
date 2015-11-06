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
/**************************ʵ�ֺ���********************************************
*����ԭ��:	   float invSqrt(float x)
*��������:	   ���ټ��� 1/Sqrt(x) 	
��������� Ҫ�����ֵ
��������� ���
*******************************************************************************/
extern float invSqrt(float x);
#endif
