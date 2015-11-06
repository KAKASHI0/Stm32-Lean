/* main.c file
编写者：lisn3188
网址：www.chiplab7.com
作者E-mail：lisn3188@163.com
编译环境：MDK-Lite  Version: 4.23
初版时间: 2012-04-25
测试： 本程序已在第七实验室的mini IMU上完成测试
功能：
姿态解算 IMU
将传感器的输出值进行姿态解算。得到目标载体的俯仰角和横滚角 和航向角
------------------------------------
 */

#include "IMU.h"
#include "Matrix.h"
#include "calcquat.h"
#include <math.h>

volatile float exInt, eyInt, ezInt;  // 误差积分
//volatile float q0, q1, q2, q3; // 全局四元数
volatile float integralFBhand,handdiff;
volatile uint32_t lastUpdate, now; // 采样周期计数 单位 us

extern float Accsensitivity;
void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);

/**************************实现函数********************************************
*函数原型:		void Initial_Timer3(void)
*功　　能:	  初始化Tim2  Tim3 将两个定时器级联，以产生一个32位的定时器来提供系统us 级的计时	
输入参数：无
输出参数：没有	
*******************************************************************************/
void Initial_Timer3(void)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3, ENABLE); 
	/* TIM2 configuration*/ 
  /* Time Base configuration 基本配置 配置定时器的时基单元*/
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure); 
  TIM_TimeBaseStructure.TIM_Period = 0xffff; //自动重装值         
  TIM_TimeBaseStructure.TIM_Prescaler = 0x0;       
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;    
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); 
  
  TIM_PrescalerConfig(TIM2, 0, TIM_PSCReloadMode_Update);
  /* Disable the TIM2 Update event */
  TIM_UpdateDisableConfig(TIM2, ENABLE);
  /* ----------------------TIM2 Configuration as slave for the TIM3 ----------*/
  /* Select the TIM2 Input Trigger: TIM3 TRGO used as Input Trigger for TIM2*/
  TIM_SelectInputTrigger(TIM2, TIM_TS_ITR2);
  /* Use the External Clock as TIM2 Slave Mode */
  TIM_SelectSlaveMode(TIM2, TIM_SlaveMode_External1);
  /* Enable the TIM2 Master Slave Mode */
  TIM_SelectMasterSlaveMode(TIM2, TIM_MasterSlaveMode_Enable);
  TIM_ARRPreloadConfig(TIM2, ENABLE);	
	/* 定时器配置:
	1.设置定时器最大计数值 50000
	2.设置时钟分频系数：TIM_CKD_DIV1
	3. 设置预分频：  1Mhz/50000= 1hz 
	4.定时器计数模式  向上计数模式
	*/		 
  	TIM_TimeBaseStructure.TIM_Period = 0xffff;     
  	TIM_TimeBaseStructure.TIM_Prescaler = 72;	 //1M 的时钟  
  	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;	
  	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
	//应用配置到TIM3 
  	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	// 使能TIM3重载寄存器ARR
  	TIM_ARRPreloadConfig(TIM3, ENABLE);	

	TIM_SelectSlaveMode(TIM3, TIM_SlaveMode_Reset);
	TIM_UpdateRequestConfig(TIM3, TIM_UpdateSource_Regular);
	/* ----------------------TIM3 Configuration as Master for the TIM2 -----------*/
  	/* Use the TIM3 Update event  as TIM3 Trigger Output(TRGO) */
  	TIM_SelectOutputTrigger(TIM3, TIM_TRGOSource_Update);
  	/* Enable the TIM3 Master Slave Mode */
  	TIM_SelectMasterSlaveMode(TIM3, TIM_MasterSlaveMode_Enable);

  	//启动定时器
	TIM_Cmd(TIM3, ENABLE); 
  	TIM_Cmd(TIM2, ENABLE);                  
}

// Fast inverse square-root
/**************************实现函数********************************************
*函数原型:	   float invSqrt(float x)
*功　　能:	   快速计算 1/Sqrt(x) 	
输入参数： 要计算的值
输出参数： 结果
*******************************************************************************/
/*float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}*/

/**************************实现函数********************************************
*函数原型:		uint32_t micros(void)
*功　　能:	  读取系统运行的时间 ，返回单位为us 的时间数。	
输入参数：无
输出参数：处理器当前时间，从上电开始计时  单位 us
*******************************************************************************/
uint32_t micros(void)
{
 	uint32_t temp=0 ;
 	temp = TIM2->CNT; //读高16位时间
 	temp = temp<<16;
 	temp += TIM3->CNT; //读低16位时间
 	return temp;
}

/**************************实现函数********************************************
*函数原型:	   void IMU_init(void)
*功　　能:	  初始化IMU相关	
			  初始化各个传感器
			  初始化四元数
			  将积分清零
			  更新系统时间
输入参数：无
输出参数：没有
*******************************************************************************/
void IMU_init(void)
{	 
	MPU6050_initialize();
	HMC5883L_SetUp();
	BMP180_init();
	delay_ms(50);
	MPU6050_initialize();
	HMC5883L_SetUp();
	BMP180_init();
	Initial_Timer3();
	// initialize quaternion
  	q0 = 1.0f;  //初始化四元数
  	q1 = 0.0f;
  	q2 = 0.0f;
  	q3 = 0.0f;
  	exInt = 0.0;
  	eyInt = 0.0;
  	ezInt = 0.0;
  	lastUpdate = micros();//更新时间
  	now = micros();
}

/**************************实现函数********************************************
*函数原型:	   void IMU_getValues(float * values)
*功　　能:	 读取加速度 陀螺仪 磁力计 的当前值  
输入参数： 将结果存放的数组首地址
输出参数：没有
*******************************************************************************/
void IMU_getValues(float * values) {  
	int16_t accgyroval[6];
	float gyrrange;
	float gyrfactor;
	int i;
	gyrrange = (GyrAndAccSettings>>2)&3;
	if (gyrrange == 0)
	   gyrfactor = 131.2;
	else if (gyrrange == 1)
	   gyrfactor = 65.6;
	else if (gyrrange == 2)
	   gyrfactor = 32.8;
	else 
	   gyrfactor = 16.4;
	//读取加速度和陀螺仪的当前ADC
    MPU6050_getMotion6(&accgyroval[0], &accgyroval[1], &accgyroval[2], &accgyroval[3], &accgyroval[4], &accgyroval[5], 1);
    for(i = 0; i<6; i++) {
      if(i < 3) {
        values[i] = (float) accgyroval[i];
      }
      else {

        values[i] = ((float) accgyroval[i]) / gyrfactor; //转成度每秒
		//这里已经将量程改成了 1000度每秒  32.8 对应 1度每秒
      }
    }
    HMC58X3_mgetValues(&values[6]);	//读取磁力计的ADC值
}


/**************************实现函数********************************************
*函数原型:	   void IMU_AHRSupdate
*功　　能:	 更新AHRS 更新四元数 
输入参数： 当前的测量值。
输出参数：没有
*******************************************************************************/
#define Kp 1.0f   // proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.53f   // integral gain governs rate of convergence of gyroscope biases
#define Km 2.0

#define twoKpDef	(2.0f * 0.5f)	// 2 * proportional gain
#define twoKiDef	(2.0f * 0.265f)	// 2 * integral gain
//calc new com value in xy plane
float comxyz[3];
float matrix[3][3]; 
void matrixmultipoint(float mt[3][3], float* pt)
{
//    float tempcom[3];
//    tempcom[0] = comxyz[0]*matrix[0][0] + comxyz[1]*matrix[1][0] + comxyz[2]*matrix[2][0];
//    tempcom[1] = comxyz[0]*matrix[0][1] + comxyz[1]*matrix[1][1] + comxyz[2]*matrix[2][1]; 
//    tempcom[2] = comxyz[0]*matrix[0][2] + comxyz[1]*matrix[1][2] + comxyz[2]*matrix[2][2];  
//	comxyz[0] = tempcom[0];
//	comxyz[1] = tempcom[1];
//	comxyz[2] = tempcom[2];
	
	  float temppt[3];
    temppt[0] = pt[0]*mt[0][0] + pt[1]*mt[1][0] + pt[2]*mt[2][0];
    temppt[1] = pt[0]*mt[0][1] + pt[1]*mt[1][1] + pt[2]*mt[2][1]; 
    temppt[2] = pt[0]*mt[0][2] + pt[1]*mt[1][2] + pt[2]*mt[2][2];  
	pt[0] = temppt[0];
	pt[1] = temppt[1];
	pt[2] = temppt[2];
}

float Gvx, Gvy, Gvz;

//int8_t firstflag = 1;
float offsetgx, offsetgy, offsetgz;
float halfT;
float ONET;

/*void IMU_AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
  float norm;
  float hx, hy, hz, bx, bz;
  float vx, vy, vz, wx, wy, wz;
  float ex, ey, ez;
  float tempq0,tempq1,tempq2,tempq3;


  // 先把这些用得到的值算好
  float q0q0 = q0*q0;
  float q0q1 = q0*q1;
  float q0q2 = q0*q2;
  float q0q3 = q0*q3;
  float q1q1 = q1*q1;
  float q1q2 = q1*q2;
  float q1q3 = q1*q3;
  float q2q2 = q2*q2;   
  float q2q3 = q2*q3;
  float q3q3 = q3*q3;         
  
  now = micros();  //读取时间
  if(now<lastUpdate){ //定时器溢出过了。
  halfT =  ((float)(now + (0xffff- lastUpdate)) / 2000000.0f);
  }
  else	{
  halfT =  ((float)(now - lastUpdate) / 2000000.0f);
  }
  lastUpdate = now;	//更新时间

  norm = invSqrt(ax*ax + ay*ay + az*az);       
  ax = ax * norm;
  ay = ay * norm;
  az = az * norm;
  //把加计的三维向量转成单位向量。

  norm = invSqrt(mx*mx + my*my + mz*mz);          
  mx = mx * norm;
  my = my * norm;
  mz = mz * norm;

  /*
  这是把四元数换算成《方向余弦矩阵》中的第三列的三个元素。
根据余弦矩阵和欧拉角的定义，地理坐标系的重力向量，转到机体坐标系，正好是这三个元素。
所以这里的vx\y\z，其实就是当前的欧拉角（即四元数）的机体坐标参照系上，换算出来的重力单位向量。
  */
  // compute reference direction of flux
 /* hx = 2*mx*(0.5f - q2q2 - q3q3) + 2*my*(q1q2 - q0q3) + 2*mz*(q1q3 + q0q2);
  hy = 2*mx*(q1q2 + q0q3) + 2*my*(0.5f - q1q1 - q3q3) + 2*mz*(q2q3 - q0q1);
  hz = 2*mx*(q1q3 - q0q2) + 2*my*(q2q3 + q0q1) + 2*mz*(0.5f - q1q1 - q2q2);         
  bx = sqrt((hx*hx) + (hy*hy));
  bz = hz;     
  
  // estimated direction of gravity and flux (v and w)
  vx = 2*(q1q3 - q0q2);
  vy = 2*(q0q1 + q2q3);
  vz = q0q0 - 0.5f + q3q3;
  wx = 2*bx*(0.5 - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2);
	
  wy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3);
  wz = 2*bx*(q0q2 + q1q3) + 2*bz*(0.5 - q1q1 - q2q2);  

  Gvx = vx;
  Gvy = vy;
  Gvz = vz;
  
  // error is sum of cross product between reference direction of fields and direction measured by sensors
  ex = (ay*vz - az*vy) + (my*wz - mz*wy);
  ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
  ez = (ax*vy - ay*vx) + (mx*wy - my*wx);

  /*
  axyz是机体坐标参照系上，加速度计测出来的重力向量，也就是实际测出来的重力向量。
axyz是测量得到的重力向量，vxyz是陀螺积分后的姿态来推算出的重力向量，它们都是机体坐标参照系上的重力向量。
那它们之间的误差向量，就是陀螺积分后的姿态和加计测出来的姿态之间的误差。
向量间的误差，可以用向量叉积（也叫向量外积、叉乘）来表示，exyz就是两个重力向量的叉积。
这个叉积向量仍旧是位于机体坐标系上的，而陀螺积分误差也是在机体坐标系，而且叉积的大小与陀螺积分误差成正比，正好拿来纠正陀螺。（你可以自己拿东西想象一下）由于陀螺是对机体直接积分，所以对陀螺的纠正量会直接体现在对机体坐标系的纠正。
  */
/*if(ex != 0.0f && ey != 0.0f && ez != 0.0f){
  exInt = exInt + ex * Ki * halfT;
  eyInt = eyInt + ey * Ki * halfT;	
  ezInt = ezInt + ez * Ki * halfT;

  // 用叉积误差来做PI修正陀螺零偏
  gx = gx + Kp*ex + exInt;
  gy = gy + Kp*ey + eyInt;
  gz = gz + Kp*ez + ezInt;

  }

  // 四元数微分方程
  tempq0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
  tempq1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
  tempq2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
  tempq3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  
  
  // 四元数规范化
  norm = invSqrt(tempq0*tempq0 + tempq1*tempq1 + tempq2*tempq2 + tempq3*tempq3);
  q0 = tempq0 * norm;
  q1 = tempq1 * norm;
  q2 = tempq2 * norm;
  q3 = tempq3 * norm;
}*/

/*volatile float twoKp = twoKpDef;											// 2 * proportional gain (Kp)
volatile float twoKi = twoKiDef;											// 2 * integral gain (Ki)
volatile float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;	// integral error terms scaled by Ki

void IMU_AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
	float recipNorm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;  
	float hx, hy, bx, bz;
	float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
	float halfex, halfey, halfez;
	float qa, qb, qc;
	float ONET;
	
	now = micros();  //读取时间
  if(now<lastUpdate){ //定时器溢出过了。
  ONET =  ((float)(now + (0xffff- lastUpdate)) / 1000000.0f);
  }
  else	{
  ONET =  ((float)(now - lastUpdate) / 1000000.0f);
  }
  lastUpdate = now;	//更新时间
	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;     

		// Normalise magnetometer measurement
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;   

        // Auxiliary variables to avoid repeated arithmetic
        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3;   

        // Reference direction of Earth's magnetic field
        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        bx = sqrt(hx * hx + hy * hy);
        bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

		// Estimated direction of gravity and magnetic field
		halfvx = q1q3 - q0q2;
		halfvy = q0q1 + q2q3;
		halfvz = q0q0 - q1q1 - q2q2 + q3q3;
        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);  
	
		// Error is sum of cross product between estimated direction and measured direction of field vectors
		halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
		halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
		halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);    

  Gvx = 2*halfvx;
  Gvy = 2*halfvy;
  Gvz = 2*halfvz;
		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			integralFBx += twoKi * halfex * ONET;	// integral error scaled by Ki
			integralFBy += twoKi * halfey * ONET;
			integralFBz += twoKi * halfez * ONET;
			gx += integralFBx;	// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		}
		else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKp * 2 * halfex;
		gy += twoKp * 2 * halfey;
		gz += twoKp * 2 * halfez;
	}
	
	// Integrate rate of change of quaternion
	gx *= (0.5f * ONET);		// pre-multiply common factors
	gy *= (0.5f * ONET);
	gz *= (0.5f * ONET);
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx); 
	
	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}*/

float mygetqval[9];	//用于存放传感器转换结果的数组
uint8_t firstcalquat = 0;
void initquat()
{
			float quat[4], pangle, rangle;
			float yprangle[3], temp;
		
			Gvx = mygetqval[0] / Accsensitivity;
			Gvy = mygetqval[1] / Accsensitivity;
			Gvz = mygetqval[2] / Accsensitivity;
	    if (Gvy>=0.9999)
				Gvy = 1.0;
	    if (Gvy<=-0.9999)
				Gvy = -1.0;
			pangle = asin(Gvy);
			rangle = -atan2(Gvx, Gvz);
			comxyz[0] = mygetqval[6];
			comxyz[1] = mygetqval[7];
			comxyz[2] = mygetqval[8];
				matrix[0][0] = cos(rangle), matrix[0][1] = 0, matrix[0][2] = -sin(rangle);
				matrix[1][0] = 0,           matrix[1][1] = 1, matrix[1][2] = 0;
			matrix[2][0] = sin(rangle), matrix[2][1] = 0, matrix[2][2] = cos(rangle);
			matrixmultipoint(matrix, comxyz);
			matrix[0][0] = 1, matrix[0][1] = 0,            matrix[0][2] = 0;
			matrix[1][0] = 0, matrix[1][1] = cos(pangle),  matrix[1][2] = sin(pangle);
			matrix[2][0] = 0, matrix[2][1] = -sin(pangle), matrix[2][2] = cos(pangle);
			matrixmultipoint(matrix, comxyz);
			yprangle[0] = -atan2(comxyz[0], comxyz[1])* 180/M_PI - 90; // yaw
			if (yprangle[0]<-180)
					yprangle[0] = yprangle[0] + 360;
			yprangle[1] = asin(Gvy)* 180/M_PI; // pitch 
			yprangle[2] = -atan2(Gvx, Gvz)* 180/M_PI; // roll*/
			EulorToQuat(quat, yprangle[2] * M_PI/180.0, yprangle[1] * M_PI/180.0, -(yprangle[0] + 180) * M_PI/180.0);
			q0 = -quat[3];
			q1 = quat[1];
			q2 = -quat[2];
			q3 = -quat[0];
			temp = invSqrt(q0*q0+q1*q1+q2*q2+q3*q3);			
			q0*= temp;
			q1*= temp;
			q2*= temp;
			q3*= temp;
}

volatile float beta = 0.1f;//default 0.1f;
//volatile float rateofgyr = 0.8f;
void IMU_AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {

	if (firstcalquat <1)
	{
		  initquat(); 
		  firstcalquat++;
	}
	else
	{
			now = micros();  //读取时间
			if(now<lastUpdate){ //定时器溢出过了。
			ONET =  ((float)(now + (0xffff- lastUpdate)) / 1000000.0f);
			}
			else	{
			ONET =  ((float)(now - lastUpdate) / 1000000.0f);
			}
			lastUpdate = now;	//更新时间
			calcquatwithgam(beta, ONET, gx, gy, gz, ax, ay, az, mx, my, mz);
			Gvx = 2*(q1*q3 - q0*q2);
			Gvy = 2*(q0*q1 + q2*q3);
			Gvz = q0*q0 - q1*q1 - q2*q2 + q3*q3;
	}
	
}

/**************************实现函数********************************************
*函数原型:	   void IMU_getQ(float * q)
*功　　能:	 更新四元数 返回当前的四元数组值
输入参数： 将要存放四元数的数组首地址
输出参数：没有
*******************************************************************************/

void IMU_getQ(float * q) {

  IMU_getValues(mygetqval);	 
  //将陀螺仪的测量值转成弧度每秒
  //加速度和磁力计保持 ADC值　不需要转换
 //IMU_AHRSupdate(-mygetqval[5] * M_PI/180, -mygetqval[4] * M_PI/180, -mygetqval[3] * M_PI/180,
 //  mygetqval[2], mygetqval[1], mygetqval[0], mygetqval[8], mygetqval[7], mygetqval[6]);
    IMU_AHRSupdate(mygetqval[3] * M_PI/180, mygetqval[4] * M_PI/180, mygetqval[5] * M_PI/180,
   mygetqval[0], mygetqval[1], mygetqval[2], mygetqval[6], mygetqval[7], mygetqval[8]);

//	 AHRS_AHRSupdate_karman(mygetqval[3] * M_PI/180, mygetqval[4] * M_PI/180, mygetqval[5] * M_PI/180,
//   mygetqval[0], mygetqval[1], mygetqval[2], mygetqval[6], mygetqval[7], mygetqval[8]);
  q[0] = q0; //返回当前值
  q[1] = q1;
  q[2] = q2;
  q[3] = q3;
}

void IMU_getgravity(/*float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, */float* grav)
{
//	float gyrrange;
//	float gyrfactor;
//	gyrrange = (GyrAndAccSettings>>2)&3;
//	if (gyrrange == 0)
//	   gyrfactor = 131.2;
//	else if (gyrrange == 1)
//	   gyrfactor = 65.6;
//	else if (gyrrange == 2)
//	   gyrfactor = 32.8;
//	else 
//	   gyrfactor = 16.4;
//   	IMU_AHRSupdate(gx / gyrfactor * M_PI/180, gy / gyrfactor * M_PI/180, gz / gyrfactor * M_PI/180, ax, ay, az, mx, my, mz);
	grav[0] = Gvx;
	grav[1] = Gvy;
	grav[2] = Gvz;
}

float preyaw, prepitch, preroll;
uint8_t firstflag = 1, zeroacccount=0;
float speed[3] = {0, 0, 0}, facclen;
float preacc[3] = {0, 0, 0};
uint16_t weightlessnesstime=0;
float gateacc;

extern volatile uint8_t weightlessness;
extern int16_t Upload_Speed;

void IMU_getQuat(float* pquat, uint16_t* acclen, float* position)
{
	float tempT, tempT1;

	float acc[3], originalacc[3], temacc[3];//, facclen;
	float quat[4];
	//float absgx, absgy, absgz;
	//float weightofyaw, tempyaw;
	
    IMU_getQ(pquat); //更新全局四元数


	originalacc[0] = mygetqval[0] / Accsensitivity - Gvx;
  originalacc[1] = mygetqval[1] / Accsensitivity - Gvy;
  originalacc[2] = mygetqval[2] / Accsensitivity - Gvz;
	facclen = 1 / invSqrt(originalacc[0] * originalacc[0] + originalacc[1] * originalacc[1] + originalacc[2] * originalacc[2]);
	*acclen = (uint16_t)(facclen * 1000);
	
	if (position != 0)
	{
	      quat[0] = pquat[1], quat[1] = pquat[2], quat[2] = pquat[3], quat[3] = pquat[0];
		    QuatMultiVec3d(quat, originalacc, acc);
				if (weightlessness)
				{
					if (zeroacccount>=5)
						weightlessness = 0;
					if (acc[2] >= 0)
							weightlessness = 0;
				}
				else
				{
				    if (acc[2]<-0.9)
						{
					    weightlessness = 1;
						}
			  }				  
				if ((myabs(acc[0]) < 0.03)&&(myabs(acc[1]) < 0.07)&&(myabs(acc[2]) < 0.03))
				{
					  acc[0] = 0;
					  acc[1] = 0;
					  acc[2] = 0;
					  zeroacccount++;
					  speed[0] *= 0.9;
					  speed[1] *= 0.9;
					  speed[2] *= 0.9;
				}
				else
					 zeroacccount = 0;
				//position[0] = acc[0];
				//position[1] = acc[1];
				//position[2] = acc[2];
				//position[0] = speed[0];
				//position[1] = speed[1];
				//position[2] = speed[2];
			temacc[0] = acc[0];
			temacc[1] = acc[1];
			temacc[2] = acc[2];
			acc[0] = (acc[0] + preacc[0]) / 2;
			acc[1] = (acc[1] + preacc[1]) / 2;
			acc[2] = (acc[2] + preacc[2]) / 2;
			//ONET = 1.0/120.0;
			tempT = 9.80665*ONET*ONET/2.0;
			//oneT = halfT * 2.0;
			position[0] = (speed[0] * ONET + acc[0]* tempT);
			position[1] = (speed[1] * ONET + acc[1]* tempT);
			position[2] = (speed[2] * ONET + acc[2]* tempT);

			tempT1 = 9.80665 * ONET;
			speed[0] = speed[0] + acc[0] * tempT1;
			speed[1] = speed[1] + acc[1] * tempT1;
			speed[2] = speed[2] + acc[2] * tempT1;
				
			//position[0] = acc[0];
      //position[1] = acc[1];
		  //position[2] = acc[2];
			
			//position[0] = speed[0];
      //position[1] = speed[1];
		  //position[2] = speed[2];
		//}
		preacc[0] = temacc[0];
		preacc[1] = temacc[1];
		preacc[2] = temacc[2];
	}
  
}

/**************************实现函数********************************************
*函数原型:	   void IMU_getYawPitchRoll(float * angles)
*功　　能:	 更新四元数 返回当前解算后的姿态数据
输入参数： 将要存放姿态角的数组首地址
输出参数：没有
*******************************************************************************/
void IMU_getYawPitchRollAndQuat(float * angles, float * quat) {
  float q[4]; //　四元数
  volatile float gx=0.0, gy=0.0, gz=0.0; //估计重力方向
  IMU_getQ(q); //更新全局四元数
  
  angles[0] = -atan2(2 * q[1] * q[2] + 2 * q[0] * q[3], -2 * q[2]*q[2] - 2 * q[3] * q[3] + 1)* 180/M_PI; // yaw
  angles[1] = -asin(-2 * q[1] * q[3] + 2 * q[0] * q[2])* 180/M_PI; // pitch
  angles[2] = atan2(2 * q[2] * q[3] + 2 * q[0] * q[1], -2 * q[1] * q[1] - 2 * q[2] * q[2] + 1)* 180/M_PI; // roll

  quat[0] = q[0];
  quat[1] = q[1];
  quat[2] = q[2];
  quat[3] = q[3];
  //if(angles[0]<0)angles[0]+=360.0f;  //将 -+180度  转成0-360度
}


void IMU_getYPRAngle(float* yprangle, uint16_t* acclen, float* position)
{
	float tempT, tempT1;
    float q[4]; //　四元数
    volatile float gx=0.0, gy=0.0, gz=0.0; //估计重力方向

	float acc[3], originalacc[3], temacc[3];//, facclen;
	float quat[4], pangle, rangle, tem;
	//float absgx, absgy, absgz;
	//float weightofyaw, tempyaw;
	
    IMU_getQ(q); //更新全局四元数


	originalacc[0] = mygetqval[0] / Accsensitivity - Gvx;
  originalacc[1] = mygetqval[1] / Accsensitivity - Gvy;
  originalacc[2] = mygetqval[2] / Accsensitivity - Gvz;
	facclen = 1 / invSqrt(originalacc[0] * originalacc[0] + originalacc[1] * originalacc[1] + originalacc[2] * originalacc[2]);
	*acclen = (uint16_t)(facclen * 1000);
	
	//test
	//yprangle[0] = -atan2(2 * q[1] * q[2] + 2 * q[0] * q[3], -2 * q[2]*q[2] - 2 * q[3] * q[3] + 1)* 180/M_PI; // yaw
  //yprangle[1] = -asin(-2 * q[1] * q[3] + 2 * q[0] * q[2])* 180/M_PI; // pitch
  //yprangle[2] = atan2(2 * q[2] * q[3] + 2 * q[0] * q[1], -2 * q[1] * q[1] - 2 * q[2] * q[2] + 1)* 180/M_PI; // roll
	//yprangle[0] = atan2(2 * q[1] * q[2] + 2 * q[0] * q[3], -2 * q[2]*q[2] - 2 * q[3] * q[3] + 1)* 180/M_PI; // yaw
	//yprangle[1] = asin(Gvy)* 180/M_PI; // pitch
	//yprangle[2] = -atan2(Gvx, Gvz)* 180/M_PI;//atan2(mygetqval[6], mygetqval[7])* 180/M_PI;
  //yprangle[2] = -atan2(Gvx, Gvz)* 180/M_PI; // roll
  //yprangle[2] = -asin(-2 * q[1] * q[3] + 2 * q[0] * q[2])* 180/M_PI; // pitch
  //yprangle[1] = atan2(2 * q[2] * q[3] + 2 * q[0] * q[1], -2 * q[1] * q[1] - 2 * q[2] * q[2] + 1)* 180/M_PI; // roll
	/*absgx = myabs(mygetqval[3]);
	absgy = myabs(mygetqval[4]);
	absgz = myabs(mygetqval[5]);
	if ((absgx >= 10.0) || (absgy >= 10.0) || (absgz >= 10.0))
		weightofyaw = 1;
	else if ((absgx >= 9.0) || (absgy >= 9.0) || (absgz >= 9.0))
		weightofyaw = 0.7;
	else if ((absgx >= 8.0) || (absgy >= 8.0) || (absgz >= 8.0))
		weightofyaw = 0.5;
	else if ((absgx >= 7.0) || (absgy >= 7.0) || (absgz >= 7.0))
		weightofyaw = 0.4;
	else if ((absgx >= 6.0) || (absgy >= 6.0) || (absgz >= 6.0))
		weightofyaw = 0.3;
	else if ((absgx >= 5.0) || (absgy >= 5.0) || (absgz >= 5.0))
		weightofyaw = 0.2;	
	else if ((absgx >= 4.0) || (absgy >= 4.0) || (absgz >= 4.0))
		weightofyaw = 0.1;
	else if ((absgx >= 3.0) || (absgy >= 3.0) || (absgz >= 3.0))
		weightofyaw = 0.0;
	else if ((absgx >= 2.0) || (absgy >= 2.0) || (absgz >= 2.0))
		weightofyaw = 0.0;
	else if ((absgx >= 1.0) || (absgy >= 1.0) || (absgz >= 1.0))
		weightofyaw = 0.0;	
  else 
    weightofyaw = 0;		
	if ((weightofyaw > 0) || (firstflag == 1))
	{*/
		/*pangle = asin(Gvy);
	    rangle = -atan2(Gvx, Gvz);
	    comxyz[0] = mygetqval[6];
	    comxyz[1] = mygetqval[7];
	    comxyz[2] = mygetqval[8];
        matrix[0][0] = cos(rangle), matrix[0][1] = 0, matrix[0][2] = -sin(rangle);
  	    matrix[1][0] = 0,           matrix[1][1] = 1, matrix[1][2] = 0;
 	    matrix[2][0] = sin(rangle), matrix[2][1] = 0, matrix[2][2] = cos(rangle);
 	    matrixmultipoint(matrix, comxyz);
 	    matrix[0][0] = 1, matrix[0][1] = 0,            matrix[0][2] = 0;
 	    matrix[1][0] = 0, matrix[1][1] = cos(pangle),  matrix[1][2] = sin(pangle);
 	    matrix[2][0] = 0, matrix[2][1] = -sin(pangle), matrix[2][2] = cos(pangle);
 	    matrixmultipoint(matrix, comxyz);
	    yprangle[0] = -atan2(comxyz[0], comxyz[1])* 180/M_PI - 90; // yaw
			if (yprangle[0]<-180)
				  yprangle[0] = yprangle[0] + 360;
			yprangle[1] = asin(Gvy)* 180/M_PI; // pitch  
      yprangle[2] = -atan2(Gvx, Gvz)* 180/M_PI; // roll
			
			EulorToQuat(quat, yprangle[2] * M_PI/180.0, yprangle[1] * M_PI/180.0, -(yprangle[0] + 180) * M_PI/180.0);
			tem = quat[0];
			quat[0] = -quat[3];
			quat[3] = -tem;
			quat[2] = -quat[2];*/
			//yprangle[0] = atan2(2 * quat[1] * quat[2] + 2 * quat[0] * quat[3], -2 * quat[2]*quat[2] - 2 * quat[3] * quat[3] + 1)* 180/M_PI; // yaw
      //yprangle[1] = -asin(-2 * quat[1] * quat[3] + 2 * quat[0] * quat[2])* 180/M_PI; // pitch
      //yprangle[2] = atan2(2 * quat[2] * quat[3] + 2 * quat[0] * quat[1], -2 * quat[1] * quat[1] - 2 * quat[2] * quat[2] + 1)* 180/M_PI; // roll
			
			yprangle[0] = atan2(2 * q[1] * q[2] + 2 * q[0] * q[3], -2 * q[2]*q[2] - 2 * q[3] * q[3] + 1)* 180/M_PI; // yaw
      yprangle[1] = -asin(-2 * q[1] * q[3] + 2 * q[0] * q[2])* 180/M_PI; // pitch
      yprangle[2] = atan2(2 * q[2] * q[3] + 2 * q[0] * q[1], -2 * q[1] * q[1] - 2 * q[2] * q[2] + 1)* 180/M_PI; // roll
	/*	preyaw = yprangle[0];
		//yprangle[0] = atan2(2 * q[1] * q[2] + 2 * q[0] * q[3], -2 * q[2]*q[2] - 2 * q[3] * q[3] + 1)* 180/M_PI; // yaw
    //yprangle[1] = -asin(-2 * q[1] * q[3] + 2 * q[0] * q[2])* 180/M_PI; // pitch
    //yprangle[2] = atan2(comxyz[0], comxyz[1])* 180/M_PI; // roll
			
		//	yprangle[0] = atan2(2 * q[1] * q[2] + 2 * q[0] * q[3], -2 * q[2]*q[2] - 2 * q[3] * q[3] + 1)* 180/M_PI; // yaw
    //  yprangle[1] = -asin(-2 * q[1] * q[3] + 2 * q[0] * q[2])* 180/M_PI; // pitch
    //  yprangle[2] = atan2(2 * q[2] * q[3] + 2 * q[0] * q[1], -2 * q[1] * q[1] - 2 * q[2] * q[2] + 1)* 180/M_PI; // roll
	/*	preyaw = yprangle[0];
		prepitch = yprangle[1];
		preroll = yprangle[2];
		firstflag = 0;
	}
	else
	{
	    moveless = 1;
		yprangle[0] = preyaw;
		yprangle[1] = prepitch;
		yprangle[2] = preroll;
	}*/
	//计算位移
	if (position != 0)
	{
	  //if (moveless == 1)
		//{
    //position[0] = 0;
		//	position[1] = 0;
		//	position[2] = 0;
		//	speed[0] = 0;
		//	speed[1] = 0;
		//	speed[2] = 0;
		//	acc[0] = 0;
		//	acc[1] = 0;
		//	acc[2] = 0;
		//}
		//else
		//{
		    //originalacc[0] = mygetqval[0] / Accsensitivity;
		    //originalacc[1] = mygetqval[1] / Accsensitivity;
		    //originalacc[2] = mygetqval[2] / Accsensitivity;
		   // if (originalacc[2] < 0)
			 //	originalacc[2] *=0.95;
				//else
				//	originalacc[2] *= 1.04;
		    //EulorToQuat(quat, yprangle[0] * M_PI/180.0, yprangle[1] * M_PI/180.0, yprangle[2] * M_PI/180.0);
		    EulorToQuat(quat, yprangle[0] * M_PI/180.0, yprangle[1] * M_PI/180.0, yprangle[2] * M_PI/180.0);
		    //quat[0] = q0;
		    //quat[1] = q1;
		    //quat[2] = q2;
				//quat[3] = q3;
		    QuatMultiVec3d(quat, originalacc, acc);
		    //acc[2] = acc[2] - 1.0;
				if (weightlessness)
				{
					if (zeroacccount>=5)
						weightlessness = 0;
					//weightlessnesstime--;
					if (acc[2] >= 0)
							weightlessness = 0;
				}
				else
				{
				    if (acc[2]<-0.9)
						{
					    weightlessness = 1;
							//if (speed[2]>0)
							//	  gateacc = 2;
							//else
							//	  gateacc = 0;
							//weightlessnesstime = speed[2]*2*Upload_Speed;
						}
			  }				  
				if ((myabs(acc[0]) < 0.03)&&(myabs(acc[1]) < 0.07)&&(myabs(acc[2]) < 0.03))
				{
					  acc[0] = 0;
					  acc[1] = 0;
					  acc[2] = 0;
					  zeroacccount++;
					  speed[0] *= 0.9;
					  speed[1] *= 0.9;
					  speed[2] *= 0.9;
				}
				else
					 zeroacccount = 0;
				//position[0] = acc[0];
				//position[1] = acc[1];
				//position[2] = acc[2];
				//position[0] = speed[0];
				//position[1] = speed[1];
				//position[2] = speed[2];
			temacc[0] = acc[0];
			temacc[1] = acc[1];
			temacc[2] = acc[2];
			acc[0] = (acc[0] + preacc[0]) / 2;
			acc[1] = (acc[1] + preacc[1]) / 2;
			acc[2] = (acc[2] + preacc[2]) / 2;
			//ONET = 1.0/120.0;
			tempT = 9.80665*ONET*ONET/2.0;
			//oneT = halfT * 2.0;
			position[0] = speed[0] * ONET + acc[0]* tempT;
			position[1] = speed[1] * ONET + acc[1]* tempT;
			position[2] = speed[2] * ONET + acc[2]* tempT;

			tempT1 = 9.80665 * ONET;
			speed[0] = speed[0] + acc[0] * tempT1;
			speed[1] = speed[1] + acc[1] * tempT1;
			speed[2] = speed[2] + acc[2] * tempT1;
				
			//position[0] = acc[0];
      //position[1] = acc[1];
		  //position[2] = acc[2];
		//}
		preacc[0] = temacc[0];
		preacc[1] = temacc[1];
		preacc[2] = temacc[2];
	}
    
}

//------------------End of File----------------------------
