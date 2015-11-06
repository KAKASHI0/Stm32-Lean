/* main.c file
编写者：lisn3188
网址：www.chiplab7.com
作者E-mail：lisn3188@163.com
编译环境：MDK-Lite  Version: 4.23
初版时间: 2012-05-05
测试： 本程序已在第七实验室的mini IMU上完成测试
Mini IMU AHRS 模块官方销售地址：Http://chiplab7.taobao.com
功能：
1.初始化各个传感器，
2.运行姿态解算和高度测量
3.将解算的姿态和各个传感器的输出上传到 MiniIMU AHRS 测试软件
4.响应 PC发送的命令
------------------------------------
*/

#include "common.h"  //包含所有的驱动 头文件
#include "comm.h"
//上传数据的状态机
#define REIMU    0x01     //上传解算的姿态数据
#define REAngle  0x02	  //上传自定义的角度
#define REAGCG   0x03	  //上传加速度计，陀螺仪，磁力计，解算后的重力加速度
#define REQuat   0x04     //上传四元数
#define REChipID 0x05     //上传芯片ID
#define REAngleAndPos 0x06 // 上传自定义的角度和位移
#define REQuatAndPos 0x07  //上传四元素和位移


#define upload_time (1000000/Upload_Speed)  //计算上传的时间。单位为us

int16_t Upload_Speed = 120;   //数据上传速度  单位 Hz
int16_t ChipID = 9;
int16_t ax, ay, az;	
int16_t gx, gy, gz;
int16_t hx, hy, hz;
int32_t Temperature = 0, Pressure = 0, Altitude = 0; 
uint32_t system_micrsecond;
int16_t hmcvalue[3];
u8 state= REQuat;  //发送特定帧 的状态机
uint8_t GyrAndAccSettings = 0;
float Accsensitivity;//加速度的灵敏度
/**************************实现函数********************************************
*函数原型:		int main(void)
*功　　能:		主程序
*******************************************************************************/
int main(void)
{
	uint32_t delta_t;//改进的采样率计算方法,Math_HZ/delta_t = HZ
  char reportdata[500];
	uint16_t acclen;
	uint16_t Math_hz=0;
	int16_t offax, offay, offaz, offhx, offhy, offhz;
	unsigned char PC_comm; //PC 命令关键字节	 
	float ypr[3]; // yaw pitch roll
	float quat[4] = {1.0, 1.0, 1.0, 1.0};
	float grav[3] = {1.0, 2.0, 3.0}; 
  float pos[3] = {0,0,0};
	/* 配置系统时钟为72M 使用外部8M晶体+PLL*/      
    SystemInit();
	delay_init(72);		//延时初始化
    Initial_LED_GPIO();	//初始化STM32-SDK板子上的LED接口
	Initial_PWMLED();
	Initial_UART1(512000L);//这里可以加快通讯波特率，可以一定程度（0.2ms）减少数据延时（最差情况延时2.2ms）
												//，但是会增加误码率。建议不大于720000
	Initial_UART2(115200L);

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_CRC, ENABLE);

	IIC_Init();	 //初始化I2C接口
	delay_ms(300);	//等待器件上电
	//UART1_Put_String("Initialize...\r\n");
	IMU_init(); //初始化IMU和传感器
	reportdata[0] = 1;

	if ((GyrAndAccSettings&3) == 0)
	    Accsensitivity = 16384.0;
	else if ((GyrAndAccSettings&3) == 1)
	    Accsensitivity = 8192.0;
	else if ((GyrAndAccSettings&3) == 2)
	    Accsensitivity = 4096.0;
	else
	    Accsensitivity = 2048.0;

	if (Upload_Speed == 60)
	    GyrAndAccSettings = GyrAndAccSettings | (1<<4);
	else if (Upload_Speed == 140)
	    GyrAndAccSettings = GyrAndAccSettings |	(3<<4);
	
	ChipID = (GyrAndAccSettings<<8) | ChipID;//不明白？？？
	
	system_micrsecond=micros();
	
	while(1)
	{	//主循环
		
		//delay_ms(1); //延时，不要算那么快。
		//IMU_getYawPitchRoll(ypr, quat); //姿态更新
		if (state == REAngle)
			IMU_getYPRAngle(ypr, &acclen, 0);
		else if (state == REIMU)
			IMU_getYawPitchRollAndQuat(ypr, quat); //姿态更新
		else if (state == REAngleAndPos)
	   IMU_getYPRAngle(ypr, &acclen, pos);
		else if (state == REQuat)
	   IMU_getQuat(quat, &acclen, 0);
		else if (state == REQuatAndPos)
		 IMU_getQuat(quat, &acclen, pos);
		else
	   IMU_getYawPitchRollAndQuat(ypr, quat);
	
		Math_hz++; //解算次数 ++
		//BMP180_Routing(); //处理BMP018 事务 开启转换和读取结果将在这个子程序中进行 

	//-------------上位机------------------------------
		//是否到了更新 上位机的时间了？
		delta_t = (micros()-system_micrsecond);
		if(delta_t>=upload_time)
		{
				Math_hz=(uint16_t) ((float)Math_hz/(delta_t/1000000.0));
			switch(state){ 
				case REIMU:
				{
					//BMP180_getTemperat(&Temperature); //读取最近的温度值
					//BMP180_getPress(&Pressure);	   //读取最近的气压测量值
					//BMP180_getAlt(&Altitude);	   //读取相对高度
					//UART1_ReportYawPitchRoll((int16_t)(ypr[0]*10.0),(int16_t)(ypr[1]*10.0),(int16_t)(ypr[2]*10.0), ChipID);
					//UART2_ReportYawPitchRoll((int16_t)(ypr[0]*10.0),(int16_t)(ypr[1]*10.0),(int16_t)(ypr[2]*10.0), ChipID);
					MPU6050_getlastMotion6(&ax, &ay, &az, &gx, &gy, &gz);
					HMC58X3_getlastValues(&hx,&hy,&hz);
					accgyrhmctoIMU_Reg(ax, ay, az, gx, gy, gz, hx, hy, hz, ChipID);
	//			UART1_ReportIMU((int16_t)(ypr[0]*10.0),(int16_t)(ypr[1]*10.0),
	//			(int16_t)(ypr[2]*10.0),Altitude/10,Temperature,Pressure/10,Math_hz);
				
					//Update_OutputIMU_Reg((int16_t)(ypr[0]*10.0),(int16_t)(ypr[1]*10.0),(int16_t)(ypr[2]*10.0),Math_hz);
					/*IMU_getgravity(grav);
					MotionAndGravToChar(ax, ay, az, gx, gy, gz, hx, hy, hz, grav, ChipID, reportdata);
					if (reportdata[0] > 1)
					{
						UART2_ReportData(reportdata);
					}
					reportdata[0] = 1; */

					//UART2_ReportIMU((int16_t)(ypr[0]*10.0),(int16_t)(ypr[1]*10.0),
					//(int16_t)(ypr[2]*10.0),Altitude/10,Temperature,Pressure/10,Math_hz);
	//				UART1_ReportMotion(ax,ay,az,gx,gy,gz,hx,hy,hz);
						//UART2_ReportMotion(ax,ay,az,gx,gy,gz,hx,hy,hz);
					//UART1_ReportQuat(quat[0], quat[1], quat[2], quat[3], ChipID); 
					//UART2_ReportQuat(quat[0], quat[1], quat[2], quat[3], ChipID); 	
					//UART1_ReportQuat(Gvx, Gvy, Gvz, 0.0, ChipID); 
					//UART2_ReportQuat(Gvx, Gvy, Gvz, 0.0, ChipID);   
					//state = REMOV; //更改状态。
					break;
				}
				case REAngle:
				{
						//UART1_ReportYawPitchRoll((int16_t)(ypr[0]*10.0), (int16_t)(ypr[1]*10.0), (int16_t)(ypr[2]*10.0)); 
						//UART2_ReportYawPitchRoll((int16_t)(ypr[0]*10.0), (int16_t)(ypr[1]*10.0), (int16_t)(ypr[2]*10.0)); 
					//YawPitchRollToChar((int16_t)(ypr[0]*10.0), (int16_t)(ypr[1]*10.0), (int16_t)(ypr[2]*10.0), ChipID, reportdata);
					EulorToIMU_Reg((int16_t)(ypr[0]*10.0), (int16_t)(ypr[1]*10.0), (int16_t)(ypr[2]*10.0), acclen, ChipID, 0);
					if (reportdata[0] > 1)
					{
							//UART1_ReportData(reportdata);
							//UART2_ReportData(reportdata);
					}
					reportdata[0] = 1;
						break;
				}
				case REAngleAndPos:
				{
					EulorToIMU_Reg((int16_t)(ypr[0]*10.0), (int16_t)(ypr[1]*10.0), (int16_t)(ypr[2]*10.0), acclen, ChipID, pos);  
				}
				case REAGCG:
				{
					MPU6050_getlastMotion6(&ax, &ay, &az, &gx, &gy, &gz);
					HMC58X3_getlastValues(&hx,&hy,&hz);
					IMU_getgravity(/*gx, gy, gz, ax, ay, az, hx, hy, hz, */grav);
					//UART1_ReportMotionAndGrav(ax,ay,az,gx,gy,gz,hx,hy,hz, grav);
					//UART2_ReportMotionAndGrav(ax,ay,az,gx,gy,gz,hx,hy,hz, grav);
					MotionAndGravToChar(ax, ay, az, gx, gy, gz, hx, hy, hz, grav, ChipID, reportdata);
					if (reportdata[0] > 1)
					{
						//UART1_ReportData(reportdata);
						UART2_ReportData(reportdata);
					}
					reportdata[0] = 1; 	
					break;
				}
				case REQuat:
				{
					QuatToIMU_Reg(quat, acclen, 0);
						//UART1_ReportQuat(quat[0], quat[1], quat[2], quat[3]); 
					//UART2_ReportQuat(quat[0], quat[1], quat[2], quat[3]); 
					//QuatToChar(quat, ChipID, reportdata);
					//UART1_ReportData(reportdata);
					//UART2_ReportData(reportdata);
					//reportdata[0] = 1;
						break;
				}
				case REQuatAndPos:
				{
					QuatToIMU_Reg(quat, acclen, pos);
					break;
				}
				case REChipID:
				{
					//  UART1_ReportChipID(ChipID);
					UART2_ReportChipID(ChipID);
					break;	    
				}
				default: 
				{
						state = REAGCG;
						break;
				}
			
			}//switch(state) 
			Math_hz=0;			 
			system_micrsecond=micros();	 //取系统时间 单位 us 
			LED_Change();	//LED1改变亮度
		}
	//--------------------------------------------------
	
		//处理PC发送来的命令
		if((PC_comm=UART2_CommandRoute(reportdata))!=0xff)
		{
			switch(PC_comm){ //检查命令标识
				case Gyro_init:			
				{
							MPU6050_InitGyro_Offset();
						break; //读取陀螺仪零偏
				}
				case Hmc_SetOffset:
				{
			    UART2_GetHmcOffset(&offhx, &offhy, &offhz);
					HMC5883L_SetHmcoffset(offhx, offhy, offhz);
					break;
				}
				case Acc_SetEextremeValue:         
				{
						MPU6050_SetAccEextremevalue(); 
					break;
				}
				case Acc_SetOffset:
				{
						UART2_GetAccOffset(&offax, &offay, &offaz);
						MPU6050_SetAccOffset(offax, offay, offaz);
					break;
				}
				case SetUploadSpeed:
				{
					Upload_Speed = UART2_CommandRoutedata(2);
					break;
				}
				case GetChipID:
				{
					//  UART1_ReportChipID(ChipID);
						UART2_ReportChipID(ChipID);
						break;
				}
				case SetReportType:
				{	state = UART2_CommandRoutedata(2); 
						break;
				}
				case High_init:			
				{
						//BMP180_ResetAlt(0); 
					break;		//气压高度 清零
				}
				case 0xA7:
				{
						reportdata[0] -= 2;
					//UART1_ReportData(reportdata);
						UART2_ReportData(reportdata);
					reportdata[0] = 1;
						break;
				}
				case 0xA4:
				{
						reportdata[0] -= 2;
					//UART1_ReportData(reportdata);
						break;
				}
				case 0xA3:
				{
						reportdata[0] -= 2;
						break;				  
				}
				default:
				{
					break;
				}
			}
		}// 处理PC 发送的命令
		else
			reportdata[0] = 1; 
		if((ypr[1]>15.0)||(ypr[1]<-15.0))
			DRDY_High();
		else if	((ypr[2]>15.0)||(ypr[2]<-15.0))
			DRDY_High();
		else
			DRDY_Low();


	}//主循环 while(1) 结束

}  //main	

//------------------End of File----------------------------
