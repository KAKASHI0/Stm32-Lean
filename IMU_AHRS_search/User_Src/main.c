/* main.c file
��д�ߣ�lisn3188
��ַ��www.chiplab7.com
����E-mail��lisn3188@163.com
���뻷����MDK-Lite  Version: 4.23
����ʱ��: 2012-05-05
���ԣ� ���������ڵ���ʵ���ҵ�mini IMU����ɲ���
Mini IMU AHRS ģ��ٷ����۵�ַ��Http://chiplab7.taobao.com
���ܣ�
1.��ʼ��������������
2.������̬����͸߶Ȳ���
3.���������̬�͸���������������ϴ��� MiniIMU AHRS �������
4.��Ӧ PC���͵�����
------------------------------------
*/

#include "common.h"  //�������е����� ͷ�ļ�
#include "comm.h"
//�ϴ����ݵ�״̬��
#define REIMU    0x01     //�ϴ��������̬����
#define REAngle  0x02	  //�ϴ��Զ���ĽǶ�
#define REAGCG   0x03	  //�ϴ����ٶȼƣ������ǣ������ƣ��������������ٶ�
#define REQuat   0x04     //�ϴ���Ԫ��
#define REChipID 0x05     //�ϴ�оƬID
#define REAngleAndPos 0x06 // �ϴ��Զ���ĽǶȺ�λ��
#define REQuatAndPos 0x07  //�ϴ���Ԫ�غ�λ��


#define upload_time (1000000/Upload_Speed)  //�����ϴ���ʱ�䡣��λΪus

int16_t Upload_Speed = 120;   //�����ϴ��ٶ�  ��λ Hz
int16_t ChipID = 9;
int16_t ax, ay, az;	
int16_t gx, gy, gz;
int16_t hx, hy, hz;
int32_t Temperature = 0, Pressure = 0, Altitude = 0; 
uint32_t system_micrsecond;
int16_t hmcvalue[3];
u8 state= REQuat;  //�����ض�֡ ��״̬��
uint8_t GyrAndAccSettings = 0;
float Accsensitivity;//���ٶȵ�������
/**************************ʵ�ֺ���********************************************
*����ԭ��:		int main(void)
*��������:		������
*******************************************************************************/
int main(void)
{
	uint32_t delta_t;//�Ľ��Ĳ����ʼ��㷽��,Math_HZ/delta_t = HZ
  char reportdata[500];
	uint16_t acclen;
	uint16_t Math_hz=0;
	int16_t offax, offay, offaz, offhx, offhy, offhz;
	unsigned char PC_comm; //PC ����ؼ��ֽ�	 
	float ypr[3]; // yaw pitch roll
	float quat[4] = {1.0, 1.0, 1.0, 1.0};
	float grav[3] = {1.0, 2.0, 3.0}; 
  float pos[3] = {0,0,0};
	/* ����ϵͳʱ��Ϊ72M ʹ���ⲿ8M����+PLL*/      
    SystemInit();
	delay_init(72);		//��ʱ��ʼ��
    Initial_LED_GPIO();	//��ʼ��STM32-SDK�����ϵ�LED�ӿ�
	Initial_PWMLED();
	Initial_UART1(512000L);//������Լӿ�ͨѶ�����ʣ�����һ���̶ȣ�0.2ms������������ʱ����������ʱ2.2ms��
												//�����ǻ����������ʡ����鲻����720000
	Initial_UART2(115200L);

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_CRC, ENABLE);

	IIC_Init();	 //��ʼ��I2C�ӿ�
	delay_ms(300);	//�ȴ������ϵ�
	//UART1_Put_String("Initialize...\r\n");
	IMU_init(); //��ʼ��IMU�ʹ�����
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
	
	ChipID = (GyrAndAccSettings<<8) | ChipID;//�����ף�����
	
	system_micrsecond=micros();
	
	while(1)
	{	//��ѭ��
		
		//delay_ms(1); //��ʱ����Ҫ����ô�졣
		//IMU_getYawPitchRoll(ypr, quat); //��̬����
		if (state == REAngle)
			IMU_getYPRAngle(ypr, &acclen, 0);
		else if (state == REIMU)
			IMU_getYawPitchRollAndQuat(ypr, quat); //��̬����
		else if (state == REAngleAndPos)
	   IMU_getYPRAngle(ypr, &acclen, pos);
		else if (state == REQuat)
	   IMU_getQuat(quat, &acclen, 0);
		else if (state == REQuatAndPos)
		 IMU_getQuat(quat, &acclen, pos);
		else
	   IMU_getYawPitchRollAndQuat(ypr, quat);
	
		Math_hz++; //������� ++
		//BMP180_Routing(); //����BMP018 ���� ����ת���Ͷ�ȡ�����������ӳ����н��� 

	//-------------��λ��------------------------------
		//�Ƿ��˸��� ��λ����ʱ���ˣ�
		delta_t = (micros()-system_micrsecond);
		if(delta_t>=upload_time)
		{
				Math_hz=(uint16_t) ((float)Math_hz/(delta_t/1000000.0));
			switch(state){ 
				case REIMU:
				{
					//BMP180_getTemperat(&Temperature); //��ȡ������¶�ֵ
					//BMP180_getPress(&Pressure);	   //��ȡ�������ѹ����ֵ
					//BMP180_getAlt(&Altitude);	   //��ȡ��Ը߶�
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
					//state = REMOV; //����״̬��
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
			system_micrsecond=micros();	 //ȡϵͳʱ�� ��λ us 
			LED_Change();	//LED1�ı�����
		}
	//--------------------------------------------------
	
		//����PC������������
		if((PC_comm=UART2_CommandRoute(reportdata))!=0xff)
		{
			switch(PC_comm){ //��������ʶ
				case Gyro_init:			
				{
							MPU6050_InitGyro_Offset();
						break; //��ȡ��������ƫ
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
					break;		//��ѹ�߶� ����
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
		}// ����PC ���͵�����
		else
			reportdata[0] = 1; 
		if((ypr[1]>15.0)||(ypr[1]<-15.0))
			DRDY_High();
		else if	((ypr[2]>15.0)||(ypr[2]<-15.0))
			DRDY_High();
		else
			DRDY_Low();


	}//��ѭ�� while(1) ����

}  //main	

//------------------End of File----------------------------
