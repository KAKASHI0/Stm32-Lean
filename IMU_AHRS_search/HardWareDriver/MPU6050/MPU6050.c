/* MPU6050.c file
��д�ߣ�lisn3188
��ַ��www.chiplab7.com
����E-mail��lisn3188@163.com
���뻷����MDK-Lite  Version: 4.23
����ʱ��: 2012-04-25
���ԣ� ���������ڵ���ʵ���ҵ�mini IMU����ɲ���
���ܣ�
�ṩMPU6050 ��ʼ�� ��ȡ��ǰ����ֵ��API
------------------------------------
 */

#include "MPU6050.h"
#include "IOI2C.h"
#include "LED.h"

#define Myabs(a) ((a)>0)?(a):(-(a))
uint8_t buffer[14];
//�����������MPU6050_ACCEL_FS_2��ƫ��ֵ
#define iniaxoffset -8
#define iniayoffset 25
#define iniazoffset 141

#define inigxoffset -14
#define inigyoffset 15
#define inigzoffset -7
int16_t  MPU6050_FIFO[6][11];
int16_t Gx_offset=0,Gy_offset=0,Gz_offset=0;
int16_t Ax_offset=0, Ay_offset=0, Az_offset=0;
int16_t Acc_max_x=0, Acc_min_x=0, Acc_max_y=0, Acc_min_y=0, Acc_max_z=0, Acc_min_z=0;
extern int16_t Upload_Speed;
extern int16_t Hx_offset;
extern int16_t Hy_offset;
extern int16_t Hz_offset;

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void  MPU6050_newValues(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz)
*��������:	    ���µ�ADC���ݸ��µ� FIFO���飬�����˲�����
*******************************************************************************/
void  MPU6050_newValues(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz)
{
unsigned char i , j;
int32_t sum=0;
int32_t nMin;
int32_t nMax;
for(i=1;i<10;i++){	//FIFO ����
MPU6050_FIFO[0][i-1]=MPU6050_FIFO[0][i];
MPU6050_FIFO[1][i-1]=MPU6050_FIFO[1][i];
MPU6050_FIFO[2][i-1]=MPU6050_FIFO[2][i];
MPU6050_FIFO[3][i-1]=MPU6050_FIFO[3][i];
MPU6050_FIFO[4][i-1]=MPU6050_FIFO[4][i];
MPU6050_FIFO[5][i-1]=MPU6050_FIFO[5][i];
}
MPU6050_FIFO[0][9]=ax;//���µ����ݷ��õ� ���ݵ������
MPU6050_FIFO[1][9]=ay;
MPU6050_FIFO[2][9]=az;
MPU6050_FIFO[3][9]=gx;
MPU6050_FIFO[4][9]=gy;
MPU6050_FIFO[5][9]=gz;

MPU6050_FIFO[0][10]=ax;//���µ����ݷ��õ� ���ݵ������
MPU6050_FIFO[1][10]=ay;
MPU6050_FIFO[2][10]=az;
MPU6050_FIFO[3][10]=gx;
MPU6050_FIFO[4][10]=gy;
MPU6050_FIFO[5][10]=gz;

/*for (j=0; j<6; j++)
{
    sum=0;
    nMin = 1<<30;
    nMax = -1<<30;
    for(i=0;i<10;i++){	//��ǰ����ĺϣ���ȡƽ��ֵ
       sum+=MPU6050_FIFO[j][i];
       if (nMin > MPU6050_FIFO[j][i])
           nMin = MPU6050_FIFO[j][i];
       if (nMax < MPU6050_FIFO[j][i])
           nMax = MPU6050_FIFO[j][i];
    }
    MPU6050_FIFO[j][10]=(sum - nMin - nMax)/8;
}*/
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MPU6050_setClockSource(uint8_t source)
*��������:	    ����  MPU6050 ��ʱ��Դ
 * CLK_SEL | Clock Source
 * --------+--------------------------------------
 * 0       | Internal oscillator
 * 1       | PLL with X Gyro reference
 * 2       | PLL with Y Gyro reference
 * 3       | PLL with Z Gyro reference
 * 4       | PLL with external 32.768kHz reference
 * 5       | PLL with external 19.2MHz reference
 * 6       | Reserved
 * 7       | Stops the clock and keeps the timing generator in reset
*******************************************************************************/
void MPU6050_setClockSource(uint8_t source){
    IICwriteBits(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);

}

/** Set full-scale gyroscope range.
 * @param range New full-scale gyroscope range value
 * @see getFullScaleRange()
 * @see MPU6050_GYRO_FS_250
 * @see MPU6050_RA_GYRO_CONFIG
 * @see MPU6050_GCONFIG_FS_SEL_BIT
 * @see MPU6050_GCONFIG_FS_SEL_LENGTH
 */
void MPU6050_setFullScaleGyroRange(uint8_t range) {
    IICwriteBits(devAddr, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MPU6050_setFullScaleAccelRange(uint8_t range)
*��������:	    ����  MPU6050 ���ٶȼƵ��������
*******************************************************************************/
void MPU6050_setFullScaleAccelRange(uint8_t range) {
    IICwriteBits(devAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MPU6050_setSleepEnabled(uint8_t enabled)
*��������:	    ����  MPU6050 �Ƿ����˯��ģʽ
				enabled =1   ˯��
			    enabled =0   ����
*******************************************************************************/
void MPU6050_setSleepEnabled(uint8_t enabled) {
    IICwriteBit(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		uint8_t MPU6050_getDeviceID(void)
*��������:	    ��ȡ  MPU6050 WHO_AM_I ��ʶ	 ������ 0x68
*******************************************************************************/
uint8_t MPU6050_getDeviceID(void) {

    IICreadBytes(devAddr, MPU6050_RA_WHO_AM_I, 1, buffer);
    return buffer[0];
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		uint8_t MPU6050_testConnection(void)
*��������:	    ���MPU6050 �Ƿ��Ѿ�����
*******************************************************************************/
uint8_t MPU6050_testConnection(void) {
   if(MPU6050_getDeviceID() == 0x68)  //0b01101000;
   return 1;
   	else return 0;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MPU6050_setI2CMasterModeEnabled(uint8_t enabled)
*��������:	    ���� MPU6050 �Ƿ�ΪAUX I2C�ߵ�����
*******************************************************************************/
void MPU6050_setI2CMasterModeEnabled(uint8_t enabled) {
    IICwriteBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT, enabled);
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MPU6050_setI2CBypassEnabled(uint8_t enabled)
*��������:	    ���� MPU6050 �Ƿ�ΪAUX I2C�ߵ�����
*******************************************************************************/
void MPU6050_setI2CBypassEnabled(uint8_t enabled) {
    IICwriteBit(devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_I2C_BYPASS_EN_BIT, enabled);
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MPU6050_initialize(void)
*��������:	    ��ʼ�� 	MPU6050 �Խ������״̬��
*******************************************************************************/
void MPU6050_initialize(void) {
	int16_t temp[6];
	//int16_t factor;
	unsigned char i;
    MPU6050_setClockSource(MPU6050_CLOCK_PLL_XGYRO); //����ʱ��
	GyrAndAccSettings = (MPU6050_GYRO_FS_2000<<2) | (MPU6050_ACCEL_FS_16);
    MPU6050_setFullScaleGyroRange((GyrAndAccSettings>>2)&3);//������������� +-1000��ÿ��
    MPU6050_setFullScaleAccelRange(GyrAndAccSettings&3);	//���ٶȶ�������� +-2G
    MPU6050_setSleepEnabled(0); //���빤��״̬
	MPU6050_setI2CMasterModeEnabled(0);	 //����MPU6050 ����AUXI2C
	MPU6050_setI2CBypassEnabled(1);	 //����������I2C��	MPU6050��AUXI2C	ֱͨ������������ֱ�ӷ���HMC5883L
	

	//����MPU6050 ���ж�ģʽ ���жϵ�ƽģʽ
	IICwriteBit(devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_INT_LEVEL_BIT, 0);
	IICwriteBit(devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_INT_OPEN_BIT, 0);
	IICwriteBit(devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_LATCH_INT_EN_BIT, 1);
	IICwriteBit(devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_INT_RD_CLEAR_BIT, 1);
	//������ת������ж�
    IICwriteBit(devAddr, MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_DATA_RDY_BIT, 1);

    for(i=0;i<10;i++){//����FIFO����
	delay_us(50);
	MPU6050_getMotion6(&temp[0],&temp[1],&temp[2],&temp[3],&temp[4],&temp[5], 1);
	}
																			 
	MPU6050_InitGyro_Offset();
	
	//��ʼ�����ٵ�ƫ��    
	//factor = GyrAndAccSettings&3;
  Ax_offset = iniaxoffset;//>>factor;
	Ay_offset = iniayoffset;//>>factor;
	Az_offset = iniazoffset;//>>factor;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		unsigned char MPU6050_is_DRY(void)
*��������:	    ��� MPU6050���ж����ţ������Ƿ����ת��
���� 1  ת�����
0 ���ݼĴ�����û�и���
*******************************************************************************/
unsigned char MPU6050_is_DRY(void)
{
    if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6)==Bit_SET){
	  return 1;
	 }
	 else return 0;
}

int16_t MPU6050_Lastax,MPU6050_Lastay,MPU6050_Lastaz
				,MPU6050_Lastgx,MPU6050_Lastgy,MPU6050_Lastgz;
/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MPU6050_getMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz) {
*��������:	    ��ȡ MPU6050�ĵ�ǰ����ֵ
*******************************************************************************/
void 
MPU6050_getMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz, int8_t Adjust) {
  
	if(MPU6050_is_DRY()){
	IICreadBytes(devAddr, MPU6050_RA_ACCEL_XOUT_H, 14, buffer);
    MPU6050_Lastax=(((int16_t)buffer[0]) << 8) | buffer[1];
    MPU6050_Lastay=(((int16_t)buffer[2]) << 8) | buffer[3];
    MPU6050_Lastaz=(((int16_t)buffer[4]) << 8) | buffer[5];
	//�����¶�ADC
    MPU6050_Lastgx=(((int16_t)buffer[8]) << 8) | buffer[9];
    MPU6050_Lastgy=(((int16_t)buffer[10]) << 8) | buffer[11];
    MPU6050_Lastgz=(((int16_t)buffer[12]) << 8) | buffer[13];
	MPU6050_newValues(MPU6050_Lastax,MPU6050_Lastay,MPU6050_Lastaz
		,MPU6050_Lastgx,MPU6050_Lastgy,MPU6050_Lastgz);
	*ax  =MPU6050_FIFO[0][10];
	*ay  =MPU6050_FIFO[1][10];
	*az = MPU6050_FIFO[2][10];
	*gx  =MPU6050_FIFO[3][10] - Gx_offset;
	*gy = MPU6050_FIFO[4][10] - Gy_offset;
	*gz = MPU6050_FIFO[5][10] - Gz_offset;
	} else {
	*ax = MPU6050_FIFO[0][10];//=MPU6050_FIFO[0][10];
	*ay = MPU6050_FIFO[1][10];//=MPU6050_FIFO[1][10];
	*az = MPU6050_FIFO[2][10];//=MPU6050_FIFO[2][10];
	*gx = MPU6050_FIFO[3][10] - Gx_offset;//=MPU6050_FIFO[3][10];
	*gy = MPU6050_FIFO[4][10] - Gy_offset;//=MPU6050_FIFO[4][10];
	*gz = MPU6050_FIFO[5][10] - Gz_offset;//=MPU6050_FIFO[5][10];
	}
	if (Adjust == 1)
	{
	    *ax -= Ax_offset;
		*ay -= Ay_offset;
		*az -= Az_offset;
	}
}

void MPU6050_getlastMotion6(int16_t* ax, int16_t* ay, 
		int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz)
{
	*ax  =MPU6050_FIFO[0][10] - Ax_offset;
	*ay  =MPU6050_FIFO[1][10] - Ay_offset;
	*az = MPU6050_FIFO[2][10] - Az_offset;
	*gx  =MPU6050_FIFO[3][10] - Gx_offset;
	*gy = MPU6050_FIFO[4][10] - Gy_offset;
	*gz = MPU6050_FIFO[5][10] - Gz_offset;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MPU6050_InitGyro_Offset(void)
*��������:	    ��ȡ MPU6050��������ƫ��
��ʱģ��Ӧ�ñ���ֹ���á��Բ��Ծ�ֹʱ�����������
*******************************************************************************/
void MPU6050_InitGyro_Offset(void)
{
	//unsigned char i;
	//int16_t temp[6];
	//int32_t	tempgx=0,tempgy=0,tempgz=0;
	Gx_offset=inigxoffset;
	Gy_offset=inigyoffset;
	Gz_offset=inigzoffset;
	/*for(i=0;i<50;i++){
  		delay_us(100);
  		MPU6050_getMotion6(&temp[0],&temp[1],&temp[2],&temp[3],&temp[4],&temp[5], 1);
  		LED_Change();
	}
 	for(i=0;i<100;i++){
		delay_us(200);
		MPU6050_getMotion6(&temp[0],&temp[1],&temp[2],&temp[3],&temp[4],&temp[5], 1);
		tempgx+= temp[3];
		tempgy+= temp[4];
		tempgz+= temp[5];
		LED_Change();
	}

	Gx_offset=tempgx/100;//MPU6050_FIFO[3][10];
	Gy_offset=tempgy/100;//MPU6050_FIFO[4][10];
	Gz_offset=tempgz/100;//MPU6050_FIFO[5][10];*/
}

unsigned char MPU6050_SetAccEextremevalue(void)
{
	unsigned char i;
	unsigned char cmdtype;
	int16_t temp[6];
	int16_t	tempax=0,tempay=0,tempaz=0;
 	for(i=0;i<1;i++){
		delay_us(200);
		MPU6050_getMotion6(&temp[0],&temp[1],&temp[2],&temp[3],&temp[4],&temp[5], 0);
		tempax+= temp[0];
		tempay+= temp[1];
		tempaz+= temp[2];
		//LED_Change();
	}
	//tempax/=100;
	//tempay/=100;
	//tempaz/=100;
    if ((myabs(tempax) >= myabs(tempay)) && (myabs(tempax) >= myabs(tempaz)))
	{
	    if (tempax < 0)
		{
		    cmdtype = 0x00;
		}
		else 
		{
		    cmdtype = 0x01;
		} 
	}
	else if ((myabs(tempay) >= myabs(tempax)) && (myabs(tempay) >= myabs(tempaz)))
	{
	    if (tempay < 0)
		    cmdtype = 0x02;
		else
		    cmdtype = 0x03; 
	}
	else
 	{
	    if (tempaz < 0)
		    cmdtype = 0x04;
		else
		    cmdtype = 0x05; 
	}
	switch(cmdtype){
	    case 0x00: 
		{
		    Acc_min_x = tempax; 
			if (Acc_max_x != 0)
	            Ax_offset = (Acc_max_x + Acc_min_x) / 2;
			break; 
		}
		case 0x01:
		{
		    Acc_max_x = tempax;
			if (Acc_min_x != 0)
		        Ay_offset = (Acc_max_y + Acc_min_y) / 2; 
			break;
		}
	    case 0x02: 
		{
		    Acc_min_y = tempay; 
			if (Acc_max_y != 0)
		        Ay_offset = (Acc_max_y + Acc_min_y) / 2;
			break;
		}
		case 0x03: 
		{
		    Acc_max_y = tempay; 
			if (Acc_min_y != 0)
		        Ay_offset = (Acc_max_y + Acc_min_y) / 2;
			break;
		}
	    case 0x04: 
		{
		    Acc_min_z = tempaz; 
		    if (Acc_max_z != 0)
		        Az_offset = (Acc_max_z + Acc_min_z) / 2;
			break;
		}
		case 0x05: 
		{
		    Acc_max_z = tempaz; 
		    if (Acc_min_z != 0)
		        Az_offset = (Acc_max_z + Acc_min_z) / 2;
			break;
		}		 
	}
	//��PC���͸��º�ļ��ٶ�ƫ��ֵ
//	UART1_ReportAccOffset(Ax_offset, Ay_offset, Az_offset);
	UART2_ReportAccOffset(Ax_offset, Ay_offset, Az_offset);

	return cmdtype;
}

void HMC5883L_SetHmcoffset(int16_t hx, int16_t hy, int16_t hz)
{
    Hx_offset = hx;
	Hy_offset = hy;
	Hz_offset = hz;
		//��PC���͸��º�ļ��ٶ�ƫ��ֵ
//	UART1_ReportAccOffset(Hx_offset, Hy_offset, Hz_offset);
	UART2_ReportAccOffset(Hx_offset, Hy_offset, Hz_offset);
}

void MPU6050_SetAccOffset(int16_t ax, int16_t ay, int16_t az)
{
    int16_t factor;
	factor = GyrAndAccSettings&3;
    Ax_offset = ax>>factor;
	Ay_offset = ay>>factor;
	Az_offset = az>>factor;
		//��PC���͸��º�ļ��ٶ�ƫ��ֵ
//	UART1_ReportAccOffset(Ax_offset, Ay_offset, Az_offset);
	UART2_ReportAccOffset(Ax_offset, Ay_offset, Az_offset);
}
//------------------End of File----------------------------
