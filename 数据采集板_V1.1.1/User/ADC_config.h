#ifndef	__ADC_CONFIG_H
#define	__ADC_CONFIG_H
	
	
#define  M   50	    			//ÿ��ͨ����50��
#define  N  	3	     				//3��ͨ��
		

extern vu16  AD_Value[N][M];
extern vu16	AD_Transform_Value[N];

void ADC_Transform(vu16 Value[N][M]);
void ADC1_Configration(void);


#endif
