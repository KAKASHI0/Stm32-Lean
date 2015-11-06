#include "stm32f10x.h"
#include 	"all_init_heard.h"

vu16  AD_Value[N][M];
vu16	AD_Transform_Value[N];
		
__task void ADC_Transform_Task(void) 
{
		for(;;)
		{
			ADC_Transform(AD_Value);
			//printf("AD_Transform_Value[0] is %d\r\n",AD_Transform_Value[0]);
		}
}

void ADC_Transform(vu16 Value[N][M])
{
		int i,j;
		vu16 Sum=0.0,Average=0.0;
		for(i=0;i<N;i++)
		{	
			for(j=0;j<M;j++)
					{
						Sum+=(vu16)Value[i][j];
					}
					Average=Sum/M;
					AD_Transform_Value[i]=Average;
					Average=0.0;
					Sum=0.0;
			}	
}	

