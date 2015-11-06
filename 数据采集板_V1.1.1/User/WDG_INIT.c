#include "stm32f10x.h"
#include "wdg_config.h"

#define OpenWDG_QUDONG

void WDG_Configration(void)
{
			IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
			/* IWDG counter clock: 40KHz(LSI) / 32 = 1.25 KHz */
			IWDG_SetPrescaler(IWDG_Prescaler_32);
			/* Set counter reload value to 349 */
			IWDG_SetReload(349);
			/*Reload IWDG counter */
			IWDG_ReloadCounter();
			/* Enable IWDG (the LSI oscillator will be enabled by hardware) */
			IWDG_Enable();
}


