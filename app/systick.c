#include "systick.h"

static __IO uint32_t timing_delay;

void SYSTICK_delay_ms(__IO uint32_t nTime)
{
  timing_delay = nTime;

  while(timing_delay != 0);
}


void SYSTICK_tick(void)
{
  if (timing_delay != 0x00)
  { 
    timing_delay--;
  }
}

void SYSTICK_Init(void)
{
  RCC_ClocksTypeDef RCC_Clocks;
  
  /* SysTick end of count event each 10ms */
  RCC_GetClocksFreq(&RCC_Clocks);
  //SysTick_Config(SystemCoreClock/100);
  SysTick_Config(RCC_Clocks.HCLK_Frequency / 1000);
}