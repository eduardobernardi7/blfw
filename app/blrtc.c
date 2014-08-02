#include "blrtc.h"

#define RTC_CLOCK_SOURCE_LSE

RTC_TimeTypeDef RTC_TimeStructure;
RTC_InitTypeDef RTC_InitStructure;

void BLRTC_Init()
{  
  __IO uint32_t AsynchPrediv = 0, SynchPrediv = 0;

  if (RTC_ReadBackupRegister(RTC_BKP_DR0) != 0x32F2)
  {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
    PWR_BackupAccessCmd(ENABLE);
    
#if defined (RTC_CLOCK_SOURCE_LSI)  /* LSI used as RTC source clock*/
    /* The RTC Clock may varies due to LSI frequency dispersion. */   
    /* Enable the LSI OSC */ 
    RCC_LSICmd(ENABLE);

    /* Wait till LSI is ready */  
    while(RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET)
    {
    }

    /* Select the RTC Clock Source */
    RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);
  
    SynchPrediv = 0xFF;
    AsynchPrediv = 0x7F;

#elif defined (RTC_CLOCK_SOURCE_LSE) /* LSE used as RTC source clock */
  /* Enable the LSE OSC */
    RCC_LSEConfig(RCC_LSE_ON);

    /* Wait till LSE is ready */  
    while(RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET)
    {
    }

    /* Select the RTC Clock Source */
    RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);
  
    SynchPrediv = 0xFF;
    AsynchPrediv = 0x7F;

#else
  #error Please select the RTC Clock source inside the main.c file
#endif /* RTC_CLOCK_SOURCE_LSI */
  
    
    RCC_RTCCLKCmd(ENABLE);
    RTC_WaitForSynchro();
    
    RTC_InitStructure.RTC_AsynchPrediv = AsynchPrediv;
    RTC_InitStructure.RTC_SynchPrediv = SynchPrediv;
    RTC_InitStructure.RTC_HourFormat = RTC_HourFormat_24;
    
    RTC_Init(&RTC_InitStructure); 
    
    RTC_TimeStructure.RTC_H12 = RTC_H12_AM;
    
    RTC_TimeStructure.RTC_Hours = 7;
    
    RTC_TimeStructure.RTC_Minutes = 7;
    
    RTC_TimeStructure.RTC_Seconds = 7;
    
    if (RTC_SetTime(RTC_Format_BIN, &RTC_TimeStructure) != ERROR)
    {
      RTC_WriteBackupRegister(RTC_BKP_DR0, 0x32F2);
    }
    
  }
  else
  {
     RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
     PWR_BackupAccessCmd(ENABLE);
     RTC_WaitForSynchro();
  }
  
}

int BLRTC_SetTime(RTC_TimeTypeDef time)
{
  if (RTC_SetTime(RTC_Format_BIN, &RTC_TimeStructure) != ERROR)
  {
    RTC_WriteBackupRegister(RTC_BKP_DR0, 0x32F2);
    return 1;
  }
  else
  {
    return 0;
  }    
}

RTC_TimeTypeDef BLRTC_GetTime()
{
  RTC_GetTime(RTC_Format_BIN, &RTC_TimeStructure);
  return RTC_TimeStructure;
}