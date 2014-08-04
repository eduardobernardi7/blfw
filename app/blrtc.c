#include "blrtc.h"

#define RTC_CLOCK_SOURCE_LSE

RTC_TimeTypeDef         RTC_TimeStructure;
RTC_InitTypeDef         RTC_InitStructure;
RTC_DateTypeDef         RTC_DateStructure;
RTC_AlarmTypeDef        RTC_AlarmStructure;

void BLRTC_Init()
{  
  NVIC_InitTypeDef  NVIC_InitStructure;
  
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
    
    RTC_DateStructure.RTC_WeekDay = RTC_Weekday_Friday;
    RTC_DateStructure.RTC_Date = 0x18;
    RTC_DateStructure.RTC_Month = RTC_Month_March;
    RTC_DateStructure.RTC_Year = 0x11;
    
    RTC_SetDate(RTC_Format_BIN, &RTC_DateStructure);    
    
    if (RTC_SetTime(RTC_Format_BIN, &RTC_TimeStructure) != ERROR)
    {
      RTC_WriteBackupRegister(RTC_BKP_DR0, 0x32F2);
    }
    
    RTC_ClearITPendingBit(RTC_IT_WUT);
  }
  else
  {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
    PWR_BackupAccessCmd(ENABLE);
    RTC_WaitForSynchro();
    RTC_ClearITPendingBit(RTC_IT_WUT);
  }
  
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  
  /* Enable the RTC Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = RTC_WKUP_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  
  RTC_WakeUpClockConfig(RTC_WakeUpClock_RTCCLK_Div16);
  RTC_SetWakeUpCounter(0x7FF);
  
  /* Enable the Wakeup Interrupt */
  RTC_ITConfig(RTC_IT_WUT, ENABLE);
  
  RTC_ClearITPendingBit(RTC_IT_WUT);
  
  /* Enable Wakeup Counter */
  RTC_WakeUpCmd(ENABLE);  
  
}

int BLRTC_SetTime(RTC_TimeTypeDef time)
{
  RTC_TimeStructure = time;
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

RTC_DateTypeDef BLRTC_GetDate()
{
  RTC_GetDate(RTC_Format_BIN, &RTC_DateStructure);
  return RTC_DateStructure;
}

void BLRTC_ConfigAlarm(RTC_AlarmTypeDef alarm)
{
  RTC_TimeStructure = BLRTC_GetTime();
  RTC_DateStructure = BLRTC_GetDate();
  
  RTC_AlarmCmd(RTC_Alarm_A, DISABLE);
  
  RTC_AlarmStructure = alarm;
  
  RTC_AlarmStructure.RTC_AlarmDateWeekDay = RTC_DateStructure.RTC_WeekDay;
  RTC_AlarmStructure.RTC_AlarmDateWeekDaySel = RTC_AlarmDateWeekDaySel_Date;
  RTC_AlarmStructure.RTC_AlarmMask = RTC_AlarmMask_DateWeekDay;
  
  RTC_SetAlarm(RTC_Format_BIN, RTC_Alarm_A, &RTC_AlarmStructure);
  
  RTC_ITConfig(RTC_IT_ALRA, ENABLE);
  
  RTC_AlarmCmd(RTC_Alarm_A, ENABLE);
}