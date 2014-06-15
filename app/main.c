#include "stm32f2xx.h"

#include "systick.h"
#include "dac.h"
#include "timer.h"
#include "adc12.h"

#define FAT_FS_TEST

#ifdef FAT_FS_TEST
#include "stm322xg_eval_sdio_sd.h"
#include "ff.h"
#include "diskio.h"
#include "string.h"
FRESULT res;
FILINFO fno;
FIL fil;
DIR dir;
FATFS fs32;

BYTE Buffer[512];
UINT BytesRead;
UINT BytesWritten;
#endif

int main()
{
  
#ifdef FAT_FS_TEST
  
  /* Interrupt Config */
  SD_InterruptEnable();

  memset(&fs32, 0, sizeof(FATFS));
  
  res = f_mount(0, &fs32);
  
  res = f_open(&fil, "DUDU.TXT", FA_READ);
  
  if(res == FR_OK)
  {
    res = f_read(&fil, Buffer, sizeof(Buffer), &BytesRead);
    
    res = f_close(&fil);
  }  
  
  res = f_open(&fil, "DUDU.TXT", FA_CREATE_ALWAYS | FA_WRITE);
  
  if (res == FR_OK)
  {
    char * s = "Hello word \r\n";
    res = f_write(&fil, s, strlen(s), &BytesWritten);
    res = f_close(&fil); // LENGTH.TXT
  }
  
#endif
  
  GPIO_InitTypeDef  GPIO_InitStructure;
  
  SYSTICK_Init();
  
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOE, &GPIO_InitStructure);
  
  GPIOE->ODR |= GPIO_Pin_3;
  
  DAC_HwInit();
  
  /* Testes de medicao e reproducao de pulsos */
  TIMER_InputCaptureCh2Init();
  TIMER_OutputCompareCh3Init();
  TIMER_DemoIrGenStructInit();
  
  ADC12_Init();
  
  for(;;)
  {
    GPIOE->ODR ^= GPIO_Pin_1;
    GPIOE->ODR ^= GPIO_Pin_2;
    GPIOE->ODR ^= GPIO_Pin_4;
    GPIOE->ODR ^= GPIO_Pin_7;    
    
    SYSTICK_delay_ms(1);
    
  }
}


