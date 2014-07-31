#include "stm32f2xx.h"

/******************************************************************************/
/* Application Switches for tests */
#define FAT_FS_TEST
#define USB_ECHO_VCP_TEST

/******************************************************************************/

/* USB STACK */
#include "usbd_cdc_core.h"
#include "usbd_usr.h"
#include "usb_conf.h"
#include "usbd_desc.h"

 #ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
  #if defined ( __ICCARM__ ) /*!< IAR Compiler */
    #pragma data_alignment=4   
  #endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */

__ALIGN_BEGIN USB_OTG_CORE_HANDLE    USB_OTG_dev __ALIGN_END ;

/******************************************************************************/

/* FATFS STACK */
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

/******************************************************************************/
/* RTOS */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#define ledSTACK_SIZE		configMINIMAL_STACK_SIZE
#define ledFLASH_RATE_BASE	( ( TickType_t ) 333 )
#define mainFLASH_TASK_PRIORITY				( tskIDLE_PRIORITY + 1 )
static void vTestTask( void *pvParameters );

/******************************************************************************/

/* Application */
#include "systick.h"
#include "dac.h"
#include "timer.h"
#include "adc12.h"
#include "IV.h"

/******************************************************************************/
/* MAIN */

//Thousands of test involving all features listed above
//Plaese, disregard the lack of organization on main function 

/******************************************************************************/
int main()
{

  //I have no idea what this means, RTOS needed nvic configured like this
  NVIC_SetVectorTable( NVIC_VectTab_FLASH, 0x0 );
  NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );
  
  SysTick_CLKSourceConfig( SysTick_CLKSource_HCLK );
  
#ifdef USB_ECHO_VCP_TEST
     USBD_Init(&USB_OTG_dev,
#ifdef USE_USB_OTG_HS 
            USB_OTG_HS_CORE_ID,
#else            
            USB_OTG_FS_CORE_ID,
#endif  
            &USR_desc, 
            &USBD_CDC_cb, 
            &USR_cb);
#endif

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
  
  //SYSTICK_Init();
  
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOE, &GPIO_InitStructure);
  
  GPIOE->ODR |= GPIO_Pin_3;
  
  DAC_HwInit();
  
  /* Pulse capture and reproduction using tim1*/
  TIMER_InputCaptureCh2Init();
  TIMER_OutputCompareCh3Init();
  TIMER_DemoIrGenStructInit();
  
  /*TIM8 will be used as a tick reference timer instead of systick */
  TIMER8_OutputcompareCh2Init();
    
  ADC12_Init();
  IV_Init();
  
#if(1)    
  xTaskCreate( vTestTask, "LEDx", ledSTACK_SIZE, NULL, mainFLASH_TASK_PRIORITY, ( TaskHandle_t * ) NULL );
  vTaskStartScheduler();
#endif
  
#if(0)
  IV_Perform_Curve();
#endif
  
  for(;;)
  {
#if(0)
    IV_Process(); 
#endif
    
#if(0)
    GPIOE->ODR ^= GPIO_Pin_1;
    GPIOE->ODR ^= GPIO_Pin_2;
    GPIOE->ODR ^= GPIO_Pin_4;
    GPIOE->ODR ^= GPIO_Pin_7;    
    
    TIM8_delay_ms(1);
#endif
    
  }
}


static void vTestTask( void *pvParameters )
{
  TickType_t xFlashRate, xLastFlashTime;
  
  xFlashRate = ledFLASH_RATE_BASE;
  xFlashRate /= portTICK_PERIOD_MS;

  xFlashRate /= ( TickType_t ) 2;
  
  xLastFlashTime = xTaskGetTickCount();
  
  for(;;)
  {
    vTaskDelayUntil( &xLastFlashTime, xFlashRate );
    
    GPIOE->ODR ^= GPIO_Pin_1;
    GPIOE->ODR ^= GPIO_Pin_2;
    GPIOE->ODR ^= GPIO_Pin_4;
    GPIOE->ODR ^= GPIO_Pin_7;  
    
    vTaskDelayUntil( &xLastFlashTime, xFlashRate );
    
    GPIOE->ODR ^= GPIO_Pin_1;
    GPIOE->ODR ^= GPIO_Pin_2;
    GPIOE->ODR ^= GPIO_Pin_4;
    GPIOE->ODR ^= GPIO_Pin_7;    
  }
}

