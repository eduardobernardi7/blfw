#include "stm32f2xx.h"

/******************************************************************************/
/* Application Switches for tests */
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
#define ledFLASH_RATE_BASE	( ( TickType_t ) 500 )
#define mainFLASH_TASK_PRIORITY				( tskIDLE_PRIORITY + 2 )
static void vTestTask( void *pvParameters );

/******************************************************************************/

/* Application */
#include "systick.h"
#include "dac.h"
#include "timer.h"
#include "adc12.h"
#include "IV.h"
#include "ihm.h"
#include "blrtc.h"

/******************************************************************************/
/* MAIN */

//Thousands of test involving all features listed above
//Plaese, disregard the lack of organization on main function 

/******************************************************************************/

/******************************************************************************/
/* USER GUIDE */

// Press S2 button to build a curve, if the curve was written successfully
// on sd card USER_LED1 will be on, otherwise USER_LED2 will be on instead
// led 0 will flash constantly indicating that FreeRtos is running !

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
     
  BLRTC_Init();   
  DAC_HwInit();
  
  /* Pulse capture and reproduction using tim1*/
  TIMER_InputCaptureCh2Init();
  TIMER_OutputCompareCh3Init();
  TIMER_DemoIrGenStructInit();
  
  /*TIM8 will be used as a tick reference timer instead of systick */
  TIMER8_OutputcompareCh2Init();
   
  ADC12_Init();
  IV_Init();
  IHM_Init();  
  
#if(1)    
  xTaskCreate( vTestTask, "LEDx", ledSTACK_SIZE, NULL, mainFLASH_TASK_PRIORITY, ( TaskHandle_t * ) NULL );
#endif  

  vTaskStartScheduler();
  
  for(;;);
}

// fancy task :D
static void vTestTask( void *pvParameters )
{
  TickType_t xFlashRate, xLastFlashTime;
  
  xFlashRate = ledFLASH_RATE_BASE;
  xFlashRate /= portTICK_PERIOD_MS;
  
  xLastFlashTime = xTaskGetTickCount();
  
  for(;;)
  {
    vTaskDelayUntil( &xLastFlashTime, xFlashRate );
    
    IHM_SetLed(USER_LED0, TOG);
    
    vTaskDelayUntil( &xLastFlashTime, xFlashRate );
    
    IHM_SetLed(USER_LED0,TOG);  
  }
}

