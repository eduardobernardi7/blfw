#include "stm32f2xx.h"

#include "ihm.h"
#include "iv.h"
#include "ivs.h"
#include "dynload.h"

//definitions and prototypes for IHM task
#define IHM_STACK_SIZE		        configMINIMAL_STACK_SIZE + 128
#define IHM_TASK_PRIORITY		( tskIDLE_PRIORITY + 4 )
#define IHM_DELAY_TICK_BASE             ( ( TickType_t ) 500 )
#define IHM_PB_EVENTS_SIZE              32

static void vIHMTask( void *pvParameters );

typedef struct IHM_TAG
{
  xTaskHandle task_handle;
  xQueueHandle pb_queue;
  xSemaphoreHandle pb_bsem;
  TickType_t xDebounceDelay;
  TickType_t xDebounceDelayTime;
  uint16_t aux_tog;
}IHM_T;

IHM_T ihm;

void IHM_TaskCreate(void);

void IHM_Init()
{
  
  EXTI_InitTypeDef   EXTI_InitStructure;
  GPIO_InitTypeDef   GPIO_InitStructure;
  NVIC_InitTypeDef   NVIC_InitStructure;
  
  IHM_TaskCreate();
  
  //Initializing local variables and rtos  
  ihm.pb_queue = xQueueCreate(IHM_PB_EVENTS_SIZE, sizeof(PB_T));  
  vSemaphoreCreateBinary(ihm.pb_bsem);
  ihm.aux_tog = 0;
  
  
  //Initializing hardware  
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6;
  GPIO_Init(GPIOE, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOE, &GPIO_InitStructure);
  
  GPIOE->ODR &= ~GPIO_Pin_1;
  GPIOE->ODR &= ~GPIO_Pin_2;
  GPIOE->ODR &= ~GPIO_Pin_3;
  GPIOE->ODR &= ~GPIO_Pin_4; 
  
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource5);
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource6);
  
  EXTI_InitStructure.EXTI_Line = EXTI_Line5;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
  
  EXTI_InitStructure.EXTI_Line = EXTI_Line6;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
  
  NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  
  NVIC_Init(&NVIC_InitStructure);  
}

void IHM_SetLed(LED_T led, LED_VALUE_T value)
{
  switch(led)
  {
  case USER_LED0:
    switch(value)
    {
    case HIGH:
      GPIOE->ODR |= GPIO_Pin_1;
      break;
      
    case LOW:
      GPIOE->ODR &= ~GPIO_Pin_1;
      break;
      
    case TOG:
      GPIOE->ODR ^= GPIO_Pin_1;
      break;
    }
    break;
    
  case USER_LED1:
    switch(value)
    {
    case HIGH:
      GPIOE->ODR |= GPIO_Pin_2;
      break;
      
    case LOW:
      GPIOE->ODR &= ~GPIO_Pin_2;
      break;
      
    case TOG:
      GPIOE->ODR ^= GPIO_Pin_2;
      break;
    } 
    break;
    
  case USER_LED2:
    switch(value)
    {
    case HIGH:
      GPIOE->ODR |= GPIO_Pin_3;
      break;
      
    case LOW:
      GPIOE->ODR &= ~GPIO_Pin_3;
      break;
      
    case TOG:
      GPIOE->ODR ^= GPIO_Pin_3;
      break;
    } 
    break;
    
  case USER_LED3:
    switch(value)
    {
    case HIGH:
      GPIOE->ODR |= GPIO_Pin_4;
      break;
      
    case LOW:
      GPIOE->ODR &= ~GPIO_Pin_4;
      break;
      
    case TOG:
      GPIOE->ODR ^= GPIO_Pin_4;
      break;
    }
    break;
  }
}

xQueueHandle * IHM_GetPBQueuePointer()
{
  return &ihm.pb_queue;
}

xTaskHandle * IHM_GetTaskHandlePointer()
{
  return &ihm.task_handle;
}

xSemaphoreHandle * IHM_GetPBSemPointer()
{
  return &ihm.pb_bsem;
}

void IHM_TaskCreate()
{
  xTaskCreate( vIHMTask, "IHM", IHM_STACK_SIZE, NULL, IHM_TASK_PRIORITY, &ihm.task_handle);    
}

static void vIHMTask( void *pvParameters )
{
  PB_T rx;
  SemaphoreHandle_t * ivs_time_mutex;
  
  ihm.xDebounceDelay = IHM_DELAY_TICK_BASE;
  ihm.xDebounceDelay /= portTICK_PERIOD_MS;
  ihm.xDebounceDelayTime = xTaskGetTickCount();
  
  ivs_time_mutex = IVS_GetTimerMutex();
  
  for(;;)
  {
    if(xQueueReceive(ihm.pb_queue, &rx, (TickType_t) 100) == pdTRUE)
    {
      switch(rx)
      {
      case USER_B1:
        IVS_StopCurve();
        
        if(!ihm.aux_tog)
        {
          DL_StartPID(400);
        }
        else
        {
          DL_StopPID();
        }
        
        ihm.aux_tog ^= 0x01;
        
        vTaskDelayUntil(&ihm.xDebounceDelayTime, ihm.xDebounceDelay);
        xQueueReset(ihm.pb_queue);
        break;
        
      case USER_B2:
        while(xSemaphoreTake(*ivs_time_mutex, (TickType_t) 100) == pdFALSE);
        
        DL_StopPID();
        
        IVS_Perform_Curve(1,3);  
        
        xSemaphoreGive(*ivs_time_mutex);
        
        vTaskDelayUntil(&ihm.xDebounceDelayTime, ihm.xDebounceDelay);
        xQueueReset(ihm.pb_queue); 
        break;
      }
    }       
  }
}