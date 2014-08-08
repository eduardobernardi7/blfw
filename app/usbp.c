#include "stm32f2xx.h"
#include "usbp.h"

#include "fsm.h"

//definitions for USBP task
#define USBP_STACK_SIZE		        configMINIMAL_STACK_SIZE + 512
#define USBP_TASK_PRIORITY		( tskIDLE_PRIORITY + 3 )

#define USBP_EVENT_LIST_SIZE            128

// Event type, parameters can be added after super
typedef struct USBP_EVENT_TAG
{
  FSM_Event super;
  uint8_t   rx_data;    
} USBP_EVENT_T;

typedef struct USBP_TAG
{
  FSM super;
  xQueueHandle  events;
  xTaskHandle   task_handle;
}USBP_T;

enum USBP_SIGNALS
{ 
  USBP_DATA_RX = FSM_USER_SIGNAL,
};

USBP_T usbp;

void USBP_TaskCreate(void);
static void vUSBPTask( void *pvParameters );


//Core methods for the fsm
int USBP_Post_Event(USBP_T *me, USBP_EVENT_T *usbp_e);
int IV_Post_EventFromISR(USBP_T *me, USBP_EVENT_T *usbp_e);
void USBP_Process(void);

// FSM states
FSM_State USBP_HAND_IDLE(USBP_T *me, FSM_Event *e);

FSM_State USBP_HAND_IDLE(USBP_T *me, FSM_Event *e)
{
  USBP_EVENT_T * event = (USBP_EVENT_T *) e;
  
  event->rx_data = 0x00;
  
  return FSM_HANDLED();
}

int USBP_Post_Event(USBP_T *me, USBP_EVENT_T *usbp_e)
{
  if( xQueueSend(me->events, (void *) usbp_e, (TickType_t) 100) == pdPASS)
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

int IV_Post_EventFromISR(USBP_T *me, USBP_EVENT_T *usbp_e)
{
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
  
  if(xQueueSendFromISR(me->events, usbp_e, &xHigherPriorityTaskWoken ) == pdTRUE)
  {    
    portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
    
    return 1;
  }
  else
  {
    return 0;
  }  
}

void USBP_Process()
{
  USBP_EVENT_T evt;
  
  if(xQueueReceive(usbp.events, &evt, (TickType_t) 100) == pdTRUE)
  {
    FSM_Dispatch (&usbp.super, &evt.super);       
  }
}


void USBP_TaskCreate()
{
  xTaskCreate(vUSBPTask, "USBP", USBP_STACK_SIZE, NULL, USBP_TASK_PRIORITY, &usbp.task_handle);
}

static void vUSBPTask( void *pvParameters )
{
  for(;;)
  {
    USBP_Process();
  }
}

void USBP_Init(void)
{
  //Signal and data queue
  usbp.events = xQueueCreate(USBP_EVENT_LIST_SIZE, sizeof(USBP_EVENT_T));  
  
  // FSM Constructor
  FSM_Ctor(&usbp.super,USBP_HAND_IDLE);
  
  // FSM Init
  FSM_Init(&usbp.super);   
  
  USBP_TaskCreate();
  
}

int USBP_ProcessDataFromISR(uint8_t data)
{
  USBP_EVENT_T event;
  
  event.rx_data = data;
  event.super.signal = USBP_DATA_RX;
  
  return IV_Post_EventFromISR(&usbp, &event);
  
}


