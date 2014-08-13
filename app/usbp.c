#include "stm32f2xx.h"
#include "usbp.h"

#include "fsm.h"
#include "usbd_cdc_vcp.h"

#include "stdio.h"
#include "string.h"

//FreeRtos
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"


//definitions for USBP task
#define USBP_STACK_SIZE		        configMINIMAL_STACK_SIZE + 512
#define USBP_TASK_PRIORITY		( tskIDLE_PRIORITY + 3 )

#define USBP_EVENT_LIST_SIZE            128

//protocol format
// |HEADER|ID|SIZE|DATA|

//protocol definitions
#define USBP_HEADER                     0xBB        
#define USBP_PACKET_MAX_SIZE            64

//protocol packet ids
#define USBP_VERSION_ID                 0x0A

//firmware version string
#define FW_VERSION_STRING_MAX_SIZE      10
const char * fw_string = "v1.0.0";

//send version packet definitions
#define USBP_SEND_VERSION_ID            0x0A
#define USBP_SEND_VERSION_DATA_SIZE     FW_VERSION_STRING_MAX_SIZE
#define USBP_SEND_VERSION_PACKET_SIZE   FW_VERSION_STRING_MAX_SIZE + 3

typedef struct USBP_PACKET_TAG
{
  uint8_t id;
  uint8_t size; 
  uint8_t data[USBP_PACKET_MAX_SIZE];
  uint8_t it;
}USBP_PACKET_T;

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
  USBP_PACKET_T cur_packet;
}USBP_T;

enum USBP_SIGNALS
{ 
  USBP_DATA_RX = FSM_USER_SIGNAL,
};

USBP_T usbp;

void USBP_TaskCreate(void);
static void vUSBPTask( void *pvParameters );

//responde to requests methods
void USBP_HandleVersionRequest(void);

//Core methods for the fsm
int USBP_Post_Event(USBP_T *me, USBP_EVENT_T *usbp_e);
int USBP_Post_EventFromISR(USBP_T *me, USBP_EVENT_T *usbp_e);
void USBP_Process(void);

// FSM states
FSM_State USBP_HAND_IDLE(USBP_T *me, FSM_Event *e);
FSM_State USBP_HAND_ID(USBP_T *me, FSM_Event *e);
FSM_State USBP_HAND_DATA_SIZE(USBP_T *me, FSM_Event *e);
FSM_State USBP_HAND_VERSION(USBP_T *me, FSM_Event *e);

FSM_State USBP_HAND_IDLE(USBP_T *me, FSM_Event *e)
{
  USBP_EVENT_T * event = (USBP_EVENT_T *) e;
  
  switch(event->super.signal)
  {
  case FSM_ENTRY_SIGNAL:
    return FSM_HANDLED();  
    
  case USBP_DATA_RX:
    if(event->rx_data == USBP_HEADER)
    {
      return FSM_TRAN(me,USBP_HAND_ID);
    }    
    break;    
    
  case FSM_EXIT_SIGNAL:
    return FSM_HANDLED();  
  }
  
  return FSM_HANDLED();
}

FSM_State USBP_HAND_ID(USBP_T *me, FSM_Event *e)
{
  USBP_EVENT_T * event = (USBP_EVENT_T *) e;
  
  switch(event->super.signal)
  {
  case FSM_ENTRY_SIGNAL:
    return FSM_HANDLED();  
    
  case USBP_DATA_RX:
    switch(event->rx_data)
    {
    case USBP_VERSION_ID:
      me->cur_packet.id = USBP_VERSION_ID;
      return FSM_TRAN(me,USBP_HAND_DATA_SIZE);
      break;
      
    default:
      me->cur_packet.id = 0;
      return FSM_TRAN(me,USBP_HAND_IDLE);
      break;
    }
    break;
    
  case FSM_EXIT_SIGNAL:
    return FSM_HANDLED();  
  }
  
  return FSM_HANDLED();
}

FSM_State USBP_HAND_DATA_SIZE(USBP_T *me, FSM_Event *e)
{
  USBP_EVENT_T * event = (USBP_EVENT_T *) e;
  
  switch(event->super.signal)
  {
  case FSM_ENTRY_SIGNAL:
    return FSM_HANDLED();  
    
  case USBP_DATA_RX:
    if(event->rx_data > 0 && event->rx_data <= USBP_PACKET_MAX_SIZE)
    {
      me->cur_packet.size = event->rx_data ;
      
      switch(me->cur_packet.id)
      {
      case USBP_VERSION_ID:
        return FSM_TRAN(me,USBP_HAND_VERSION);
        break;
        
      default:
        return FSM_TRAN(me,USBP_HAND_IDLE);
        break;        
      }
    }    
    else
    {
      return FSM_TRAN(me,USBP_HAND_IDLE);
    }
    break;    
    
  case FSM_EXIT_SIGNAL:
    return FSM_HANDLED();  
  }
  
  return FSM_HANDLED();
}

FSM_State USBP_HAND_VERSION(USBP_T *me, FSM_Event *e)
{
  USBP_EVENT_T * event = (USBP_EVENT_T *) e;
  
  switch(event->super.signal)
  {
  case FSM_ENTRY_SIGNAL:
    me->cur_packet.it = 0;
    return FSM_HANDLED();  
    
  case USBP_DATA_RX:
    if(me->cur_packet.it == me->cur_packet.size - 1)
    {
      me->cur_packet.data[me->cur_packet.it] = event->rx_data;     
      USBP_HandleVersionRequest();
    }
    return FSM_TRAN(me,USBP_HAND_IDLE);
    break;    
    
  case FSM_EXIT_SIGNAL:
    return FSM_HANDLED();  
  }
  
  return FSM_HANDLED();
}



void USBP_HandleVersionRequest()
{
  uint8_t i,j;
  uint8_t packet[USBP_SEND_VERSION_PACKET_SIZE];
  uint32_t fw_len;
  
  fw_len = strlen(fw_string);
  
  if(fw_len >= FW_VERSION_STRING_MAX_SIZE)
  {
    return;
  }
  
  packet[0] = USBP_HEADER;
  packet[1] = USBP_SEND_VERSION_ID;
  packet[2] = USBP_SEND_VERSION_DATA_SIZE;
  
  j = 0;
  for(i = 3; i < USBP_SEND_VERSION_PACKET_SIZE; i++)
  {
    if(j < fw_len)
    {
      packet[i] = (uint8_t) fw_string[j];
      j++;
    }
    else
    {
      packet[i] = 0x00;
    }
  } 
  
  USB_SendPacket(packet,USBP_SEND_VERSION_PACKET_SIZE);
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

int USBP_Post_EventFromISR(USBP_T *me, USBP_EVENT_T *usbp_e)
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
  
  return USBP_Post_EventFromISR(&usbp, &event);
  
}


