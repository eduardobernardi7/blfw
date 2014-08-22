// dynload.c
// Source file for Dynamic Load module

#include "dynload.h"
#include "dac.h"
#include "pid.h"
#include "adc12.h"
#include <stdint.h>

#include "fsm.h"
#include "ihm.h"


//FreeRtos
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

//definitions for DL task
#define DL_STACK_SIZE		        configMINIMAL_STACK_SIZE + 256
#define DL_TASK_PRIORITY		( tskIDLE_PRIORITY + 5 )

#define DL_EVENT_LIST_SIZE              128

#define DL_TICK_TIME                    5

typedef struct DL_PROBE_TAG
{
  uint16_t measured_voltage;
  uint16_t correction;
  uint16_t dac_val;  
  int probe_locked; 
}DL_PROBE_T;

// Event type, parameters can be added after super
typedef struct DL_EVENT_TAG
{
  FSM_Event super;
  uint16_t  desired_current_ma;
} DL_EVENT_T;

typedef struct DL_TAG
{
  FSM super;
  SemaphoreHandle_t     tick_mutex;
  uint32_t              timeout;
  xQueueHandle          events;
  xTaskHandle           task_handle;
  PID_T                 pid;
  DL_PROBE_T            probe;
  uint8_t (*completeCallBack)(uint16_t);
}DL_T;

enum DL_SIGNALS
{ 
  DL_START_PID = FSM_USER_SIGNAL,
  DL_TIMEOUT,
  DL_STOP_PID,
};

DL_T dl;

//private methods
void DL_TaskCreate(void);
static void vDLTask( void *pvParameters );
void DL_PIDProcess(void);
uint8_t DL_SetTimeout(uint32_t timeout);
void DL_SetCurrent(uint16_t set_current_in_ma);

//FSM post,process
int DL_Post_Event(DL_T *me, DL_EVENT_T *dl_e);
int DL_Post_EventFromISR(DL_T *me, DL_EVENT_T *dl_e);
void DL_Process(void);

// FSM states
FSM_State DL_HAND_IDLE(DL_T *me, FSM_Event *e);
FSM_State DL_HAND_PID(DL_T *me, FSM_Event *e);


FSM_State DL_HAND_IDLE(DL_T *me, FSM_Event *e)
{
  DL_EVENT_T * event = (DL_EVENT_T *) e;
  
  switch(event->super.signal)
  {
  case FSM_ENTRY_SIGNAL:
    return FSM_HANDLED();  
    
  case DL_START_PID:
    DL_SetCurrent(event->desired_current_ma);
    return FSM_TRAN(me,DL_HAND_PID);
    break;    
    
  case FSM_EXIT_SIGNAL:
    return FSM_HANDLED();  
  }
  
  return FSM_HANDLED();
}

FSM_State DL_HAND_PID(DL_T *me, FSM_Event *e)
{
  DL_EVENT_T * event = (DL_EVENT_T *) e;
  
  switch(event->super.signal)
  {
  case FSM_ENTRY_SIGNAL:
    DL_SetTimeout(DL_TICK_TIME);
    return FSM_HANDLED();  
    
  case DL_TIMEOUT:
    DL_PIDProcess();
    DL_SetTimeout(DL_TICK_TIME);
    IHM_SetLed(USER_LED3, TOG);
    break;    
    
  case DL_STOP_PID:
    return FSM_TRAN(me,DL_HAND_IDLE);    
    break;
    
  case FSM_EXIT_SIGNAL:
    IHM_SetLed(USER_LED3, LOW);
    return FSM_HANDLED();  
  }
  
  return FSM_HANDLED();  
}

int DL_Post_Event(DL_T *me, DL_EVENT_T *dl_e)
{
  if( xQueueSend(me->events, (void *) dl_e, (TickType_t) 100) == pdPASS)
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

int DL_Post_EventFromISR(DL_T *me, DL_EVENT_T *dl_e)
{
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
  
  if(xQueueSendFromISR(me->events, dl_e, &xHigherPriorityTaskWoken ) == pdTRUE)
  {    
    portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
    
    return 1;
  }
  else
  {
    return 0;
  }  
}

void DL_Process()
{
  DL_EVENT_T evt;
  
  if(xQueueReceive(dl.events, &evt, (TickType_t) 100) == pdTRUE)
  {
    FSM_Dispatch (&dl.super, &evt.super);       
  }
}

void DL_Init()
{
  PID_Init(&dl.pid, DL_PID_KP, DL_PID_KI, DL_PID_KD);
  
  //Signal and data queue
  dl.events = xQueueCreate(DL_EVENT_LIST_SIZE, sizeof(DL_EVENT_T));  
  
  //tick mutex
  dl.tick_mutex = xSemaphoreCreateMutex();
  
  //timeout for timing apps
  DL_SetTimeout(0);
  
  // FSM Constructor
  FSM_Ctor(&dl.super,DL_HAND_IDLE);
  
  // FSM Init
  FSM_Init(&dl.super);   
  
  //callback
  dl.completeCallBack = NULL;
  
  DL_TaskCreate();  
}

uint8_t DL_SetTimeout(uint32_t timeout)
{
  if(xSemaphoreTake(dl.tick_mutex, (TickType_t) 100) == pdFALSE)
  {
    return 0;
  }
  else
  {
    dl.timeout = timeout;
    
    xSemaphoreGive(dl.tick_mutex);
    return 1;
  }
}

void DL_TickFromISR()
{
  portBASE_TYPE TaskWokenTake;
  portBASE_TYPE TaskWokenGive;
  DL_EVENT_T    event;
  
  if (xSemaphoreTakeFromISR(dl.tick_mutex, &TaskWokenTake) == pdTRUE)
  {
    if(dl.timeout != 0)
    {
      dl.timeout -= 1;
      
      if(dl.timeout == 0)
      {
        event.super.signal = DL_TIMEOUT;
        DL_Post_EventFromISR(&dl, &event);
      }
    }
    
    xSemaphoreGiveFromISR(dl.tick_mutex, &TaskWokenGive);
    
    portEND_SWITCHING_ISR(TaskWokenGive);   
  }
}

// Set the current to draw with electronic load
void DL_SetCurrent(uint16_t set_current_in_ma)
{    
  uint16_t target_voltage = (uint16_t) (set_current_in_ma * DL_CURRENT_RESISTOR);
  
  PID_SetRefValue(&dl.pid, (int16_t) target_voltage); 
}

void DL_PIDProcess()
{
  uint16_t measured_voltage = dl.probe.measured_voltage = ADC12_GetOutputBufferSample(DL_ADC_ISENSE_LB1);
  
  int16_t correction = dl.probe.correction = PID_Process(&dl.pid, dl.probe.measured_voltage);
  
  uint16_t dac_val = DAC_DacValToMilivolts(DAC_GetDataOutputValue(DAC_Channel_1));
  
  if (dac_val + correction < 0)
  {
    dac_val = 0;
  }
  else
  {
    dac_val += correction;
  }
  
  
  dl.probe.dac_val = dac_val;  
  
  dl.probe.probe_locked = PID_LockStatus(&dl.pid);
  
  DAC_SetDACValInMilivolts(DAC_Channel_1,dac_val);
}

void DL_DetCompleteCallbBack(uint8_t (*completeCallBack)(uint16_t))
{
  if(completeCallBack != NULL)
  {
    dl.completeCallBack = completeCallBack;
  }
}

void DL_TaskCreate()
{
  xTaskCreate(vDLTask, "DL", DL_STACK_SIZE, NULL, DL_TASK_PRIORITY, &dl.task_handle);
}

static void vDLTask( void *pvParameters )
{
  for(;;)
  {
    DL_Process();
  }
}

void DL_StartPID(uint16_t desired_current_ma)
{
  DL_EVENT_T event;
  
  event.desired_current_ma = desired_current_ma;
  event.super.signal = DL_START_PID;
  
  DL_Post_Event(&dl, &event);    
}

void DL_StopPID()
{
  DL_EVENT_T event;
  
  event.super.signal = DL_STOP_PID;
  
  DL_Post_Event(&dl, &event);  
  
}
