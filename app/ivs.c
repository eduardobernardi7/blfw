#include "stm32f2xx.h"
#include "ivs.h"

#include "fsm.h"
#include "fifo.h"
#include "dac.h"
#include "adc12.h"
#include "ihm.h"
#include "blrtc.h"

//FATFS stack
#include "stm322xg_eval_sdio_sd.h"
#include "ff.h"
#include "diskio.h"
#include "string.h"
#include "stdio.h"

//definitions for IVS task
#define IVS_STACK_SIZE		        configMINIMAL_STACK_SIZE + 512
#define IVS_TASK_PRIORITY		( tskIDLE_PRIORITY + 3 )

//definitions for IVS timer task 
#define IVS_TIMER_TASK_STACK_SIZE       configMINIMAL_STACK_SIZE
#define IVS_TIMER_TASK_PRIORITY         tskIDLE_PRIORITY + 2

#define IVS_CURRENT_RESISTOR         (0.1f)
#define IVS_CURRENT_CURVE_STEP       (100)
#define IVS_CURRENT_DAC_STEP         ((uint16_t) (IVS_CURRENT_RESISTOR*IVS_CURRENT_CURVE_STEP))
#define IVS_VOLTAGE_DIVIDER_FACTOR   (33.4f)
#define IVS_VOLTAGE_SC_TOL           2000//(500)
#define IVS_DEFAULT_POINT_DELAY      (5)
#define IVS_DELAY_TICK_BASE          ( ( TickType_t ) 5 )
#define IVS_EVENT_LIST_SIZE          (10)
#define IVS_CURVE_SIZE               (ADC12_TYPICAL_VREF / IVS_CURRENT_DAC_STEP)
#define IVS_MAX_FATFS_ATTEMPT        10

#define IVS_MAX_TIME_HOURS           10      

#define IVS_TIMING_PRESCALER         10

// Continuous curve building status
typedef enum CURVE_STATUS_TAG
{
  CURVE_IDLE,
  CURVE_COMPLETED,
  CURVE_CANCELED,
  CURVE_PENDING
}CURVE_STATUS_T;

// Time intervals for the curve building process
typedef struct IVS_TIME_CONFIG_TAG
{
  uint8_t durationHours;
  uint8_t intervalMins;
  
}IVS_TIME_CONFIG_T;

// FatFs stack essential variables
typedef struct IVS_FATFS_TAG
{
  FRESULT res;
  FILINFO fno;
  FIL fil;
  DIR dir;
  FATFS fs32;  
}IVS_FATFS_T;

// Event type, parameters can be added after super
typedef struct IVS_EVENT_TAG
{
  FSM_Event super;
} IVS_EVENT_T;

// IV Point type
typedef struct IVS_POINT_TAG
{
  uint16_t v;
  uint16_t i;
  uint16_t correct_i;
} IVS_POINT_T;

// IV Curve type
typedef struct IVS_CURVE_TAG
{
  FIFO_T     super;
  IVS_POINT_T points[IVS_CURVE_SIZE];
} IVS_CURVE_T;

// class attributes
typedef struct IVS_TRACER_TAG
{
  FSM super;
  xQueueHandle          events;
  SemaphoreHandle_t     sem_isr_tick;
  IVS_CURVE_T           curve;
  IVS_POINT_T           last_point;
  TickType_t            xPointDelay;
  TickType_t            xPointDelayTime;
  xTaskHandle           task_handle;
  xTaskHandle           timer_task_handle;
  IVS_FATFS_T           fatfs;
  IVS_TIME_CONFIG_T     time_config;
  SemaphoreHandle_t     time_config_mutex;
  RTC_TimeTypeDef       start_time;
  RTC_TimeTypeDef       last_time;
  CURVE_STATUS_T        curve_progress_status;
  uint32_t              timing_prescaler_cnt;
} IVS_TRACER_T;

//Valid signal for the fsm processor
enum IVS_SIGNALS
{ 
  IVS_START_NEW_CURVE = FSM_USER_SIGNAL,
  IVS_POINT_DELAY_TIMEOUT, 
  IVS_SHORT_CIRCUIT,
  IVS_TIMEOUT,
  IVS_NEW_CURVE,
  IVS_NEW_POINT,
  IVS_DAC_FULL_SCALE
};

//Core methods for the fsm
int IVS_Post_Event(IVS_TRACER_T *me, IVS_EVENT_T *iv_e);
int IVS_Post_EventFromISR(IVS_TRACER_T *me, IVS_EVENT_T *iv_e);
void IVS_Process(void);

//Tasks and its constructors methods
void IVS_TaskCreate(void);
void IVS_TimerTaskCreate(void);

//this tasks runs the fsm
static void vIVSTask( void *pvParameters );

//this task implements all timing events for the continuous curve building
static void vIVSTimerTask(void * pvParameters);

void IVS_Request_Point(void);

//main function of the timing task - TimerTask
void IVS_RTCTick(void);

// hardware related methods
void IVS_SetCurrent(uint16_t current_in_ma);
uint16_t IVS_Get_Panel_Voltage(void);
uint16_t IVS_Get_Panel_Current(void);

//aux methods
int IVS_Curve2File(char * filename, IVS_CURVE_T * curve, IVS_FATFS_T * fatfs_handle);
uint32_t IVS_TimeDifSeconds(RTC_TimeTypeDef cur, RTC_TimeTypeDef last);
void time2string(RTC_TimeTypeDef time, char * hold_string);

//states
FSM_State IVS_HAND_IDLE(IVS_TRACER_T *me, FSM_Event *e);
FSM_State IVS_HAND_OPER(IVS_TRACER_T *me, FSM_Event *e);

//locals
static IVS_TRACER_T ivs_tracer;


FSM_State IVS_HAND_IDLE(IVS_TRACER_T *me, FSM_Event *e)
{
  switch (e->signal)
  {
  case FSM_ENTRY_SIGNAL:
    return FSM_HANDLED();    
    
  case  IVS_START_NEW_CURVE:
    FIFO_Init(&me->curve.super, &me->curve.points, IVS_CURVE_SIZE, sizeof(IVS_POINT_T));  
    IVS_Request_Point();
    return FSM_TRAN(me,IVS_HAND_OPER);
    
  case FSM_EXIT_SIGNAL:
    return FSM_HANDLED();  
  }
  
  return FSM_HANDLED();   
}

FSM_State IVS_HAND_OPER(IVS_TRACER_T *me, FSM_Event *e)
{
  switch (e->signal)
  {
  case FSM_ENTRY_SIGNAL:
    return FSM_HANDLED();
  case IVS_NEW_POINT:             
    
    // Gets Panel Measure
    me->last_point.v = IVS_Get_Panel_Voltage();
    me->last_point.correct_i = IVS_Get_Panel_Current();
    
    // Puts point on curve FIFO
    FIFO_Post(&me->curve.super, &me->last_point);
    
    // Increments current by a step
    me->last_point.i += IVS_CURRENT_CURVE_STEP;
    IVS_SetCurrent(me->last_point.i);
    
    //Delay for next point
    vTaskDelayUntil(&me->xPointDelayTime, me->xPointDelay);
    
    IVS_Request_Point(); // next point !                             
    
    return FSM_HANDLED(); 
    
  case IVS_SHORT_CIRCUIT:
  case IVS_DAC_FULL_SCALE:
    // Curve done
    // Set DAC to zero here, to reduce temperature
    me->last_point.i = 0;
    IVS_SetCurrent(me->last_point.i);
    
    vTaskPrioritySet(me->task_handle, IVS_TASK_PRIORITY + 2);
    
    if(IVS_Curve2File("CURVA_S.TXT", &me->curve, &me->fatfs) > 0)
    {
      IHM_SetLed(USER_LED1, HIGH);
      IHM_SetLed(USER_LED2, LOW);
    }
    else
    {
      IHM_SetLed(USER_LED2, HIGH);
      IHM_SetLed(USER_LED1, LOW);
    }
    
    vTaskPrioritySet(me->task_handle, IVS_TASK_PRIORITY);
    
    
    return FSM_TRAN(me,IVS_HAND_IDLE);
    
  case FSM_EXIT_SIGNAL:    
    return FSM_HANDLED();   
  }
  
  // Default: Handled
  return FSM_HANDLED();
}

// Post an event to the IV Event list
int IVS_Post_Event(IVS_TRACER_T *me, IVS_EVENT_T *iv_e)
{
  if( xQueueSend(me->events, (void *) iv_e, (TickType_t) 100) == pdPASS)
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

// Post an event to the IV Event list
int ISV_Post_EventFromISR(IVS_TRACER_T *me, IVS_EVENT_T *iv_e)
{       
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
  
  xQueueSendFromISR(me->events, iv_e, &xHigherPriorityTaskWoken );
  
  portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
  
  return 1;
}


// Event dispatcher for IV Curve tracer
void IVS_Process()
{
  IVS_EVENT_T evt;
  
  if(xQueueReceive(ivs_tracer.events, &evt, (TickType_t) 100) == pdTRUE)
  {
    FSM_Dispatch (&ivs_tracer.super, &evt.super);       
  }
}

void IVS_TaskCreate()
{
  xTaskCreate( vIVSTask, "IVS", IVS_STACK_SIZE, NULL, IVS_TASK_PRIORITY, &ivs_tracer.task_handle);    
}

void IVS_TimerTaskCreate()
{
  xTaskCreate( vIVSTimerTask, "IVS_TIMER", IVS_TIMER_TASK_STACK_SIZE, NULL, IVS_TIMER_TASK_PRIORITY, &ivs_tracer.timer_task_handle); 
}

//ivs_tracer timer task
static void vIVSTimerTask(void * pvParameters)
{
  for(;;)
  {
    if( xSemaphoreTake( ivs_tracer.sem_isr_tick, ( TickType_t ) 100 ) == pdTRUE )
    {
      IVS_RTCTick();              
    }       
  }
}

void IVS_TickFromISR()
{
  portBASE_TYPE TaskWoken;
  
  if(ivs_tracer.timing_prescaler_cnt == 0)
  {    
    xSemaphoreGiveFromISR(ivs_tracer.sem_isr_tick, &TaskWoken );    
    portEND_SWITCHING_ISR(TaskWoken);  
    ivs_tracer.timing_prescaler_cnt = IVS_TIMING_PRESCALER;
  }
  else
  {
    ivs_tracer.timing_prescaler_cnt -= 1;
  }
}

// ivs_tracer task
static void vIVSTask(void *pvParameters)
{  
  //Adjusting delay between two measures
  ivs_tracer.xPointDelay = IVS_DELAY_TICK_BASE;
  ivs_tracer.xPointDelay /= portTICK_PERIOD_MS;
  ivs_tracer.xPointDelayTime = xTaskGetTickCount();
  
  //ADC12_CalibrateVref();
  
  for(;;)
  {
    IVS_Process(); 
  }
}

SemaphoreHandle_t * IVS_GetTimerTickSem()
{
  return &ivs_tracer.sem_isr_tick;
}

SemaphoreHandle_t * IVS_GetTimerMutex()
{
  return &ivs_tracer.time_config_mutex;
}

// Initialization function
void IVS_Init(void) 
{
  // This task will run the ivs fsm machine 
  IVS_TaskCreate();
  
  IVS_TimerTaskCreate();
  
  //Initializes queue
  ivs_tracer.events = xQueueCreate(IVS_EVENT_LIST_SIZE, sizeof(IVS_EVENT_T));  
  
  //Initializes timer tick isr sem
  vSemaphoreCreateBinary(ivs_tracer.sem_isr_tick);
  
  //No pending curves to build, lets wait a external event
  ivs_tracer.curve_progress_status = CURVE_IDLE;
  
  //Initializes mutexes
  ivs_tracer.time_config_mutex = xSemaphoreCreateMutex();
  
  //resets prescaler counter of timing task
  ivs_tracer.timing_prescaler_cnt = IVS_TIMING_PRESCALER;
  
  // FSM Constructor
  FSM_Ctor(&ivs_tracer.super,IVS_HAND_IDLE);
  
  // FSM Init
  FSM_Init(&ivs_tracer.super);   
  
}


// Starts a new IV Curve
void IVS_Perform_Curve(unsigned int periodHours, unsigned int periodIntervalMins)
{
  IVS_EVENT_T ivs_e;
  RTC_TimeTypeDef init_time;
  
  if(periodHours == 0 || periodIntervalMins == 0 || periodHours > IVS_MAX_TIME_HOURS)
  {
    return;
  }
  
  vTaskResume(ivs_tracer.timer_task_handle);
  
  ivs_tracer.curve_progress_status = CURVE_PENDING;
  
  init_time.RTC_H12 = RTC_H12_AM;
  init_time.RTC_Hours = 1;
  init_time.RTC_Minutes = 1;
  init_time.RTC_Seconds = 1;
  
  BLRTC_SetTime(init_time);
  
  ivs_tracer.time_config.durationHours = periodHours;
  ivs_tracer.time_config.intervalMins = periodIntervalMins;
  
  ivs_tracer.start_time = ivs_tracer.last_time = BLRTC_GetTime();  
  
  ivs_e.super.signal = IVS_START_NEW_CURVE;
  
  IVS_Post_Event(&ivs_tracer, &ivs_e); 
}

// Starts a new IV Curve
void IVS_Perform_CurveFromISR(unsigned int periodHours, unsigned int periodIntervalMins)
{
  IVS_EVENT_T ivs_e;
  
  if(periodHours == 0 || periodIntervalMins == 0)
  {
    return;
  }
  
  ivs_tracer.time_config.durationHours = periodHours;
  ivs_tracer.time_config.intervalMins = periodIntervalMins;
  
  ivs_e.super.signal = IVS_START_NEW_CURVE;
  
  IVS_Post_EventFromISR(&ivs_tracer, &ivs_e); 
}

//fornece os eventos de temporizacao para construcao de curvas
//unica rotina executada em loop na task TimerTask
void IVS_RTCTick()
{
  RTC_TimeTypeDef current_time;
  IVS_EVENT_T ivs_e;  
  uint32_t seconds_dif;
  
  
  if(ivs_tracer.curve_progress_status != CURVE_PENDING)
  {
    return;
  }
  
  current_time = BLRTC_GetTime();
  
  if(xSemaphoreTake(ivs_tracer.time_config_mutex, (TickType_t) 100) == pdFALSE)
  {
    return;
  }
  
  if(current_time.RTC_Hours > ivs_tracer.start_time.RTC_Hours + ivs_tracer.time_config.durationHours)
  {
    ivs_tracer.curve_progress_status = CURVE_COMPLETED;
  }  
  else
  {    
    seconds_dif = IVS_TimeDifSeconds(current_time, ivs_tracer.last_time);
    
    if(seconds_dif >= ivs_tracer.time_config.intervalMins)
    {    
      ivs_tracer.last_time = current_time;
      ivs_e.super.signal = IVS_START_NEW_CURVE;
      IVS_Post_Event(&ivs_tracer, &ivs_e);
      
      IHM_SetLed(USER_LED3, TOG);
    }
  }
  
  xSemaphoreGive(ivs_tracer.time_config_mutex);
  
  if(ivs_tracer.curve_progress_status == CURVE_COMPLETED)
  {
    vTaskSuspend(ivs_tracer.timer_task_handle);    
  }
  
}

//apenas retorna a diferenca em segundos entre 2 horarios
uint32_t IVS_TimeDifSeconds(RTC_TimeTypeDef cur, RTC_TimeTypeDef last)
{
  uint32_t sum_cur;
  uint32_t sum_last;
  
  sum_cur = cur.RTC_Hours * 3600 + cur.RTC_Minutes * 60 + cur.RTC_Seconds;
  sum_last = last.RTC_Hours * 3600 + last.RTC_Minutes * 60 + last.RTC_Seconds;
  
  if(sum_cur >= sum_last)
  {
    return sum_cur - sum_last;
  }
  else
  {
    return 0;
  }
  
}

//requests a new point for the current curve
void IVS_Request_Point()
{
  IVS_EVENT_T ivs_e;
  
  ivs_e.super.signal = IVS_NEW_POINT;
  IVS_Post_Event(&ivs_tracer, &ivs_e); 
  
}

// Gets the panel voltage
uint16_t IVS_Get_Panel_Voltage()
{
  IVS_EVENT_T ivs_e;
  
  uint16_t voltage = (uint16_t) (ADC12_GetOutputBufferSample(ADC12_CH1)*IVS_VOLTAGE_DIVIDER_FACTOR);
  
  // If panel voltage drops too much, issues short circuit
  if (voltage <= IVS_VOLTAGE_SC_TOL)
  {
    ivs_e.super.signal = IVS_SHORT_CIRCUIT;
    IVS_Post_Event(&ivs_tracer, &ivs_e);
  }
  
  return (voltage); 
}

// Gets the panel voltage
uint16_t IVS_Get_Panel_Current()
{    
  return ((uint16_t) (ADC12_GetOutputBufferSample(ADC12_CH2)/IVS_CURRENT_RESISTOR)); 
}

// Set the current to draw from panel
void IVS_SetCurrent(uint16_t current_in_ma)
{
  IVS_EVENT_T ivs_e;
  
  uint16_t dac_val = ((uint16_t) (current_in_ma * IVS_CURRENT_RESISTOR));
  
  if (DAC_SetDACValInMilivolts(DAC_Channel_1, dac_val) == DAC_VALUE_OUTSIDE_BOUNDARIES)
  {
    ivs_e.super.signal = IVS_DAC_FULL_SCALE;
    IVS_Post_Event(&ivs_tracer, &ivs_e);
  }
  
}

//TODO: typedef enum for return values
// fatfs_handle is the object responsible for fat32 interface
int IVS_Curve2File(char * filename, IVS_CURVE_T * curve, IVS_FATFS_T * fatfs_handle)
{
  uint32_t i;
  char header[64];
  char data_string[128];
  UINT BytesWritten;
  unsigned int n_attempt;
  
  //check if the caller is a orc
  if(filename == 0)
  {
    return 0;
  }
  
  //check if the requested curve has enough points
  if(curve->super.elements == 0)
  {
    return 0;
  }
  
  SD_InterruptEnable();
  
  memset(&fatfs_handle->fs32, 0, sizeof(FATFS));
  
  fatfs_handle->res = f_mount(0, &fatfs_handle->fs32);
  
  if(fatfs_handle->res != FR_OK)
  {
    return -1;
  }
  
  fatfs_handle->res = f_close(&fatfs_handle->fil);
  
  n_attempt = IVS_MAX_FATFS_ATTEMPT;
  
  do{
    fatfs_handle->res = f_open(&fatfs_handle->fil, filename, FA_CREATE_ALWAYS | FA_WRITE);
    n_attempt -= 1;
  }while(n_attempt > 0 && fatfs_handle->res != FR_OK);
  
  if(n_attempt == 0)
  {
    fatfs_handle->res = f_close(&fatfs_handle->fil);
    return -2;  // very bad, check if the retarded user inserted the sd card
  }
  
  f_lseek(&fatfs_handle->fil, (fatfs_handle->fil.fsize)); // EOF please
  
  time2string(BLRTC_GetTime(), header);
  
  fatfs_handle->res = f_write(&fatfs_handle->fil, header, strlen(header), &BytesWritten);
  
  if (fatfs_handle->res != FR_OK)
  {
    return -3;    
  }  
  
  //Log the whole curve even the empty points, improve this!
  for(i = 0; i < IVS_CURVE_SIZE; i++)
  {
    sprintf(data_string, "V[%d] = %d; I[%d] = %d;\r\n", i, curve->points[i].v, i, curve->points[i].correct_i);
    fatfs_handle->res = f_write(&fatfs_handle->fil, data_string, strlen(data_string), &BytesWritten);
  }  
  
  fatfs_handle->res = f_close(&fatfs_handle->fil);
  
  return 1;
}

void time2string(RTC_TimeTypeDef time, char * hold_string)
{
  if(time.RTC_H12 == RTC_H12_AM)
  {
    sprintf(hold_string, "START LOG - RTC -> %d :: %d :: %d  [AM]\r\n", time.RTC_Hours, time.RTC_Minutes, time.RTC_Seconds);
  }
  else
  {
    sprintf(hold_string, "RTC - %d :: %d :: %d  [PM]\r\n", time.RTC_Hours, time.RTC_Minutes, time.RTC_Seconds);
  }  
}

void IVS_StopCurve()
{
  ivs_tracer.curve_progress_status = CURVE_CANCELED;
  
  // a task de temporizacao nao eh mais necessaria
  vTaskSuspend(ivs_tracer.timer_task_handle); 
  
  IHM_SetLed(USER_LED2, LOW);
  IHM_SetLed(USER_LED1, LOW);
}