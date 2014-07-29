//IV.c
// Source file for I-V curve point extraction
// MAH - 07/2014 - Initial Version

#include "fsm.h"
#include "fifo.h"
#include "dac.h"
#include "IV.h"
#include "adc12.h"

//FATFS stack
#include "stm322xg_eval_sdio_sd.h"
#include "ff.h"
#include "diskio.h"
#include "string.h"
#include "stdio.h"

#define IV_CURRENT_RESISTOR         (0.1f)
#define IV_CURRENT_CURVE_STEP       (100)
#define IV_CURRENT_DAC_STEP         ((uint16_t) (IV_CURRENT_RESISTOR*IV_CURRENT_CURVE_STEP))
#define IV_VOLTAGE_DIVIDER_FACTOR   (33.4f)
#define IV_VOLTAGE_SC_TOL           2000//(500)
#define IV_DEFAULT_POINT_DELAY      (5)
#define IV_EVENT_LIST_SIZE          (10)
#define IV_CURVE_SIZE               (ADC12_TYPICAL_VREF / IV_CURRENT_DAC_STEP)

typedef struct IV_FATFS_TAG
{
  FRESULT res;
  FILINFO fno;
  FIL fil;
  DIR dir;
  FATFS fs32;  
}IV_FATFS_T;

// Event type, parameters can be added after super
typedef struct IV_EVENT_TAG
{
    FSM_Event super;
} IV_EVENT_T;

// Event FIFO type
typedef struct IV_EVENT_LIST_TAG
{
    FIFO_T     super;
    IV_EVENT_T list[IV_EVENT_LIST_SIZE];
} IV_EVENT_LIST_T;
 
// IV Point type
typedef struct IV_POINT_TAG
{
    uint16_t v;
    uint16_t i;
    uint16_t correct_i;
} IV_POINT_T;

// IV Curve type
typedef struct IV_CURVE_TAG
{
    FIFO_T     super;
    IV_POINT_T points[IV_CURVE_SIZE];
} IV_CURVE_T;

typedef struct IV_TRACER_TAG
{
    FSM super;
    IV_EVENT_LIST_T events;
    IV_CURVE_T curve;
    IV_POINT_T last_point;
    uint32_t point_delay_ms;
    uint32_t point_delay_counter;
    IV_FATFS_T fatfs;
} IV_TRACER_T;

enum IV_SIGNALS
{ 
    IV_START_NEW_CURVE = FSM_USER_SIGNAL,
    IV_POINT_DELAY_TIMEOUT, 
    IV_SHORT_CIRCUIT,
    IV_DAC_FULL_SCALE
};

static IV_TRACER_T iv_tracer;

FSM_State IV_HAND_INITIAL(IV_TRACER_T *me, FSM_Event *e);
FSM_State IV_HAND_IDLE(IV_TRACER_T *me, FSM_Event *e);
FSM_State IV_HAND_OPER(IV_TRACER_T *me, FSM_Event *e);
int IV_Post_Event(IV_TRACER_T *me, IV_EVENT_T *e);
uint16_t IV_Get_Panel_Voltage(void);
uint16_t IV_Get_Panel_Current(void);
void IV_SetCurrent(uint16_t current_in_ma);
int IV_Curve2File(char * filename, IV_CURVE_T * curve, IV_FATFS_T * fatfs_handle);

// Initial state. Just performs the initial transition
FSM_State IV_HAND_INITIAL(IV_TRACER_T *me, FSM_Event *e)
{
    // Inits Curve FIFO
    FIFO_Init(&me->curve.super, &me->curve.points, IV_CURVE_SIZE, sizeof(IV_POINT_T));
        
    // Goes to IDLE mode
    return FSM_TRAN(me,IV_HAND_IDLE);
}

// Idle, waiting timeout to get the next IV Curve
FSM_State IV_HAND_IDLE(IV_TRACER_T *me, FSM_Event *e)
{
    switch (e->signal)
    {
        case FSM_ENTRY_SIGNAL:
            return FSM_HANDLED();
        case IV_START_NEW_CURVE:
            return FSM_TRAN(me,IV_HAND_OPER);
        case FSM_EXIT_SIGNAL:
            return FSM_HANDLED();   
    }
    
    // Default: Handled
    return FSM_HANDLED();
}

// Operation, getting all points until DAC Full scale or Short Circuit
FSM_State IV_HAND_OPER(IV_TRACER_T *me, FSM_Event *e)
{
    switch (e->signal)
    {
        case FSM_ENTRY_SIGNAL:
            // Sets the point delay for first time
            me->point_delay_counter = me->point_delay_ms;
            return FSM_HANDLED();
        case IV_POINT_DELAY_TIMEOUT:
            // Gets Panel Voltage
            me->last_point.v = IV_Get_Panel_Voltage();
            me->last_point.correct_i = IV_Get_Panel_Current();
            // Puts point on curve FIFO
            FIFO_Post(&me->curve.super, &me->last_point);
            // Increments current by a step
            me->last_point.i += IV_CURRENT_CURVE_STEP;
            IV_SetCurrent(me->last_point.i);
            // Sets the point delay
            me->point_delay_counter = me->point_delay_ms;
            return FSM_HANDLED();
        case IV_SHORT_CIRCUIT:
        case IV_DAC_FULL_SCALE:
            // Curve done
            return FSM_TRAN(me,IV_HAND_IDLE);
        case FSM_EXIT_SIGNAL:
            // Set DAC to zero here, to reduce temperature
            me->last_point.i = 0;
            IV_SetCurrent(me->last_point.i);
            IV_Curve2File("CURVA.TXT", &me->curve, &me->fatfs);
            return FSM_HANDLED();   
    }
    // Default: Handled
    return FSM_HANDLED();
}


// Starts a new IV Curve
void IV_Perform_Curve(void)
{
    IV_EVENT_T iv_e;
    
    iv_e.super.signal = IV_START_NEW_CURVE;
    IV_Post_Event(&iv_tracer, &iv_e);  
}

// Gets the panel voltage
uint16_t IV_Get_Panel_Voltage(void)
{
    IV_EVENT_T iv_e;
    
    uint16_t voltage = (uint16_t) (ADC12_GetOutputBufferSample(ADC12_CH1)*IV_VOLTAGE_DIVIDER_FACTOR);
    
    // If panel voltage drops too much, issues short circuit
    if (voltage <= IV_VOLTAGE_SC_TOL)
    {
        iv_e.super.signal = IV_SHORT_CIRCUIT;
        IV_Post_Event(&iv_tracer, &iv_e);
    }
    
    return (voltage); 
}

// Gets the panel voltage
uint16_t IV_Get_Panel_Current(void)
{    
    return ((uint16_t) (ADC12_GetOutputBufferSample(ADC12_CH2)/IV_CURRENT_RESISTOR)); 
}

// Set the current to draw from panel
void IV_SetCurrent(uint16_t current_in_ma)
{
    IV_EVENT_T iv_e;
    
    uint16_t dac_val = ((uint16_t) (current_in_ma * IV_CURRENT_RESISTOR));
    
    if (DAC_SetDAC1ValInMilivolts(dac_val) == DAC_VALUE_OUTSIDE_BOUNDARIES)
    {
        iv_e.super.signal = IV_DAC_FULL_SCALE;
        IV_Post_Event(&iv_tracer, &iv_e);
    }
        
}

// Post an event to the IV Event list
int IV_Post_Event(IV_TRACER_T *me, IV_EVENT_T *iv_e)
{       
    FIFO_T* element_list_as_fifo = (FIFO_T *) &me->events;
    
    int result = FIFO_Post(element_list_as_fifo, iv_e);
    
    return result; 
}

// Initialization function
void IV_Init(void) 
{
    // Initializes point delay in ms
    iv_tracer.point_delay_ms = IV_DEFAULT_POINT_DELAY;
    
    // Initializes FIFO
    FIFO_Init(&iv_tracer.events.super, 
              &iv_tracer.events.list ,
              IV_EVENT_LIST_SIZE, 
              sizeof(IV_EVENT_T));
    
    // FSM Constructor
    FSM_Ctor(&iv_tracer.super,IV_HAND_INITIAL);
    
    // FSM Init
    FSM_Init(&iv_tracer.super);
}

// Event dispatcher for IV Curve tracer
void IV_Process(void)
{
   IV_EVENT_T evt;
  
   if (FIFO_Get(&iv_tracer.events.super, &evt) == FIFO_NOERROR)
       FSM_Dispatch (&iv_tracer.super, &evt.super);  
}

// IV Timertick
void IV_Timertick (void)
{
    IV_EVENT_T iv_e; 

    if(iv_tracer.point_delay_counter > 0)
    {
        iv_tracer.point_delay_counter--;
        if (iv_tracer.point_delay_counter == 0)
        {
            iv_e.super.signal = IV_POINT_DELAY_TIMEOUT;
            IV_Post_Event(&iv_tracer, &iv_e);
        }
    }
}

//TODO: typedef enum for return values
// fatfs_handle is the object responsible for fat32 interface
int IV_Curve2File(char * filename, IV_CURVE_T * curve, IV_FATFS_T * fatfs_handle)
{
  uint32_t i;
  char welcome_string[32];
  char data_string[128];
  UINT BytesWritten;
  
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
  
  fatfs_handle->res = f_open(&fatfs_handle->fil, filename, FA_CREATE_ALWAYS | FA_WRITE);
  
  if (fatfs_handle->res != FR_OK)
  {
    return -2;  // very bad, check if the retarded user inserted the sd card     
  }
  
  f_lseek(&fatfs_handle->fil, (fatfs_handle->fil.fsize)); // EOF please
  
  strcpy(welcome_string, "Starting LOG:\r\n"); 
  
  fatfs_handle->res = f_write(&fatfs_handle->fil, welcome_string, strlen(welcome_string), &BytesWritten);
  
  if (fatfs_handle->res != FR_OK)
  {
    return -3;    
  }  
  
  //Log the whole curve even the empty points, improve this!
  for(i = 0; i < IV_CURVE_SIZE; i++)
  {
    sprintf(data_string, "V[%d] = %d; I[%d] = %d;\r\n", i, curve->points[i].v, i, curve->points[i].correct_i);
    fatfs_handle->res = f_write(&fatfs_handle->fil, data_string, strlen(data_string), &BytesWritten);
  }  
  
  fatfs_handle->res = f_close(&fatfs_handle->fil);
  
  return 1;
}
