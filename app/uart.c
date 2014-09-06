#include "uart.h"

//lib c
#include "stdio.h"
#include "string.h"

//FreeRtos
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

//definitions for UART task
#define UART_STACK_SIZE		        configMINIMAL_STACK_SIZE + 512
#define UART_TASK_PRIORITY		( tskIDLE_PRIORITY + 3 )

#define TC_DELAY_RATE_BASE	        ( ( TickType_t ) 32 )

#define UART_RX_LIST_SIZE               128
#define UART_TX_LIST_SIZE               128

#define UART_DEFAULT_BAUD_RATE          115200

#define UART_TC_TIMEOUT                 32768

typedef struct TC_CTL_TAG
{
  TickType_t    xDelayRate;
  TickType_t    xLastDelayTime;
  uint32_t      timeout_counter;
  uint8_t       timeout_flag;  
}TC_T;

typedef struct UART_TAG
{
  xQueueHandle  rx_queue;
  xQueueHandle  tx_queue;
  xTaskHandle   task_handle;
  TC_T          tc_control;
}UART_T;



UART_T uart;

static void vUARTTask( void *pvParameters );
void UART_TaskCreate();


void UART2_Init(uint32_t baud_rate);

void UART_TC_CTL_Init(TC_T * tc_ctl);
void UART_TCDelay(TC_T * tc_ctl);
uint8_t UART_TCTimeoutFlagCheck(TC_T * tc_ctl);
void UART_TCTimeoutFlagClear(TC_T * tc_ctl);


void UART_Init()
{
  UART2_Init(UART_DEFAULT_BAUD_RATE); // hw init
  
  UART_TC_CTL_Init(&uart.tc_control); //timings for transmission complete flag check
  
  //queue init
  uart.rx_queue = xQueueCreate(UART_RX_LIST_SIZE, sizeof(uint16_t));
  uart.tx_queue = xQueueCreate(UART_TX_LIST_SIZE, sizeof(uint16_t));
  
  //task init
  UART_TaskCreate();
  
}

void UART_TaskCreate()
{
  xTaskCreate(vUARTTask, "UART", UART_STACK_SIZE, NULL, UART_TASK_PRIORITY, &uart.task_handle);
}

static void vUARTTask( void *pvParameters )
{
  uint16_t data;
  for(;;)
  {
    if(xQueuePeek(uart.tx_queue, &data, (TickType_t) 100) == pdTRUE)
    {
      USART_ITConfig (USART2, USART_IT_TXE , ENABLE);
            
      while ((USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET) && (UART_TCTimeoutFlagCheck(&uart.tc_control) != 1) )
      {
        UART_TCDelay(&uart.tc_control);
      }
      
      if(UART_TCTimeoutFlagCheck(&uart.tc_control) == 1)
      {
        //TODO: Handle timeout in tc flag check
        UART_TCTimeoutFlagClear(&uart.tc_control);
      }  
      
    }    
  }
}



void UART_TCDelay(TC_T * tc_ctl)
{  
  vTaskDelayUntil( &tc_ctl->xLastDelayTime, tc_ctl->xDelayRate );
  
  if(tc_ctl == 0)
  {
    tc_ctl->timeout_counter = UART_TC_TIMEOUT;
    tc_ctl->timeout_flag = 1;
  }
  else
  {
    tc_ctl->timeout_counter -= 1;
  }
  
}

uint8_t UART_TCTimeoutFlagCheck(TC_T * tc_ctl)
{
  return tc_ctl->timeout_flag;
}

void UART_TCTimeoutFlagClear(TC_T * tc_ctl)        
{
  tc_ctl->timeout_flag = 0;
}

void UART_TC_CTL_Init(TC_T * tc_ctl)
{
  tc_ctl->timeout_counter = UART_TC_TIMEOUT;
  tc_ctl->timeout_flag = 0;
  
  tc_ctl->xDelayRate = TC_DELAY_RATE_BASE;
  tc_ctl->xDelayRate /= portTICK_PERIOD_MS;
  
  tc_ctl->xLastDelayTime = xTaskGetTickCount();
}

int UART_Get_RxData(uint16_t * data)
{
  if( xQueueReceive(uart.rx_queue, data, (TickType_t) 100) == pdPASS)
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

int UART_PostTxData(uint16_t * data)
{
  if( xQueueSend(uart.tx_queue, (void *) data, (TickType_t) 100) == pdPASS)
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

int UART_Get_TxDataFromISR(uint16_t * data)
{
  BaseType_t xTaskWokenByReceive = pdFALSE;
  
  if(xQueueReceiveFromISR(uart.tx_queue, (void *) data, &xTaskWokenByReceive) == pdTRUE )
  {
    portEND_SWITCHING_ISR( xTaskWokenByReceive );
    
    return 1;
  }
  else
  {
    return 0;
  }
}

int UART_Post_RxDataFromISR(uint16_t data)
{
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
  
  if(xQueueSendFromISR(uart.rx_queue, &data, &xHigherPriorityTaskWoken ) == pdTRUE)
  {    
    portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
    
    return 1;
  }
  else
  {
    return 0;
  }  
}

void UART2_Init(uint32_t baud_rate)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  
  
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
  
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource3, GPIO_AF_USART2);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource4, GPIO_AF_USART2);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_USART2);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART2);
  
  USART_InitStructure.USART_BaudRate = baud_rate;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_RTS_CTS;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  
  USART_Init(USART2, &USART_InitStructure);
  
  USART_HalfDuplexCmd(USART2, ENABLE);
  
  USART_ITConfig (USART2, USART_IT_RXNE , ENABLE);
  
  
  
  USART_Cmd(USART2, ENABLE); 
  
}


