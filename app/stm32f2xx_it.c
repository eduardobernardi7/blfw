/**
******************************************************************************
* @file    Project/STM32F2xx_StdPeriph_Template/stm32f2xx_it.c 
* @author  MCD Application Team
* @version V1.1.3
* @date    05-March-2012
* @brief   Main Interrupt Service Routines.
*          This file provides template for all exceptions handler and 
*          peripherals interrupt service routine.
******************************************************************************
* @attention
*
* <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
*
* Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
* You may not use this file except in compliance with the License.
* You may obtain a copy of the License at:
*
*        http://www.st.com/software_license_agreement_liberty_v2
*
* Unless required by applicable law or agreed to in writing, software 
* distributed under the License is distributed on an "AS IS" BASIS, 
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*
******************************************************************************
*/ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f2xx_it.h"
#include "stm322xg_eval_sdio_sd.h"
#include "systick.h"
#include "timer.h"
#include "adc12.h"
#include "IV.h"
#include "ivs.h"
#include "ihm.h"
#include "dynload.h"
#include "uart.h"

/* USB */
#include "usb_core.h"
#include "usbd_core.h"
#include "usbd_cdc_core.h"

/* FreeRtos */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
extern USB_OTG_CORE_HANDLE           USB_OTG_dev;
extern uint32_t USBD_OTG_ISR_Handler (USB_OTG_CORE_HANDLE *pdev);

#ifdef USB_OTG_HS_DEDICATED_EP1_ENABLED 
extern uint32_t USBD_OTG_EP1IN_ISR_Handler (USB_OTG_CORE_HANDLE *pdev);
extern uint32_t USBD_OTG_EP1OUT_ISR_Handler (USB_OTG_CORE_HANDLE *pdev);
#endif



/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/** @addtogroup Template_Project
* @{
*/

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
__IO uint32_t capture;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
* @brief   This function handles NMI exception.
* @param  None
* @retval None
*/
void NMI_Handler(void)
{
}

/**
* @brief  This function handles Hard Fault exception.
* @param  None
* @retval None
*/
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
* @brief  This function handles Memory Manage exception.
* @param  None
* @retval None
*/
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
* @brief  This function handles Bus Fault exception.
* @param  None
* @retval None
*/
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
* @brief  This function handles Usage Fault exception.
* @param  None
* @retval None
*/
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
* @brief  This function handles SVCall exception.
* @param  None
* @retval None
*/
void SVC_Handler(void)
{
}

/**
* @brief  This function handles Debug Monitor exception.
* @param  None
* @retval None
*/
void DebugMon_Handler(void)
{
}

/**
* @brief  This function handles PendSVC exception.
* @param  None
* @retval None
*/
void PendSV_Handler(void)
{
}

/* Somente para Testes de modos Capture e Compare */
void TIM1_CC_IRQHandler(void)
{ 
  tim_capture * capture_p;
  tim_ir_gen * ir_p;
  
  capture_p = TIMER_GetCaptureCh2Struct(); // estrura auxiliar para medicao de largura de pulso
  ir_p = TIMER_GetIrGenStruct(); // estrutura auxiliar para geracao de pulso
  
  /* Geracao de pulso */
  if (TIM_GetITStatus(TIM1, TIM_IT_CC3) == SET)
  {    
    TIM_ClearITPendingBit(TIM1, TIM_IT_CC3 );
    capture = TIM_GetCapture3(TIM1);
    TIM_SetCompare3(TIM1, capture + ir_p->pulses[ir_p->iterator]);
    GPIOE->ODR ^= GPIO_Pin_12;    
    
    
    ir_p->iterator += 1;
    if(ir_p->iterator >= ir_p->n_pulses)
    {
      ir_p->iterator = 0;
    }
  }  
  
  /* Medicao de largura de pulso */
  if(TIM_GetITStatus(TIM1, TIM_IT_CC2) == SET) 
  {      
    TIM_ClearITPendingBit(TIM1, TIM_IT_CC2);   
    if(capture_p->cap_flag == 0)
    {
      capture_p->capture[0] = TIM_GetCapture2(TIM1);
      capture_p->cap_flag = 0x01;
    }
    else if(capture_p->cap_flag == 1)
    {
      capture_p->capture[1] = TIM_GetCapture2(TIM1);
      
      if (capture_p->capture[1] > capture_p->capture[0])
      {
        capture_p->pulse_len = (capture_p->capture[1] - capture_p->capture[0]); 
      }
      else if (capture_p->capture[1] < capture_p->capture[0])
      {
        capture_p->pulse_len = ((0xFFFF - capture_p->capture[0]) + capture_p->capture[1]); 
      }
      else
      {
        capture_p->pulse_len = 0;
      }
      capture_p->cap_flag = 0x00;
      capture_p->freq = (uint32_t) SystemCoreClock / capture_p->pulse_len;
    }
    
    
  }
  
}

//I used time8 as output compare timer, the counter counts until 0xFFFF,
//but every time it reaches the compare value a interrupt is generated
// in this way i have a very accurate timing base
void TIM8_CC_IRQHandler(void)
{
  __IO uint32_t current_capture;
  
  if (TIM_GetITStatus(TIM8, TIM_IT_CC2) == SET)
  { 
    TIM_ClearITPendingBit(TIM8, TIM_IT_CC2 );
    current_capture = TIM_GetCapture2(TIM8);
    //tim8 clock 600 Khz, to reach 10Khz we need 60 ticks !
    TIM_SetCompare2(TIM8, current_capture + 60); // every interrupt on 60 ticks  
    
    TIM8_tick(); 
    IVS_TickFromISR(); 
    DL_TickFromISR();    
    
  }
  
}

/* IHM - PUSH BUTTONS */
void EXTI9_5_IRQHandler(void)
{
  PB_T pb_flag;
  xQueueHandle * pb_queue;
  portBASE_TYPE TaskWokenByPost;
  
  TaskWokenByPost = pdFALSE;
  
  pb_queue = IHM_GetPBQueuePointer();
  
  if(EXTI_GetITStatus(EXTI_Line5) != RESET)
  {    
    pb_flag = USER_B1;
    
    TaskWokenByPost = xQueueSendFromISR( *pb_queue, &pb_flag, &TaskWokenByPost );
    
    EXTI_ClearITPendingBit(EXTI_Line5);
  } 
  
  if(EXTI_GetITStatus(EXTI_Line6) != RESET)
  {    
    pb_flag = USER_B2;
    
    TaskWokenByPost = xQueueSendFromISR( *pb_queue, &pb_flag, &TaskWokenByPost );
    
    EXTI_ClearITPendingBit(EXTI_Line6);
  }  
  
  portEND_SWITCHING_ISR(TaskWokenByPost);
}

/* DMA ADC */
void DMA2_Stream0_IRQHandler(void)
{
  if (DMA_GetITStatus(DMA2_Stream0, DMA_IT_TCIF0) != RESET)
  {
#ifndef ADC12_IIR_DISABLED
    ADC12_FilterDMASamples(); 
#endif
    DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_TCIF0);
  }
}

/* RTC ALARM  - NOT WORKING */
void RTC_Alarm_IRQHandler(void)
{
  if(RTC_GetITStatus(RTC_IT_ALRA) != RESET)
  {
    RTC_ClearITPendingBit(RTC_IT_ALRA);
  }  
}

/* RTC WakeUp Counter - NOT WORKING */
void RTC_WKUP_IRQHandler(void)
{
  if(RTC_GetITStatus(RTC_IT_WUT) != RESET)
  {
    RTC_ClearITPendingBit(RTC_IT_WUT);
  }
}

/* SDIO */

void SDIO_IRQHandler(void)
{
  /* Process All SDIO Interrupt Sources */
  SD_ProcessIRQSrc();
}

void SD_SDIO_DMA_IRQHANDLER(void)
{
  /* Process DMA2 Stream3 or DMA2 Stream6 Interrupt Sources */
  SD_ProcessDMAIRQ();
}

/* USART 2 */
void USART2_IRQHandler(void)
{
  uint16_t rx_data, tx_data;
  
  if (USART_GetITStatus(USART2, USART_IT_RXNE) == SET)
  {
    rx_data = (uint8_t) USART_ReceiveData(USART2);  
    
    UART_Post_RxDataFromISR(rx_data);
  }
  
  if (USART_GetITStatus(USART2, USART_IT_TXE) == SET)
  {
    if(UART_Get_TxDataFromISR(&tx_data))
    {
      USART_SendData(USART2, tx_data);
    }
    else
    {
      USART_ITConfig (USART2, USART_IT_TXE , DISABLE);
    }
  }
}


/* USB */


/**
* @brief  This function handles EXTI15_10_IRQ Handler.
* @param  None
* @retval None
*/
#ifdef USE_USB_OTG_FS  
void OTG_FS_WKUP_IRQHandler(void)
{
  if(USB_OTG_dev.cfg.low_power)
  {
    *(uint32_t *)(0xE000ED10) &= 0xFFFFFFF9 ; 
    SystemInit();
    USB_OTG_UngateClock(&USB_OTG_dev);
  }
  EXTI_ClearITPendingBit(EXTI_Line18);
}
#endif

/**
* @brief  This function handles EXTI15_10_IRQ Handler.
* @param  None
* @retval None
*/
#ifdef USE_USB_OTG_HS  
void OTG_HS_WKUP_IRQHandler(void)
{
  if(USB_OTG_dev.cfg.low_power)
  {
    *(uint32_t *)(0xE000ED10) &= 0xFFFFFFF9 ; 
    SystemInit();
    USB_OTG_UngateClock(&USB_OTG_dev);
  }
  EXTI_ClearITPendingBit(EXTI_Line20);
}
#endif

/**
* @brief  This function handles OTG_HS Handler.
* @param  None
* @retval None
*/
#ifdef USE_USB_OTG_HS  
void OTG_HS_IRQHandler(void)
#else
void OTG_FS_IRQHandler(void)
#endif
{
  USBD_OTG_ISR_Handler (&USB_OTG_dev);
}

#ifdef USB_OTG_HS_DEDICATED_EP1_ENABLED 
/**
* @brief  This function handles EP1_IN Handler.
* @param  None
* @retval None
*/
void OTG_HS_EP1_IN_IRQHandler(void)
{
  USBD_OTG_EP1IN_ISR_Handler (&USB_OTG_dev);
}

/**
* @brief  This function handles EP1_OUT Handler.
* @param  None
* @retval None
*/
void OTG_HS_EP1_OUT_IRQHandler(void)
{
  USBD_OTG_EP1OUT_ISR_Handler (&USB_OTG_dev);
}
#endif

/******************************************************************************/
/*                 STM32F2xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f2xx.s).                                               */
/******************************************************************************/

/**
* @brief  This function handles PPP interrupt request.
* @param  None
* @retval None
*/
/*void PPP_IRQHandler(void)
{
}*/

/**
* @}
*/ 


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
