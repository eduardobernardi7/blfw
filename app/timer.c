#include "timer.h"


tim_capture     capture_ch2;
tim_ir_gen      ir_gen;

void TIMER_CaptureStructInit(void);

void TIMER_DemoIrGenStructInit()
{
  ir_gen.iterator = 0;
  ir_gen.n_pulses = 3;
  ir_gen.pulses[0] = 10000;
  ir_gen.pulses[1] = 20000;
  ir_gen.pulses[2] = 5000;
}

tim_ir_gen * TIMER_GetIrGenStruct()
{
  return &ir_gen;
}

tim_capture * TIMER_GetCaptureCh2Struct()
{
  return &capture_ch2;
}

void TIMER_CaptureStructInit()
{
  capture_ch2.capture[0] = 0x00;
  capture_ch2.capture[1] = 0x00;
  capture_ch2.cap_flag = 0x00;
  capture_ch2.pulse_len = 0x00;  
}

void TIMER_InputCaptureCh2Init()
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  TIM_ICInitTypeDef  TIM_ICInitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
  
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOE, &GPIO_InitStructure);
  
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1);
  
  NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  /* Lembre-se que o TIM1 esta com o clock de 120 MHz e o timer estoura em 0xFFFF*/
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
  
  TIM_ICInit(TIM1, &TIM_ICInitStructure);
  
  TIM_Cmd(TIM1, ENABLE);
  
  TIM_ITConfig(TIM1, TIM_IT_CC2, ENABLE);
  
  TIMER_CaptureStructInit(); 
  
  
}

void TIMER_OutputCompareCh3Init()
{
  uint16_t PrescalerValue = 0;
  
  GPIO_InitTypeDef              GPIO_InitStructure;
  NVIC_InitTypeDef              NVIC_InitStructure;
  TIM_TimeBaseInitTypeDef       TIM_TimeBaseStructure;
  TIM_OCInitTypeDef             TIM_OCInitStructure;
  
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOE, &GPIO_InitStructure); 
  
  GPIOE->ODR &= ~GPIO_Pin_12;

  NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  /* Como o TIM1 pode ser clockeado com 120 Mhz, deixa o prescaler em 0 mesmo */
  PrescalerValue = (uint16_t) ((SystemCoreClock) / 120000000) - 1;

  TIM_TimeBaseStructure.TIM_Period = 0xFFFF; // 16 bits de resolucao
  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; // 0 - 65535
  
  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
  
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  
  TIM_OC3Init(TIM1, &TIM_OCInitStructure);
  
  TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Disable);
  
  TIM_Cmd(TIM1, ENABLE);

  TIM_ITConfig(TIM1, TIM_IT_CC3, ENABLE);
  
}

