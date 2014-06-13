#include "dac.h"


void DAC_HwInit()
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  DAC_InitTypeDef   DAC_InitStructure;
  
  /* Clocks para DAC e GPIO correspondente */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  /* Inicializando DAC, canais 1 e 2, modo trig. software, sincronizados ! */
  DAC_InitStructure.DAC_Trigger = DAC_Trigger_None;
  DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;
  DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Enable;
  
  DAC_Init(DAC_Channel_1, &DAC_InitStructure);
  DAC_Init(DAC_Channel_2, &DAC_InitStructure);
  
  DAC_Cmd(DAC_Channel_1, ENABLE);
  DAC_Cmd(DAC_Channel_2, ENABLE);
  
  /* Para testes - Canal 1(Test probe DRV1) -  aprox 2V, Canal 2 - aprox 1V */
  DAC_SetDualChannelData(DAC_Align_12b_R, 1242, 2484);
  
  
}