#include "adc12.h"
#include "utils.h"
#include "dac.h"

#include "usbd_cdc_vcp.h"

#include "stdio.h"
#include "string.h"
#include "intrinsics.h"


/* NUM OF CHANNELS definition */
#define ADC12_NUM_OF_CHANNELS   6  

/* endereco do reg ADC1 pg51 e 249 do reference manual ! */
#define ADC_CDR_ADDRESS         ((uint32_t)0x4001204C) 

/* Buffer length definitions */
#define MEASURE_BUF_SIZE 512

/* Time interval for reporting all measures */
#define MEASURE_REPORT_TIME_MS  1000

/* Private types */
typedef struct measure_s
{
  uint16_t vsense1[MEASURE_BUF_SIZE];
  uint16_t isense1[MEASURE_BUF_SIZE];
  int16_t write_index;
  int16_t read_index;  
  
  uint32_t vsense1_last_result;
  uint32_t isense1_last_result;
}measure_t;

/* Private attributes */
uint8_t adc12_rdy = 0;

uint8_t dac_tog = 0;

measure_t measure;

static __IO uint32_t adc12_1ms_tick;

__IO    uint16_t adc12buf[ADC12_NUM_OF_CHANNELS];


/* Private methods */
void ADC12_InitMeasureStruct(measure_t * _measure);
void ADC12_Measure2USB(measure_t * _measure);
void ADC12_DacTog(void);


void ADC12_InitMeasureStruct(measure_t * _measure)
{
  uint32_t i;
  
  for(i = 0; i < MEASURE_BUF_SIZE; i++)
  {
    _measure->isense1[i] = 0x00;    
  }
  
  for(i = 0; i < MEASURE_BUF_SIZE; i++)
  {
    _measure->vsense1[i] = 0x00;
  }
  
  _measure->write_index = _measure->read_index = 0;
  
}

void ADC12_Measure2USB(measure_t * _measure)
{
  char usb_string[100];
  
  memset(usb_string, 0, sizeof(usb_string));  
  
  sprintf(usb_string, "ISENSE1 -> %d \r\nVSENSE1 -> %d \r\n", _measure->isense1_last_result, _measure->vsense1_last_result);
  
  USB_SendString(usb_string);  
}

void ADC12_DacTog(void)
{
  dac_tog ^= 0x01;
  
  if(dac_tog)
  {
    DAC_SetVctr(0,124);    
  }
  else
  {
    DAC_SetVctr(0,372); 
  }  
}

void ADC12_Init(void)
{
  ADC_InitTypeDef       ADC_InitStructure;
  ADC_CommonInitTypeDef ADC_CommonInitStructure;
  DMA_InitTypeDef       DMA_InitStructure;
  GPIO_InitTypeDef      GPIO_InitStructure;
  NVIC_InitTypeDef      NVIC_InitStructure;
  
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2 | RCC_AHB1Periph_GPIOC, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_2 | GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  
  DMA_InitStructure.DMA_Channel = DMA_Channel_0;  
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC_CDR_ADDRESS;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&adc12buf;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize = ADC12_NUM_OF_CHANNELS;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA2_Stream0, &DMA_InitStructure);
  
  
  DMA_ITConfig(DMA2_Stream0, DMA_IT_TC, ENABLE);
  
  NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream0_IRQn ;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);  
  
  DMA_Cmd(DMA2_Stream0, ENABLE);  
  
  /*  5cycles delay - 6Msps (Ui) */
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_20Cycles; //5
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;  
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2; 
  ADC_CommonInit(&ADC_CommonInitStructure);
  
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfConversion = ADC12_NUM_OF_CHANNELS;
  ADC_Init(ADC1, &ADC_InitStructure);
  
  /* ADC 10,12,13,VBAT,VREF e VTEMP */ 
  ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_3Cycles);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 2, ADC_SampleTime_3Cycles);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 3, ADC_SampleTime_3Cycles);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_Vbat, 4, ADC_SampleTime_3Cycles);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_Vrefint, 5, ADC_SampleTime_3Cycles);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_TempSensor, 6, ADC_SampleTime_3Cycles);
  
  ADC_TempSensorVrefintCmd(ENABLE);
  ADC_VBATCmd(ENABLE); 
  
  ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE); // single adc, so vamos utilizar o ADC1
  //ADC_MultiModeDMARequestAfterLastTransferCmd(ENABLE); //multi adc
  
  ADC_DMACmd(ADC1, ENABLE);
  
  ADC_Cmd(ADC1, ENABLE);
  
  ADC_SoftwareStartConv(ADC1);
  
  ADC12_InitMeasureStruct(&measure);
  
  adc12_1ms_tick = MEASURE_REPORT_TIME_MS;
  
  adc12_rdy = 1;
}

void ADC12_Tick(void)
{
  if(adc12_1ms_tick != 0x00)
  {
    adc12_1ms_tick -= 1;
  }
  else
  {
    adc12_1ms_tick = MEASURE_REPORT_TIME_MS;
    if(adc12_rdy)
    {
      ADC12_DacTog();
      DMA_ITConfig(DMA2_Stream0, DMA_IT_TC, DISABLE); 
      ADC12_Measure2USB(&measure); 
      DMA_ITConfig(DMA2_Stream0, DMA_IT_TC, ENABLE);
    }
  }  
}

void ADC12_HandleData(void)
{
  uint32_t i;
  
  measure.isense1[measure.write_index] = adc12buf[1];
  measure.vsense1[measure.write_index] = adc12buf[0];
  measure.write_index = circindex(measure.write_index, 1, MEASURE_BUF_SIZE);  
  
  if(measure.write_index == 0)
  {
    measure.isense1_last_result = measure.vsense1_last_result = 0;
    
    for(i = 0; i < MEASURE_BUF_SIZE; i++)
    {
      measure.isense1_last_result += measure.isense1[i];
      measure.vsense1_last_result += measure.vsense1[i];      
    }
    
    measure.isense1_last_result /= MEASURE_BUF_SIZE;
    measure.vsense1_last_result /= MEASURE_BUF_SIZE;
  }
  
}


