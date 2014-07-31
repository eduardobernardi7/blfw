
// ADC12.C
// Source file for ADC routines
// EB - 07/2014 - Initial Version

#include "adc12.h"
#include "timer.h" // freertos owns systick, using tim8 instead

// ADC1 address as stated in RM0033 PG51 and 249
#define ADC_CDR_ADDRESS         ((uint32_t)0x4001204C) 

// ADC DMA Buffer, which contain "Live" samples
static volatile uint16_t adc12_dma_buffer[ADC12_NUM_OF_CHANNELS];

// ADC Output Buffer, that has IIR Filtered samples
static uint16_t adc12_output_buffer[ADC12_NUM_OF_CHANNELS];

static uint16_t adc12_vref = ADC12_TYPICAL_VREF; 

// Set Vref
void ADC12_SetVref(uint16_t val)
{
    adc12_vref = val;
}

// Get Vref
uint16_t ADC12_GetVref(void)
{
    return adc12_vref;
}

// Calibrate Vref to a better value using internal voltage reference
void ADC12_CalibrateVref(void)
{    
    ADC12_SetVref((uint16_t) (ADC12_INTREF_VOLTAGE * ADC12_MAX_VALUE / adc12_output_buffer[ADC12_CH5]));
}

// Get ADC Output Buffer
uint16_t ADC12_GetOutputBufferSample(enum ADC12_CHANNELS const ch)
{
    return ((uint16_t) (adc12_output_buffer[ch]*ADC12_GetVref()/ADC12_MAX_VALUE));
}

// Init structure
void ADC12_Init(void)
{
    // Init structures required to put the ADC to work
    ADC_InitTypeDef       ADC_InitStructure;
    ADC_CommonInitTypeDef ADC_CommonInitStructure;
    DMA_InitTypeDef       DMA_InitStructure;
    GPIO_InitTypeDef      GPIO_InitStructure;
    NVIC_InitTypeDef      NVIC_InitStructure;
    
    // Enables clock to DMA2 and GPIOC through AHB1
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2 | RCC_AHB1Periph_GPIOC, ENABLE);
    
    // Enables clock to ADC1 through APB2
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
  
    // Puts PC pins 0, 2 and 3 in Analog Mode with no pull up resistor
    // PC0 -> ADC123_IN10 - VSENSE
    // PC2 -> ADC123_IN12 - ISENSE for LB1
    // PC3 -> ADC123_IN13 - ISENSE for LB2
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
  
    // Initialize the DMA to get the samples from the ADC
    DMA_InitStructure.DMA_Channel = DMA_Channel_0;  
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) ADC_CDR_ADDRESS;
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t) &adc12_dma_buffer;
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

    // Turn on DMA2 Stream 0 Interrupt
    DMA_ITConfig(DMA2_Stream0, DMA_IT_TC, ENABLE);
  
    // Configure DMA2 Stream 0 IRQ on NVIC
    NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream0_IRQn ;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);  
  
    // Enable DMA2 Stream 0
    DMA_Cmd(DMA2_Stream0, ENABLE);     
    
    // ADC Mode set to independent
    ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
    // Two sampling delay set to 5 Cycles, but this is irrelevant for 
    // independent mode. 
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_20Cycles;
    // Disables DMA Access
    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;  
    // Uses a divide by 2 prescaler, so ADCclk = Pclk/2
    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2; 
    // Initializes ADC Common configs
    ADC_CommonInit(&ADC_CommonInitStructure);
      
    // ADC1 initialization
    
    // 12 bit resolution
    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
    // Multichannel operation
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;
    // Continuous mode
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    // No external trigger
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    // Timer 1 CH1? << I'm not certain about that
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
    // ADC data align right
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    // Converts ADC12_NUM_OF_CHANNELS
    ADC_InitStructure.ADC_NbrOfConversion = ADC12_NUM_OF_CHANNELS;
    // Initializes ADC1 configs
    ADC_Init(ADC1, &ADC_InitStructure);
 
    // Configs each ADC1 Channel individually
    // ADC1 CH 10, 12 and 13 are hooked up to actual inputs
    // The rest is tied to VBAT, VREF and VTEMP
    ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_3Cycles);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 2, ADC_SampleTime_3Cycles);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 3, ADC_SampleTime_3Cycles);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_Vbat, 4, ADC_SampleTime_3Cycles);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_Vrefint, 5, ADC_SampleTime_3Cycles);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_TempSensor, 6, ADC_SampleTime_3Cycles);
  
    // Enables Temp Sensor and VBAT Measurement
    ADC_TempSensorVrefintCmd(ENABLE);
    ADC_VBATCmd(ENABLE); 
  
    // Enables DMA for ADC1
    ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);   
    ADC_DMACmd(ADC1, ENABLE);

    
    // Enables ADC1
    ADC_Cmd(ADC1, ENABLE);
    ADC_SoftwareStartConv(ADC1);
    
    TIM8_delay_ms(1);
    
    // Calibrate Vref
    ADC12_CalibrateVref();

}


// Calculates a simple IIR filtered sample 
uint16_t ADC12_IIRFilterSamples(uint16_t yn_1, uint16_t xn)
{
    uint16_t result = (uint16_t) ((ADC_IIR_C0*yn_1 + ADC_IIR_C1*xn));
    
    return ((result));
}

// Filters the samples for all DMA channels
void ADC12_FilterDMASamples(void)
{
    for (int i=0 ; i < ADC12_NUM_OF_CHANNELS; i++)
    {        
        adc12_output_buffer[i] = ADC12_IIRFilterSamples(adc12_output_buffer[i], 
                                                        adc12_dma_buffer[i]);
    }   
}


void ADC12_Timertic(void)
{
    // I do nothing woohoo!!
}
