#include "stm32f0xx.h"
#include "stm32f0xx_adc.h"
#include "stm32f0_discovery.h"
#include "Twister.h"
#include "setup.h"

MersenneTwister random;

const uint16_t Sine12bit[32] = {
  2047, 2447, 2831, 3185, 3498, 3750, 3939, 4056, 4095, 4056,
  3939, 3750, 3495, 3185, 2831, 2447, 2047, 1647, 1263, 909, 
  599, 344, 155, 38, 0, 38, 155, 344, 599, 909, 1263, 1647
};

__IO uint32_t TempSensVoltmv = 0, VrefIntVoltmv = 0;
__IO uint16_t RegularConvData_Tab[2];

#define DAC_DHR12R1_ADDRESS      0x40007408
#define DAC_DHR8R1_ADDRESS       0x40007410

void setup(){
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

  random.seed(5489);
  ADC_Config();
  DAC_Config();

  /* TIM2 Periph clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  
  /* Time base configuration */
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure); 
  TIM_TimeBaseStructure.TIM_Period = 0xFF;          
  TIM_TimeBaseStructure.TIM_Prescaler = 0x0;       
  TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;    
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

  /* TIM2 TRGO selection */
  TIM_SelectOutputTrigger(TIM2, TIM_TRGOSource_Update);
  
  /* TIM2 enable counter */
  TIM_Cmd(TIM2, ENABLE);
  
  /* Configures Button GPIO and EXTI Line */
  STM_EVAL_PBInit(BUTTON_USER, BUTTON_MODE_EXTI);

  configureSine();
}

void configureSine(){
  /* The sine wave has been selected */
  /* Sine Wave generator ---------------------------------------------*/
  DAC_DeInit(); 

DAC_InitTypeDef            DAC_InitStructure;
DMA_InitTypeDef            DMA_InitStructure;
          
  /* DAC channel1 Configuration */
  DAC_InitStructure.DAC_Trigger = DAC_Trigger_T2_TRGO;
  DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Enable;
          
  /* DMA channel3 Configuration */
  DMA_DeInit(DMA1_Channel3); 
  DMA_InitStructure.DMA_PeripheralBaseAddr = DAC_DHR12R1_ADDRESS;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&Sine12bit;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
  DMA_InitStructure.DMA_BufferSize = 32;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel3, &DMA_InitStructure);

  /* Enable DMA1 Channel3 */
  DMA_Cmd(DMA1_Channel3, ENABLE);

  /* DAC Channel1 Init */
  DAC_Init(DAC_Channel_1, &DAC_InitStructure);

  /* Enable DAC Channel1: Once the DAC channel1 is enabled, PA.04 is 
     automatically connected to the DAC converter. */
  DAC_Cmd(DAC_Channel_1, ENABLE);

  /* Enable DMA for DAC Channel1 */
  DAC_DMACmd(DAC_Channel_1, ENABLE);
}

void loop(){
    /* Test DMA1 TC flag */
    while((DMA_GetFlagStatus(DMA1_FLAG_TC1)) == RESET ); 
    
    /* Clear DMA TC flag */
    DMA_ClearFlag(DMA1_FLAG_TC1);
    
    /* Convert temperature sensor voltage value in mv */
    TempSensVoltmv = (uint32_t)((RegularConvData_Tab[0]* 3300) / 0xFFF);
    
    /* Convert Vref voltage value in mv */
    VrefIntVoltmv  = (uint32_t)((RegularConvData_Tab[1]* 3300) / 0xFFF);  
}


#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line){
  for(;;);
}
#endif
