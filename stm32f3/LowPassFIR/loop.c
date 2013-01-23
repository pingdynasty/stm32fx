#include "main.h"

#include <stdbool.h>

/* #define ARM_MATH_CM4 defined in Makefile */
#include "arm_math.h"
/* #include "math_helper.h"  */

__IO uint16_t calibration_value = 0;
static void ADC_Config(void);
static void DAC_Config(void);

/* FIR */
#define BLOCK_SIZE			32 
#define NUM_TAPS			30

/* ------------------------------------------------------------------- 
 * Declare State buffer of size (numTaps + blockSize - 1) 
 * ------------------------------------------------------------------- */ 

q15_t firOutput[BLOCK_SIZE]; 
q15_t firInput[BLOCK_SIZE]; 
q15_t firStateQ15[BLOCK_SIZE + NUM_TAPS - 1]; 
const uint32_t blocksize = BLOCK_SIZE; 

q15_t dacValues[BLOCK_SIZE]; 
q15_t adcValues[BLOCK_SIZE]; 

uint32_t inputCounter = 0;
uint32_t outputCounter = 0;
volatile bool recalculate = false;
volatile bool firDone = false;

arm_fir_instance_q15 S;

/* float32_t firCoeffsF32[NUM_TAPS] = {  */
/* 0.000028,  */
/* 0.000903,  */
/* 0.003915,  */
/* 0.008252,  */
/* 0.010417,  */
/* 0.008252,  */
/* 0.003915,  */
/* 0.000903,  */
/* 0.000028 */
/* };  */

float32_t firCoeffsF32[NUM_TAPS] = {
  -0.0018225230f, -0.0015879294f, +0.0000000000f, +0.0036977508f, +0.0080754303f, +0.0085302217f, -0.0000000000f, -0.0173976984f,
  -0.0341458607f, -0.0333591565f, +0.0000000000f, +0.0676308395f, +0.1522061835f, +0.2229246956f, +0.2504960933f, +0.2229246956f,
  +0.1522061835f, +0.0676308395f, +0.0000000000f, -0.0333591565f, -0.0341458607f, -0.0173976984f, -0.0000000000f, +0.0085302217f,
  +0.0080754303f, +0.0036977508f, +0.0000000000f, -0.0015879294f, -0.0018225230f
  , 0.00f /* pad the filter to
	     the next higher even number of taps by using a zero coefficient for the additional term */
};

q15_t firCoeffsQ15[NUM_TAPS];

void FIR_Config(void){
  arm_float_to_q15(&firCoeffsF32[0], &firCoeffsQ15[0], NUM_TAPS);
  /* Call FIR init function to initialize the instance structure. */
  arm_status status;  
  status = arm_fir_init_q15(&S, NUM_TAPS, &firCoeffsQ15[0], &firStateQ15[0], blocksize); 
  assert_param(status == ARM_MATH_SUCCESS);
}

DAC_InitTypeDef    DAC_InitStructure;
GPIO_InitTypeDef   GPIO_InitStructure;

static void DAC_Config(void)
{

  /* Enable GPIOA clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
  
  /* Configure PA.04 (DAC1_OUT1) in analog mode -------------------------*/
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Enable DAC clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);
  
  /* DAC1 channel1 Configuration */
  DAC_InitStructure.DAC_Trigger = DAC_Trigger_None;
  DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;
  DAC_InitStructure.DAC_LFSRUnmask_TriangleAmplitude = DAC_LFSRUnmask_Bits2_0;
  DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Disable;
  DAC_Init(DAC_Channel_1, &DAC_InitStructure);
  
  /* Enable DAC1 Channel1 */
  DAC_Cmd(DAC_Channel_1, ENABLE);
}

__IO uint16_t CCR1_Val = 54618;
/* __IO uint16_t CCR2_Val = 27309; */
/* __IO uint16_t CCR3_Val = 13654; */
/* __IO uint16_t CCR4_Val = 6826; */

void TIM_Config(){
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef        TIM_OCInitStructure;
  NVIC_InitTypeDef         NVIC_InitStructure;
  uint16_t PrescalerValue = 0;

  /* TIM3 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

  /* Enable the TIM3 gloabal Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Compute the prescaler value */
  PrescalerValue = (uint16_t) ((SystemCoreClock / 2) / 500000) - 1;

  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 65535;
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

  /* Prescaler configuration */
  TIM_PrescalerConfig(TIM3, PrescalerValue, TIM_PSCReloadMode_Immediate);

  /* Output Compare Timing Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR1_Val;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  TIM_OC1Init(TIM3, &TIM_OCInitStructure);

  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Disable);

  /* TIM Interrupts enable */
  TIM_ITConfig(TIM3, TIM_IT_CC1, ENABLE);

  /* TIM3 enable counter */
  TIM_Cmd(TIM3, ENABLE);
}

/**
  * @brief  ADC1 channel1 configuration
  */
static void ADC_Config(void)
{  

  ADC_InitTypeDef       ADC_InitStructure;
  ADC_CommonInitTypeDef ADC_CommonInitStructure;
  GPIO_InitTypeDef      GPIO_InitStructure;
  NVIC_InitTypeDef      NVIC_InitStructure;

  ADC_StopConversion(ADC1);

  /* GPIOC Periph clock enable */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
  
  /* Configure the ADC clock */
  RCC_ADCCLKConfig(RCC_ADC12PLLCLK_Div2);
    
  /* ADC1 Periph clock enable */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ADC12, ENABLE);
  
  /* Setup SysTick Timer for 1 µsec interrupts  */
/*   if (SysTick_Config(SystemCoreClock / 1000000)) */
  /* changed to 10 or 1 ms interrupts */
  SysTick_Config(SystemCoreClock / 100);
  
  /* ADC Channel configuration */
  /* Configure ADC Channel7 (PC1) as analog input */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 ;
  /* changed to channel 6 -> PC0 -> GPIO pin 0 */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
    
  ADC_StructInit(&ADC_InitStructure);
  
  /* Calibration procedure */ 
  ADC_VoltageRegulatorCmd(ADC1, ENABLE);
    
  ADC_SelectCalibrationMode(ADC1, ADC_CalibrationMode_Single);
  ADC_StartCalibration(ADC1);
  
  while(ADC_GetCalibrationStatus(ADC1) != RESET );
  calibration_value = ADC_GetCalibrationValue(ADC1);
  
  /* Configure the ADC1 in continuous mode */  
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;                                                                    
  ADC_CommonInitStructure.ADC_Clock = ADC_Clock_AsynClkMode;                    
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;             
  ADC_CommonInitStructure.ADC_DMAMode = ADC_DMAMode_OneShot;                  
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = 0;          
  
  ADC_CommonInit(ADC1, &ADC_CommonInitStructure);
  
  ADC_InitStructure.ADC_ContinuousConvMode = ADC_ContinuousConvMode_Enable;
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b; 
  ADC_InitStructure.ADC_ExternalTrigConvEvent = ADC_ExternalTrigConvEvent_0;         
  ADC_InitStructure.ADC_ExternalTrigEventEdge = ADC_ExternalTrigInjecConvEvent_0;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_OverrunMode = ADC_OverrunMode_Disable;   
  ADC_InitStructure.ADC_AutoInjMode = ADC_AutoInjec_Disable;  
  ADC_InitStructure.ADC_NbrOfRegChannel = 1;
  ADC_Init(ADC1, &ADC_InitStructure);

  /* ADC1 regular channel7 (PC1) configuration */ 
/*   ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 1, ADC_SampleTime_7Cycles5);   */
/*   changed to channel 6 PC0, ADC12_IN6, sample time 61.5 cycles */
  ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 1, ADC_SampleTime_61Cycles5);

  /* Enable End Of Conversion interrupt */
  ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);  

  /* Configure and enable ADC1 interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);  
  
  /* Enable ADC1 */
  ADC_Cmd(ADC1, ENABLE);
  
  /* wait for ADRDY */
  while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_RDY));
  
  /* ADC1 Start Conversion */ 
  ADC_StartConversion(ADC1);
}

void fir(){
  recalculate = false;
  q15_t *input, *output;
  input = &firInput[0];
  output = &firOutput[0];
  arm_fir_q15(&S, input, output, blocksize);
  firDone = true;
  STM_EVAL_LEDToggle(LED10);
}

void setup(){
  STM_EVAL_LEDInit(LED3);  // red
  STM_EVAL_LEDInit(LED4);  // blue
  STM_EVAL_LEDInit(LED5);  // orange
  STM_EVAL_LEDInit(LED6);  // green left
  STM_EVAL_LEDInit(LED7);  // green right
  STM_EVAL_LEDInit(LED8);  // orange
  STM_EVAL_LEDInit(LED10); // 

  FIR_Config();
  TIM_Config();

  DAC_Config();
  ADC_Config();
}

void loop(){
/*   if(recalculate && !firDone) */
/*     fir(); */
  STM_EVAL_LEDToggle(LED4);
}

void NMI_Handler(void){}
void MemManage_Handler(void)
{ for(;;); }
void BusFault_Handler(void)
{ for(;;); }
void UsageFault_Handler(void)
{ for(;;); }
void WWDG_IRQHandler(void){}
void SVC_Handler(void){}
void DebugMon_Handler(void){}
void PendSV_Handler(void){}
void EXTI0_IRQHandler(void){}
void EXTI1_IRQHandler(void){}
void SysTick_Handler(void){
/*   if(recalculate && !firDone) */
/*     fir(); */
  STM_EVAL_LEDToggle(LED3);
}

uint16_t capture = 0;
void TIM3_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET)
  {
    TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
    STM_EVAL_LEDToggle(LED5);     // appears to be running at 6kHz -> 192kHz sample rate with 7.5 cycle ADC
    // toggles at 2.4kHz, 4.8kHz, 153.6kHz sample rate with 61.5 cycles
    if(recalculate && !firDone)
      fir();
    capture = TIM_GetCapture1(TIM3);
    TIM_SetCompare1(TIM3, capture + CCR1_Val);
  }
}

#define UNIPOLAR_OFFSET 2047
void ADC1_2_IRQHandler(void)
{
  if(ADC_GetITStatus(ADC1, ADC_IT_EOC) != RESET){
    /* Get converted value */
    adcValues[inputCounter] = ADC_GetConversionValue(ADC1) - UNIPOLAR_OFFSET;

    if(++inputCounter == blocksize){
      STM_EVAL_LEDToggle(LED8);
      inputCounter = 0;
      arm_copy_q15(adcValues, firInput, blocksize);
      if(firDone)
	recalculate = true;
      else
	STM_EVAL_LEDToggle(LED7);
    }

    if(++outputCounter == blocksize){
      outputCounter = 0;
      if(firDone){
	arm_copy_q15(firOutput, dacValues, blocksize);
	firDone = false;
      }else{
	STM_EVAL_LEDToggle(LED6);
/* 	arm_copy_q15(firInput, dacValues, blocksize); */
      }
    }

    /* Output converted value on DAC1_OUT1 */
    DAC_SetChannel1Data(DAC_Align_12b_R, dacValues[outputCounter] + UNIPOLAR_OFFSET);

/*     uint32_t ADCVal; */
/*     ADCVal = ADC_GetConversionValue(ADC1); */    
/*     /\* Output converted value on DAC1_OUT1 *\/ */
/*     DAC_SetChannel1Data(DAC_Align_12b_R, ADCVal); */
    
    /* Clear EOC Flag */
    ADC_ClearFlag(ADC1, ADC_FLAG_EOC);
  }
}
