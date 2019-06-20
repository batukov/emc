/**
  ******************************************************************************
  * File Name          : ADC.c
  * Description        : This file provides code for the configuration
  *                      of the ADC instances.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "adc.h"
#include "gpio.h"



DMA_HandleTypeDef hdma_adc1;
volatile uint16_t ADC_data[4]= {0,0,0,0}; // DMA-Buffer
volatile uint16_t adc_data[40]; //struct with uint16_t units??

uint16_t avrg_v_1_1 = 0;
uint16_t avrg_v_2_1 = 0;
uint16_t avrg_v_3_1 = 0;
uint16_t avrg_c_1_1 = 0;
uint16_t avrg_c_2_1 = 0;

uint16_t avrg_v_1_2 = 0;
uint16_t avrg_v_2_2 = 0;
uint16_t avrg_v_3_2 = 0;
uint16_t avrg_c_1_2 = 0;
uint16_t avrg_c_2_2 = 0;




ADC_HandleTypeDef hadc1;



/* ADC1 init function */
void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig = {0};

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 6;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }


  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if(HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
      Error_Handler();
  }
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if(HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
      Error_Handler();
  }
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 3;
  if(HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
      Error_Handler();
  }
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 4;
  if(HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
      Error_Handler();
  }
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 5;
  if(HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
      Error_Handler();
  }
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 6;
  if(HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
      Error_Handler();
  }
}

void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(adcHandle->Instance==ADC1)
  {

    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /**ADC1 GPIO Configuration    
    PA0-WKUP     ------> ADC1_IN0
    PA1     ------> ADC1_IN1
    PA2     ------> ADC1_IN2
    PA3     ------> ADC1_IN3 
    */

    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    HAL_DMA_Init(adcHandle->DMA_Handle);



    hdma_adc1.Instance = DMA2_Stream0;
    hdma_adc1.Init.Channel = DMA_CHANNEL_0;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc1.Init.Mode = DMA_CIRCULAR;
    hdma_adc1.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    hdma_adc1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK)
    {
        Error_Handler();
    }

    __HAL_LINKDMA(adcHandle,DMA_Handle,hdma_adc1);
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle)
{

  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspDeInit 0 */

  /* USER CODE END ADC1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC1_CLK_DISABLE();

  
    /**ADC1 GPIO Configuration    
    PA0-WKUP     ------> ADC1_IN0
    PA1     ------> ADC1_IN1
    PA2     ------> ADC1_IN2
    PA3     ------> ADC1_IN3
    PB0     ------> ADC2_IN8
    PB1     ------> ADC1_IN9
    */

    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_0|GPIO_PIN_1);
    /* ADC1 DMA DeInit */
    HAL_DMA_DeInit(adcHandle->DMA_Handle);
  }
}

void MX_DMA_Init(void)
{
    /* DMA controller clock enable */
    __HAL_RCC_DMA2_CLK_ENABLE();

    /* DMA interrupt init */
    /* DMA2_Stream0_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 6, 6);
    HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//#define SAMPLE_RATE 20
//#define NUM_CHANNELS 6
//#define ADCCONVERTEDVALUES_BUFFER_SIZE ((uint32_t)  (SAMPLE_RATE * NUM_CHANNELS * 2))     /* Size of array containing ADC converted values */

//__IO uint16_t	aADCxConvertedValues[ADCCONVERTEDVALUES_BUFFER_SIZE]; /* ADC conversion results table of regular group, channel on rank1 */
//uint16_t        uhADCxConvertedValue_Regular_Avg_half1[NUM_CHANNELS];  /* Average of the 1st half of ADC conversion results table of regular group, channel on rank1 */
//uint16_t        uhADCxConvertedValue_Regular_Avg_half2[NUM_CHANNELS];  /* Average of the 2nd half of ADC conversion results table of regular group, channel on rank1 */
//uint16_t*       puhADCxConvertedValue_Regular_Avg;       /* Pointer to the average of the 1st or 2nd half of ADC conversion results table of regular group, channel on rank1*/
//uint8_t					ADC_new = 0;

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *AdcHandle)
{
    avrg_v_1_2 = (adc_data[20] + adc_data[25] + adc_data[30] + adc_data[35]) >> 2;
    avrg_v_2_2 = (adc_data[21] + adc_data[26] + adc_data[31] + adc_data[36]) >> 2;
    avrg_v_3_2 = (adc_data[22] + adc_data[27] + adc_data[32] + adc_data[37]) >> 2;
    avrg_c_1_2 = (adc_data[23] + adc_data[28] + adc_data[33] + adc_data[38]) >> 2;
    avrg_c_2_2 = (adc_data[24] + adc_data[29] + adc_data[34] + adc_data[39]) >> 2;

/*    uint32_t avg_index;
    uint32_t ch_index;
    uint32_t tmp_average;

    for (ch_index = 0; ch_index < NUM_CHANNELS; ch_index++)
    {
        tmp_average = 0;
        for (avg_index = 0; avg_index < SAMPLE_RATE; avg_index++)
        {
            tmp_average += aADCxConvertedValues[SAMPLE_RATE*NUM_CHANNELS + ch_index + avg_index*NUM_CHANNELS];
        }
        uhADCxConvertedValue_Regular_Avg_half2[ch_index] = (uint16_t)(tmp_average/SAMPLE_RATE);0
    }
    puhADCxConvertedValue_Regular_Avg = (uint16_t*)&uhADCxConvertedValue_Regular_Avg_half2;
    ADC_new = 1;
    */
}

/**
  * @brief  Conversion DMA half-transfer callback in non blocking mode
  * @param  hadc: ADC handle
  * @retval None
  */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
    avrg_v_1_1 = (adc_data[0] + adc_data[5] + adc_data[10] + adc_data[15]) >> 2;
    avrg_v_2_1 = (adc_data[1] + adc_data[6] + adc_data[11] + adc_data[16]) >> 2;
    avrg_v_3_1 = (adc_data[2] + adc_data[7] + adc_data[12] + adc_data[17]) >> 2;
    avrg_c_1_1 = (adc_data[3] + adc_data[8] + adc_data[13] + adc_data[18]) >> 2;
    avrg_c_2_1 = (adc_data[4] + adc_data[9] + adc_data[14] + adc_data[19]) >> 2;
/*    uint32_t avg_index;
    uint32_t ch_index;
    uint32_t tmp_average;

    for (ch_index = 0; ch_index < NUM_CHANNELS; ch_index++)
    {
        tmp_average = 0;
        for (avg_index = 0; avg_index < SAMPLE_RATE; avg_index++)
        {
            tmp_average += aADCxConvertedValues[ch_index + avg_index*NUM_CHANNELS];
        }
        uhADCxConvertedValue_Regular_Avg_half1[ch_index] = (uint16_t)(tmp_average/SAMPLE_RATE);
    }
    puhADCxConvertedValue_Regular_Avg = (uint16_t*)&uhADCxConvertedValue_Regular_Avg_half1;
    ADC_new = 1;
    */
}