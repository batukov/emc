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
#include <stdlib.h>
#include "adc.h"
#include "gpio.h"



DMA_HandleTypeDef hdma_adc1;

volatile uint32_t adc_data[DMA_BUFF_SIZE] = {0}; //struct with uint16_t units??



ADC_HandleTypeDef hadc1;

uint32_t avrg_v_1 = 0; //0
uint32_t avrg_v_2 = 0; //1
uint32_t avrg_v_3 = 0; //2
uint32_t avrg_c_1 = 0; //3
uint32_t avrg_c_2 = 0; //4

uint32_t v_1_buff[32] = {0};
uint32_t v_2_buff[32] = {0};
uint32_t v_3_buff[32] = {0};
uint32_t c_1_buff[32] = {0};
uint32_t c_2_buff[32] = {0};


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
  //hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_FALLING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 5;//6;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }


  sConfig.Channel = ADC_CHANNEL_0; //ADC_CHANNEL_0
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;//ADC_SAMPLETIME_480CYCLES
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

  /*sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 4;
  if(HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
      Error_Handler();
  }*/
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 4;
  if(HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
      Error_Handler();
  }
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 5;
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
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    HAL_DMA_Init(adcHandle->DMA_Handle);



    hdma_adc1.Instance = DMA2_Stream0;
    hdma_adc1.Init.Channel = DMA_CHANNEL_0;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;//DMA_PDATAALIGN_HALFWORD;
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;//DMA_MDATAALIGN_HALFWORD;
    hdma_adc1.Init.Mode = DMA_CIRCULAR;
    hdma_adc1.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    hdma_adc1.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
    hdma_adc1.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_1QUARTERFULL;
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
    split_data();
    get_filtered_values();
    int hh= 0;
    //avrg_v_1 = ( adc_data[97] + adc_data[103]  + adc_data[109] + adc_data[115] + adc_data[121] + adc_data[127] + adc_data[133] + adc_data[139] + adc_data[145] + adc_data[151] + adc_data[157] + adc_data[163] + adc_data[169] + adc_data[175] + adc_data[181] + adc_data[187]) >> 4;
    //avrg_v_2 = ( adc_data[98] + adc_data[104]  + adc_data[110] + adc_data[116] + adc_data[122] + adc_data[128] + adc_data[134] + adc_data[140] + adc_data[146] + adc_data[152] + adc_data[158] + adc_data[164] + adc_data[170] + adc_data[176] + adc_data[182] + adc_data[188]) >> 4;
    //avrg_v_3 = ( adc_data[99] + adc_data[105]  + adc_data[111] + adc_data[117] + adc_data[123] + adc_data[129] + adc_data[135] + adc_data[141] + adc_data[147] + adc_data[153] + adc_data[159] + adc_data[165] + adc_data[171] + adc_data[177] + adc_data[183] + adc_data[189]) >> 4;
    //avrg_c_1 = (adc_data[100] + adc_data[106]  + adc_data[112] + adc_data[118] + adc_data[124] + adc_data[130] + adc_data[136] + adc_data[142] + adc_data[148] + adc_data[154] + adc_data[160] + adc_data[166] + adc_data[172] + adc_data[178] + adc_data[184] + adc_data[190]) >> 4;
    //avrg_c_2 = (adc_data[101] + adc_data[107]  + adc_data[113] + adc_data[119] + adc_data[125] + adc_data[131] + adc_data[137] + adc_data[143] + adc_data[149] + adc_data[155] + adc_data[161] + adc_data[167] + adc_data[173] + adc_data[179] + adc_data[185] + adc_data[191]) >> 4;
}

/**
  * @brief  Conversion DMA half-transfer callback in non blocking mode
  * @param  hadc: ADC handle
  * @retval None
  */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{

    //avrg_v_1 = (adc_data[1] + adc_data[7]  + adc_data[13] + adc_data[19] + adc_data[25] + adc_data[31] + adc_data[37] + adc_data[43] + adc_data[49] + adc_data[55] + adc_data[61] + adc_data[67] + adc_data[73] + adc_data[79] + adc_data[85] + adc_data[91]) >> 4;
    //avrg_v_2 = (adc_data[2] + adc_data[8]  + adc_data[14] + adc_data[20] + adc_data[26] + adc_data[32] + adc_data[38] + adc_data[44] + adc_data[50] + adc_data[56] + adc_data[62] + adc_data[68] + adc_data[74] + adc_data[80] + adc_data[86] + adc_data[92]) >> 4;
    //avrg_v_3 = (adc_data[3] + adc_data[9]  + adc_data[15] + adc_data[21] + adc_data[27] + adc_data[33] + adc_data[39] + adc_data[45] + adc_data[51] + adc_data[57] + adc_data[63] + adc_data[69] + adc_data[75] + adc_data[81] + adc_data[87] + adc_data[93]) >> 4;
    //avrg_c_1 = (adc_data[4] + adc_data[10] + adc_data[16] + adc_data[22] + adc_data[28] + adc_data[34] + adc_data[40] + adc_data[46] + adc_data[52] + adc_data[58] + adc_data[64] + adc_data[70] + adc_data[76] + adc_data[82] + adc_data[88] + adc_data[94]) >> 4;
    //avrg_c_2 = (adc_data[5] + adc_data[11] + adc_data[17] + adc_data[23] + adc_data[29] + adc_data[35] + adc_data[41] + adc_data[47] + adc_data[53] + adc_data[59] + adc_data[65] + adc_data[71] + adc_data[77] + adc_data[83] + adc_data[89] + adc_data[95]) >> 4;
}



void split_data()
{
    for(int i=0; i<DMA_VAR_BUF_SIZE; i++)
    {
        v_1_buff[i] = adc_data[i*5];
        v_2_buff[i] = adc_data[i*5+1];
        v_3_buff[i] = adc_data[i*5+2];
        c_1_buff[i] = adc_data[i*5+3];
        c_2_buff[i] = adc_data[i*5+4];
    }
}
int compare(const void * x1, const void * x2)   // функция сравнения элементов массива
{
    return ( *(int*)x1 - *(int*)x2 );              // если результат вычитания равен 0, то числа равны, < 0: x1 < x2; > 0: x1 > x2
}
void get_filtered_values(){

    qsort(v_1_buff, DMA_VAR_BUF_SIZE, sizeof(uint32_t), compare);
    avrg_v_1 = v_1_buff[16];
    qsort(v_2_buff, DMA_VAR_BUF_SIZE, sizeof(uint32_t), compare);
    avrg_v_2 = v_2_buff[16];
    qsort(v_3_buff, DMA_VAR_BUF_SIZE, sizeof(uint32_t), compare);
    avrg_v_3 = v_3_buff[16];
    qsort(c_1_buff, DMA_VAR_BUF_SIZE, sizeof(uint32_t), compare);
    avrg_c_1 = c_1_buff[16];
    qsort(c_2_buff, DMA_VAR_BUF_SIZE, sizeof(uint32_t), compare);
    avrg_c_2 = c_2_buff[16];
}











































