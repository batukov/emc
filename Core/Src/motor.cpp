//
// Created by diehigh on 17.09.18.
//

#include <stm32f405xx.h>
//#include "stm32f4xx_hal.h"
//#include <stm32f4xx_hal_tim.h>
#include <stm32f4xx_ll_tim.h>
//#include <stm32f4xx_hal_tim.h>
#include "main.h"
#include "adc.h"
#include "motor.h"
#include "tim.h"

#include "math.h"
#include "vector"
#include "algorithm"
//#include <stm32f4xx_hal_tim_ex.h>


#define pi 3.1415926535F
extern volatile uint16_t ADC_data[4];

extern uint32_t tim4_counter;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim4;
extern motor * motor_pointer;

uint32_t old_time;
uint32_t new_time;


volatile uint32_t amplitude = 15;//30;
volatile uint32_t max_amp = 100;



motor::motor(int time){
    this->current_state = State(0);
    BaseType_t val = xTaskCreate(this->motor_task,
                                 "ya_mon",
                                 1000,
                                 (void *) this,
                                 4,
                                 nullptr);
    int hh = 0;
}

void motor::set_pwm(uint32_t pwm_value) {
    TIM1->CCR1 = pwm_value;
    TIM1->CCR2 = pwm_value;
    TIM1->CCR3 = pwm_value;
}

uint16_t get_hell_data(void);

void motor::switch_mode(const int &value) {
    this->mode = value;
    switch (this->mode)
    {
        case 0:
            TIM1->CCR1 = 0;
            TIM1->CCR2 = 0;
            TIM1->CCR3 = 0;
            HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);    //starts PWM on CH1 pin
            HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);    //starts PWM on CH1 pin
            HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);    //starts PWM on CH1 pin

            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8 , GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9 , GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);

            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
            break;
        case 1:
            HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);    //starts PWM on CH1 pin
            HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);    //starts PWM on CH1 pin
            HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);    //starts PWM on CH1 pin
            break;
        default:
            break;
    }
}

void motor::do_commutation(uint16_t halls_data) {
    switch (this->mode) {
    case 0:
        //blablabla
            break;
    case 1:
        this->set_state(halls_data);
        if (this->current_state.key_1_H) {
            LL_TIM_CC_EnableChannel(htim1.Instance, LL_TIM_CHANNEL_CH1);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);

        } else {
            LL_TIM_CC_DisableChannel(htim1.Instance, LL_TIM_CHANNEL_CH1);
            if (this->current_state.key_1_L) {
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
            } else {
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
            }
        }
        if (this->current_state.key_0_H) {
            LL_TIM_CC_EnableChannel(htim1.Instance, LL_TIM_CHANNEL_CH2);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
        } else {
            LL_TIM_CC_DisableChannel(htim1.Instance, LL_TIM_CHANNEL_CH2);
            if (this->current_state.key_0_L) {
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
            } else {
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
            }
        }
        if (this->current_state.key_2_H) {
            LL_TIM_CC_EnableChannel(htim1.Instance, LL_TIM_CHANNEL_CH3);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
        } else {
            LL_TIM_CC_DisableChannel(htim1.Instance, LL_TIM_CHANNEL_CH3);
            if (this->current_state.key_2_L) {
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
            } else {
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);

            }
        }
        break;
     default:
        break;
    }
}


uint16_t motor::set_state(uint16_t new_state) {
    this->current_state = State(new_state);
    return new_state;
}
void motor::motor_task(void *pvParameters){

    motor *obj = (motor * )pvParameters;

    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_10,GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);

    // нессущая чистота 84000, предделитель 8399, период 10000,длина импульса всякая там
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);    //starts PWM on CH1 pin
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);    //starts PWM on CH1 pin
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);    //starts PWM on CH1 pin

    MX_TIM4_Init();

    uint32_t  adc[2] = {0};
    uint32_t adc_v[4] = {0};

    while(1)
    {
        //adc_v[0] = ADC1->JDR4;
        //adc_v[1] = ADC1->JDR3;//3
        //adc_v[2] = ADC1->JDR2;//2
        //adc_v[3] = ADC1->JDR1;//1
        //ADC1->CR2 |= ADC_CR2_JSWSTART;

        //adc[0] = ADC2->JDR1;
        //adc[1] = ADC2->JDR2;
        //ADC2->CR2 |= ADC_CR2_JSWSTART;
        obj->set_pwm(amplitude);
        //vTaskDelay(500);
    }
}

uint16_t get_hell_data(void) {
    return (uint8_t) (( HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) << 0) |  (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) << 1) |  (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_11) << 2));
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance==TIM4) //check if the interrupt comes from TIM3
    {
        //uint16_t state = get_hell_data();
        motor_pointer->do_commutation(get_hell_data());
    }
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
    motor_pointer->do_commutation(get_hell_data());
}
