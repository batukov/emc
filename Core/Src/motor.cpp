//
// Created by diehigh on 17.09.18.
//

#include <stm32f405xx.h>
//#include <stm32f4xx_hal_tim.h>
#include "motor.h"
#include "tim.h"
#include "stm32f4xx_hal.h"
#include "math.h"
#include "vector"
#include "algorithm"
#include "motor.h"


#define pi 3.1415926535F
extern volatile uint16_t ADC_data[4];
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern uint32_t tim4_counter;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim4;
extern motor * motor_pointer;
extern uint8_t it_done;
extern uint32_t it_current_time;
extern uint32_t it_last_time;
extern uint32_t it_awaiting_time;

volatile uint8_t states_order[100] = {0};
volatile uint8_t states_order_counter = 0;


motor::motor(int time){
    this->current_state = State(0);
    BaseType_t val = xTaskCreate(this->motor_task,
                                 "ya_mon",
                                 15000,
                                 (void *) this,
                                 1,
                                 nullptr);

}
void motor::set_cfg(int channel, int mode) {

    TIM_OC_InitTypeDef sConfigOC;


    sConfigOC.Pulse = 10;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;

    if(mode)
    {
        sConfigOC.OCMode = TIM_OCMODE_PWM1;
    }else{
        sConfigOC.OCMode = TIM_OCMODE_FORCED_ACTIVE;
        //sConfigOC.OCMode = TIM_OCMODE_ACTIVE;
    }

    switch(channel)
    {
        case 1:
            HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);
            break;
        case 2:
            HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2);
            break;
        case 3:
            HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3);
            break;
    }
}
void motor::set_pwm(uint32_t pwm_value) {
    TIM1->CCR1 = pwm_value;
    TIM1->CCR2 = pwm_value;
    TIM1->CCR3 = pwm_value;
}
uint16_t get_hell_data(void);
void motor::do_commutation(uint16_t halls_data) {

    //this->set_state(halls_data);
// HAL_TIMEx_OCN_Start
    if(this->current_state.key_0_H){
        this->set_cfg(1,1); // channel_1, mode - pwm_1
        HAL_TIM_PWM_Start(&htim1, 1);
        HAL_TIMEx_PWMN_Start(&htim1, 1);
    }else{
        HAL_TIM_PWM_Stop(&htim1, 1);
        HAL_TIMEx_PWMN_Stop(&htim1, 1);
        if(this->current_state.key_0_L){
            this->set_cfg(1,0); // channel_1, mode - forced_active
            HAL_TIMEx_OCN_Start(&htim1, 1);
            //HAL_TIMEx_PWMN_Start(&htim1, 1);
        }else{
            HAL_TIMEx_OCN_Stop(&htim1, 1);
            //HAL_TIMEx_PWMN_Stop(&htim1, 1);
        }
    }

    if(this->current_state.key_1_H){
        this->set_cfg(2,1);
        HAL_TIM_PWM_Start(&htim1, 2);
        HAL_TIMEx_PWMN_Start(&htim1, 2);
    }else{
        HAL_TIM_PWM_Stop(&htim1, 2);
        HAL_TIMEx_PWMN_Stop(&htim1, 2);
        if(this->current_state.key_1_L){
            this->set_cfg(2,0);
            HAL_TIMEx_OCN_Start(&htim1, 2);
            //HAL_TIMEx_PWMN_Start(&htim1, 2);
        }else{
            HAL_TIMEx_OCN_Stop(&htim1, 2);
            //HAL_TIMEx_PWMN_Stop(&htim1, 2);
        }
    }

    if(this->current_state.key_2_H){
        this->set_cfg(3,1);
        HAL_TIM_PWM_Start(&htim1, 3);
        HAL_TIMEx_PWMN_Start(&htim1, 3);
    }else{
        HAL_TIM_PWM_Stop(&htim1, 3);
        HAL_TIMEx_PWMN_Stop(&htim1, 3);
        if(this->current_state.key_2_L){
            this->set_cfg(3,0);
            HAL_TIMEx_OCN_Start(&htim1, 3);
            //HAL_TIMEx_PWMN_Start(&htim1, 3);
        }else{
            HAL_TIMEx_OCN_Start(&htim1, 3);
            //HAL_TIMEx_PWMN_Stop(&htim1, 3);
        }
    }
}


uint16_t motor::set_state(uint16_t new_state) {
    this->current_state = State(new_state);
    return new_state;
}
void motor::motor_task(void *pvParameters){

    uint32_t amplitude = 300;

    motor *obj = (motor * )pvParameters;

    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_10,GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);

    // нессущая чистота 84000, предделитель 8399, период 10000,длина импульса всякая там
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);    //starts PWM on CH1 pin
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1); //starts PWM on CH1N pin

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);    //starts PWM on CH1 pin
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2); //starts PWM on CH1N pin

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);    //starts PWM on CH1 pin
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3); //starts PWM on CH1N pin

    htim1.Instance->CCR1 = 0;
    htim1.Instance->CCR2 = 0;
    htim1.Instance->CCR3 = 0;

    uint8_t cur_state = get_hell_data();

    while(1)
    {
        /*
        if( get_time() > it_awaiting_time)
        {
            cur_state = get_hell_data();
            //motor_pointer->do_commutation(motor_pointer->set_state(cur_state-1));
            //cur_state = obj->current_state.state_number;
            switch(cur_state)
            {
                case 1:
                    motor_pointer->do_commutation(motor_pointer->set_state(3));
                    //motor_pointer->do_commutation(motor_pointer->set_state(2));
                    break;
                case 3:
                    motor_pointer->do_commutation(motor_pointer->set_state(2));
                    //motor_pointer->do_commutation(motor_pointer->set_state(6));
                    break;
                case 2:
                    motor_pointer->do_commutation(motor_pointer->set_state(6));
                    //motor_pointer->do_commutation(motor_pointer->set_state(4));
                    break;
                case 6:
                    motor_pointer->do_commutation(motor_pointer->set_state(4));
                    //motor_pointer->do_commutation(motor_pointer->set_state(5));
                    break;
                case 4:
                    motor_pointer->do_commutation(motor_pointer->set_state(5));
                    //motor_pointer->do_commutation(motor_pointer->set_state(1));
                    break;
                case 5:
                    motor_pointer->do_commutation(motor_pointer->set_state(1));
                    //motor_pointer->do_commutation(motor_pointer->set_state(3));
                    break;
                default:
                    //motor_pointer->do_commutation(1);
                    break;
            }
            //motor_pointer->do_commutation(7);
        }
         */

        //motor_pointer->do_commutation(motor_pointer->set_state(2));
        obj->set_pwm(amplitude);

    }

}

uint16_t get_hell_data(void) {
    return (uint8_t) (( HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) << 2) |  (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) << 1) |  HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_11) << 0 );
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance==TIM4) //check if the interrupt comes from TIM3
    {

        motor_pointer->do_commutation(motor_pointer->set_state(get_hell_data()));
        //it_last_time = it_current_time;
        //it_current_time = get_time();
        //it_awaiting_time = it_current_time*3 - it_last_time*2;

    }
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
    //motor_pointer->do_commutation(motor_pointer->set_state(get_hell_data()));
    //it_last_time = it_current_time;
    //it_current_time = get_time();
    //it_awaiting_time = it_current_time*3 - it_last_time*2;
}

//{ 0,1	,	0,0	,	1,0 }, //1 001  -
//{ 0,0	,	0,1	,	1,0 }, //3 011  -
//{ 1,0	,	0,1	,	0,0 }, //2 010  +
//{ 1,0	,   0,0	,   0,1 }, //6 110  -
//{ 0,0	,	1,0	,	0,1 }, //4 100  +
//{ 0,1	,	1,0	,	0,0 }, //5 101  +
