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



motor::motor(commands_buf * new_buf){
    this->cmd_buffer = new_buf;
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
    TIM1->CCR1 = pwm_value; // устанавливаем значение шим
    TIM1->CCR2 = pwm_value;
    TIM1->CCR3 = pwm_value;
}

uint16_t get_hell_data(void);

void motor::switch_mode(const int &value) {
    this->mode = value;
    switch (this->mode)
    {
        case 0:
            TIM1->CCR1 = 0; // устанавливаем значение шим в 0
            TIM1->CCR2 = 0;
            TIM1->CCR3 = 0;
            HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);    // останавливаем шим
            HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
            HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);

            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8 , GPIO_PIN_RESET); // закрываем верхние ключи
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9 , GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);

            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET); // закрываем нижние ключи
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
            break;
        case 1:
            TIM1->CCR1 = 0; // устанавливаем значение шим в 0
            TIM1->CCR2 = 0;
            TIM1->CCR3 = 0;
            HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);    // запускаем шим
            HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);    //
            HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);    //
            break;
        default:
            break;
    }
}

void motor::do_commutation(uint16_t halls_data) {
    switch (this->mode) {
    case 0: // если режим остановки
        //blablabla
            break;
    case 1: // если режим шим
    // если верхний ключ 1, а нижний 0 - значит шим по верхнему ключу, нижний ключ закрыт
    // если верхний ключ 0, а нижний 1 - значит верхний ключ закрыт и шим остановлен, нижний ключ открыт
    // если верхний ключ 0, а нижний 0 - значит верхний ключ закрыт, шим остановлен, и нижний ключ закрыт
    // если верхний ключ 1, а нижний 1 - а такого не должно быть, все сломается, сгорит, взорвется, палево

        if(halls_data == 7) {break;} // если все датчики холла 1 - значит что-то пошло не так, ничего не делаем
        this->set_state(halls_data); // устанавливаем состояние системы в соответствие с датчиками холла

        if (this->current_state.key_1_H) { // если верхний ключ 1
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET); // нижний ключ закрываем
            LL_TIM_CC_EnableChannel(htim1.Instance, LL_TIM_CHANNEL_CH1); // включаем шим на верхнем
        } else { // если верхний ключ 0
            LL_TIM_CC_DisableChannel(htim1.Instance, LL_TIM_CHANNEL_CH1); // выключаем шим на верхнем ключе
            if (this->current_state.key_1_L) { // если нижний ключ 1
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET); // открываем нижний ключ
            } else { // если нижний ключ 0
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET); // закрываем нижний ключ
            }
        } // и так далее
        if (this->current_state.key_0_H) {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
            LL_TIM_CC_EnableChannel(htim1.Instance, LL_TIM_CHANNEL_CH2);
        } else {
            LL_TIM_CC_DisableChannel(htim1.Instance, LL_TIM_CHANNEL_CH2);
            if (this->current_state.key_0_L) {
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
            } else {
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
            }
        }
        if (this->current_state.key_2_H) {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
            LL_TIM_CC_EnableChannel(htim1.Instance, LL_TIM_CHANNEL_CH3);
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
    this->current_state = State(new_state); // устанавливаем состояние
    return new_state; // возвращаем значение состояния
}
void motor::motor_task(void *pvParameters){

    motor *obj = (motor * )pvParameters;

    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_10,GPIO_PIN_SET); // выставляем высокий уровень на С10 (ENABLE GATE) - позволяем дергать ключи на драйвере
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET); // выставляем низкий уровень на B12 (DC CALIB) - отключаем процесс калибровки драйвера

    // нессущая чистота 84000, предделитель 8399, период 10000,длина импульса всякая там
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);    // запускаем шим на верхних ключах
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

    MX_TIM4_Init(); // запускаем 4 таймер, который отвечает за коммутацию обмоток

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
        obj->set_pwm(amplitude); // устанавливаем значение шим
        obj->process_last_msg(); // обрабатываем последнюю сообщеньку
        //vTaskDelay(500);
    }
}

uint16_t get_hell_data(void) {
    return (uint8_t) (( HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) << 0) |
    (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) << 1) |
    (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_11) << 2)); // формируем число описывающие значение 3х датчиков холла
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance==TIM4) {motor_pointer->do_commutation(get_hell_data());} // если тикнул 4 таймер -
    // то мутим коммутацию обмоток в соответствии с датчиками холла
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
    motor_pointer->do_commutation(get_hell_data()); // если пришло прерывания от датчика холла (да хоть какова) -
    // то мутим коммутацию обмоток в соответствии с датчиками холла
}

int motor::process_last_msg() {
    char buffer_to_process[30] = {0}; // создаем буфер для сообщеня
    int value_to_process = 0; // создаем буфер для значеня в сообщени
    if(!this->cmd_buffer->get_last_incoming_cmd(buffer_to_process, value_to_process)) // если есть какой-нить сообщеня
    {
        if(!strcmp("stop",buffer_to_process)){ // если сообщеня стоп и тд
            // а здесь всякие методцы для обработки, мон
            return 0;
        }
        if(!strcmp("starter_par",buffer_to_process)){
            return 0;
        }
        if(!strcmp("starter_imax",buffer_to_process)){
            return 0;
        }
        if(!strcmp("starter_vmin",buffer_to_process)){
            return 0;
        }
        if(!strcmp("starter_run",buffer_to_process)){
            return 0;
        }
        if(!strcmp("starter_forvard",buffer_to_process)){
            return 0;
        }
        if(!strcmp("starter_reverse",buffer_to_process)){
            return 0;
        }
        if(!strcmp("starter_pwm",buffer_to_process)){
            return 0;
        }
        if(!strcmp("generator_par",buffer_to_process)){
            return 0;
        }
        if(!strcmp("generator_run",buffer_to_process)){
            return 0;
        }
        if(!strcmp("generator_pwm",buffer_to_process)){
            return 0;
        }
        if(!strcmp("servo_par",buffer_to_process)){
            return 0;
        }
        if(!strcmp("servo_set",buffer_to_process)){
            return 0;
        }
        if(!strcmp("servo_on",buffer_to_process)){
            return 0;
        }
        if(!strcmp("servo_off",buffer_to_process)){
            return 0;
        }
        if(!strcmp("servo_auto",buffer_to_process)){
            return 0;
        }
        if(!strcmp("servo_vt",buffer_to_process)){
            return 0;
        }
        if(!strcmp("servo_kp",buffer_to_process)){
            return 0;
        }
        if(!strcmp("servo_ki",buffer_to_process)){
            return 0;
        }
        if(!strcmp("servo_kd",buffer_to_process)){
            return 0;
        }
        if(!strcmp("servo_min",buffer_to_process)){
            return 0;
        }
        if(!strcmp("servo_max",buffer_to_process)){
            return 0;
        }
        if(!strcmp("save",buffer_to_process)){
            return 0;
        }
        if(!strcmp("telem",buffer_to_process)){
            return 0;
        }
        this->cmd_buffer->del_last_msg(); // если сообщеня есть а смысла нет - удаляем его
        return (-1); // осмысленного сообщени не нашлося
    }else{return (-1);} // сообщени не нашлося

}