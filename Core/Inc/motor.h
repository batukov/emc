//
// Created by diehigh on 17.09.18.
//

//#ifndef MOTOR_H
//#define MOTOR_H
#pragma once
#include "FreeRTOS.h"
#include "task.h"
//#include "stm32f4xx_hal.h"
//#include "stm32f4xx_hal_tim.h"
#include <string.h>
#include <usb.h>

#define DMA_VAR_BUF_SIZE 32
#define DMA_BUFF_SIZE DMA_VAR_BUF_SIZE*5

extern ADC_HandleTypeDef hadc1;
extern uint32_t avrg_v_1;
extern uint32_t avrg_v_2;
extern uint32_t avrg_v_3;
extern uint32_t avrg_c_1;
extern uint32_t avrg_c_2;

struct State{

    uint8_t key_0_H;
    uint8_t key_0_L;
    uint8_t key_1_H;
    uint8_t key_1_L;
    uint8_t key_2_H;
    uint8_t key_2_L;

    uint8_t free_wiring = 0;
    uint8_t state_number;

    State(uint8_t state_value)
    {
        switch(state_value) //6 4 5 1 3 2
        {

            case 6://1
                this->key_0_H = 0;
                this->key_0_L = 1;
                this->key_1_H = 0;
                this->key_1_L = 0;
                this->key_2_H = 1;
                this->key_2_L = 0;
                break;
            case 5://2
                this->key_0_H = 1;
                this->key_0_L = 0;
                this->key_1_H = 0;
                this->key_1_L = 1;
                this->key_2_H = 0;
                this->key_2_L = 0;
                break;
            case 4://3
                this->key_0_H = 0;
                this->key_0_L = 0;
                this->key_1_H = 0;
                this->key_1_L = 1;
                this->key_2_H = 1;
                this->key_2_L = 0;
                break;
            case 3://4
                this->key_0_H = 0;
                this->key_0_L = 0;
                this->key_1_H = 1;
                this->key_1_L = 0;
                this->key_2_H = 0;
                this->key_2_L = 1;
                break;
            case 2://5
                this->key_0_H = 0;
                this->key_0_L = 1;
                this->key_1_H = 1;
                this->key_1_L = 0;
                this->key_2_H = 0;
                this->key_2_L = 0;
                break;
            case 1://6
                this->key_0_H = 1;
                this->key_0_L = 0;
                this->key_1_H = 0;
                this->key_1_L = 0;
                this->key_2_H = 0;
                this->key_2_L = 1;
                break;
        }
        this->state_number = state_value;
    }
};


class motor {
    commands_buf *cmd_buffer;
public:
    motor(commands_buf * new_buf);
    static void motor_task(void *pvParameters);
    uint16_t set_state(uint16_t new_state);
    void do_commutation(uint16_t halls_data);
    void set_pwm(uint32_t pwm_value);
    State current_state = State(0);
    int mode = 0;
    void switch_mode(const int &value);
    int process_last_msg();
    void form_string_to_send();
    uint32_t amplitude;

    uint32_t current_1 = 0;
    uint32_t current_2 = 0;

    uint32_t voltage_1 = 0;
    uint32_t voltage_2 = 0;
    uint32_t voltage_3 = 0;

    void set_currents(const uint32_t &new_cur_1, const uint32_t &new_cur_2);
    void set_voltages(const uint32_t &new_vol_1, const uint32_t &new_vol_2, const uint32_t &new_vol_3);

};
void action();
//#endif MOTOR_H
