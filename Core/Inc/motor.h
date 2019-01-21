//
// Created by diehigh on 17.09.18.
//

#ifndef HGUXHCFB_MY_CLASS_H
#define HGUXHCFB_MY_CLASS_H
#include "FreeRTOS.h"
#include "task.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"
#include "algorithm"
#include <string.h>
//#include "vector"

#define pwm_port_forward        GPIOA
#define pwm_port_backward       GPIOB

#define pwm_for_drw_ch1_f_pin   10
#define pwm_for_drw_ch1_b_pin   15

#define pwm_for_drw_ch2_f_pin   9
#define pwm_for_drw_ch2_b_pin   14

#define pwm_for_drw_ch3_f_pin   8
#define pwm_for_drw_ch3_b_pin   13

#define zero_mask 0b00000000000000000000000000000000
#define mask_ch1__forward   0b00000000001100000000000000000000
#define mask_ch1__backward  0b11000000000000000000000000000000

#define mask_ch2__forward   0b00000000000011000000000000000000
#define mask_ch2__backward  0b00110000000000000000000000000000

#define mask_ch3__forward   0b00000000000000110000000000000000
#define mask_ch3__backward  0b00001100000000000000000000000000

#define PULL_UP_KEY 1
#define PULL_DOWN_KEY 2
#define NO_PULL_KEY 0

static const uint8_t PMSM_BRIDGE_STATE_FORWARD[8][6] =
        {
            //	UH,UL		VH,VL	WH,WL
                { 0,1	,	0,0	,	1,0 }, //1 001
                { 1,0	,	0,1	,	0,0 }, //2 010
                { 0,0	,	0,1	,	1,0 }, //3 011
                { 0,0	,	1,0	,	0,1 }, //4 100
                { 0,1	,	1,0	,	0,0 }, //5 101
                { 1,0	,   0,0	,   0,1 }, //6 110
        };


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
        switch(state_value)
        {
            case 0:
                this->key_0_H = 0;
                this->key_0_L = 0;
                this->key_1_H = 0;
                this->key_1_L = 0;
                this->key_2_H = 0;
                this->key_2_L = 0;
                break;
            case 1:
                this->key_0_H = 0;
                this->key_0_L = 1;
                this->key_1_H = 0;
                this->key_1_L = 0;
                this->key_2_H = 1;
                this->key_2_L = 0;
                break;
            case 2:
                this->key_0_H = 1;
                this->key_0_L = 0;
                this->key_1_H = 0;
                this->key_1_L = 1;
                this->key_2_H = 0;
                this->key_2_L = 0;
                break;
            case 3:
                this->key_0_H = 0;
                this->key_0_L = 0;
                this->key_1_H = 0;
                this->key_1_L = 1;
                this->key_2_H = 1;
                this->key_2_L = 0;
                break;
            case 4:
                this->key_0_H = 0;
                this->key_0_L = 0;
                this->key_1_H = 1;
                this->key_1_L = 0;
                this->key_2_H = 0;
                this->key_2_L = 1;
                break;
            case 5:
                this->key_0_H = 0;
                this->key_0_L = 1;
                this->key_1_H = 1;
                this->key_1_L = 0;
                this->key_2_H = 0;
                this->key_2_L = 0;
                break;
            case 6:
                this->key_0_H = 1;
                this->key_0_L = 0;
                this->key_1_H = 0;
                this->key_1_L = 0;
                this->key_2_H = 0;
                this->key_2_L = 1;
                break;
            case 7:
                this->key_0_H = 0;
                this->key_0_L = 0;
                this->key_1_H = 0;
                this->key_1_L = 0;
                this->key_2_H = 0;
                this->key_2_L = 0;
                break;
        }
        /*if(wiring_0 == -1){this->key_1_H = 0; this->key_1_L = 1;}
        if(wiring_0 ==  0){this->key_1_H = 0; this->key_1_L = 0; this->free_wiring = 0;}
        if(wiring_0 ==  1){this->key_1_H = 1; this->key_1_L = 0;}

        if(wiring_1 == -1){this->key_1_H = 0; this->key_1_L = 1;}
        if(wiring_1 ==  0){this->key_1_H = 0; this->key_1_L = 0; this->free_wiring = 1;}
        if(wiring_1 ==  1){this->key_1_H = 1; this->key_1_L = 0;}

        if(wiring_2 == -1){this->key_1_H = 0; this->key_1_L = 1;}
        if(wiring_2 ==  0){this->key_1_H = 0; this->key_1_L = 0; this->free_wiring = 2;}
        if(wiring_2 ==  1){this->key_1_H = 1; this->key_1_L = 0;}
         */

        this->state_number = state_value;
        //this->next_state = next;
    }
    //State(State* next, int wiring_0, int wiring_1, int wiring_2, uint8_t state_value)
/*    State(int wiring_0, int wiring_1, int wiring_2, uint8_t state_value)
    {
        if(wiring_0 == -1){this->key_1_H = 0; this->key_1_L = 1;}
        if(wiring_0 ==  0){this->key_1_H = 0; this->key_1_L = 0; this->free_wiring = 0;}
        if(wiring_0 ==  1){this->key_1_H = 1; this->key_1_L = 0;}

        if(wiring_1 == -1){this->key_1_H = 0; this->key_1_L = 1;}
        if(wiring_1 ==  0){this->key_1_H = 0; this->key_1_L = 0; this->free_wiring = 1;}
        if(wiring_1 ==  1){this->key_1_H = 1; this->key_1_L = 0;}

        if(wiring_2 == -1){this->key_1_H = 0; this->key_1_L = 1;}
        if(wiring_2 ==  0){this->key_1_H = 0; this->key_1_L = 0; this->free_wiring = 2;}
        if(wiring_2 ==  1){this->key_1_H = 1; this->key_1_L = 0;}

        this->state_number = state_value;
        //this->next_state = next;
    }
    */
    //State*  next_state;
    //void set_next_state(State* next);
};


class motor {
public:
    motor(int time);
    static void motor_task(void *pvParameters);
    uint16_t set_state(uint16_t new_state);
    void do_commutation(uint16_t halls_data);
    void set_cfg(int channel, int mode);
    void set_pwm(uint32_t pwm_value);

    State current_state = State(0);

};
void action();

/*
 State(uint8_t state_value)
    {
        switch(state_value)
        {
            case 0:
                this->key_0_H = 0;
                this->key_0_L = 0;
                this->key_1_H = 0;
                this->key_1_L = 0;
                this->key_2_H = 0;
                this->key_2_L = 0;
                break;
            case 1:
                this->key_0_H = 0;
                this->key_0_L = 1;
                this->key_1_H = 0;
                this->key_1_L = 0;
                this->key_2_H = 1;
                this->key_2_L = 0;
                break;
            case 2:
                this->key_0_H = 1;
                this->key_0_L = 0;
                this->key_1_H = 0;
                this->key_1_L = 1;
                this->key_2_H = 0;
                this->key_2_L = 0;
                break;
            case 3:
                this->key_0_H = 0;
                this->key_0_L = 0;
                this->key_1_H = 0;
                this->key_1_L = 1;
                this->key_2_H = 1;
                this->key_2_L = 0;
                break;
            case 4:
                this->key_0_H = 0;
                this->key_0_L = 0;
                this->key_1_H = 1;
                this->key_1_L = 0;
                this->key_2_H = 0;
                this->key_2_L = 1;
                break;
            case 5:
                this->key_0_H = 0;
                this->key_0_L = 1;
                this->key_1_H = 1;
                this->key_1_L = 0;
                this->key_2_H = 0;
                this->key_2_L = 0;
                break;
            case 6:
                this->key_0_H = 1;
                this->key_0_L = 0;
                this->key_1_H = 0;
                this->key_1_L = 0;
                this->key_2_H = 0;
                this->key_2_L = 1;
                break;
            case 7:
                this->key_0_H = 0;
                this->key_0_L = 0;
                this->key_1_H = 0;
                this->key_1_L = 0;
                this->key_2_H = 0;
                this->key_2_L = 0;
                break;
        }*/
#endif //HGUXHCFB_MY_CLASS_H
