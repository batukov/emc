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



struct State{
    uint8_t bits_A;
    uint8_t bits_B;
    int wiring;
    int8_t statte_1;
    int8_t statte_2;
    int8_t statte_3;
    bool direction;
    int state_h;
    //uint
    State(int8_t state_1, int8_t state_2, int8_t state_3, bool dir, State* next, int state_num)
    {
        this->wiring = 10;
        this->bits_A = 0;
        this->bits_B = 0;
        if(state_1 ==  1){this->bits_A = this->bits_A^0b01000000;   this->bits_B = this->bits_B^0b00100000;}
        if(state_1 == -1){this->bits_A = this->bits_A^0b00100000;   this->bits_B = this->bits_B^0b01000000;}
        if(state_1 ==  0){this->bits_A = this->bits_A^0b00100000;   this->bits_B = this->bits_B^0b00100000; this->wiring = 0;}
        if(state_2 ==  1){this->bits_A = this->bits_A^0b00010000;   this->bits_B = this->bits_B^0b00001000;}
        if(state_2 == -1){this->bits_A = this->bits_A^0b00001000;   this->bits_B = this->bits_B^0b00010000;}
        if(state_2 ==  0){this->bits_A = this->bits_A^0b00001000;   this->bits_B = this->bits_B^0b00001000; this->wiring = 1;}
        if(state_3 ==  1){this->bits_A = this->bits_A^0b00000100;   this->bits_B = this->bits_B^0b00000010;}
        if(state_3 == -1){this->bits_A = this->bits_A^0b00000010;   this->bits_B = this->bits_B^0b00000100;}
        if(state_3 ==  0){this->bits_A = this->bits_A^0b00000010;   this->bits_B = this->bits_B^0b00000010; this->wiring = 2;}
        if(dir){this->bits_A = this->bits_A^0b00000001;}
        this->next_state = next;
        statte_1 = state_1;
        statte_2 = state_2;
        statte_3 = state_3;
        direction = dir;
        state_h = state_num;
    }
    State*  next_state;
    void set_next_state(State* next);
};


struct comparison
{
    inline bool operator() (const uint32_t &val1, const uint32_t &val2)
    {
        return (val1 < val2);
    }
};

struct period_median_struct
{
    //const array_size = 9;
    static int compare( const void * val_1, const void * val_2 )
    {
        return ( *(uint32_t*)val_1 - *(uint32_t*)val_2 );
    };
    const static int array_length = 5;
    int current_index;
    uint32_t period_array[array_length];
    uint32_t get_period(uint32_t new_period){
        this->current_index++;
        if(this->current_index > array_length-1){ this->current_index = 0;}
        this->period_array[this->current_index] = new_period;
        qsort(period_array,array_length,sizeof(uint32_t),compare);
        return period_array[array_length/2];
    };
};

struct sin_cos{
    float sin_val;
    float cos_val;
};

class my_class {
public:
    my_class(int time);
    static void morgat_blyat(void *pvParameters);
    //void set_state(State * state);
    void set_width_new_way(float ampl, float time, int channel);
    float median_filter(float* array, int array_size);
    static int compare(const void * x1, const void * x2) ;  // функция сравнения элементов массива
    uint32_t expected_switch_time;
    uint32_t lasst_time;
    int state_handler;
    float angle;
    float P_d = 0.02F;
    float I_d = 0.0F;
    float P_q = 0.02F;
    float I_q = 0.0F;
    void kirhgoff(int val_1, int val_2);
    void forward_clark();
    void forward_park();
    void pid_d();
    void pid_q();
    void backward_clark();
    void backward_park();

    //kirhgoff
    int cur_clear_3;
    int cur_clear_1;
    int cur_clear_2;

    //fwrd_clrk
    float cur_a;
    float cur_b;

    //fwrd clrk
    float cur_d;
    float cur_q;

    //pid
    float target_d = 0.0F;
    float target_q = 1.0F;
    float vol_d_p;
    float vol_q_p;
    float vol_d_i = 0.0F;
    float vol_q_i = 0.0F;

    //bck prk
    float vol_d_sup;
    float vol_q_sup;

    //bck clrk
    float vol_a;
    float vol_b;
    float vol_c;

};


#endif //HGUXHCFB_MY_CLASS_H
