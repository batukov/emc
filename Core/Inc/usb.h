//
// Created by diehigh on 01.04.19.
//

#pragma once

#include "usb_device.h"
#include "FreeRTOS.h"
#include "task.h"

struct data {
    char name[30];
    int value = 0;
};

class commands_buf {
    data in_cmd_list[10];
    int  in_iterator;

    data out_cmd_list[10];
    int  out_iterator;

public:
    commands_buf();
    int add_incoming_cmd(char *cmd, int new_value);
    int get_last_incoming_cmd(char (&cmd_buffer)[30], int &o_value); //void f(int (&arr)[123]) {}

    int add_outcoming_cmd(const char *cmd, const int new_value);
    int get_last_outcoming_cmd(char (&cmd_buffer)[30], int &o_value);

    int del_last_msg();
};

class usb {
    commands_buf * cmd_buffer;
public:
    usb(commands_buf * new_buf);
    static void usb_task(void *pvParameters);
    void read_data(uint8_t *rx_buffer);
    void send_data(char *cmd, int new_value);
    uint8_t cut_the_string(char *string_to_cut, char *result_buffer);
    void split_to_words(const char *string_to_parse, uint8_t string_size);
};

//extern uint32_t amplitude;
//extern motor *motor_pointer;
extern uint8_t got_msg;

//////////////////////////////////////////////////////////////////////////////////



//commands_buf();
//extern uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];

/** Data to send over USB CDC are stored in this buffer   */
//extern uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];
