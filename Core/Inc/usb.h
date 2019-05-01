//
// Created by diehigh on 01.04.19.
//

#pragma once

#include "usb_device.h"
#include "FreeRTOS.h"
#include "task.h"
#include <motor.h>

class usb {
public:
    usb(int time);
    static void usb_task(void *pvParameters);
    void read_data(uint8_t *rx_buffer, uint8_t buf_size);
};

extern uint32_t amplitude;
extern motor *motor_pointer;


//extern uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];

/** Data to send over USB CDC are stored in this buffer   */
//extern uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];
