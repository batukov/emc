//
// Created by diehigh on 01.04.19.
//

#include <algorithm>
#include "usb.h"
//#include "usbd_cdc_if.c"

extern int8_t CDC_Receive_FS(uint8_t* pbuf, uint32_t *Len);

usb::usb(int time){
    BaseType_t val = xTaskCreate(this->usb_task,
                                 "usb_mon",
                                 1000,
                                 (void *) this,
                                 4,
                                 nullptr);
int gg = 0;
}

void usb::usb_task(void *pvParameters){

    usb *obj = (usb * )pvParameters;

    MX_USB_DEVICE_Init();

    //uint8_t testDataToSend[8];
    uint8_t a = 's';
    uint8_t buf[4];
    uint32_t len = 4;
/*
    for (uint8_t i = 0; i < 8; i++)
    {
        testDataToSend[i] = i;
    }
*/
    while(1)
    {
        //CDC_Transmit_FS(testDataToSend, 8);
        //CDC_Rece
        //CDC_Transmit_FS(&a, sizeof(a));
//        CDC_Receive_FS(buf, &len);
        //buf = receive();
        //amplitude = uint32_t (buf);
        //vTaskDelay(5);
        obj->read_data(UserRxBufferFS, APP_RX_DATA_SIZE);
    }
}
void usb::read_data(uint8_t *rx_buffer, uint8_t buf_size)
{
    //int buf_size = 9;
    //const char pwm[3] = {'p', 'w', 'm'};
    const char pwm[] = {"pwm"};
    const char stp[] = {"stp"};
    const char str[] = {"str"};
    char cmd[4];
    cmd[0] = rx_buffer[0];
    cmd[1] = rx_buffer[1];
    cmd[2] = rx_buffer[2];

    char  val[3];
    val[0] = rx_buffer[4];
    val[1] = rx_buffer[5];
    val[2] = rx_buffer[6];

    if(!strcmp(cmd, pwm))
    {
        amplitude = std::min(atoi(val),255);
        return;
    }
    if(!strcmp(cmd, stp))
    {
        motor_pointer->switch_mode(0);
        //amplitude = 0;
        return;
    }
    if(!strcmp(cmd, str))
    {
        motor_pointer->switch_mode(1);
        //amplitude = 10;
        return;
    }
}