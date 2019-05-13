//
// Created by diehigh on 01.04.19.
//

#include <algorithm>
#include "usb.h"

extern int8_t CDC_Receive_FS(uint8_t* pbuf, uint32_t *Len);


commands_buf::commands_buf() {this->in_iterator = -1; this->out_iterator = -1;}

int commands_buf::add_incoming_cmd(char *cmd, int new_value) {
    if(this->in_iterator > 9) {return -1;} // если буфер полный возвращаем -1
    this->in_iterator++; // увеличиваем итератор
    strcpy(this->in_cmd_list[this->in_iterator].name, cmd); // копируем название сообщения в буфер
    this->in_cmd_list[this->in_iterator].value = new_value; // копируем числовое значение в буфер
    return 0; // если все успешной возвращаем 0
}
int commands_buf::get_last_incoming_cmd(char (&cmd_buffer)[30], int &o_value) {
    if(this->in_iterator < 0) { return -1;} // если итератор отрицательный (значит нет сообщений на обработку) возвращаем -1

    strcpy(cmd_buffer, this->in_cmd_list[this->in_iterator].name); // копируем во входящий буфер последнее сообщение
    o_value = this->in_cmd_list[this->in_iterator].value; // копируем во входящий буфер числовое сообщение
    memset(this->in_cmd_list[this->in_iterator].name, 0, sizeof(this->in_cmd_list[this->in_iterator].name)); // зануляем память хранящую последнее сообщение
    this->in_cmd_list[this->in_iterator].value = 0; // зануляем численное значение
    this->in_iterator--; // уменьшаем итератор
    return 0; // если все ок возвращаем 0
}

int commands_buf::del_last_msg() {
    if(in_iterator < 0) {return -1;} // если итератор отрицательный (значит нет сообщений на обработку) возвращаем -1
    memset(this->in_cmd_list[this->in_iterator].name, 0, sizeof(this->in_cmd_list[this->in_iterator].name)); // зануляем память хранящую последнее сообщение
    this->in_cmd_list[this->in_iterator].value = 0; // зануляем численное значение
    this->in_iterator--; // уменьшаем итератор
    return 0;
}
int commands_buf::add_outcoming_cmd(const char *cmd, const int new_value) {
    if(this->out_iterator > 9){return -1;} // если буфер полный возвращаем -1
    this->out_iterator++; // увеличиваем итератор
    strcpy(this->out_cmd_list[this->out_iterator].name, cmd); // копируем название сообщения в буфер
    this->out_cmd_list[this->out_iterator].value = new_value; // копируем числовое значение в буфер
    return 0; // если все успешной возвращаем 0
}
int commands_buf::get_last_outcoming_cmd(char (&cmd_buffer)[30] , int &o_value) {
    if(this->out_iterator < 0) {return -1;} // если итератор отрицательный (значит нет сообщений на обработку) возвращаем -1

    strcpy(cmd_buffer, this->out_cmd_list[this->out_iterator].name); // копируем во входящий буфер последнее сообщение
    o_value = this->out_cmd_list[this->out_iterator].value; // копируем во входящий буфер числовое сообщение
    memset(this->out_cmd_list[this->out_iterator].name, 0, sizeof(this->out_cmd_list[this->out_iterator].name)); // зануляем память хранящую последнее сообщение
    this->out_cmd_list[this->out_iterator].value = 0; // зануляем численное значение
    this->out_iterator--; // уменьшаем итератор
    return 0;
}


usb::usb(commands_buf * new_buf){
    this->cmd_buffer = new_buf;
    BaseType_t val = xTaskCreate(this->usb_task,
                                 "usb_mon",
                                 1500,
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
    char buffer_to_process[30] = {0}; // создаем буфер сoобщений на отправку
    int value_to_process = 0; // создаем буфер значений на отправку


    while(1)
    {
        //CDC_Transmit_FS(testDataToSend, 8);
        //CDC_Transmit_FS(&a, sizeof(a));
        if(got_msg == 1) // проверяем если есть входящее сообщение
        {
            obj->read_data(rx_stuff); // записываем сообщение в буфер и делаем с ним всякие штуки
            got_msg = 0; // сбрасываем флаг сообщений
        }

        if(!obj->cmd_buffer->get_last_outcoming_cmd(buffer_to_process, value_to_process)) // если есть какой-нить сообщеня
        {
            obj->send_data(buffer_to_process, value_to_process); // отправляем его прямиком в отправщик сообщений
        }
    }
}
uint8_t usb::cut_the_string(char *string_to_cut, char *result_buffer)
{
    char *pos = strchr(string_to_cut, 0x20); // ищем вхождение пробела
    if(pos) // если что-то нашлося
    {
        strncpy(result_buffer, string_to_cut, pos - string_to_cut); // копируем часть строки с начала до этого момента
        return 1;
    }else{return 0;}
}
void usb::split_to_words(const char *string_to_parse, uint8_t string_size)
{
    char result[30][30] = {{0}}; // буфер для хранения слов сообщения
    uint8_t index_of_free_string = 0; // индекс свободной строки буфера

    char tmp_value[30] = {0}; // переменная для временного хранения слова
    char new_tmp_value[30] = {0}; // еще одна переменная для временного хранения слова
    strcpy(tmp_value,string_to_parse); // копируем строку, которую нужно парсить во временный буфер
    while(this->cut_the_string(tmp_value, result[index_of_free_string]))
    {
        strcpy(new_tmp_value, &tmp_value[strlen(result[index_of_free_string]) + 1]); // записываю новые значения в массив
        memset(tmp_value, 0, sizeof(tmp_value));  // зануляю массив
        strcpy(tmp_value,new_tmp_value); // записываю в зануленный массив новую строку
        index_of_free_string++; // инкрементирую индекс
    }
    index_of_free_string--; // уменьшаем итератор на 1, так как последний обход цикла был холостой (надо поправить это дело)
    char tmp = 0x5f; // вводим сомвол подчеркивания
    switch(index_of_free_string) // в зависимости от количества найденных слов
    {
        case 0:
            this->cmd_buffer->add_incoming_cmd(result[0], 0); // добавляем новое сообщение в буфер на обработку
            break;
        case 1:
            strcat(result[0], &tmp); // дописываем подчеркивание в конце первого слова
            this->cmd_buffer->add_incoming_cmd(strcat(result[0], result[1]), 0); //объединяем 2 первых строки
            break;
        case 2:
            strcat(result[0], &tmp); // дописываем подчеркивание в конце первого слова
            this->cmd_buffer->add_incoming_cmd(strcat(result[0], result[1]), atoi(result[2])); //объединяем 2 первых строки и еще заносим целочисленное значение
            break;
        default:
            break;
    }
}
void usb::read_data(uint8_t *rx_buffer)
{
    this->split_to_words((char *)rx_buffer, 30);
}
void usb::send_data(char *cmd, int new_value)
{
    CDC_Transmit_FS((uint8_t *)cmd, 30); //uint8_t* Buf, uint16_t Len)
}