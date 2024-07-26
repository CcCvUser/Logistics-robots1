/*
 * VCAN_VisualScope.c
 *
 *  Created on: 2021年9月6日
 *      Author: 1
 */
#include "VCAN_VisualScope.h"

uint8 data_uint8[4];
uint8 data_uint8_head[2];
uint8 data_uint8_end[2];

//4通道 8位数据
void send_uint8_data(uint8 data1,uint8 data2,uint8 data3,uint8 data4)
{

    data_uint8_head[0]=0x03;
    data_uint8_head[1]=0xFC;
    uart_write_buffer(UART_PIN,&data_uint8_head[0],2);

    data_uint8[0]=data1;
    data_uint8[1]=data2;
    data_uint8[2]=data3;
    data_uint8[3]=data4;
    uart_write_buffer(UART_PIN,&data_uint8[0],4);

    data_uint8_end[0]=0xFC;
    data_uint8_end[1]=0x03;

    uart_write_buffer(UART_PIN,&data_uint8_end[0],2);

}

//4通道 16位数据
void send_uint16_data(uint16 data1,uint16 data2,uint16 data3,uint16 data4)
{
    data_uint8_head[0]=0x03;
    data_uint8_head[1]=0xFC;
    uart_write_buffer(UART_PIN,&data_uint8_head[0],2);

    uint8 dat[8];

    dat[0] = (uint8)((uint16)data1&0xff);
    dat[1] = (uint8)((uint16)data1>>8);

    dat[2] = (uint8)((uint16)data2&0xff);
    dat[3] = (uint8)((uint16)data2>>8);

    dat[4] = (uint8)((uint16)data3&0xff);
    dat[5] = (uint8)((uint16)data3>>8);

    dat[6] = (uint8)((uint16)data4&0xff);
    dat[7] = (uint8)((uint16)data4>>8);
    uart_write_buffer(UART_PIN,&dat[0],8);

    data_uint8_end[0]=0xFC;
    data_uint8_end[1]=0x03;

    uart_write_buffer(UART_PIN,&data_uint8_end[0],2);
}

void send_int16_data(int16 data1,int16 data2,int16 data3,int16 data4)
{
    data_uint8_head[0]=0x03;
    data_uint8_head[1]=0xFC;
    uart_write_buffer(UART_PIN,&data_uint8_head[0],2);

    uint8 dat[8];

    dat[0] = (uint8)((uint16)data1&0xff);
    dat[1] = (uint8)((uint16)data1>>8);

    dat[2] = (uint8)((uint16)data2&0xff);
    dat[3] = (uint8)((uint16)data2>>8);

    dat[4] = (uint8)((uint16)data3&0xff);
    dat[5] = (uint8)((uint16)data3>>8);

    dat[6] = (uint8)((uint16)data4&0xff);
    dat[7] = (uint8)((uint16)data4>>8);
    uart_write_buffer(UART_PIN,&dat[0],8);

    data_uint8_end[0]=0xFC;
    data_uint8_end[1]=0x03;

    uart_write_buffer(UART_PIN,&data_uint8_end[0],2);
}
