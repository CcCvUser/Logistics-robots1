/*
 * VCAN_VisualScope.h
 *
 *  Created on: 2021年9月6日
 *      Author: 1
 */

#ifndef CODE_VCAN_VISUALSCOPE_H_
#define CODE_VCAN_VISUALSCOPE_H_
#include "headfile.h"


#define UART_PIN UART_3
//使用前需要初始化uart


void send_uint8_data(uint8 data1,uint8 data2,uint8 data3,uint8 data4);

void send_uint16_data(uint16 data1,uint16 data2,uint16 data3,uint16 data4);

void send_int16_data(int16 data1,int16 data2,int16 data3,int16 data4);
#endif /* CODE_VCAN_VISUALSCOPE_H_ */
