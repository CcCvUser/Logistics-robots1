/*
 * beep.h
 *
 *  Created on: 2022Äê7ÔÂ28ÈÕ
 *      Author: zh'n
 */

#ifndef CODE_BEEP_H_
#define CODE_BEEP_H_

#include "headfile.h"
#define BEEP_ENABLE B11

#define BEEP(i)         \
do {                    \
    BEEP_on();          \
    buzzerTime = i;\
} while (0);

void BEEP_init();
void BEEP_on();
void BEEP_off();
void BEEP_deal();
extern int buzzerTime;
#endif /* CODE_BEEP_H_ */
