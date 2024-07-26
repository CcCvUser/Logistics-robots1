#ifndef _UARTSET_h
#define _UARTSET_h

#include "headfile.h"

void uart3_init(void);
void uart4_init(void);
void uart4_callback(LPUART_Type *base, lpuart_handle_t *handle, status_t status, void *userData);
void uart1_init(void);
void get_openart1_data(LPUART_Type *base, lpuart_handle_t *handle, status_t status, void *userData);
void uart5_init(void);
void uart8_init(void);

#endif