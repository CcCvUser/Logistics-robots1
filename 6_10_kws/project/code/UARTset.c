#include "UARTset.h"

extern int get_TGT_flag;
extern int get_class_flag;
extern int small_move_flag;
extern int second_classify_finish_flag;
extern int intervene_flag;
extern int intervene_can;
extern int servo_can;

uint8               USART_4_rx_buffer;
lpuart_transfer_t   USART_4_receivexfer;
lpuart_handle_t     USART_4_lpuartHandle;

uint8_t nowData;
uint8   uart4_receive[65];

void get_openart_data(LPUART_Type *base, lpuart_handle_t *handle, status_t status, void *userData)
{       
    static uint8_t uart4_receive_num = 0;
    static uint8_t uart4_receive_flag = 0;

    if(kStatus_LPUART_RxIdle == status){
      //BEEP(200);
        if(uart4_receive_flag == 0 && nowData == 0xf0){
            uart4_receive_flag=1;
            uart4_receive[0]=0xf0;
        }
        else if (uart4_receive_flag==1)
            uart4_receive[++uart4_receive_num] = nowData;
        if(uart4_receive[uart4_receive_num]==0xff){
          
          
           uart4_receive_flag = 0; 
           uart4_receive_num=0;
           if(get_TGT_flag==1){
                get_TGT();
                get_TGT_flag=0;
                //BEEP(200);
           }
           /*else if(get_class_flag==1&&!servo_can){
              if(uart4_receive[1]!=0xfe){
                  get_class();
                  //get_drift_updata_STcar1();
                  get_class_flag=0;
                  BEEP(100);
              }
           }*/
           
           
        }
    }
          handle->rxDataSize = USART_4_receivexfer.dataSize;  //还原缓冲区长度
          handle->rxData = USART_4_receivexfer.data;          //还原缓冲区地址
}

void uart4_init(void)
{
    //初始化串口
    uart_init (UART_4, 115200,UART4_TX_C16,UART4_RX_C17);
    NVIC_SetPriority(LPUART4_IRQn,2);  //设置优先级
    uart_rx_irq(UART_4,1);// 打开串口1接收中断
    //配置串口接收的缓冲区及缓冲区长度
    USART_4_receivexfer.dataSize = 1;
    USART_4_receivexfer.data = &nowData;
    //配置串口中断
    uart_set_handle(UART_4, &USART_4_lpuartHandle, get_openart_data, NULL, 0, USART_4_receivexfer.data, 1);    
}


extern int update_path_flag;
extern int add_finish_flag;
    
uint8               USART_1_rx_buffer;
lpuart_transfer_t   USART_1_receivexfer;
lpuart_handle_t     USART_1_lpuartHandle;

uint8_t nowData_U1;
uint8   uart1_receive[25];

uint8_t uart1_receive_num = 0;
uint8_t uart1_receive_flag = 0;
void get_openart1_data(LPUART_Type *base, lpuart_handle_t *handle, status_t status, void *userData){          
    if(kStatus_LPUART_RxIdle == status){
        //BEEP(200);
        if(nowData_U1==0xfe){
          add_finish_flag=1;
          handle->rxDataSize = USART_1_receivexfer.dataSize;  //还原缓冲区长度
          handle->rxData = USART_1_receivexfer.data;          //还原缓冲区地址
          return;
        }
        if(uart1_receive_flag == 0 && nowData_U1 == 0xf0){               
          memset(uart1_receive,0,sizeof(uart1_receive)); 
          uart1_receive_flag=1;
            uart1_receive[0]=0xf0;
        }
        else if (uart1_receive_flag==1)
            uart1_receive[++uart1_receive_num] = nowData_U1;
        
        if(nowData_U1==0xff){
            if((uart1_receive_num-1)%3!=0){
              //uart_putchar(UART_1,0xfc);
              BEEP(2000);
            }
            else {
                receive_unknownTGT(uart1_receive);
              }
            uart1_receive_flag = 0;
            uart1_receive_num = 0;
            /*if(uart1_receive[1]==0xfe){
              add_finish_flag=1;
            
            }*/
        }
        
    }
    handle->rxDataSize = USART_1_receivexfer.dataSize;  //还原缓冲区长度
    handle->rxData = USART_1_receivexfer.data;          //还原缓冲区地址
}

void uart1_init(void)
{
    //初始化串口
    uart_init (UART_1, 115200,UART1_TX_B12,UART1_RX_B13);
    NVIC_SetPriority(LPUART1_IRQn,15);  //设置优先级
    uart_rx_irq(UART_1,1);// 打开串口1接收中断
    //配置串口接收的缓冲区及缓冲区长度
    USART_1_receivexfer.dataSize = 1;
    USART_1_receivexfer.data = &nowData_U1;
    //配置串口中断
    uart_set_handle(UART_1, &USART_1_lpuartHandle, get_openart1_data, NULL, 0, USART_1_receivexfer.data, 1);    
}
extern int motor_test_flag;
//#define  DEBUG_pid
extern PID Speed_pid[4];
uint8               USART_3_rx_buffer;
lpuart_transfer_t   USART_3_receivexfer;
lpuart_handle_t     USART_3_lpuartHandle;

uint8_t nowData_U3;
uint8   uart3_receive[24];

uint8_t uart3_receive_num = 0;
uint8_t uart3_receive_flag = 0;
void get_openart2_data(LPUART_Type *base, lpuart_handle_t *handle, status_t status, void *userData){//发给辅车
    if(kStatus_LPUART_RxIdle == status){
        if(uart3_receive_flag == 0 && nowData_U3 == 0xf0){               
            uart3_receive_flag=1;
            uart3_receive[0]=0xf0;
        }
        else if (uart3_receive_flag==1)
            uart3_receive[++uart3_receive_num] = nowData_U3;
        if(nowData_U3==0xff){
          
          
            uart3_receive_flag = 0;
            uart3_receive_num = 0;
#ifdef DEBUG_pid
            if(motor_test_flag==0){
               Speed_pid[0].Kp=Speed_pid[0].Kp+uart3_receive[1]*10;
               Speed_pid[0].Ki=Speed_pid[0].Ki+uart3_receive[2];
               Speed_pid[0].Kd=Speed_pid[0].Kd+uart3_receive[3];
               Speed_pid[1].Kp=Speed_pid[1].Kp+uart3_receive[4]*10;
               Speed_pid[1].Ki=Speed_pid[1].Ki+uart3_receive[5];
               Speed_pid[1].Kd=Speed_pid[1].Kd+uart3_receive[6];
            }
            else{
               Speed_pid[2].Kp=Speed_pid[2].Kp+uart3_receive[1]*10;
               Speed_pid[2].Ki=Speed_pid[2].Ki+uart3_receive[2];
               Speed_pid[2].Kd=Speed_pid[2].Kd+uart3_receive[3];
               Speed_pid[3].Kp=Speed_pid[3].Kp+uart3_receive[4]*10;
               Speed_pid[3].Ki=Speed_pid[3].Ki+uart3_receive[5];
               Speed_pid[3].Kd=Speed_pid[3].Kd+uart3_receive[6];
            }
#endif               
        }
        
        
    }
    handle->rxDataSize = USART_3_receivexfer.dataSize;  //还原缓冲区长度
    handle->rxData = USART_3_receivexfer.data;          //还原缓冲区地址
}

void uart3_init(void)
{
    //初始化串口
    uart_init (UART_3, 115200,UART3_TX_B22,UART3_RX_B23);
    NVIC_SetPriority(LPUART3_IRQn,3);  //设置优先级
    uart_rx_irq(UART_3,1);// 打开串口3接收中断
    //配置串口接收的缓冲区及缓冲区长度
    USART_3_receivexfer.dataSize = 1;
    USART_3_receivexfer.data = &nowData_U3;
    //配置串口中断
    uart_set_handle(UART_3, &USART_3_lpuartHandle, get_openart2_data, NULL, 0, USART_3_receivexfer.data, 1);    
}

uint8               USART_5_rx_buffer;
lpuart_transfer_t   USART_5_receivexfer;
lpuart_handle_t     USART_5_lpuartHandle;

uint8_t nowData_U5;
uint8   uart5_receive[24];

uint8_t uart5_receive_num = 0;
uint8_t uart5_receive_flag = 0;
void get_openart3_data(LPUART_Type *base, lpuart_handle_t *handle, status_t status, void *userData){          
    if(kStatus_LPUART_RxIdle == status){

        //BEEP(200);       
        if(uart5_receive_flag == 0 && nowData_U5 == 0xfe){
        //memset(uart5_receive,0,sizeof(uart5_receive));          
            uart5_receive_flag=1;
            uart5_receive[0]=0xfe;
        }
        else if (uart5_receive_flag==1)
            uart5_receive[++uart5_receive_num] = nowData_U5;
        if(nowData_U5==0xff){
          
          
            uart5_receive_flag = 0;
            uart5_receive_num = 0;
            if(intervene_flag==1&&uart5_receive[0]==0xfe){
                get_error(uart5_receive);
                
                //BEEP(200);
            }
            
        }
        
        
    }
    handle->rxDataSize = USART_5_receivexfer.dataSize;  //还原缓冲区长度
    handle->rxData = USART_5_receivexfer.data;          //还原缓冲区地址
}

void uart5_init(void)
{
    //初始化串口
    uart_init (UART_5, 115200,UART5_TX_C28,UART5_RX_C29);
    NVIC_SetPriority(LPUART5_IRQn,4);  //设置优先级
    uart_rx_irq(UART_5,1);// 打开串口3接收中断
    //配置串口接收的缓冲区及缓冲区长度
    USART_5_receivexfer.dataSize = 1;
    USART_5_receivexfer.data = &nowData_U5;
    //配置串口中断
    uart_set_handle(UART_5, &USART_5_lpuartHandle, get_openart3_data, NULL, 0, USART_5_receivexfer.data, 1);    
}

uint8               USART_8_rx_buffer;
lpuart_transfer_t   USART_8_receivexfer;
lpuart_handle_t     USART_8_lpuartHandle;

uint8_t nowData_U8;
uint8   uart8_receive[24];

uint8_t uart8_receive_num = 0;
uint8_t uart8_receive_flag = 0;
void Upper_computer(LPUART_Type *base, lpuart_handle_t *handle, status_t status, void *userData){          
    if(kStatus_LPUART_RxIdle == status){

      //BEEP(200);       
        if(uart8_receive_flag == 0 && nowData_U8 == 0xf0){
        //memset(uart8_receive,0,sizeof(uart8_receive));          
            uart8_receive_flag=1;
            uart8_receive[0]=0xf0;
        }
        else if (uart8_receive_flag==1)
            uart8_receive[++uart8_receive_num] = nowData_U8;
        if(nowData_U8==0xff){
          
          
            uart8_receive_flag = 0;
            uart8_receive_num = 0;
            
        }
        
        
    }
    handle->rxDataSize = USART_8_receivexfer.dataSize;  //还原缓冲区长度
    handle->rxData = USART_8_receivexfer.data;          //还原缓冲区地址
}

void uart8_init(void)
{
    //初始化串口
    uart_init (UART_8, 115200,UART8_TX_B26,UART8_RX_B27);
    NVIC_SetPriority(LPUART8_IRQn,5);  //设置优先级
    uart_rx_irq(UART_8,1);// 打开串口3接收中断
    //配置串口接收的缓冲区及缓冲区长度
    USART_8_receivexfer.dataSize = 1;
    USART_8_receivexfer.data = &nowData_U5;
    //配置串口中断
    uart_set_handle(UART_8, &USART_8_lpuartHandle, Upper_computer, NULL, 0, USART_8_receivexfer.data, 1);    
}
