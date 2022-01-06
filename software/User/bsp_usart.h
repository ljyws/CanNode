#ifndef BSP_USART_H
#define BSP_USART_H

#include "stm32f4xx.h"
#include "usart.h"
#include "dma.h"

#define UART_RX_DMA_SIZE (1024)


#define USART3_BUFLEN 28
#define USART3_MAX_LEN USART3_BUFLEN * 2

//#define USART6_BUFLEN 28
//#define USART6_MAX_LEN USART6_BUFLEN * 2

//#define UART4_BUFLEN 28
//#define UART4_MAX_LEN UART4_BUFLEN * 2

//#define UART5_BUFLEN 28
//#define UART5_MAX_LEN UART5_BUFLEN * 2

//extern uint8_t usart1_buf[USART1_BUFLEN];
//extern uint8_t usart2_buf[USART2_BUFLEN];
extern uint8_t usart3_buf[USART3_BUFLEN];
//extern uint8_t usart6_buf[USART6_BUFLEN];
//extern uint8_t uart4_buf[UART4_BUFLEN];
//extern uint8_t uart5_buf[UART5_BUFLEN];


//ACTION定位模块数据联合体
typedef union _imu_data
{
    uint8_t data[24];
    float ActVal[6];
} imudata_t;
extern imudata_t imudata;

void uart_init(void);
void uart_receive_handler(UART_HandleTypeDef *huart);
void uart_receive_init(   UART_HandleTypeDef *huart);
void usart1_callback_handler(uint8_t *buff);
void usart2_callback_handler(uint8_t *buff);
void usart3_callback_handler(uint8_t *buff);
void usart4_callback_handler(uint8_t *buff);
void usart5_callback_handler(uint8_t *buff);
void usart6_callback_handler(uint8_t *buff);
	
#define ABS(x) ((x > 0) ? (x) : (-x))

#endif
