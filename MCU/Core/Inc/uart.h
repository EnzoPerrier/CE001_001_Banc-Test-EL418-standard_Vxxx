/*
 * uart.h
 *
 *  Created on: Jun 30, 2025
 *      Author: Enzo Perrier
 */

#ifndef INC_UART_H_
#define INC_UART_H_

#include "main.h"
#include <stdbool.h>

#define RX_BUFFER1_SIZE 300
#define RX_BUFFER2_SIZE 50
#define RX_BUFFER3_SIZE 100


//RS232_418
extern UART_HandleTypeDef huart1;

extern uint8_t message_complete1;
extern uint8_t rx_char1;
extern uint16_t rx_index1;
extern uint8_t rx_buffer1[RX_BUFFER1_SIZE];

extern volatile uint32_t last_rx_tick1;

void MX_USART1_UART_Init(void);


void send_UART1(const char *msg);
void Check_UART1_Timeout(void);
void clear_rx_buffer1(void);

// RS232_COM
extern UART_HandleTypeDef huart3;

extern uint8_t message_complete3;
extern uint8_t rx_char3;
extern uint8_t rx_buffer3[RX_BUFFER3_SIZE];

void MX_USART3_UART_Init(void);

void send_UART3(const char *msg);
void UART3_MutexInit(void);
void clear_rx_buffer3(void);


//RS485
extern UART_HandleTypeDef huart2;

extern uint8_t message_complete2;
extern uint8_t rx_char2;
extern uint8_t rx_buffer2[RX_BUFFER2_SIZE];

void MX_USART2_UART_Init(void);
void send_UART2(const char *msg);
void clear_rx_buffer2(void);

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

#endif /* INC_UART_H_ */
