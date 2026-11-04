/*
 * uart.c
 *
 *  Created on: Jun 30, 2025
 *      Author: Enzo Perrier
 */

#include "uart.h"
#include "main.h"
#include "cmsis_os.h"

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_uart.h"
#include "stm32f1xx_it.h"

#include <string.h> //DEBUG
#include <stdio.h>

#define RX_TIMEOUT_MS 200 // 200 ms timeout

// RS232_418
UART_HandleTypeDef huart1;

uint8_t message_complete1 = 0;
uint8_t rx_char1 = 0;
uint8_t rx_buffer1[RX_BUFFER1_SIZE] = {0};
uint16_t rx_index1 = 0;

volatile uint32_t last_rx_tick1 = 0;

// RS232_COM
UART_HandleTypeDef huart3;

uint8_t message_complete3 = 0;
uint8_t rx_char3 = 0;
uint8_t rx_buffer3[RX_BUFFER3_SIZE] = {0};
uint16_t rx_index3 = 0;

// RS485
UART_HandleTypeDef huart2;

uint8_t message_complete2 = 0;
uint8_t rx_char2 = 0;
uint8_t rx_buffer2[RX_BUFFER2_SIZE] = {0};
uint16_t rx_index2 = 0;

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
  HAL_UART_Receive_IT(&huart1, &rx_char1, 1);

  /* USER CODE END USART1_Init 2 */
}

void send_UART1(const char *msg)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
}

void clear_rx_buffer1(void)
{
  memset(rx_buffer1, 0, sizeof(rx_buffer1));
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
  HAL_UART_Receive_IT(&huart2, &rx_char2, 1);

  /* USER CODE END USART2_Init 2 */
}

void send_UART2(const char *msg)
{
  HAL_GPIO_WritePin(RTS_485_GPIO_Port, RTS_485_Pin, GPIO_PIN_SET);
  HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
}

void clear_rx_buffer2(void)
{
  memset(rx_buffer2, 0, sizeof(rx_buffer2));
}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;

  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN USART3_Init 2 */
  HAL_UART_Receive_IT(&huart3, &rx_char3, 1);
  /* USER CODE END USART3_Init 2 */
}

//MUTEX pour protéger l'envoi car le thread statemachine ecrit en meme temps que la mesure de courant tension

static osMutexId_t uart3Mutex = NULL;

void UART3_MutexInit(void)
{
  uart3Mutex = osMutexNew(NULL);
}

void send_UART3(const char *msg)
{
  // Pas de mutex avant le démarrage du scheduler, ni dans Error_Handler (IRQ coupées)
  uint8_t use_mutex = (uart3Mutex != NULL) &&
                      (osKernelGetState() == osKernelRunning) &&
                      !__get_PRIMASK();

  if (use_mutex) osMutexAcquire(uart3Mutex, osWaitForever);
  HAL_UART_Transmit(&huart3, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
  if (use_mutex) osMutexRelease(uart3Mutex);
}

void clear_rx_buffer3(void)
{
  memset(rx_buffer3, 0, sizeof(rx_buffer3));
}

// Callback
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  //HAL_GPIO_TogglePin(OUT4_GPIO_Port, OUT4_Pin); // DEBUG

  // RS232_418
  if (huart->Instance == USART1)
  {
    // Stocker le tick de réception à chaque char reçu
    last_rx_tick1 = HAL_GetTick();

    if (rx_char1 != '\0' && rx_index1 < RX_BUFFER1_SIZE - 1)
    {
      rx_buffer1[rx_index1++] = rx_char1;
    }
    else
    {
      rx_buffer1[rx_index1] = '\0';
      message_complete1 = 1;
      rx_index1 = 0;
    }

    HAL_UART_Receive_IT(&huart1, &rx_char1, 1);
  }

  // RS232_COM
  if (huart->Instance == USART3)
  {
    if (rx_char3 != '\r' && rx_index3 < RX_BUFFER3_SIZE - 1)
    {
      rx_buffer3[rx_index3++] = rx_char3;
    }
    else
    {
      rx_buffer3[rx_index3] = '\r';
      message_complete3 = 1;
      rx_index3 = 0;
    }

    HAL_UART_Receive_IT(&huart3, &rx_char3, 1);
    // send_UART3("COM!"); // DEBUG
  }

  // RS485
  if (huart->Instance == USART2)
  {
    if (rx_char2 != '\0' && rx_index2 < RX_BUFFER2_SIZE - 1)
    {
      rx_buffer2[rx_index2++] = rx_char2;
    }
    else
    {

      rx_buffer2[rx_index2] = '\0';
      message_complete2 = 1;
      rx_index2 = 0;
    }
    HAL_UART_Receive_IT(&huart2, &rx_char2, 1);
  }
}

void Check_UART1_Timeout(void)
{
  if (!message_complete1 && rx_index1 > 0)
  {
    if ((HAL_GetTick() - last_rx_tick1) > RX_TIMEOUT_MS)
    {

      // Sécurité : s'assurer de ne pas dépasser la taille du buffer
      if (rx_index1 >= RX_BUFFER1_SIZE)
      {
        rx_index1 = RX_BUFFER1_SIZE - 1;
      }

      rx_buffer1[rx_index1] = '\0'; // Fin de chaîne propre
      message_complete1 = 1;        // Signal que le message est complet
      rx_index1 = 0;                // Réinitialisation de l'index
    }
  }
}
