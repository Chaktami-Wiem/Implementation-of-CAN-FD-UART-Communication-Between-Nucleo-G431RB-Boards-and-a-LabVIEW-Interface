/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
FDCAN_HandleTypeDef hfdcan1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
//UART Variable
uint8_t* received_buffer = NULL ;  // Buffer de stockage
uint8_t received_data;
uint8_t my_index = 0;

// Variable TX_CANFD
FDCAN_FilterTypeDef sFilterConfig;
FDCAN_TxHeaderTypeDef TxHeader;
HAL_FDCAN_StateTypeDef myCanState;
HAL_StatusTypeDef myState;
#define MAX_TXDATA_SIZE 64

// variable FDCAN_RxFifo0Callback
uint8_t messageIndex = 0;
FDCAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[64];
#define MAX_MESSAGES 2  // Taille maximale du tableau pour stocker les messages

// variable Process_Command
char param[20];
char value_str[50];  // Read the value as a string
int value = 0;
uint16_t temp_id;

//variable send_Frame
char id_str[4] = {0};
uint32_t Tx_identifier;

//variable pour Extract_Data_Until_CR
uint8_t TxData[MAX_TXDATA_SIZE] = {0};
char hex_pair[3] = {0};

//variable RX_identifier
char id_Rx[4] = {0};
uint32_t Rx_identifier;
char ID;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void Process_Command(char *command);
void UART_Send_Response(char *response);
void send_Frame(uint8_t *Frame);
void Extract_Data_Until_CR(uint8_t *Frame);
void SET_RXHeader(uint16_t ID);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Structure pour stocker les messages reçus
typedef struct {
    uint16_t Identifier;  // Identifiant du message
    uint8_t Data[64];      // Données du message (8 octets)
} FDCAN_Message;
FDCAN_Message receivedMessages[MAX_MESSAGES];

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */


  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_FDCAN1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart2, &received_data, 1);
  myCanState = HAL_FDCAN_GetState(&hfdcan1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  }

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */



  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 1;
  hfdcan1.Init.NominalSyncJumpWidth = 5;
  hfdcan1.Init.NominalTimeSeg1 = 24;
  hfdcan1.Init.NominalTimeSeg2 = 6;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 3;
  hfdcan1.Init.DataTimeSeg1 = 3;
  hfdcan1.Init.DataTimeSeg2 = 2;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  TxHeader.Identifier = 0x7E2;  // ID de l'émetteur (STM32 avec ID 0x7E2)
  TxHeader.IdType = FDCAN_STANDARD_ID;
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;
  TxHeader.DataLength = FDCAN_DLC_BYTES_64;  // DLC = 15 → 64 octets pour CAN FD
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_ON;  // Activation du Bit Rate Switching
  TxHeader.FDFormat = FDCAN_FD_CAN;
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker = 0;

  /* Configure reception filter to Rx FIFO 0 on both FDCAN instances */
   sFilterConfig.IdType = FDCAN_STANDARD_ID;
   sFilterConfig.FilterIndex = 0;
   sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
   sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
   sFilterConfig.FilterID1 = 0x7EA;
   sFilterConfig.FilterID2 =  0x7FF;

   if(HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
   	{
   		Error_Handler();
   	}

   	if(HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
   	{
   		Error_Handler();
   	}

   	if(HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
   	{
   		Error_Handler();
   	}

   	//Cette ligne de code active une interruption qui se déclenchera lorsqu'un nouveau message est reçu dans RX FIFO0.
   	if(HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
   	{
   		Error_Handler();
   	}


  /* Activer l'interruption FDCAN dans le NVIC */
  HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 230400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_EnableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void UART_Send_Response(char *response) {
    HAL_UART_Transmit(&huart2, (uint8_t*)response, strlen(response), HAL_MAX_DELAY);

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)  // Vérifie que l'UART2 a reçu une donnée
    {
    	//Réallocation dynamique du buffer pour stocker la nouvelle donnée
    	        uint8_t* temp = (uint8_t*)realloc(received_buffer, my_index + 1);
    	        if (temp == NULL)  // Vérification de l'échec de l'allocation
    	        {
    	            HAL_UART_Transmit(&huart2, (uint8_t*)"Erreur mémoire\r\n", 16, HAL_MAX_DELAY);
    	            return;
    	        }
    	        received_buffer = temp;
        received_buffer[my_index++] = received_data;  // Stocker l'octet reçu
        // Vérifier si toute la trame est reçue
        if (received_data == '\r')
        {
        	if(received_buffer[0] == 'S')
        	{
        		 Process_Command((char*)received_buffer);
        		 myCanState = HAL_FDCAN_GetState(&hfdcan1);
        	}
        	else
        	{
        		send_Frame(received_buffer);
        		myCanState = HAL_FDCAN_GetState(&hfdcan1);
        	}

            // 5. Libérer la mémoire et réinitialiser les variables
            free(received_buffer);
            received_buffer = NULL;
            my_index = 0;
        }
        // Relancer la réception en interruption pour le prochain octet
        HAL_UART_Receive_IT(&huart2, &received_data, 1);
    }
}

void Process_Command(char *command) {
    // Lire "SET PARAM VALEUR"
        if (sscanf(command, "SET %s %s", param, value_str) == 2) {
            // Vérification si c'est un nombre hexadécimal
            if (value_str[0] == '0' && (value_str[1] == 'x' || value_str[1] == 'X')) {
                sscanf(value_str, "%x", &value);
            } else {
                sscanf(value_str, "%d", &value);
            }
        if(strcmp(param, "Init_CAN") == 0)
        		{
            		MX_FDCAN1_Init();
                    UART_Send_Response("OK");
                    return;
            	}
        else if (strcmp(param, "CLOCK_DIVIDER") == 0) {
                    switch (value) {
                        case 1: hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1; break;
                        case 2: hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV2; break;
                        case 4: hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV4; break;
                        case 6: hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV6; break;
                        case 8: hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV8; break;
                        case 10: hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV10; break;
                        case 12: hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV12; break;
                        case 14: hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV14; break;
                        case 16: hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV16; break;
                        case 18: hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV18; break;
                        case 20: hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV20; break;
                        case 22: hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV22; break;
                        case 24: hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV24; break;
                        case 26: hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV26; break;
                        case 28: hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV28; break;
                        case 30: hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV30; break;
                        default: UART_Send_Response("Valeur invalide pour CLOCK_DIVIDER\r\n"); return;
                    }
                }
        else if (strcmp(param,"FRAME_FORMAT")==0){
        	if(value ==0) hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
        	else if(value ==1) hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_NO_BRS;
        	else if(value ==2) hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
        	else { UART_Send_Response("Valeur invalide pour FRAME_FORMAT\r\n"); return; }
        }
        else if (strcmp(param, "MODE") == 0) {
            if (value == 0) hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
            else if (value == 1) hfdcan1.Init.Mode = FDCAN_MODE_RESTRICTED_OPERATION;
            else if (value == 2) hfdcan1.Init.Mode = FDCAN_MODE_BUS_MONITORING;
            else if (value == 3) hfdcan1.Init.Mode = FDCAN_MODE_INTERNAL_LOOPBACK;
            else if (value == 4) hfdcan1.Init.Mode = FDCAN_MODE_EXTERNAL_LOOPBACK;
            else { UART_Send_Response("Valeur invalide pour MODE\r\n"); return; }
        }
        else if (strcmp(param, "AUTO_RETRANSMISSION") == 0) {
        	if (value == 0) hfdcan1.Init.AutoRetransmission = DISABLE;
        	else if (value == 1) hfdcan1.Init.AutoRetransmission = ENABLE;
        	else { UART_Send_Response("Valeur invalide pour AUTO_RETRANSMISSION\r\n"); return; }
        }
        else if (strcmp(param, "TRANSMIT_PAUSE") == 0) {
        	if (value == 0) hfdcan1.Init.TransmitPause = DISABLE;
        	else if (value == 1) hfdcan1.Init.TransmitPause = ENABLE;
        	else { UART_Send_Response("Valeur invalide pour TRANSMIT_PAUSE\r\n"); return; }
        }
        else if (strcmp(param, "PROTOCOL_EXCEPTION") == 0) {
        	if (value == 0) hfdcan1.Init.ProtocolException = DISABLE;
        	else if (value == 1) hfdcan1.Init.ProtocolException = ENABLE;
        	else { UART_Send_Response("Valeur invalide pour PROTOCOL_EXCEPTION\r\n"); return; }
        }
        else if (strcmp(param, "NOMINAL_PRESCALER") == 0) {
            hfdcan1.Init.NominalPrescaler = value;
        }
        else if (strcmp(param, "NOMINAL_SYNC_JW") == 0) {
            hfdcan1.Init.NominalSyncJumpWidth = value;
        }
        else if (strcmp(param, "NOMINAL_TIME_SEG1") == 0) {
            hfdcan1.Init.NominalTimeSeg1 = value;
        }
        else if (strcmp(param, "NOMINAL_TIME_SEG2") == 0) {
            hfdcan1.Init.NominalTimeSeg2 = value;
        }
        else if (strcmp(param, "DATA_PRESCALER") == 0) {
            hfdcan1.Init.DataPrescaler = value;
        }
        else if (strcmp(param, "DATA_TIM_SEG1") == 0) {
            hfdcan1.Init.DataTimeSeg1 = value;
        }
        else if (strcmp(param, "DATA_TIM_SEG2") == 0) {
            hfdcan1.Init.DataTimeSeg2 = value;
        }
        else if (strcmp(param, "DATA_SYNC_JW") == 0) {
            hfdcan1.Init.DataSyncJumpWidth = value;
        }
        else if (strcmp(param, "STD_FILTERS_NBR") == 0) {
            hfdcan1.Init.StdFiltersNbr = value;
        }
        else if (strcmp(param, "EXT_FILTERS_NBR") == 0) {
            hfdcan1.Init.ExtFiltersNbr = value;
        }
        else if (strcmp(param, "TX_FIFO_QUEUE_MODE") == 0) {
        	if (value == 0) hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
        	else if (value == 1) hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_QUEUE_OPERATION;
        	else { UART_Send_Response("Valeur invalide pour TX_FIFO_QUEUE_MOD\r\n"); return; }
        }
        else if (strcmp(param, "RxHeader_ID") == 0) {
                   if (sscanf(value_str, "%hx", &temp_id) == 1) {
                       SET_RXHeader(temp_id);}
                       else { UART_Send_Response("Valeur invalide pour RxHeader_ID\r\n"); return; }

               }
        else {
            UART_Send_Response("Parametre inconnu\r\n");
            return;
        }
     UART_Send_Response("OK");
    } else {
     UART_Send_Response("Format invalide\r\n");
     }

}

void SET_RXHeader(uint16_t ID)
{
sFilterConfig.FilterID1 = ID;

}


void send_Frame(uint8_t *Frame)
 {
//  Vérification de la longueur minimale avant extraction
	  if (strlen((char *)Frame) < 3) return;

// Extraire l'identifiant (3 caractères après 'F')
	  strncpy(id_str, (char *)&Frame[0], 3);
	  id_str[3] = '\0';  // Ajout d'un '\0' pour éviter les erreurs
//  Convertir en hexadécimal
	  if(sscanf(id_str, "%lx", &Tx_identifier) == 1)
	  {
		 TxHeader.Identifier = Tx_identifier;
		// UART_Send_Response("ID_OK\r\n");
	  }
	  else {
	          UART_Send_Response("Erreur_config_ID\r\n");
	          free(Frame);
	       }

	  Extract_Data_Until_CR(Frame);

 }

void Extract_Data_Until_CR(uint8_t *Frame)
{
    // Allocation du buffer temporaire pour stocker la chaîne
    char *Frame_str = (char *)malloc(256 * sizeof(char));
    if (Frame_str == NULL)
    {
        HAL_UART_Transmit(&huart2, (uint8_t*)"Erreur mémoire pour Frame_str\r\n", 30, HAL_MAX_DELAY);
        return;
    }
    // Trouver le '\r'
    char *end_pos = strchr((char *)Frame, '\r');
    if (end_pos)
    {
        int length = end_pos - (char *)&Frame[4];
        if (length > 0 && length < 256 && length %2 == 0)
        {
            // Copier la section de la trame dans Frame_str
            strncpy(Frame_str, (char *)&Frame[4], length);
            Frame_str[length] = '\0';  // Ajouter la fin de chaîne
            for (int i = 0; i < length / 2 && i < MAX_TXDATA_SIZE; i++)
            {
                strncpy(hex_pair, &Frame_str[i * 2], 2);  // Extraire 2 caractères
                hex_pair[2] = '\0';  // Assurer la terminaison de chaîne
                // Convertir la sous-chaîne hexadécimale en nombre
                TxData[i] = (uint8_t)strtol(hex_pair, NULL, 16);
            }
            // Ajouter le message à la FIFO de transmission du FDCAN;
            HAL_FDCAN_Init(&hfdcan1);
            HAL_FDCAN_Start(&hfdcan1);
            HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
            while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0);
            if((myState = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData)) != HAL_OK)
            {
            	UART_Send_Response("Error send Frame To SEG\r\n");
                // Libérer la mémoire allouée pour Frame_str
                free(Frame_str);
                return;
            }
            //UART_Send_Response("Frame Sent To SEG\r\n");
            free(Frame_str);
            return;
        }
    }
    // Libérer la mémoire allouée pour Frame_str
    free(Frame_str);
}

void UART_received_Response(FDCAN_Message receivedMessages[], uint8_t messageCount)
{
    char buffer[200]; // Adapter la taille selon le besoin
    for (int j = 0; j < messageCount; j++)
    {
        int offset = snprintf(buffer, sizeof(buffer), "ID: %03X Data: ", receivedMessages[j].Identifier);

        for (int i = 0; i < 64; i++)
        {
            offset += snprintf(buffer + offset, sizeof(buffer) - offset, "%02X ", receivedMessages[j].Data[i]);
        }

        snprintf(buffer + offset, sizeof(buffer) - offset, "\r\n");
        HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
    }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
  {
    /* Retrieve Rx messages from RX FIFO0 */
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
    {
    Error_Handler();
    }
    /* Display LEDx */
    if ((RxHeader.IdType == FDCAN_STANDARD_ID) && (RxHeader.DataLength == FDCAN_DLC_BYTES_64))
    {
    	if(messageIndex < MAX_MESSAGES)
    	{
    		 receivedMessages[messageIndex].Identifier = RxHeader.Identifier;
    		 memcpy(receivedMessages[messageIndex].Data, RxData, 64);
    		 UART_received_Response(&receivedMessages[messageIndex], 1);
    		 messageIndex++;
    	}
    	if (messageIndex >= MAX_MESSAGES)
    	    {
    	        messageIndex = 0;  // Évite de réinitialiser trop tôt
    	    }

    }
  }
}




/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
