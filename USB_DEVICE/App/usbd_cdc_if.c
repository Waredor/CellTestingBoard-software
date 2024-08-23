/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_cdc_if.c
  * @version        : v2.0_Cube
  * @brief          : Usb device for Virtual Com Port.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "usbd_cdc_if.h"

/* USER CODE BEGIN INCLUDE */
#include "string.h"
/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief Usb device library.
  * @{
  */

/** @addtogroup USBD_CDC_IF
  * @{
  */

/** @defgroup USBD_CDC_IF_Private_TypesDefinitions USBD_CDC_IF_Private_TypesDefinitions
  * @brief Private types.
  * @{
  */

/* USER CODE BEGIN PRIVATE_TYPES */

/* USER CODE END PRIVATE_TYPES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Defines USBD_CDC_IF_Private_Defines
  * @brief Private defines.
  * @{
  */

/* USER CODE BEGIN PRIVATE_DEFINES */
/* USER CODE END PRIVATE_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Macros USBD_CDC_IF_Private_Macros
  * @brief Private macros.
  * @{
  */

/* USER CODE BEGIN PRIVATE_MACRO */

/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Variables USBD_CDC_IF_Private_Variables
  * @brief Private variables.
  * @{
  */
/* Create buffer for reception and transmission           */
/* It's up to user to redefine and/or remove those define */
/** Received data over USB are stored in this buffer      */
uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];

/** Data to send over USB CDC are stored in this buffer   */
uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

/* USER CODE BEGIN PRIVATE_VARIABLES */

/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_Variables USBD_CDC_IF_Exported_Variables
  * @brief Public variables.
  * @{
  */

extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE BEGIN EXPORTED_VARIABLES */
extern uint8_t receive_buf[5];

extern char status1[5];
extern char status2[5];
extern char status3[5];
extern char status4[5];
extern char status5[5];

extern int flg;
/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_FunctionPrototypes USBD_CDC_IF_Private_FunctionPrototypes
  * @brief Private functions declaration.
  * @{
  */

static int8_t CDC_Init_FS(void);
static int8_t CDC_DeInit_FS(void);
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Receive_FS(uint8_t* pbuf, uint32_t *Len);

/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */
int cell_selector(char* receive_buf)
{
	char str1[] = "cell1\n";
	char str2[] = "cell2\n";
	char str3[] = "cell3\n";
	if (strcmp(receive_buf, str1) == 0)
	{
		flg = 1;
	}

	else if (strcmp(receive_buf, str2) == 0)
	{
		flg = 2;
	}

	else if (strcmp(receive_buf, str3) == 0)
	{
		flg = 3;
	}
	else {
		flg = 999;
	}
return flg;

}

char *cell_switch(int flg, char *status1, char *status2, char *status3)
{
	if (flg ==1)
	{
		char st1[] = "ACT  ";
		char st2[] = "OFF  ";
		char st3[] = "OFF  ";
		strcpy(status1, st1);
		strcpy(status2, st2);
		strcpy(status3, st3);
		HAL_GPIO_WritePin(GPIOA, cell2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, cell3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_TogglePin(GPIOA, cell1_Pin);
		return status1;
		return status2;
		return status3;
	}

	else if (flg == 2)
	{
		char st1[] = "OFF  ";
		char st2[] = "ACT  ";
		char st3[] = "OFF  ";
		strcpy(status1, st1);
		strcpy(status2, st2);
		strcpy(status3, st3);
		HAL_GPIO_WritePin(GPIOA, cell1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, cell3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_TogglePin(GPIOA, cell2_Pin);
		return status1;
		return status2;
		return status3;
	}

	else if (flg == 3)
	{
		char st1[] = "OFF  ";
		char st2[] = "OFF  ";
		char st3[] = "ACT  ";
		strcpy(status1, st1);
		strcpy(status2, st2);
		strcpy(status3, st3);
		HAL_GPIO_WritePin(GPIOA, cell1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, cell2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_TogglePin(GPIOA, cell3_Pin);
		return status1;
		return status2;
		return status3;
	}
	else if (flg == 999){
		char st1[] = "OFF  ";
		char st2[] = "OFF  ";
		char st3[] = "OFF  ";
		strcpy(status1, st1);
		strcpy(status2, st2);
		strcpy(status3, st3);
		return status1;
		return status2;
		return status3;
	}
return 0;
}

char *Buf_compile(char *Buf, const char *status1, const char *status2, const char *status3, const char *status4, const char *status5, float volt1, float volt2, float volt3, float volt4, float volt5, float cur1, float cur2, float cur3, float cur4, float cur5)
{
	memset(Buf, '\0', 250);
	for (int i = 1; i<=5; i++)
	{
		switch(i)
		{
			case 1: ; //ячейка с номером 1
				char cell1[9] = "cell 1  ";
				Buf_compile_str(Buf, cell1, status1, volt1, cur1);
				break;
			case 2: ; //ячейка с номером 2
				char cell2[9] = "cell 2  ";
				Buf_compile_str(Buf, cell2, status2, volt2, cur2);
				break;
			case 3: ; //ячейка с номером 3
				char cell3[9] = "cell 3  ";
				Buf_compile_str(Buf, cell3, status3, volt3, cur3);
				break;
			case 4: ; //ячейка с номером 4
				char cell4[9] = "cell 4  ";
				Buf_compile_str(Buf, cell4, status4, volt4, cur4);
				break;
			case 5: ; //ячейка с номером 5
				char cell5[9] = "cell 5  ";
				Buf_compile_str(Buf, cell5, status5, volt5, cur5);
				char space[] ="\n";
				strcat(Buf, space);
				break;
			default:
				break;
		}
	}
	return Buf;
}

char *Buf_compile_str(char *Buf, char *cell, const char *status, float volt, float cur) /*Эта функция собирает данные об ячейке в строку*/
{
	char VOLT[] = " V  ";
	char CUR[] = " A\n";
	char numvolt[5];
	char numcur[5];
	strcat(Buf, cell);
	strcat(Buf, status);
	sprintf(numvolt, "%.2f", volt);
	strcat(Buf, numvolt);
	strcat(Buf, VOLT);
	sprintf(numcur, "%.2f", cur);
	strcat(Buf, numcur);
	strcat(Buf, CUR);
	return Buf;
}
/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
  * @}
  */

USBD_CDC_ItfTypeDef USBD_Interface_fops_FS =
{
  CDC_Init_FS,
  CDC_DeInit_FS,
  CDC_Control_FS,
  CDC_Receive_FS
};

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Initializes the CDC media low layer over the FS USB IP
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Init_FS(void)
{
  /* USER CODE BEGIN 3 */
  /* Set Application Buffers */
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
  return (USBD_OK);
  /* USER CODE END 3 */
}

/**
  * @brief  DeInitializes the CDC media low layer
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_DeInit_FS(void)
{
  /* USER CODE BEGIN 4 */
  return (USBD_OK);
  /* USER CODE END 4 */
}

/**
  * @brief  Manage the CDC class requests
  * @param  cmd: Command code
  * @param  pbuf: Buffer containing command data (request parameters)
  * @param  length: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
  /* USER CODE BEGIN 5 */
  switch(cmd)
  {
    case CDC_SEND_ENCAPSULATED_COMMAND:

    break;

    case CDC_GET_ENCAPSULATED_RESPONSE:

    break;

    case CDC_SET_COMM_FEATURE:

    break;

    case CDC_GET_COMM_FEATURE:

    break;

    case CDC_CLEAR_COMM_FEATURE:

    break;

  /*******************************************************************************/
  /* Line Coding Structure                                                       */
  /*-----------------------------------------------------------------------------*/
  /* Offset | Field       | Size | Value  | Description                          */
  /* 0      | dwDTERate   |   4  | Number |Data terminal rate, in bits per second*/
  /* 4      | bCharFormat |   1  | Number | Stop bits                            */
  /*                                        0 - 1 Stop bit                       */
  /*                                        1 - 1.5 Stop bits                    */
  /*                                        2 - 2 Stop bits                      */
  /* 5      | bParityType |  1   | Number | Parity                               */
  /*                                        0 - None                             */
  /*                                        1 - Odd                              */
  /*                                        2 - Even                             */
  /*                                        3 - Mark                             */
  /*                                        4 - Space                            */
  /* 6      | bDataBits  |   1   | Number Data bits (5, 6, 7, 8 or 16).          */
  /*******************************************************************************/
    case CDC_SET_LINE_CODING:

    break;

    case CDC_GET_LINE_CODING:

    break;

    case CDC_SET_CONTROL_LINE_STATE:

    break;

    case CDC_SEND_BREAK:

    break;

  default:
    break;
  }

  return (USBD_OK);
  /* USER CODE END 5 */
}

/**
  * @brief  Data received over USB OUT endpoint are sent over CDC interface
  *         through this function.
  *
  *         @note
  *         This function will issue a NAK packet on any OUT packet received on
  *         USB endpoint until exiting this function. If you exit this function
  *         before transfer is complete on CDC interface (ie. using DMA controller)
  *         it will result in receiving more data while previous ones are still
  *         not sent.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
{
  /* USER CODE BEGIN 6 */
	USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
	USBD_CDC_ReceivePacket(&hUsbDeviceFS);
	uint8_t len = (uint8_t) *Len;
	memset(receive_buf, '\0', 5);
	memcpy(receive_buf, Buf, len);
	memset(Buf, '\0', len);
	return (USBD_OK);
  /* USER CODE END 6 */
}

/**
  * @brief  CDC_Transmit_FS
  *         Data to send over USB IN endpoint are sent over CDC interface
  *         through this function.
  *         @note
  *
  *
  * @param  Buf: Buffer of data to be sent
  * @param  Len: Number of data to be sent (in bytes)
  * @retval USBD_OK if all operations are OK else USBD_FAIL or USBD_BUSY
  */
uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 7 */
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
  if (hcdc->TxState != 0){
    return USBD_BUSY;
  }
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, Buf, Len);
  result = USBD_CDC_TransmitPacket(&hUsbDeviceFS);
  /* USER CODE END 7 */
  return result;
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */

/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @}
  */

/**
  * @}
  */
