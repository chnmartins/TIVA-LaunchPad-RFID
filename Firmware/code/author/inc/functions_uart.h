/**
  ******************************************************************************
  * @file    C_HEADER_TEMPLATE.h
  * @author  Header template author and its contact email
  * @version Version 1 Issue 1
  * @date    Date when the file was released.	(i.e. 05-March-2014)
  * @brief   Short description about the header template.
  ******************************************************************************
  * @attention
  *
  *		Copyright Information (C)
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FUNCTIONS_UART_H
#define __FUNCTIONS_UART_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <stdint.h>
#include "functions_gpio.h"
#include "hw_memmap.h"
#include "uart.h"
#include "conf.h"

/* Exported types ------------------------------------------------------------*/

typedef struct
{
    uint32_t    Module;
    uint32_t    WordBits;
    uint32_t    StopBits;
    uint32_t    Parity;
    uint32_t    ClockSource;
    uint32_t	BaudRate;

    uint32_t*	GpioPin;
    uint32_t*	GpioPort;
    uint32_t*	GpioCurrent;
    uint32_t*	GpioType;
    uint32_t*	GpioAlternateFunction;
    uint8_t		nPins;
} fUart_InitStruct;

typedef struct
{
	void (*Init) (fUart_InitStruct* InitStruct, fGpio_Class* GpioClass);
	void (*SendByte) (uint32_t FUART_MODULEx, uint8_t byte);
	void (*SendBytes) (uint32_t FUART_MODULEx, uint8_t* bytes, uint8_t length);

	void (*IntInit) (fUart_InitStruct* InitStruct, fGpio_Class* GpioClass, uint32_t FUART_INTx, void (*UART_IRQ) (void));
	uint8_t (*IntTest) (uint32_t FUART_MODULEx, uint32_t FUART_INTx, uint8_t OnlyMaskedInt);
	uint32_t (*IntGet) (uint32_t FUART_MODULEx, uint32_t FUART_INTx, uint8_t OnlyMaskedInt);
	void (*IntStatus) (uint32_t FUART_MODULEx, uint32_t FUART_INTx, uint8_t FUART_STATUSx);
	void (*IntClear) (uint32_t FUART_MODULEx, uint32_t FUART_INTx);

} fUart_Class;

/* Exported constants --------------------------------------------------------*/
#define FUART_MODULE_0   		UART0_BASE
#define FUART_MODULE_1   		UART1_BASE
#define FUART_MODULE_2   		UART2_BASE
#define FUART_MODULE_3   		UART3_BASE
#define FUART_MODULE_4   		UART4_BASE
#define FUART_MODULE_5   		UART5_BASE
#define FUART_MODULE_6   		UART6_BASE
#define FUART_MODULE_7   		UART7_BASE

#define FUART_WORD_BITS_EIGTH  		UART_CONFIG_WLEN_8
#define FUART_WORD_BITS_SEVEN  		UART_CONFIG_WLEN_7
#define FUART_WORD_BITS_SIX    		UART_CONFIG_WLEN_6
#define FUART_WORD_BITS_FIVE   		UART_CONFIG_WLEN_5

#define FUART_STOP_BITS_ONE    				UART_CONFIG_STOP_ONE
#define FUART_STOP_BITS_TWO    				UART_CONFIG_STOP_TWO

#define FUART_PARITY_NONE    UART_CONFIG_PAR_NONE
#define FUART_PARITY_EVEN    UART_CONFIG_PAR_EVEN
#define FUART_PARITY_ODD     UART_CONFIG_PAR_ODD
#define FUART_PARITY_ONE     UART_CONFIG_PAR_ONE
#define FUART_PARITY_ZERO    UART_CONFIG_PAR_ZERO

#define FUART_CLOCK_SOURCE_SYSTEM      UART_CLOCK_SYSTEM
#define FUART_CLOCK_SOURCE_PIOSC    UART_CLOCK_PIOSC

#define FUART_BAUDRATE_110          110
#define FUART_BAUDRATE_300          300
#define FUART_BAUDRATE_600          600
#define FUART_BAUDRATE_1200         1200
#define FUART_BAUDRATE_2400         2400
#define FUART_BAUDRATE_4800         4800
#define FUART_BAUDRATE_9600         9600
#define FUART_BAUDRATE_14400        14400
#define FUART_BAUDRATE_19200        19200
#define FUART_BAUDRATE_38400        38400
#define FUART_BAUDRATE_57600        57600
#define FUART_BAUDRATE_115200       115200
#define FUART_BAUDRATE_230400       230400
#define FUART_BAUDRATE_460800       460800
#define FUART_BAUDRATE_921600       921600

#define FUART_INT_9BIT                UART_INT_9BIT
#define FUART_INT_OVERRUN_ERROR       UART_INT_OE
#define FUART_INT_BREAK_ERROR         UART_INT_BE
#define FUART_INT_PARITY_ERROR        UART_INT_PE
#define FUART_INT_FRAMING_ERROR       UART_INT_FE
#define FUART_INT_RECEIVE_TIMEOUT     UART_INT_RT
#define FUART_INT_TRANSMIT            UART_INT_TX
#define FUART_INT_RECEIVE             UART_INT_RX
#define FUART_INT_DSR                 UART_INT_DSR
#define FUART_INT_DCD                 UART_INT_DCD
#define FUART_INT_CTS                 UART_INT_CTS
#define FUART_INT_RI                  UART_INT_RI

#define FUART_STATUS_ON				 (0x01)
#define FUART_STATUS_OFF			 (0x00)

/* Exported macro ------------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
fUart_Class* fUart_CreateClass (void);
void fUart_DestroyClass (fUart_Class * class);

#ifdef __cplusplus
}
#endif

#endif /*__FUNCTIONS_UART_H */
 
