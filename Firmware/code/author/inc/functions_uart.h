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
#ifndef __C_HEADER_TEMPLATE_H
#define __C_HEADER_TEMPLATE_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <stdint.h>
#include "functions_gpio.h"
#include "hw_memmap.h"
#include "uart.h"

/* Exported types ------------------------------------------------------------*/

typedef struct
{
    uint32_t    Pin;
    uint32_t    Port;
    uint32_t    AlternateFunction;
    uint32_t    Current;
    uint32_t    Type;
} fUart_Pin;

typedef struct
{
    uint32_t    Module;
    fUart_Pin*   Rx;
    fUart_Pin*   Tx;
    uint32_t    Wlen;
    uint32_t    Stop;
    uint32_t    Parity;
    uint32_t    ClockSource;
    uint32_t    BaudRate;
} fUart_Mod;

/* Exported constants --------------------------------------------------------*/
#define MOD_UART0   UART0_BASE
#define MOD_UART1   UART1_BASE
#define MOD_UART2   UART2_BASE
#define MOD_UART3   UART3_BASE
#define MOD_UART4   UART4_BASE
#define MOD_UART5   UART5_BASE
#define MOD_UART6   UART6_BASE
#define MOD_UART7   UART7_BASE

#define WLEN_EIGTH  UART_CONFIG_WLEN_8
#define WLEN_SEVEN  UART_CONFIG_WLEN_7
#define WLEN_SIX    UART_CONFIG_WLEN_6
#define WLEN_FIVE   UART_CONFIG_WLEN_5

#define STOP_ONE    UART_CONFIG_STOP_ONE
#define STOP_TWO    UART_CONFIG_STOP_TWO

#define PAR_EVEN    UART_CONFIG_PAR_EVEN
#define PAR_ODD     UART_CONFIG_PAR_ODD
#define PAR_ONE     UART_CONFIG_PAR_ONE
#define PAR_ZERO    UART_CONFIG_PAR_ZERO

#define CLK_SYSTEM      UART_CLOCK_SYSTEM
#define CLK_INTERNAL    UART_CLOCK_PIOSC

#define BR_110          110
#define BR_300          300
#define BR_600          600
#define BR_1200         1200
#define BR_2400         2400
#define BR_4800         4800
#define BR_9600         9600
#define BR_14400        14400
#define BR_19200        19200
#define BR_38400        38400
#define BR_57600        57600
#define BR_115200       115200
#define BR_230400       230400
#define BR_460800       460800
#define BR_921600       921600

/* Exported macro ------------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
void fUart_enableSysCtl (fUart_Mod* const Uart_Pin);
void fUart_setPins (fUart_Mod* const Uart_Mod);
void fUart_DeInit (fUart_Mod* const Uart_Mod);
void fUart_setConfig (fUart_Mod* const Uart_Mod);
void fUart_Start (fUart_Mod* const Uart_Mod);
void fUart_Init (fUart_Mod* const Uart_Mod);


#ifdef __cplusplus
}
#endif

#endif /*__C_HEADER_TEMPLATE_H */
 
