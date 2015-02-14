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

/* Exported types ------------------------------------------------------------*/
typedef struct
{
    uint32_t    Module;
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

/* Exported macro ------------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
void fUart_enableSysCtl (fUart_Mod* const Uart_Pin);


#ifdef __cplusplus
}
#endif

#endif /*__C_HEADER_TEMPLATE_H */
 
