/**
  ******************************************************************************
  * @file    C_SOURCE_TEMPLATE.C
  * @author  Source template author and its contact email
  * @version Version 1 Issue 1
  * @date    Date when the file was released.	(i.e. 05-March-2014)
  * @brief   Short description about the functionalities this file provides.
  *  @verbatim
  *			Full information on how to use this driver.
  *  @endverbatim        
  *
  ******************************************************************************
  * @attention
  *		
  *		(Copyright Information)
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "functions_uart.h"
#include "hw_memmap.h"
#include "sysctl.h"
#include "conf.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
#define ASSERT_UART(x)          ((x) == UART0_BASE || \
                                 (x) == UART1_BASE || \
                                 (x) == UART2_BASE || \
                                 (x) == UART3_BASE || \
                                 (x) == UART4_BASE || \
                                 (x) == UART5_BASE || \
                                 (x) == UART6_BASE || \
                                 (x) == UART7_BASE)


/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/*
 * Function to enable the SysCtl clock.
 */

void fUart_enableSysCtl (fUart_Mod* const Uart_Mod)
{
#ifdef  DEBUG
    ASSERT_PARAM(ASSERT_UART(Uart_Mod->Module));
#endif

    switch (Uart_Mod->Module)
    {
    case MOD_UART0:
        SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
        break;
    case MOD_UART1:
        SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
        break;
    case MOD_UART2:
        SysCtlPeripheralEnable(SYSCTL_PERIPH_UART2);
        break;
    case MOD_UART3:
        SysCtlPeripheralEnable(SYSCTL_PERIPH_UART3);
        break;
    case MOD_UART4:
        SysCtlPeripheralEnable(SYSCTL_PERIPH_UART4);
        break;
    case MOD_UART5:
        SysCtlPeripheralEnable(SYSCTL_PERIPH_UART5);
        break;
    case MOD_UART6:
        SysCtlPeripheralEnable(SYSCTL_PERIPH_UART6);
        break;
    case MOD_UART7:
        SysCtlPeripheralEnable(SYSCTL_PERIPH_UART7);
        break;
    }
}
