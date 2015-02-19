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
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "conf.h"
#include "ektm4c123gxl.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

void assert_failed (char* file, uint32_t line)
{
    printf("> ASSERT FAILED: Wrong parameter at file %s on line %i.\n", file, line);

    while (1)
    {

    }
}

void IRQHandler (void)
{
    uint8_t val = brd_PushButtonGetInt(PB1 | PB2);

    if (val & PB1)
    {
        brd_LedInteract(LEDR, LED_ON);
        brd_LedInteract(LEDG, LED_ON);
    }
    if (val & PB2)
    {
        brd_LedInteract(LEDR, LED_OFF);
        brd_LedInteract(LEDG, LED_OFF);
    }
}

/*
 * Main function.
 */

int main (void)
{
    brd_LedInit(LEDG | LEDR | LEDB);
    brd_PushButtonInit(PB1 | PB2);
    brd_PushButtonInitInt(PB1 | PB2, IRQHandler);

    brd_LedInteract(LEDG | LEDR | LEDB, LED_OFF);

    brd_UartInit(UARTDBG);

	while (1)
	{
	    brd_UartParse(UARTDBG);
	    brd_delay(1);
	    brd_LedInteract(LEDR, LED_TOGGLE);
	}
}
