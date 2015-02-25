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
static ektm4c123gxl_Class* board;

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

void assert_failed (char* file, uint32_t line)
{
    printf("> ASSERT FAILED: Wrong parameter at file %s on line %i.\n", file, line);

    while (1)
    {

    }
}

void PB_IRQ (void)
{
    if (board->PB_IntTest(EKTM4C123GXL_PB1) == EKTM4C123GXL_STATUS_ON)
    {
        board->LED_Toggle(EKTM4C123GXL_LEDB);
        board->PB_IntClear(EKTM4C123GXL_PB1);
    }

    if (board->PB_IntTest(EKTM4C123GXL_PB2) == EKTM4C123GXL_STATUS_ON)
    {
        board->LED_Toggle(EKTM4C123GXL_LEDG);
        board->PB_IntClear(EKTM4C123GXL_PB2);
    }
}

/*
 * Main function.
 */

int main (void)
{
    board = ektm4c123gxl_CreateClass();

    board->LED_Init(EKTM4C123GXL_LEDB | EKTM4C123GXL_LEDG | EKTM4C123GXL_LEDR);
    board->PB_IntInit(EKTM4C123GXL_PB1 | EKTM4C123GXL_PB2, PB_IRQ);
    board->PB_IntStatus(EKTM4C123GXL_PB1 | EKTM4C123GXL_PB2, EKTM4C123GXL_STATUS_ON);

    board->LED_Off(EKTM4C123GXL_LEDB | EKTM4C123GXL_LEDG | EKTM4C123GXL_LEDR);

    board->UART_IntInit(EKTM4C123GXL_UART_DBG);

    board->UART_SendString(EKTM4C123GXL_UART_DBG, "I'm here and I work properly.\r\n");

	while (1)
	{
	    board->UART_Parse(EKTM4C123GXL_UART_DBG);
	}
}
