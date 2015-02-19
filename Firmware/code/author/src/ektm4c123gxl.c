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
#include <stdlib.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>
#include "ektm4c123gxl.h"
#include "functions_gpio.h"
#include "functions_uart.h"
#include "functions_tim.h"
#include "pin_map.h"
#include "conf.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define LEDR_PIN     GPIO_PIN_1
#define LEDR_PORT    GPIO_PORTF_BASE
#define LEDB_PIN     GPIO_PIN_2
#define LEDB_PORT    GPIO_PORTF_BASE
#define LEDG_PIN     GPIO_PIN_3
#define LEDG_PORT    GPIO_PORTF_BASE

#define PB1_PIN      GPIO_PIN_4
#define PB1_PORT     GPIO_PORTF_BASE
#define PB2_PIN      GPIO_PIN_0
#define PB2_PORT     GPIO_PORTF_BASE

#define UARTDBG_RX_PIN  GPIO_PIN_0
#define UARTDBG_RX_PORT GPIO_PORTA_BASE
#define UARTDBG_RX_AF   GPIO_PA0_U0RX

#define UARTDBG_TX_PIN  GPIO_PIN_1
#define UARTDBG_TX_PORT GPIO_PORTA_BASE
#define UARTDBG_TX_AF   GPIO_PA1_U0TX

#define UARTDBG_RXBUF_SIZE   50
#define UARTDBG_TXBUF_SIZE   50

#define UARTDBG_CMD_DELIMITER   '\n'
#define UARTDBG_CMD_HELLO       "Are you there?\r\n"
#define UARTDBG_CMD_HEY         "Yes, I'm here.\r\n"
#define UARTDBG_CMD_UNKNOWN     "I have no idea what you are talking about.\r\n"

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
fUart_Mod* UartDbg;

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/*
 * Initialize the LEDs.
 */

void brd_LedInit (uint8_t LEDx)
{
    fGpio_Pin temp = {.Current = CURR_2MA, .Direction = DIR_OUT, .Type = TYPE_PP_PU};

    if (LEDx & LEDR)
    {
        temp.Pin = LEDR_PIN, temp.Port = LEDR_PORT;
        fGpio_Init(&temp);
    }
    if (LEDx & LEDG)
    {
        temp.Pin = LEDG_PIN, temp.Port = LEDG_PORT;
        fGpio_Init(&temp);
    }
    if (LEDx & LEDB)
    {
        temp.Pin = LEDB_PIN, temp.Port = LEDB_PORT;
        fGpio_Init(&temp);
    }
}

/*
 * Interact with the LEDs.
 */

void brd_LedInteract (uint8_t LEDx, uint8_t LED_x)
{
    fGpio_Pin temp;
    void (*fnPtr) (fGpio_Pin* const);

    switch (LED_x)
    {
    case LED_ON:
        fnPtr = fGpio_setHigh;
        break;
    case LED_OFF:
        fnPtr = fGpio_setLow;
        break;
    case LED_TOGGLE:
        fnPtr = fGpio_setToggle;
        break;
    }

    if (LEDx & LEDR)
    {
        temp.Pin = LEDR_PIN, temp.Port = LEDR_PORT;
        fnPtr(&temp);
    }
    if (LEDx & LEDG)
    {
        temp.Pin = LEDG_PIN, temp.Port = LEDG_PORT;
        fnPtr(&temp);
    }
    if (LEDx & LEDB)
    {
        temp.Pin = LEDB_PIN, temp.Port = LEDB_PORT;
        fnPtr(&temp);
    }
}

/*
 * Initialize the pushbuttons.
 */

void brd_PushButtonInit (uint8_t PBx)
{
    fGpio_Pin temp = {.Current = CURR_2MA, .Direction = DIR_IN, .Type = TYPE_PP_PU};

    if (PBx & PB1)
    {
        temp.Pin = PB1_PIN, temp.Port = PB1_PORT;
        fGpio_Init(&temp);
    }
    if (PBx & PB2)
    {
        temp.Pin = PB2_PIN, temp.Port = PB2_PORT;
        fGpio_Init(&temp);
    }
}

/*
 * Read the pushbuttons.
 */

uint8_t brd_PushButtonRead (uint8_t PBx)
{
    fGpio_Pin temp;
    uint8_t val = 0;

    if (PBx & PB1)
    {
        temp.Pin = PB1_PIN, temp.Port = PB1_PORT;
        val |= fGpio_getLevel(&temp) ? PB1 : 0;
    }
    if (PBx & PB2)
    {
        temp.Pin = PB2_PIN, temp.Port = PB2_PORT;
        val |= fGpio_getLevel(&temp) ? PB2 : 0;
    }

    return ~val;
}

/*
 * Sets the external interrupts on the pushbuttons.
 */

void brd_PushButtonInitInt (uint8_t PBx, void (*IntIRQ) (void))
{
    fGpio_Pin temp = {.IntType = INTTYPE_FALLING_EDGE, .IntIRQ = IntIRQ};

    if (PBx & PB1)
    {
        temp.Pin = PB1_PIN, temp.Port = PB1_PORT;
        fGpio_IntInit(&temp);
    }
    if (PBx & PB2)
    {
        temp.Pin = PB2_PIN, temp.Port = PB2_PORT;
        fGpio_IntInit(&temp);
    }
}

/*
 * Gets the status of the interrupt and clears it if active.
 */

uint8_t brd_PushButtonGetInt (uint8_t PBx)
{
    fGpio_Pin temp;
    uint8_t val = 0;

    if (PBx & PB1)
    {
        temp.Pin = PB1_PIN, temp.Port = PB1_PORT;
        if (fGpio_IntGet(&temp) & temp.Pin)
        {
            val |= PB1;
            fGpio_IntClear(&temp);
        }
    }

    if (PBx & PB2)
    {
        temp.Pin = PB2_PIN, temp.Port = PB2_PORT;
        if (fGpio_IntGet(&temp) & temp.Pin)
        {
            val |= PB2;
            fGpio_IntClear(&temp);
        }
    }

    return val;
}

/*
 * Initializes the specified UART interface.
 */

bool brd_UartInit (uint8_t UARTx)
{
    switch (UARTx)
    {
    case UARTDBG:
    	UartDbg = (fUart_Mod*) calloc (1, sizeof(fUart_Mod));
    	if (UartDbg == (fUart_Mod*) NULL)
    		return false;
    	UartDbg->Rx = (fUart_Pin*) calloc(1, sizeof(fUart_Pin));
        if (UartDbg->Rx == (fUart_Pin*) NULL)
            return false;
    	UartDbg->Tx = (fUart_Pin*) calloc(1, sizeof(fUart_Pin));
        if (UartDbg->Tx == (fUart_Pin*) NULL)
            return false;

        UartDbg->Rx->Pin = UARTDBG_RX_PIN;
        UartDbg->Rx->Port = UARTDBG_RX_PORT;
        UartDbg->Rx->AlternateFunction = UARTDBG_RX_AF;
        UartDbg->Rx->Current = CURR_2MA;
        UartDbg->Rx->Type = TYPE_PP_PD;

        UartDbg->Tx->Pin = UARTDBG_TX_PIN;
        UartDbg->Tx->Port = UARTDBG_TX_PORT;
        UartDbg->Tx->AlternateFunction = UARTDBG_TX_AF;
        UartDbg->Tx->Current = CURR_2MA;
        UartDbg->Tx->Type = TYPE_PP_PD;

        UartDbg->Module = MOD_UART0;
        UartDbg->ClockSource = CLK_SYSTEM;
        UartDbg->Parity = PAR_NONE;
        UartDbg->Stop = STOP_ONE;
        UartDbg->Wlen = WLEN_EIGTH;
        UartDbg->BaudRate = BR_115200;

        UartDbg->Interrupts = INT_RECEIVE | INT_TRANSMIT | INT_OVERRUN_ERROR | INT_BREAK_ERROR | INT_PARITY_ERROR | INT_FRAMING_ERROR | INT_RECEIVE_TIMEOUT;
        UartDbg->IntIRQ = brd_UartDbgISR;

        UartDbg->RxBuf = (uint8_t*) calloc(UARTDBG_RXBUF_SIZE, sizeof(uint8_t));
        if (UartDbg->RxBuf == NULL)
        	return false;
        UartDbg->RxBufProcIndex = 0;
        UartDbg->RxBufUnprocIndex = 0;
        UartDbg->RxBufLength = UARTDBG_RXBUF_SIZE;

        UartDbg->TxBuf = (uint8_t*) calloc(UARTDBG_TXBUF_SIZE, sizeof(uint8_t));
        if (UartDbg->TxBuf == NULL)
        	return false;
        UartDbg->TxBufProcIndex = 0;
        UartDbg->TxBufUnprocIndex = 0;
        UartDbg->TxBufLength = UARTDBG_TXBUF_SIZE;

        fUart_Init(UartDbg);

        return true;
    default:
    	return false;
    }
}

/*
 * Interrupt Service Routine for the UART Debug interface.
 */

void brd_UartDbgISR (void)
{
	fUart_IRQHandler(UartDbg);
}

/*
 * Sends data over the UART interface.
 */

void brd_UartSend (uint8_t UARTx, const uint8_t* data)
{
    uint8_t length = 0;

    switch (UARTx)
    {
    case UARTDBG:
        length = 0;

        while (*(data++))
            length++;

        fUart_BeginTransfer(UartDbg, data - length - 1, length);

        break;
    default:


        break;
    }
}

/*
 * Parses the data received on the specified UART interface.
 */

void brd_UartParse (uint8_t UARTx)
{
    uint8_t RxBufIndex;
    uint8_t CmdIndex;
    uint8_t Cmd[20];

    switch (UARTx)
    {
    case UARTDBG:
        RxBufIndex = UartDbg->RxBufProcIndex;
        CmdIndex = 0;
        memset(Cmd, 0x00, sizeof(Cmd) * sizeof(uint8_t));

        while (RxBufIndex != UartDbg->RxBufUnprocIndex)
        {
            if (CmdIndex < sizeof(Cmd) / sizeof(uint8_t))
            {
                *(Cmd + CmdIndex) = *(UartDbg->RxBuf + RxBufIndex++);
                if (RxBufIndex >= UartDbg->RxBufLength)
                    RxBufIndex = 0;

                if (*(Cmd + CmdIndex) == UARTDBG_CMD_DELIMITER)
                {
                    if (!(strcmp((char*) Cmd, UARTDBG_CMD_HELLO)))
                    {
                        brd_UartSend(UARTDBG, UARTDBG_CMD_HEY);
                    } else {
                        brd_UartSend(UARTDBG, UARTDBG_CMD_UNKNOWN);
                    }

                    UartDbg->RxBufProcIndex = RxBufIndex;
                    CmdIndex = 0;
                    memset(Cmd, 0x00, sizeof(Cmd) * sizeof(uint8_t));
                }
                else
                {
                    CmdIndex++;
                }
            }
            else
            {
                break;
            }
        }

        break;
    default:

        break;
    }
}

/*
 *
 */
fTim_Mod Tim;

void brd_TimInit (void)
{

    Tim.Module = MOD_TIM0;
    Tim.Type = TYPE_PERIODIC_UP;
    Tim.sTime = 0.1;
    Tim.Int = INT_TIMEOUT;
    Tim.IntIRQ = test;

    fTim_Init(&Tim);
}

/*
 *
 */

void test (void)
{
    fTim_IRQ(&Tim);
    brd_LedInteract(LEDR, LED_TOGGLE);
}
