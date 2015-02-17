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
#include "ektm4c123gxl.h"
#include "functions_gpio.h"
#include "functions_uart.h"
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

#define UARTDBG_RXBUF_SIZE   30
#define UARTDBG_TXBUF_SIZE   30

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
uint8_t* RxBuf;
uint8_t iRxBuf = 0;

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

void brd_UartInit (uint8_t UARTx, void (*IntIRQ) (void))
{
    fUart_Pin Rx, Tx;
    fUart_Mod Mod;

    switch (UARTx)
    {
    case UARTDBG:
        //RxBuf = (uint8_t*) calloc(UARTDBG_RXBUF_SIZE, sizeof(uint8_t));

        Rx.Pin = UARTDBG_RX_PIN;
        Rx.Port = UARTDBG_RX_PORT;
        Rx.AlternateFunction = UARTDBG_RX_AF;
        Rx.Current = CURR_2MA;
        Rx.Type = TYPE_PP_PD;

        Tx.Pin = UARTDBG_TX_PIN;
        Tx.Port = UARTDBG_TX_PORT;
        Tx.AlternateFunction = UARTDBG_TX_AF;
        Tx.Current = CURR_2MA;
        Tx.Type = TYPE_PP_PD;

        Mod.Module = MOD_UART0;
        Mod.Rx = &Rx;
        Mod.Tx = &Tx;
        Mod.ClockSource = CLK_SYSTEM;
        Mod.Parity = PAR_NONE;
        Mod.Stop = STOP_ONE;
        Mod.Wlen = WLEN_EIGTH;
        Mod.BaudRate = BR_115200;
        Mod.Interrupts = INT_RECEIVE | INT_TRANSMIT;
        Mod.IntIRQ = IntIRQ;
        break;
    }

    fUart_Init(&Mod);
}

/*
 * Default IRQ handler for the UART Peripheral.
 */

void brd_UartDbgDefIRQHandler (void)
{
    static uint8_t tx_index = 0;
    static uint8_t rx_index = 0;
    fUart_Mod tempe = {.Module = MOD_UART0, .Interrupts = INT_RECEIVE | INT_TRANSMIT};
    uint32_t val;

    val = fUart_IntGet(&tempe);

    if (val & INT_RECEIVE)
    {
        //fUart_receiveByte(&tempe);

        brd_LedInteract(LEDR, LED_TOGGLE);

        //*(RxBuf + rx_index) = temp.RxByte;

        //if (++rx_index >= UARTDBG_RXBUF_SIZE)
            //rx_index = 0;

        tempe.Interrupts = INT_RECEIVE;
        fUart_IntClear(&tempe);
    }

    if (val & INT_TRANSMIT)
    {



        //if (++tx_index >= UARTDBG_TXBUF_SIZE)
            //tx_index = 0;

        tempe.Interrupts = INT_TRANSMIT;
        fUart_IntClear(&tempe);
    }
}

/*
 *
 */
