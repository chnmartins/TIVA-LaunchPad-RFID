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
#include "pin_map.h"
#include "conf.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define EKTM4C123GXL_LEDR_PIN     FGPIO_PIN_1
#define EKTM4C123GXL_LEDR_PORT    FGPIO_PORT_F
#define EKTM4C123GXL_LEDB_PIN     FGPIO_PIN_2
#define EKTM4C123GXL_LEDB_PORT    FGPIO_PORT_F
#define EKTM4C123GXL_LEDG_PIN     FGPIO_PIN_3
#define EKTM4C123GXL_LEDG_PORT    FGPIO_PORT_F

#define EKTM4C123GXL_PB1_PIN      FGPIO_PIN_4
#define EKTM4C123GXL_PB1_PORT     FGPIO_PORT_F
#define EKTM4C123GXL_PB2_PIN      FGPIO_PIN_0
#define EKTM4C123GXL_PB2_PORT     FGPIO_PORT_F

#define	EKTM4C123GXL_UARTDBG_MODULE			FUART_MODULE_0
#define	EKTM4C123GXL_UARTDBG_WORD_BITS		FUART_WORD_BITS_EIGTH
#define EKTM4C123GXL_UARTDBG_STOP_BITS		FUART_STOP_BITS_ONE
#define EKTM4C123GXL_UARTDBG_PARITY			FUART_PARITY_NONE
#define EKTM4C123GXL_UARTDBG_CLOCKSOURCE	FUART_CLOCK_SOURCE_SYSTEM
#define EKTM4C123GXL_UARTDBG_BAUDRATE		FUART_BAUDRATE_115200

#define EKTM4C123GXL_UARTDBG_RX_PIN  		FGPIO_PIN_0
#define EKTM4C123GXL_UARTDBG_RX_PORT 		FGPIO_PORT_A
#define EKTM4C123GXL_UARTDBG_RX_AF   		GPIO_PA0_U0RX
#define EKTM4C123GXL_UARTDBG_RX_TYPE		FGPIO_TYPE_PUSH_PULL_PULLDOWN
#define EKTM4C123GXL_UARTDBG_RX_CURRENT		FGPIO_CURRENT_2MA

#define EKTM4C123GXL_UARTDBG_TX_PIN  		FGPIO_PIN_1
#define EKTM4C123GXL_UARTDBG_TX_PORT 		FGPIO_PORT_A
#define EKTM4C123GXL_UARTDBG_TX_AF   		GPIO_PA1_U0TX
#define EKTM4C123GXL_UARTDBG_TX_TYPE		FGPIO_TYPE_PUSH_PULL_PULLDOWN
#define EKTM4C123GXL_UARTDBG_TX_CURRENT		FGPIO_CURRENT_2MA

#define EKTM4C123GXL_UARTDBG_RXBUF_SIZE   50
#define EKTM4C123GXL_UARTDBG_TXBUF_SIZE   50

#define EKTM4C123GXL_UARTDBG_CMD_DELIMITER   '\n'
#define EKTM4C123GXL_UARTDBG_CMD_HELLO       "Are you there?\r\n"
#define EKTM4C123GXL_UARTDBG_CMD_HEY         "Yes, I'm here.\r\n"
#define EKTM4C123GXL_UARTDBG_CMD_UNKNOWN     "I have no idea what you are talking about.\r\n"

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
fGpio_Class* GpioClass;
fUart_Class* UartClass;

/* Private function prototypes -----------------------------------------------*/
static void ektm4c123gxl_LED_Init (uint8_t LEDx);
static void ektm4c123gxl_LED_On (uint8_t LEDx);
static void ektm4c123gxl_LED_Off (uint8_t LEDx);
static void ektm4c123gxl_LED_Toggle (uint8_t LEDx);

static void ektm4c123gxl_PB_Init (uint8_t PBx);
static uint8_t ektm4c123gxl_PB_Read (uint8_t EKTM4C123GXL_PBx);
static void ektm4c123gxl_PB_IntInit (uint8_t PBx, void (*PB_INTIRQ) (void));
static uint8_t ektm4c123gxl_PB_IntTest (uint8_t EKTM4C123GXL_PBx);
static void ektm4c123gxl_PB_IntClear (uint8_t EKTM4C123GXL_PBx);
static void ektm4c123gxl_PB_IntStatus (uint8_t EKTM4C123GXL_PBx, uint8_t EKTM4C123GXL_STATUSx);

static uint8_t ektm4c123gxl_UART_IntInit (uint8_t EKTM4C123GXL_UARTx);
static void ektm4c123gxl_UartDbg_Irq (void);

/* Private functions ---------------------------------------------------------*/

/*
 *
 */

ektm4c123gxl_Class* ektm4c123gxl_CreateClass (void)
{
    ektm4c123gxl_Class* temp = malloc(sizeof(ektm4c123gxl_Class));
    if (temp == NULL)
        return NULL;

    GpioClass = fGpio_CreateClass();
    if (GpioClass == NULL)
        return NULL;

    UartClass = fUart_CreateClass();
    if (UartClass == NULL)
    	return NULL;

    temp->LED_Init = ektm4c123gxl_LED_Init;
    temp->LED_Off = ektm4c123gxl_LED_Off;
    temp->LED_On = ektm4c123gxl_LED_On;
    temp->LED_Toggle = ektm4c123gxl_LED_Toggle;

    temp->PB_Init = ektm4c123gxl_PB_Init;
    temp->PB_Read = ektm4c123gxl_PB_Read;
    temp->PB_IntInit = ektm4c123gxl_PB_IntInit;
    temp->PB_IntStatus = ektm4c123gxl_PB_IntStatus;
    temp->PB_IntClear = ektm4c123gxl_PB_IntClear;
    temp->PB_IntTest = ektm4c123gxl_PB_IntTest;

    return temp;
}

/*
 *
 */

void ektm4c123gxl_DestroyClass (ektm4c123gxl_Class* class)
{
    free(class);
}

/*
 * Wrapper functions LEDs.
 */

static void ektm4c123gxl_LED_Init (uint8_t EKTM4C123GXL_LEDx)
{
    if (EKTM4C123GXL_LEDx & EKTM4C123GXL_LEDR)
        GpioClass->InitOutput(EKTM4C123GXL_LEDR_PIN, EKTM4C123GXL_LEDR_PORT, FGPIO_TYPE_PUSH_PULL_PULLDOWN, FGPIO_CURRENT_2MA);
    if (EKTM4C123GXL_LEDx & EKTM4C123GXL_LEDB)
        GpioClass->InitOutput(EKTM4C123GXL_LEDB_PIN, EKTM4C123GXL_LEDB_PORT, FGPIO_TYPE_PUSH_PULL_PULLDOWN, FGPIO_CURRENT_2MA);
    if (EKTM4C123GXL_LEDx & EKTM4C123GXL_LEDG)
        GpioClass->InitOutput(EKTM4C123GXL_LEDG_PIN, EKTM4C123GXL_LEDG_PORT, FGPIO_TYPE_PUSH_PULL_PULLDOWN, FGPIO_CURRENT_2MA);
}

/*
 * Wrapper functions LEDs.
 */

static void ektm4c123gxl_LED_On (uint8_t EKTM4C123GXL_LEDx)
{
    if (EKTM4C123GXL_LEDx & EKTM4C123GXL_LEDR)
        GpioClass->OutputInteract(EKTM4C123GXL_LEDR_PIN, EKTM4C123GXL_LEDR_PORT, FGPIO_OUTPUT_HIGH);
    if (EKTM4C123GXL_LEDx & EKTM4C123GXL_LEDB)
        GpioClass->OutputInteract(EKTM4C123GXL_LEDB_PIN, EKTM4C123GXL_LEDB_PORT, FGPIO_OUTPUT_HIGH);
    if (EKTM4C123GXL_LEDx & EKTM4C123GXL_LEDG)
        GpioClass->OutputInteract(EKTM4C123GXL_LEDG_PIN, EKTM4C123GXL_LEDG_PORT, FGPIO_OUTPUT_HIGH);
}

/*
 * Wrapper functions LEDs.
 */

static void ektm4c123gxl_LED_Off (uint8_t EKTM4C123GXL_LEDx)
{
    if (EKTM4C123GXL_LEDx & EKTM4C123GXL_LEDR)
        GpioClass->OutputInteract(EKTM4C123GXL_LEDR_PIN, EKTM4C123GXL_LEDR_PORT, FGPIO_OUTPUT_LOW);
    if (EKTM4C123GXL_LEDx & EKTM4C123GXL_LEDB)
        GpioClass->OutputInteract(EKTM4C123GXL_LEDB_PIN, EKTM4C123GXL_LEDB_PORT, FGPIO_OUTPUT_LOW);
    if (EKTM4C123GXL_LEDx & EKTM4C123GXL_LEDG)
        GpioClass->OutputInteract(EKTM4C123GXL_LEDG_PIN, EKTM4C123GXL_LEDG_PORT, FGPIO_OUTPUT_LOW);
}

/*
 * Wrapper functions LEDs.
 */

static void ektm4c123gxl_LED_Toggle (uint8_t EKTM4C123GXL_LEDx)
{
    if (EKTM4C123GXL_LEDx & EKTM4C123GXL_LEDR)
        GpioClass->OutputInteract(EKTM4C123GXL_LEDR_PIN, EKTM4C123GXL_LEDR_PORT, FGPIO_OUTPUT_TOGGLE);
    if (EKTM4C123GXL_LEDx & EKTM4C123GXL_LEDB)
        GpioClass->OutputInteract(EKTM4C123GXL_LEDB_PIN, EKTM4C123GXL_LEDB_PORT, FGPIO_OUTPUT_TOGGLE);
    if (EKTM4C123GXL_LEDx & EKTM4C123GXL_LEDG)
        GpioClass->OutputInteract(EKTM4C123GXL_LEDG_PIN, EKTM4C123GXL_LEDG_PORT, FGPIO_OUTPUT_TOGGLE);
}

/*
 * Wrapper functions PBs.
 */

static void ektm4c123gxl_PB_Init (uint8_t EKTM4C123GXL_PBx)
{
    if (EKTM4C123GXL_PBx & EKTM4C123GXL_PB1)
        GpioClass->InitInput(EKTM4C123GXL_PB1_PIN, EKTM4C123GXL_PB1_PORT, FGPIO_TYPE_PUSH_PULL_PULLUP, FGPIO_CURRENT_2MA);
    if (EKTM4C123GXL_PBx & EKTM4C123GXL_PB2)
        GpioClass->InitInput(EKTM4C123GXL_PB2_PIN, EKTM4C123GXL_PB2_PORT, FGPIO_TYPE_PUSH_PULL_PULLUP, FGPIO_CURRENT_2MA);
}

/*
 * Wrapper functions PBs.
 */

static uint8_t ektm4c123gxl_PB_Read (uint8_t EKTM4C123GXL_PBx)
{
    if (EKTM4C123GXL_PBx & EKTM4C123GXL_PB1)
    {
        if (GpioClass->InputRead(EKTM4C123GXL_PB1_PIN, EKTM4C123GXL_PB1_PORT) == FGPIO_INPUT_HIGH)
            return EKTM4C123GXL_STATUS_OFF;
    }
    else if (EKTM4C123GXL_PBx & EKTM4C123GXL_PB2)
    {
        if (GpioClass->InputRead(EKTM4C123GXL_PB2_PIN, EKTM4C123GXL_PB2_PORT) == FGPIO_INPUT_HIGH)
            return EKTM4C123GXL_STATUS_OFF;
    }

    return EKTM4C123GXL_STATUS_ON;
}

/*
 * Wrapper functions PBs.
 */

static void ektm4c123gxl_PB_IntInit (uint8_t EKTM4C123GXL_PBx, void (*EKTM4C123GXL_PB_INTIRQ) (void))
{
    if (EKTM4C123GXL_PBx & EKTM4C123GXL_PB1)
        GpioClass->InitInputInt(EKTM4C123GXL_PB1_PIN, EKTM4C123GXL_PB1_PORT, FGPIO_TYPE_PUSH_PULL_PULLUP, FGPIO_CURRENT_2MA, FGPIO_INTTYPE_FALLING_EDGE, EKTM4C123GXL_PB_INTIRQ);
    if (EKTM4C123GXL_PBx & EKTM4C123GXL_PB2)
        GpioClass->InitInputInt(EKTM4C123GXL_PB2_PIN, EKTM4C123GXL_PB2_PORT, FGPIO_TYPE_PUSH_PULL_PULLUP, FGPIO_CURRENT_2MA, FGPIO_INTTYPE_FALLING_EDGE, EKTM4C123GXL_PB_INTIRQ);
}

/*
 * Wrapper functions PBs.
 */

static uint8_t ektm4c123gxl_PB_IntTest (uint8_t EKTM4C123GXL_PBx)
{
    if (EKTM4C123GXL_PBx & EKTM4C123GXL_PB1)
    {
        if (GpioClass->IntTest(EKTM4C123GXL_PB1_PIN, EKTM4C123GXL_PB1_PORT) == FGPIO_INT_ON)
            return EKTM4C123GXL_STATUS_ON;
    }
    else if (EKTM4C123GXL_PBx & EKTM4C123GXL_PB2)
    {
        if (GpioClass->IntTest(EKTM4C123GXL_PB2_PIN, EKTM4C123GXL_PB2_PORT) == FGPIO_INT_ON)
            return EKTM4C123GXL_STATUS_ON;
    }

    return EKTM4C123GXL_STATUS_OFF;
}

/*
 * Wrapper functions PBs.
 */

static void ektm4c123gxl_PB_IntClear (uint8_t EKTM4C123GXL_PBx)
{
    if (EKTM4C123GXL_PBx & EKTM4C123GXL_PB1)
        GpioClass->IntClear(EKTM4C123GXL_PB1_PIN, EKTM4C123GXL_PB1_PORT);
    if (EKTM4C123GXL_PBx & EKTM4C123GXL_PB2)
        GpioClass->IntClear(EKTM4C123GXL_PB2_PIN, EKTM4C123GXL_PB2_PORT);
}

/*
 * Wrapper functions PBs.
 */

static void ektm4c123gxl_PB_IntStatus (uint8_t EKTM4C123GXL_PBx, uint8_t EKTM4C123GXL_STATUSx)
{
    if (EKTM4C123GXL_PBx & EKTM4C123GXL_PB1)
        (EKTM4C123GXL_STATUSx == EKTM4C123GXL_STATUS_ON) ?  GpioClass->IntStatus(EKTM4C123GXL_PB1_PIN, EKTM4C123GXL_PB1_PORT, FGPIO_INT_ON) :  GpioClass->IntStatus(EKTM4C123GXL_PB1_PIN, EKTM4C123GXL_PB1_PORT, FGPIO_INT_OFF);
    if (EKTM4C123GXL_PBx & EKTM4C123GXL_PB2)
        (EKTM4C123GXL_STATUSx == EKTM4C123GXL_STATUS_ON) ?  GpioClass->IntStatus(EKTM4C123GXL_PB2_PIN, EKTM4C123GXL_PB2_PORT, FGPIO_INT_ON) :  GpioClass->IntStatus(EKTM4C123GXL_PB2_PIN, EKTM4C123GXL_PB2_PORT, FGPIO_INT_OFF);
}

/*
 * Initializes the specified UART interface.
 */

static uint8_t ektm4c123gxl_UART_IntInit (uint8_t EKTM4C123GXL_UARTx)
{
	uint32_t *GpioPin, *GpioPort, *GpioCurrent, *GpioType, *GpioAlternateFunction;
	fUart_InitStruct* UartStruct;
	uint8_t result = EKTM4C123GXL_STATUS_OFF;

	switch (EKTM4C123GXL_UARTx)
	{
	case EKTM4C123GXL_UART_DBG:
		UartStruct = malloc(sizeof(fUart_InitStruct));
		if (UartStruct == NULL) return result;

		UartStruct->nPins = 2;
		GpioPin = malloc(sizeof(uint32_t) * UartStruct->nPins);
		if (GpioPin == NULL) return result;
		GpioPort = malloc(sizeof(uint32_t) * UartStruct->nPins);
		if (GpioPort == NULL) return result;
		GpioCurrent = malloc(sizeof(uint32_t) * UartStruct->nPins);
		if (GpioCurrent == NULL) return result;
		GpioType = malloc(sizeof(uint32_t) * UartStruct->nPins);
		if (GpioType == NULL) return result;
		GpioAlternateFunction = malloc(sizeof(uint32_t) * UartStruct->nPins);
		if (GpioAlternateFunction == NULL) return result;

		GpioPin[0] = EKTM4C123GXL_UARTDBG_RX_PIN;
		GpioPort[0] = EKTM4C123GXL_UARTDBG_RX_PORT;
		GpioCurrent[0] = EKTM4C123GXL_UARTDBG_RX_CURRENT;
		GpioType[0] = EKTM4C123GXL_UARTDBG_RX_TYPE;
		GpioAlternateFunction[0] = EKTM4C123GXL_UARTDBG_RX_AF;

		GpioPin[1] = EKTM4C123GXL_UARTDBG_TX_PIN;
		GpioPort[1] = EKTM4C123GXL_UARTDBG_TX_PORT;
		GpioCurrent[1] = EKTM4C123GXL_UARTDBG_TX_CURRENT;
		GpioType[1] = EKTM4C123GXL_UARTDBG_TX_TYPE;
		GpioAlternateFunction[1] = EKTM4C123GXL_UARTDBG_TX_AF;

		UartStruct->GpioPin = GpioPin;
		UartStruct->GpioPort = GpioPort;
		UartStruct->GpioCurrent = GpioCurrent;
		UartStruct->GpioType = GpioType;
		UartStruct->GpioAlternateFunction = GpioAlternateFunction;

		UartStruct->Module = EKTM4C123GXL_UARTDBG_MODULE;
		UartStruct->WordBits = EKTM4C123GXL_UARTDBG_WORD_BITS;
		UartStruct->StopBits = EKTM4C123GXL_UARTDBG_STOP_BITS;
		UartStruct->ClockSource = EKTM4C123GXL_UARTDBG_CLOCKSOURCE;
		UartStruct->Parity = EKTM4C123GXL_UARTDBG_PARITY;
		UartStruct->BaudRate = EKTM4C123GXL_UARTDBG_BAUDRATE;

		UartClass->IntInit(UartStruct, GpioClass, FUART_INT_RECEIVE, ektm4c123gxl_UartDbg_Irq);

		free(GpioPin);
		free(GpioPort);
		free(GpioType);
		free(GpioCurrent);
		free(GpioAlternateFunction);
		free(UartStruct);


		result = EKTM4C123GXL_STATUS_ON;
		break;
	}

	return result;
}

/*
 * IRQ Handler for the UART DBG.
 */

static void ektm4c123gxl_UartDbg_Irq (void)
{
	uint32_t IntVal;

	IntVal = UartClass->IntGet(EKTM4C123GXL_UARTDBG_MODULE, FUART_INT_RECEIVE | FUART_INT_TRANSMIT);


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
