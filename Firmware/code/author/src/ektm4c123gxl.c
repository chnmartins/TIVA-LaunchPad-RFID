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
#include "functions_spi.h"
#include "mfrc522.h"
#include "pin_map.h"
#include "conf.h"

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
    uint8_t* data;
    uint8_t Length;
    uint8_t IndexProc;
    uint8_t IndexUnproc;
} ektm4c123gxl_CircularBuffer;

/* Private define ------------------------------------------------------------*/
/******** LEDS */
#define EKTM4C123GXL_LEDR_PIN     FGPIO_PIN_1
#define EKTM4C123GXL_LEDR_PORT    FGPIO_PORT_F
#define EKTM4C123GXL_LEDB_PIN     FGPIO_PIN_2
#define EKTM4C123GXL_LEDB_PORT    FGPIO_PORT_F
#define EKTM4C123GXL_LEDG_PIN     FGPIO_PIN_3
#define EKTM4C123GXL_LEDG_PORT    FGPIO_PORT_F

/******** PUSHBUTTONS */
#define EKTM4C123GXL_PB1_PIN      FGPIO_PIN_4
#define EKTM4C123GXL_PB1_PORT     FGPIO_PORT_F
#define EKTM4C123GXL_PB2_PIN      FGPIO_PIN_0
#define EKTM4C123GXL_PB2_PORT     FGPIO_PORT_F

/******** UART DEBUG */
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

/******** SPI RFID */

#define EKTM4C123GXL_SPIRFID_MODULE				FSPI_MODULE_0
#define EKTM4C123GXL_SPIRFID_PROTOCOL			FSPI_PROTOCOL_POL0_PHA0
#define EKTM4C123GXL_SPIRFID_CLOCK_SOURCE		FSPI_CLOCK_SOURCE_SYSTEM
#define EKTM4C123GXL_SPIRFID_DATA_WIDTH			(16)
#define EKTM4C123GXL_SPIRFID_MODE				FSPI_MODE_MASTER
#define EKTM4C123GXL_SPIRFID_BITRATE			(50000)

#define EKTM4C123GXL_SPIRFID_MISO_PIN        	FGPIO_PIN_4
#define EKTM4C123GXL_SPIRFID_MISO_PORT       	FGPIO_PORT_A
#define	EKTM4C123GXL_SPIRFID_MISO_TYPE			FGPIO_TYPE_PUSH_PULL_PULLDOWN
#define EKTM4C123GXL_SPIRFID_MISO_CURRENT		FGPIO_CURRENT_2MA
#define EKTM4C123GXL_SPIRFID_MISO_AF         	GPIO_PA4_SSI0RX

#define EKTM4C123GXL_SPIRFID_MOSI_PIN       	FGPIO_PIN_5
#define EKTM4C123GXL_SPIRFID_MOSI_PORT       	FGPIO_PORT_A
#define	EKTM4C123GXL_SPIRFID_MOSI_TYPE			FGPIO_TYPE_PUSH_PULL_PULLDOWN
#define EKTM4C123GXL_SPIRFID_MOSI_CURRENT		FGPIO_CURRENT_2MA
#define EKTM4C123GXL_SPIRFID_MOSI_AF         	GPIO_PA5_SSI0TX

#define EKTM4C123GXL_SPIRFID_NSS_PIN         	FGPIO_PIN_3
#define EKTM4C123GXL_SPIRFID_NSS_PORT        	FGPIO_PORT_A
#define	EKTM4C123GXL_SPIRFID_NSS_TYPE			FGPIO_TYPE_PUSH_PULL_PULLUP
#define EKTM4C123GXL_SPIRFID_NSS_CURRENT		FGPIO_CURRENT_2MA
#define EKTM4C123GXL_SPIRFID_NSS_AF          	GPIO_PA3_SSI0FSS

#define EKTM4C123GXL_SPIRFID_CLK_PIN         	FGPIO_PIN_2
#define EKTM4C123GXL_SPIRFID_CLK_PORT        	FGPIO_PORT_A
#define	EKTM4C123GXL_SPIRFID_CLK_TYPE			FGPIO_TYPE_PUSH_PULL_PULLDOWN
#define EKTM4C123GXL_SPIRFID_CLK_CURRENT		FGPIO_CURRENT_2MA
#define EKTM4C123GXL_SPIRFID_CLK_AF          	GPIO_PA2_SSI0CLK

#define EKTM4C123GXL_SPIRFID_RST_PIN         	FGPIO_PIN_2
#define EKTM4C123GXL_SPIRFID_RST_PORT        	FGPIO_PORT_E
#define	EKTM4C123GXL_SPIRFID_RST_TYPE			FGPIO_TYPE_PUSH_PULL_PULLUP
#define EKTM4C123GXL_SPIRFID_RST_CURRENT		FGPIO_CURRENT_2MA

#define EKTM4C123GXL_SPIRFID_IRQ_PIN         	FGPIO_PIN_3
#define EKTM4C123GXL_SPIRFID_IRQ_PORT        	FGPIO_PORT_E
#define	EKTM4C123GXL_SPIRFID_IRQ_TYPE			FGPIO_TYPE_PUSH_PULL_PULLDOWN
#define EKTM4C123GXL_SPIRFID_IRQ_CURRENT		FGPIO_CURRENT_2MA

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
fGpio_Class* GpioClass;
fUart_Class* UartClass;
ektm4c123gxl_CircularBuffer* UartDbgTxBuf;
ektm4c123gxl_CircularBuffer* UartDbgRxBuf;
fTim_Class* TimerClass;
fSpi_Class*	SpiClass;

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
uint8_t ektm4c123gxl_UART_SendString (uint8_t EKTM4C123GXL_UARTx, uint8_t* string);
static void ektm4c123gxl_UART_Parse (uint8_t EKTM4C123GXL_UARTx);
static void ektm4c123gxl_UartDbg_Irq (void);

static void ektm4c123gxl_Delay (double sTime);

static uint8_t ektm4c123gxl_SPI_Init (uint8_t EKTM4C123GXL_SPIx);

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
    {
    	free(temp);
    	return NULL;
    }

    UartClass = fUart_CreateClass();
    if (UartClass == NULL)
    {
    	free(GpioClass);
    	free(temp);
    	return NULL;
    }

    TimerClass = fTim_CreateClass();
    if (TimerClass == NULL)
    {
    	free(UartClass);
    	free(GpioClass);
    	free(temp);
    	return NULL;
    }

    SpiClass = fSpi_CreateClass();
    if (SpiClass == NULL)
    {
    	free(TimerClass);
    	free(UartClass);
    	free(GpioClass);
    	free(temp);
    	return NULL;
    }


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

    temp->UART_IntInit = ektm4c123gxl_UART_IntInit;
    temp->UART_SendString = ektm4c123gxl_UART_SendString;
    temp->UART_Parse = ektm4c123gxl_UART_Parse;

    temp->Delay = ektm4c123gxl_Delay;

    temp->SPI_Init = ektm4c123gxl_SPI_Init;

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
	fUart_InitStruct* UartStruct;
	uint8_t result = EKTM4C123GXL_STATUS_OFF;

	switch (EKTM4C123GXL_UARTx)
	{
	case EKTM4C123GXL_UART_DBG:
		UartStruct = malloc(sizeof(fUart_InitStruct));
		if (UartStruct == NULL) return result;

		UartDbgRxBuf = malloc(sizeof(ektm4c123gxl_CircularBuffer));
		if (UartDbgRxBuf == NULL) return result;

		UartDbgTxBuf = malloc(sizeof(ektm4c123gxl_CircularBuffer));
		if (UartDbgTxBuf == NULL) return result;

        UartDbgRxBuf->Length = EKTM4C123GXL_UARTDBG_RXBUF_SIZE;
		UartDbgRxBuf->data = malloc(sizeof(uint8_t) * UartDbgRxBuf->Length);
		if (UartDbgRxBuf->data == NULL) return result;

        UartDbgTxBuf->Length = EKTM4C123GXL_UARTDBG_TXBUF_SIZE;
		UartDbgTxBuf->data = malloc(sizeof(uint8_t) * UartDbgTxBuf->Length);
		if (UartDbgTxBuf->data == NULL) return result;

		UartDbgTxBuf->IndexProc = 0;
		UartDbgTxBuf->IndexUnproc = 0;
		UartDbgRxBuf->IndexProc = 0;
		UartDbgRxBuf->IndexUnproc = 0;

		UartStruct->nPins = 2;
		UartStruct->GpioPin = malloc(sizeof(uint32_t) * UartStruct->nPins);
		if (UartStruct->GpioPin == NULL) return result;
		UartStruct->GpioPort = malloc(sizeof(uint32_t) * UartStruct->nPins);
		if (UartStruct->GpioPort == NULL) return result;
		UartStruct->GpioCurrent = malloc(sizeof(uint32_t) * UartStruct->nPins);
		if (UartStruct->GpioCurrent == NULL) return result;
		UartStruct->GpioType = malloc(sizeof(uint32_t) * UartStruct->nPins);
		if (UartStruct->GpioType == NULL) return result;
		UartStruct->GpioAlternateFunction = malloc(sizeof(uint32_t) * UartStruct->nPins);
		if (UartStruct->GpioAlternateFunction == NULL) return result;

		UartStruct->GpioPin[0] = EKTM4C123GXL_UARTDBG_RX_PIN;
		UartStruct->GpioPort[0] = EKTM4C123GXL_UARTDBG_RX_PORT;
		UartStruct->GpioCurrent[0] = EKTM4C123GXL_UARTDBG_RX_CURRENT;
		UartStruct->GpioType[0] = EKTM4C123GXL_UARTDBG_RX_TYPE;
		UartStruct->GpioAlternateFunction[0] = EKTM4C123GXL_UARTDBG_RX_AF;

		UartStruct->GpioPin[1] = EKTM4C123GXL_UARTDBG_TX_PIN;
		UartStruct->GpioPort[1] = EKTM4C123GXL_UARTDBG_TX_PORT;
		UartStruct->GpioCurrent[1] = EKTM4C123GXL_UARTDBG_TX_CURRENT;
		UartStruct->GpioType[1] = EKTM4C123GXL_UARTDBG_TX_TYPE;
		UartStruct->GpioAlternateFunction[1] = EKTM4C123GXL_UARTDBG_TX_AF;

		UartStruct->Module = EKTM4C123GXL_UARTDBG_MODULE;
		UartStruct->WordBits = EKTM4C123GXL_UARTDBG_WORD_BITS;
		UartStruct->StopBits = EKTM4C123GXL_UARTDBG_STOP_BITS;
		UartStruct->ClockSource = EKTM4C123GXL_UARTDBG_CLOCKSOURCE;
		UartStruct->Parity = EKTM4C123GXL_UARTDBG_PARITY;
		UartStruct->BaudRate = EKTM4C123GXL_UARTDBG_BAUDRATE;

		UartClass->IntInit(UartStruct, GpioClass, FUART_INT_RECEIVE, ektm4c123gxl_UartDbg_Irq);

		free(UartStruct->GpioPin);
		free(UartStruct->GpioPort);
		free(UartStruct->GpioCurrent);
		free(UartStruct->GpioType);
		free(UartStruct->GpioAlternateFunction);
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

	IntVal = UartClass->IntGet(EKTM4C123GXL_UARTDBG_MODULE, FUART_INT_RECEIVE | FUART_INT_TRANSMIT, EKTM4C123GXL_STATUS_ON);

	if (IntVal & FUART_INT_RECEIVE)
	{
	    UartDbgRxBuf->data[UartDbgRxBuf->IndexUnproc++] = UartClass->ReadByte(EKTM4C123GXL_UARTDBG_MODULE);

	    if (UartDbgRxBuf->IndexUnproc >= UartDbgRxBuf->Length)
	        UartDbgRxBuf->IndexUnproc = 0;

	    UartClass->IntClear(EKTM4C123GXL_UARTDBG_MODULE, FUART_INT_RECEIVE);
	}
	if (IntVal & FUART_INT_TRANSMIT)
	{
	    if (UartDbgTxBuf->IndexUnproc != UartDbgTxBuf->IndexProc)
	    {
	        UartClass->SendByte(EKTM4C123GXL_UARTDBG_MODULE, UartDbgTxBuf->data[UartDbgTxBuf->IndexProc++]);

            if (UartDbgTxBuf->IndexProc >= UartDbgTxBuf->Length)
                UartDbgTxBuf->IndexProc = 0;
	    }
	    else
	    {
            UartClass->IntStatus(EKTM4C123GXL_UARTDBG_MODULE, FUART_INT_TRANSMIT, EKTM4C123GXL_STATUS_OFF);
	    }

        UartClass->IntClear(EKTM4C123GXL_UARTDBG_MODULE, FUART_INT_TRANSMIT);
	}
}

/*
 * Sends data over the specified UART interface.
 */

uint8_t ektm4c123gxl_UART_SendString (uint8_t EKTM4C123GXL_UARTx, uint8_t* string)
{
    uint8_t i = 1;
    uint8_t result = EKTM4C123GXL_STATUS_OFF;

    switch (EKTM4C123GXL_UARTx)
    {
    case EKTM4C123GXL_UART_DBG:
        // Wait till previous transmission ends.
        while (UartDbgTxBuf->IndexProc != UartDbgTxBuf->IndexUnproc);

        while (string[i] != '\0')
        {
            UartDbgTxBuf->data[UartDbgTxBuf->IndexUnproc++] = string[i++];


            if (UartDbgTxBuf->IndexUnproc >= UartDbgTxBuf->Length)
                UartDbgTxBuf->IndexUnproc = 0;
        }

        UartClass->SendByte(EKTM4C123GXL_UARTDBG_MODULE, string[0]);
        UartClass->IntStatus(EKTM4C123GXL_UARTDBG_MODULE, FUART_INT_TRANSMIT, EKTM4C123GXL_STATUS_ON);

        result = EKTM4C123GXL_STATUS_ON;
        break;
    }

    return result;
}

/*
 * Parses the data received on the specified UART interface.
 */

static void ektm4c123gxl_UART_Parse (uint8_t EKTM4C123GXL_UARTx)
{
    uint8_t RxBufIndexProc;
    uint8_t CmdIndex;
    uint8_t Cmd[20];

    switch (EKTM4C123GXL_UARTx)
    {
    case EKTM4C123GXL_UART_DBG:
        RxBufIndexProc = UartDbgRxBuf->IndexProc;
        CmdIndex = 0;

        while (RxBufIndexProc != UartDbgRxBuf->IndexUnproc)
        {
            if (CmdIndex < (sizeof(Cmd) / sizeof(uint8_t)))
            {
                Cmd[CmdIndex] = UartDbgRxBuf->data[RxBufIndexProc++];
                if (RxBufIndexProc >= UartDbgRxBuf->Length)
                    RxBufIndexProc = 0;

                if (Cmd[CmdIndex] == EKTM4C123GXL_UARTDBG_CMD_DELIMITER)
                {
                    if (memcmp(Cmd, EKTM4C123GXL_UARTDBG_CMD_HELLO, sizeof(EKTM4C123GXL_UARTDBG_CMD_HELLO) - 1) == 0)
                    {
                        ektm4c123gxl_UART_SendString(EKTM4C123GXL_UART_DBG, EKTM4C123GXL_UARTDBG_CMD_HEY);
                    } else {
                        ektm4c123gxl_UART_SendString(EKTM4C123GXL_UART_DBG, EKTM4C123GXL_UARTDBG_CMD_UNKNOWN);
                    }

                    UartDbgRxBuf->IndexProc = RxBufIndexProc;
                    CmdIndex = 0;
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
    }
}

/*
 * Causes a delay and stops the flow execution till it finishes.
 */

static void ektm4c123gxl_Delay (double sTime)
{
    TimerClass->Init(FTIMER_MODULE_0, sTime, FTIMER_TYPE_ONE_SHOT_DOWN);
    while (TimerClass->IntTest(FTIMER_MODULE_0, FTIMER_INT_TIMEOUT, FTIMER_STATUS_OFF) == FTIMER_STATUS_OFF);
    TimerClass->IntClear(FTIMER_MODULE_0, FTIMER_INT_TIMEOUT);
    TimerClass->DeInit(FTIMER_MODULE_0);

}

/*
 * Initialize the RFID interface.
 */

static uint8_t ektm4c123gxl_SPI_Init (uint8_t EKTM4C123GXL_SPIx)
{
    fSpi_Struct* InitStruct;
    uint8_t result = EKTM4C123GXL_STATUS_OFF;

    switch (EKTM4C123GXL_SPIx)
    {
    case EKTM4C123GXL_SPI_RFID:
    	InitStruct = fSpi_CreateInitStruct(4);
    	if (InitStruct == NULL)
    		return result;

    	InitStruct->Module = EKTM4C123GXL_SPIRFID_MODULE;
    	InitStruct->Mode = EKTM4C123GXL_SPIRFID_MODE;
    	InitStruct->Protocol = EKTM4C123GXL_SPIRFID_PROTOCOL;
    	InitStruct->DataWidth = EKTM4C123GXL_SPIRFID_DATA_WIDTH;
    	InitStruct->ClockSource = EKTM4C123GXL_SPIRFID_CLOCK_SOURCE;
    	InitStruct->BitRate = EKTM4C123GXL_SPIRFID_BITRATE;

    	InitStruct->GpioPin[0] = EKTM4C123GXL_SPIRFID_MISO_PIN;
    	InitStruct->GpioPort[0] = EKTM4C123GXL_SPIRFID_MISO_PORT;
    	InitStruct->GpioCurrent[0] = EKTM4C123GXL_SPIRFID_MISO_CURRENT;
    	InitStruct->GpioType[0] = EKTM4C123GXL_SPIRFID_MISO_TYPE;
    	InitStruct->GpioAlternateFunction[0] = EKTM4C123GXL_SPIRFID_MISO_AF;

    	InitStruct->GpioPin[1] = EKTM4C123GXL_SPIRFID_MOSI_PIN;
    	InitStruct->GpioPort[1] = EKTM4C123GXL_SPIRFID_MOSI_PORT;
    	InitStruct->GpioCurrent[1] = EKTM4C123GXL_SPIRFID_MOSI_CURRENT;
    	InitStruct->GpioType[1] = EKTM4C123GXL_SPIRFID_MOSI_TYPE;
    	InitStruct->GpioAlternateFunction[1] = EKTM4C123GXL_SPIRFID_MOSI_AF;

    	InitStruct->GpioPin[2] = EKTM4C123GXL_SPIRFID_NSS_PIN;
    	InitStruct->GpioPort[2] = EKTM4C123GXL_SPIRFID_NSS_PORT;
    	InitStruct->GpioCurrent[2] = EKTM4C123GXL_SPIRFID_NSS_CURRENT;
    	InitStruct->GpioType[2] = EKTM4C123GXL_SPIRFID_NSS_TYPE;
    	InitStruct->GpioAlternateFunction[2] = EKTM4C123GXL_SPIRFID_NSS_AF;

    	InitStruct->GpioPin[3] = EKTM4C123GXL_SPIRFID_CLK_PIN;
    	InitStruct->GpioPort[3] = EKTM4C123GXL_SPIRFID_CLK_PORT;
    	InitStruct->GpioCurrent[3] = EKTM4C123GXL_SPIRFID_CLK_CURRENT;
    	InitStruct->GpioType[3] = EKTM4C123GXL_SPIRFID_CLK_TYPE;
    	InitStruct->GpioAlternateFunction[3] = EKTM4C123GXL_SPIRFID_CLK_AF;

    	SpiClass->Init(InitStruct, GpioClass);

		fSpi_DestroyInitStruct(InitStruct);

    	result = EKTM4C123GXL_STATUS_ON;
}

void brd_RfidTest (void)
{
    mfrc522_Initialization(RfidDev);
}
