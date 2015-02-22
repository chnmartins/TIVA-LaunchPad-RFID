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


#define DBGUART_RX_PIN  GPIO_PIN_0
#define DBGUART_RX_PORT GPIO_PORTA_BASE
#define DBGUART_RX_AF   GPIO_PA0_U0RX

#define DBGUART_TX_PIN  GPIO_PIN_1
#define DBGUART_TX_PORT GPIO_PORTA_BASE
#define DBGUART_TX_AF   GPIO_PA1_U0TX

#define DBGUART_RXBUF_SIZE   50
#define DBGUART_TXBUF_SIZE   50

#define DBGUART_CMD_DELIMITER   '\n'
#define DBGUART_CMD_HELLO       "Are you there?\r\n"
#define DBGUART_CMD_HEY         "Yes, I'm here.\r\n"
#define DBGUART_CMD_UNKNOWN     "I have no idea what you are talking about.\r\n"


#define SPIRFID_MISO_PIN        GPIO_PIN_4
#define SPIRFID_MISO_PORT       GPIO_PORTA_BASE
#define SPIRFID_MISO_AF         GPIO_PA4_SSI0RX

#define SPIRFID_MOSI_PIN        GPIO_PIN_5
#define SPIRFID_MOSI_PORT       GPIO_PORTA_BASE
#define SPIRFID_MOSI_AF         GPIO_PA5_SSI0TX

#define SPIRFID_NSS_PIN         GPIO_PIN_3
#define SPIRFID_NSS_PORT        GPIO_PORTA_BASE
#define SPIRFID_NSS_AF          GPIO_PA3_SSI0FSS

#define SPIRFID_CLK_PIN         GPIO_PIN_2
#define SPIRFID_CLK_PORT        GPIO_PORTA_BASE
#define SPIRFID_CLK_AF          GPIO_PA2_SSI0CLK

#define SPIRFID_RST_PIN         GPIO_PIN_2
#define SPIRFID_RST_PORT        GPIO_PORTE_BASE

#define SPIRFID_IRQ_PIN         GPIO_PIN_3
#define SPIRFID_IRQ_PORT        GPIO_PORTE_BASE

#define SPIRFID_RXBUF_SIZE      10
#define SPIRFID_TXBUF_SIZE      10

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
fUart_Mod* UartDbg;
fSpi_Mod* SpiRfid;
fGpio_Pin*  RstRfid;
fGpio_Pin*  IrqRfid;
mfrc522_Mod* RfidDev;

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
    case DBGUART:
    	UartDbg = (fUart_Mod*) calloc (1, sizeof(fUart_Mod));
    	if (UartDbg == (fUart_Mod*) NULL)
    		return false;
    	UartDbg->Rx = (fUart_Pin*) calloc(1, sizeof(fUart_Pin));
        if (UartDbg->Rx == (fUart_Pin*) NULL)
            return false;
    	UartDbg->Tx = (fUart_Pin*) calloc(1, sizeof(fUart_Pin));
        if (UartDbg->Tx == (fUart_Pin*) NULL)
            return false;

        UartDbg->Rx->Pin = DBGUART_RX_PIN;
        UartDbg->Rx->Port = DBGUART_RX_PORT;
        UartDbg->Rx->AlternateFunction = DBGUART_RX_AF;
        UartDbg->Rx->Current = CURR_2MA;
        UartDbg->Rx->Type = TYPE_PP_PD;

        UartDbg->Tx->Pin = DBGUART_TX_PIN;
        UartDbg->Tx->Port = DBGUART_TX_PORT;
        UartDbg->Tx->AlternateFunction = DBGUART_TX_AF;
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

        UartDbg->RxBuf = (uint8_t*) calloc(DBGUART_RXBUF_SIZE, sizeof(uint8_t));
        if (UartDbg->RxBuf == NULL)
        	return false;
        UartDbg->RxBufProcIndex = 0;
        UartDbg->RxBufUnprocIndex = 0;
        UartDbg->RxBufLength = DBGUART_RXBUF_SIZE;

        UartDbg->TxBuf = (uint8_t*) calloc(DBGUART_TXBUF_SIZE, sizeof(uint8_t));
        if (UartDbg->TxBuf == NULL)
        	return false;
        UartDbg->TxBufProcIndex = 0;
        UartDbg->TxBufUnprocIndex = 0;
        UartDbg->TxBufLength = DBGUART_TXBUF_SIZE;

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
    case DBGUART:
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
    case DBGUART:
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

                if (*(Cmd + CmdIndex) == DBGUART_CMD_DELIMITER)
                {
                    if (!(strcmp((char*) Cmd, DBGUART_CMD_HELLO)))
                    {
                        brd_UartSend(DBGUART, DBGUART_CMD_HEY);
                    } else {
                        brd_UartSend(DBGUART, DBGUART_CMD_UNKNOWN);
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
 * Causes a delay and stops the flow execution till it finishes.
 */

void brd_delay (double sTime)
{
    fTim_Mod Tim_Mod = {.Module = MOD_TIM0, .Type = TYPE_ONE_SHOT_DOWN, .sTime = sTime, .Int = INT_NONE};

    fTim_Init(&Tim_Mod);

    while (!(fTim_IsTimeout(&Tim_Mod)));

}

/*
 * Initialize the RFID interface.
 */

bool brd_RfidHwInit (void)
{
    uint8_t i;

    SpiRfid = (fSpi_Mod*) calloc(1, sizeof(fSpi_Mod));
    if (SpiRfid == (fSpi_Mod*) NULL)
        return false;

    SpiRfid->nPins = 4;
    SpiRfid->Pins = (fGpio_Pin**) calloc(SpiRfid->nPins, sizeof(fGpio_Pin*));
    if (SpiRfid->Pins == (fGpio_Pin**) NULL)
        return false;

    for (i = 0; i < SpiRfid->nPins; i++)
    {
        (*(SpiRfid->Pins + i)) = (fGpio_Pin*) calloc(1, sizeof(fGpio_Pin));
        if ((*(SpiRfid->Pins + i)) == (fGpio_Pin*) NULL)
            return false;
        (*(SpiRfid->Pins + i))->Type = TYPE_PP_PD;
        (*(SpiRfid->Pins + i))->Direction = DIR_HW;
        (*(SpiRfid->Pins + i))->Current = CURR_2MA;
    }

    (*(SpiRfid->Pins + 0))->Pin = SPIRFID_MISO_PIN;
    (*(SpiRfid->Pins + 0))->Port = SPIRFID_MISO_PORT;
    (*(SpiRfid->Pins + 0))->AlternateFunction = SPIRFID_MISO_AF;

    (*(SpiRfid->Pins + 1))->Pin = SPIRFID_MOSI_PIN;
    (*(SpiRfid->Pins + 1))->Port = SPIRFID_MOSI_PORT;
    (*(SpiRfid->Pins + 1))->AlternateFunction = SPIRFID_MOSI_AF;

    (*(SpiRfid->Pins + 2))->Pin = SPIRFID_CLK_PIN;
    (*(SpiRfid->Pins + 2))->Port = SPIRFID_CLK_PORT;
    (*(SpiRfid->Pins + 2))->AlternateFunction = SPIRFID_CLK_AF;

    (*(SpiRfid->Pins + 3))->Pin = SPIRFID_NSS_PIN;
    (*(SpiRfid->Pins + 3))->Port = SPIRFID_NSS_PORT;
    (*(SpiRfid->Pins + 3))->AlternateFunction = SPIRFID_NSS_AF;
    (*(SpiRfid->Pins + 3))->Type = TYPE_PP_PU;

    SpiRfid->ClockSource = FSPI_CLK_SYSTEM;
    SpiRfid->DataWidth = 16;
    SpiRfid->Mode = MODE_MASTER;
    SpiRfid->BitRate = 50000;
    SpiRfid->Module = SPI_MOD0;
    SpiRfid->Protocol = PROT_POL0_PHA0;
    SpiRfid->Int = INT_NONE;
    fSpi_Init(SpiRfid);

    RstRfid = calloc(1, sizeof(fGpio_Pin));
    if (RstRfid == NULL)
        return 1;

    RstRfid->Pin = SPIRFID_RST_PIN;
    RstRfid->Port = SPIRFID_RST_PORT;
    RstRfid->Current = CURR_2MA;
    RstRfid->Direction = DIR_OUT;
    RstRfid->Type = TYPE_PP_PU;
    fGpio_Init(RstRfid);

    return true;
}

/*
 * Function to write an address.
 */

void brd_RfidWriteAddress (uint8_t address, uint8_t value)
{
    // SPI fifo gets cleared by reading the byte.
    uint32_t a = (address << 8) | value;
    uint32_t b = 0;

    fSpi_SendReceive(SpiRfid, a, &b);
}

/*
 * Function to read an address.
 */

void brd_RfidReadAddress (uint8_t address, uint8_t* value)
{
    // Send a random character to generate the clock pulses.
    // Send 0x80, it equals read address 0x00 on the device.
    uint32_t a = (address << 8) | 0x00;
    uint32_t b = 0;

    fSpi_SendReceive(SpiRfid, a, &b);

    *value = (uint8_t) (b & 0x000000FF);
}

/*
 * Manages the reset pin on the RFID device.
 */

void brd_RfidRstControl (uint8_t status)
{
    if (status)
    {
        fGpio_setHigh(RstRfid);
    } else {
        fGpio_setLow(RstRfid);
    }
}

/*
 * Initializes the RFID device completely.
 */

bool brd_RfidInit (void)
{
    RfidDev = calloc(1, sizeof(mfrc522_Mod));
    if (RfidDev == NULL)
        return false;

    RfidDev->Delay = brd_delay;
    RfidDev->HwInit = brd_RfidHwInit;
    RfidDev->RstCtrl = brd_RfidRstControl;
    RfidDev->WriteAddress = brd_RfidWriteAddress;
    RfidDev->ReadAddress = brd_RfidReadAddress;

    mfrc522_Init(RfidDev);

    return true;
}
