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
#include "hw_uart.h"
#include "hw_types.h"
#include "interrupt.h"
#include "functions_gpio.h"
#include "hw_memmap.h"
#include "sysctl.h"
#include "conf.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define PIOSC_NOMINAL_FREQUENCY     16000000

/* Private macro -------------------------------------------------------------*/
#define ASSERT_UART(x)          ((x) == UART0_BASE || \
                                 (x) == UART1_BASE || \
                                 (x) == UART2_BASE || \
                                 (x) == UART3_BASE || \
                                 (x) == UART4_BASE || \
                                 (x) == UART5_BASE || \
                                 (x) == UART6_BASE || \
                                 (x) == UART7_BASE)

#define ASSERT_WLEN(x)          ((x) == WLEN_EIGTH || \
                                 (x) == WLEN_SEVEN || \
                                 (x) == WLEN_SIX || \
                                 (x) == WLEN_FIVE)

#define ASSERT_STOP(x)          ((x) == STOP_ONE || \
                                 (x) == STOP_TWO)

#define ASSERT_PARITY(x)        ((x) == PAR_NONE || \
                                 (x) == PAR_EVEN || \
                                 (x) == PAR_ODD || \
                                 (x) == PAR_ONE || \
                                 (x) == PAR_ZERO)

#define ASSERT_CLOCK_SOURCE(x)  ((x) == UART_CLOCK_SYSTEM || \
                                 (x) == UART_CLOCK_PIOSC)

#define ASSERT_BAUD_RATE(x)     ((x) == BR_110 || \
                                 (x) == BR_300 || \
                                 (x) == BR_600 || \
                                 (x) == BR_1200 || \
                                 (x) == BR_2400 || \
                                 (x) == BR_4800 || \
                                 (x) == BR_9600 || \
                                 (x) == BR_14400 || \
                                 (x) == BR_19200 || \
                                 (x) == BR_38400 || \
                                 (x) == BR_57600 || \
                                 (x) == BR_115200 || \
                                 (x) == BR_230400 || \
                                 (x) == BR_460800 || \
                                 (x) == BR_921600)

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/*
 * Function to enable the SysCtl clock.
 */

void fUart_enableSysCtl (const fUart_Mod* Uart_Mod)
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

/*
 * Function to configure the GPIO pins to be managed by the UART hardware peripheral.
 */

void fUart_setPins (const fUart_Mod* Uart_Mod)
{
    fGpio_Pin Rx_Pin = {.Pin = Uart_Mod->Rx->Pin, \
                        .Port = Uart_Mod->Rx->Port, \
                        .AlternateFunction = Uart_Mod->Rx->AlternateFunction, \
                        .Current = Uart_Mod->Rx->Current, \
                        .Type = Uart_Mod->Rx->Type, \
                        .Direction = DIR_HW};

    fGpio_Pin Tx_Pin = {.Pin = Uart_Mod->Tx->Pin, \
                        .Port = Uart_Mod->Tx->Port, \
                        .AlternateFunction = Uart_Mod->Tx->AlternateFunction, \
                        .Current = Uart_Mod->Tx->Current, \
                        .Type = Uart_Mod->Tx->Type, \
                        .Direction = DIR_HW};

    fGpio_enableSysCtl(&Rx_Pin);
    fGpio_enableSysCtl(&Tx_Pin);

    fGpio_setDirection(&Rx_Pin);
    fGpio_setDirection(&Tx_Pin);

    fGpio_setAlternateFunction(&Rx_Pin);
    fGpio_setAlternateFunction(&Tx_Pin);

    fGpio_setConfig(&Rx_Pin);
    fGpio_setConfig(&Tx_Pin);
}

/*
 * Disable the UART peripheral.
 */

void fUart_DeInit (const fUart_Mod* Uart_Mod)
{
#ifdef  DEBUG
    ASSERT_PARAM(ASSERT_UART(Uart_Mod->Module));
#endif

    UARTDisable(Uart_Mod->Module);
}

/*
 * Configure the UART peripheral.
 */

void fUart_setConfig (const fUart_Mod* Uart_Mod)
{
#ifdef  DEBUG
    ASSERT_PARAM(ASSERT_UART(Uart_Mod->Module));
    ASSERT_PARAM(ASSERT_WLEN(Uart_Mod->Wlen));
    ASSERT_PARAM(ASSERT_STOP(Uart_Mod->Stop));
    ASSERT_PARAM(ASSERT_PARITY(Uart_Mod->Parity));
    ASSERT_PARAM(ASSERT_BAUD_RATE(Uart_Mod->BaudRate));
    ASSERT_PARAM(ASSERT_CLOCK_SOURCE(Uart_Mod->ClockSource));
#endif

    uint32_t ClockFrequency;

    UARTClockSourceSet(Uart_Mod->Module, Uart_Mod->ClockSource);

    if (Uart_Mod->ClockSource == CLK_SYSTEM)
        ClockFrequency = SysCtlClockGet();
    else if (Uart_Mod->ClockSource == CLK_INTERNAL)
        ClockFrequency = PIOSC_NOMINAL_FREQUENCY;

    UARTConfigSetExpClk(Uart_Mod->Module, ClockFrequency, Uart_Mod->BaudRate, Uart_Mod->Parity | Uart_Mod->Stop | Uart_Mod->Wlen);
    UARTFIFODisable(Uart_Mod->Module);
}

/*
 * Starts the specified UART peripheral.
 */

void fUart_Start (const fUart_Mod* Uart_Mod)
{
#ifdef  DEBUG
    ASSERT_PARAM(ASSERT_UART(Uart_Mod->Module));
#endif

    // Using UARTEnable causes the FIFO to be initialized aswell.
    HWREG(Uart_Mod->Module + UART_O_CTL) |= (UART_CTL_UARTEN | UART_CTL_TXE | UART_CTL_RXE);
}

/*
 * Function to initialize and start the UARt interface specified.
 */

void fUart_Init (const fUart_Mod* Uart_Mod)
{
#ifdef  DEBUG
    ASSERT_PARAM(ASSERT_UART(Uart_Mod->Module));
    ASSERT_PARAM(ASSERT_WLEN(Uart_Mod->Wlen));
    ASSERT_PARAM(ASSERT_STOP(Uart_Mod->Stop));
    ASSERT_PARAM(ASSERT_PARITY(Uart_Mod->Parity));
    ASSERT_PARAM(ASSERT_BAUD_RATE(Uart_Mod->BaudRate));
    ASSERT_PARAM(ASSERT_CLOCK_SOURCE(Uart_Mod->ClockSource));
#endif

    fUart_enableSysCtl(Uart_Mod);
    fUart_setPins(Uart_Mod);
    fUart_DeInit(Uart_Mod);
    fUart_setConfig(Uart_Mod);
    fUart_IntInit(Uart_Mod);
    fUart_Start(Uart_Mod);
}

/*
 * Initializes the specified interrupts and sets the IRQ handler.
 *
 * NOTE: Interrupts are a logical OR and as such the function doesn't assert this parameter. It depends on the user to give correct parameters.
 * NOTE: The IntIRQ parameter isn't checked, it depends on the user to give correct parameters.
 */

void fUart_IntInit (const fUart_Mod* Uart_Mod)
{
#ifdef  DEBUG
    ASSERT_PARAM(ASSERT_UART(Uart_Mod->Module));
#endif

    if (Uart_Mod->Interrupts != INT_NONE)
    {
        IntMasterEnable();
        UARTIntRegister(Uart_Mod->Module, Uart_Mod->IntIRQ);
        UARTIntEnable(Uart_Mod->Module, Uart_Mod->Interrupts);
    }
}

/*
 * Gets the status of the specified masked interrupts.
 */

uint32_t fUart_IntGet (const fUart_Mod* Uart_Mod)
{
#ifdef  DEBUG
    ASSERT_PARAM(ASSERT_UART(Uart_Mod->Module));
#endif

    return (UARTIntStatus(Uart_Mod->Module, true) & Uart_Mod->Interrupts);
}

/*
 * Function to call on the IRQ handler of the UART interface.
 */

void fUart_IRQHandler (fUart_Mod* Uart_Mod)
{
    uint32_t val;

    val = fUart_IntGet(Uart_Mod);

    if (val & INT_9BIT)
    {

    	UARTIntClear(Uart_Mod->Module, INT_9BIT);
    }
    if (val & INT_OVERRUN_ERROR)
    {

    	UARTIntClear(Uart_Mod->Module, INT_OVERRUN_ERROR);
    }
    if (val & INT_BREAK_ERROR)
    {

    	UARTIntClear(Uart_Mod->Module, INT_BREAK_ERROR);
    }
    if (val & INT_PARITY_ERROR)
    {

    	UARTIntClear(Uart_Mod->Module, INT_PARITY_ERROR);
    }
    if (val & INT_FRAMING_ERROR)
    {

    	UARTIntClear(Uart_Mod->Module, INT_FRAMING_ERROR);
    }
    if (val & INT_RECEIVE_TIMEOUT)
    {

    	UARTIntClear(Uart_Mod->Module, INT_RECEIVE_TIMEOUT);
    }
    if (val & INT_TRANSMIT)
    {
        if (Uart_Mod->TxBufProcIndex != Uart_Mod->TxBufUnprocIndex)
        {
            UARTCharPut(Uart_Mod->Module, *(Uart_Mod->TxBuf + Uart_Mod->TxBufProcIndex++));

            if (Uart_Mod->TxBufProcIndex >= Uart_Mod->TxBufLength)
                Uart_Mod->TxBufProcIndex = 0;
        }
        else
        {
            UARTIntDisable(Uart_Mod->Module, INT_TRANSMIT);
        }

    	UARTIntClear(Uart_Mod->Module, INT_TRANSMIT);
    }
    if (val & INT_RECEIVE)
    {
        *(Uart_Mod->RxBuf + Uart_Mod->RxBufUnprocIndex++) = UARTCharGet(Uart_Mod->Module);

        if (Uart_Mod->RxBufUnprocIndex >= Uart_Mod->RxBufLength)
            Uart_Mod->RxBufUnprocIndex = 0;

    	UARTIntClear(Uart_Mod->Module, INT_RECEIVE);
    }
    if (val & INT_DSR)
    {

    	UARTIntClear(Uart_Mod->Module, INT_DSR);
    }
    if (val & INT_DCD)
    {

    	UARTIntClear(Uart_Mod->Module, INT_DCD);
    }
    if (val & INT_CTS)
    {

    	UARTIntClear(Uart_Mod->Module, INT_CTS);
    }
    if (val & INT_RI)
    {

    	UARTIntClear(Uart_Mod->Module, INT_RI);
    }
}

/*
 * Transmits the specified data over on TxBuf over the specified interface.
 */

bool fUart_BeginTransfer (fUart_Mod* Uart_Mod, const uint8_t* data, uint8_t length)
{
#ifdef  DEBUG
    ASSERT_PARAM(ASSERT_UART(Uart_Mod->Module));
#endif

    if (length >= Uart_Mod->TxBufLength)
        return false;

    while (Uart_Mod->TxBufProcIndex != Uart_Mod->TxBufUnprocIndex);

    while (length--) {
        *(Uart_Mod->TxBuf + Uart_Mod->TxBufUnprocIndex++) = *(data++);


        if (Uart_Mod->TxBufUnprocIndex >= Uart_Mod->TxBufLength)
            Uart_Mod->TxBufUnprocIndex = 0;
    }

    UARTCharPut(Uart_Mod->Module, 0);
    UARTIntEnable(Uart_Mod->Module, INT_TRANSMIT);

    return true;
}
