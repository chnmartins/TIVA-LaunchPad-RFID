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
#include <stddef.h>
#include <stdlib.h>

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
#define FUART_ASSERT_MODULE(x)            ((x) == FUART_MODULE_0 || \
                                 	   	   (x) == FUART_MODULE_1 || \
                                 	   	   (x) == FUART_MODULE_2 || \
                                 	   	   (x) == FUART_MODULE_3 || \
                                 	   	   (x) == FUART_MODULE_4 || \
                                 	   	   (x) == FUART_MODULE_5 || \
                                 	   	   (x) == FUART_MODULE_6 || \
                                 	   	   (x) == FUART_MODULE_7)

#define FUART_ASSERT_WORD_BITS(x)          ((x) == FUART_WORD_BITS_EIGTH 	|| \
                                 	 	 	(x) == FUART_WORD_BITS_SEVEN 	|| \
                                 	 	 	(x) == FUART_WORD_BITS_SIX	 	|| \
                                 	 	 	(x) == FUART_WORD_BITS_FIVE)

#define FUART_ASSERT_STOP_BITS(x)          ((x) == FUART_STOP_BITS_ONE 		|| \
                                 	 	 	(x) == FUART_STOP_BITS_TWO)

#define FUART_ASSERT_PARITY(x)        	   ((x) == FUART_PARITY_NONE 		|| \
											(x) == FUART_PARITY_EVEN 		|| \
											(x) == FUART_PARITY_ODD 		|| \
											(x) == FUART_PARITY_ONE 		|| \
											(x) == FUART_PARITY_ZERO)

#define FUART_ASSERT_CLOCK_SOURCE(x)  	   ((x) == FUART_CLOCK_SOURCE_SYSTEM || \
                                 	 	 	(x) == FUART_CLOCK_SOURCE_PIOSC)

#define FUART_ASSERT_BAUDRATE(x)     	   ((x) == FUART_BAUDRATE_110 		|| \
                                 	 	 	(x) == FUART_BAUDRATE_300 		|| \
                                 	 	 	(x) == FUART_BAUDRATE_600 		|| \
                                 	 	 	(x) == FUART_BAUDRATE_1200 		|| \
                                 	 	 	(x) == FUART_BAUDRATE_2400  	|| \
                                 	 	 	(x) == FUART_BAUDRATE_4800 		|| \
                                 	 	 	(x) == FUART_BAUDRATE_9600		|| \
                                 	 	 	(x) == FUART_BAUDRATE_14400 	|| \
                                 	 	 	(x) == FUART_BAUDRATE_19200 	|| \
                                 	 	 	(x) == FUART_BAUDRATE_38400 	|| \
                                 	 	 	(x) == FUART_BAUDRATE_57600 	|| \
                                 	 	 	(x) == FUART_BAUDRATE_115200 	|| \
                                 	 	 	(x) == FUART_BAUDRATE_230400 	|| \
                                 	 	 	(x) == FUART_BAUDRATE_460800 	|| \
                                 	 	 	(x) == FUART_BAUDRATE_921600)

#define	FUART_ASSERT_INT(x)				   ((x) == FUART_INT_9BIT				||	\
											(x) == FUART_INT_OVERRUN_ERROR 		||	\
											(x) == FUART_INT_BREAK_ERROR 		||	\
											(x) == FUART_INT_PARITY_ERROR 		||	\
											(x) == FUART_INT_FRAMING_ERROR 		||	\
											(x) == FUART_INT_RECEIVE_TIMEOUT 	||	\
											(x) == FUART_INT_TRANSMIT 			||	\
											(x) == FUART_INT_RECEIVE 			||	\
											(x) == FUART_INT_DSR 				||	\
											(x) == FUART_INT_DCD 				||	\
											(x) == FUART_INT_CTS 				||	\
											(x) == FUART_INT_RI)

#define FUART_ASSERT_STATUS(x)			   ((x) == FUART_STATUS_OFF 			|| \
											(x) == FUART_STATUS_ON)

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
static void fUart_SysCtlStatus (uint32_t FUART_MODULEx, uint8_t FUART_STATUSx);
static void fUart_GpioInit (fGpio_Class* class, uint32_t* FGPIO_PINx, uint32_t* FGPIO_PORTx, uint32_t* FGPIO_TYPEx, uint32_t* FGPIO_CURRENTx, uint32_t* GPIO_AF, uint8_t nPins);
static void fUart_DeInit (uint32_t FUART_MODULEx);
static void fUart_Config (uint32_t FUART_MODULEx, uint32_t FUART_WORD_BITSx, uint32_t FUART_STOP_BITSx, uint32_t FUART_PARITYx, uint32_t FUART_BAUDRATEx, uint32_t FUART_CLOCK_SOURCEx);
static void fUart_Init (fUart_InitStruct* InitStruct, fGpio_Class* GpioClass);

static void fUart_IntInit (fUart_InitStruct* InitStruct, fGpio_Class* GpioClass, uint32_t FUART_INTx, void (*UART_IRQ) (void));
static uint8_t fUart_IntTest (uint32_t FUART_MODULEx, uint32_t FUART_INTx, uint8_t OnlyMaskedInt);
static uint32_t fUart_IntGet (uint32_t FUART_MODULEx, uint32_t FUART_INTx, uint8_t OnlyMaskedInt);
static void fUart_IntStatus (uint32_t FUART_MODULEx, uint32_t FUART_INTx, uint8_t FUART_STATUSx);
static void fUart_IntClear (uint32_t FUART_MODULEx, uint32_t FUART_INTx);

static void fUart_SendByte (uint32_t FUART_MODULEx, uint8_t byte);
static void fUart_SendBytes (uint32_t FUART_MODULEx, uint8_t* bytes, uint8_t length);
static uint8_t fUart_ReadByte (uint32_t FUART_MODULEx);

/* Private functions ---------------------------------------------------------*/

/*
 * Creates a class.
 */

fUart_Class* fUart_CreateClass (void)
{
	fUart_Class* class = malloc(sizeof(fUart_Class));
	if (class == NULL)
		return NULL;

	class->Init = fUart_Init;
	class->SendByte = fUart_SendByte;
	class->SendBytes = fUart_SendBytes;
	class->ReadByte = fUart_ReadByte;

	class->IntInit = fUart_IntInit;
	class->IntTest = fUart_IntTest;
	class->IntStatus = fUart_IntStatus;
	class->IntGet = fUart_IntGet;
	class->IntClear = fUart_IntClear;

	return class;
}

/*
 * Destroys a class.
 */

void fUart_DestroyClass (fUart_Class * class)
{
	free(class);
}

/*
 * Function to enable the SysCtl clock.
 */

static void fUart_SysCtlStatus (uint32_t FUART_MODULEx, uint8_t FUART_STATUSx)
{
#ifdef  DEBUG
    ASSERT_PARAM(FUART_ASSERT_MODULE(FUART_MODULEx));
    ASSERT_PARAM(FUART_ASSERT_STATUS(FUART_STATUSx));
#endif

    switch (FUART_MODULEx)
    {
    case FUART_MODULE_0:
        (FUART_STATUSx == FUART_STATUS_ON) ? SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0) : SysCtlPeripheralDisable(SYSCTL_PERIPH_UART0);
        break;
    case FUART_MODULE_1:
    	(FUART_STATUSx == FUART_STATUS_ON) ? SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1) : SysCtlPeripheralDisable(SYSCTL_PERIPH_UART1);
        break;
    case FUART_MODULE_2:
    	(FUART_STATUSx == FUART_STATUS_ON) ? SysCtlPeripheralEnable(SYSCTL_PERIPH_UART2) : SysCtlPeripheralDisable(SYSCTL_PERIPH_UART2);
        break;
    case FUART_MODULE_3:
    	(FUART_STATUSx == FUART_STATUS_ON) ? SysCtlPeripheralEnable(SYSCTL_PERIPH_UART3) : SysCtlPeripheralDisable(SYSCTL_PERIPH_UART3);
        break;
    case FUART_MODULE_4:
    	(FUART_STATUSx == FUART_STATUS_ON) ? SysCtlPeripheralEnable(SYSCTL_PERIPH_UART4) : SysCtlPeripheralDisable(SYSCTL_PERIPH_UART4);
        break;
    case FUART_MODULE_5:
    	(FUART_STATUSx == FUART_STATUS_ON) ? SysCtlPeripheralEnable(SYSCTL_PERIPH_UART5) : SysCtlPeripheralDisable(SYSCTL_PERIPH_UART5);
        break;
    case FUART_MODULE_6:
    	(FUART_STATUSx == FUART_STATUS_ON) ? SysCtlPeripheralEnable(SYSCTL_PERIPH_UART6) : SysCtlPeripheralDisable(SYSCTL_PERIPH_UART6);
        break;
    case FUART_MODULE_7:
    	(FUART_STATUSx == FUART_STATUS_ON) ? SysCtlPeripheralEnable(SYSCTL_PERIPH_UART7) : SysCtlPeripheralDisable(SYSCTL_PERIPH_UART7);
        break;
    default:
    	break;
    }
}

/*
 * Function to configure the GPIO pins to be managed by the UART hardware peripheral.
 */

static void fUart_GpioInit (fGpio_Class* class, uint32_t* FGPIO_PINx, uint32_t* FGPIO_PORTx, uint32_t* FGPIO_TYPEx, uint32_t* FGPIO_CURRENTx, uint32_t* GPIO_AF, uint8_t nPins)
{
    uint8_t i;

    for (i = 0; i < nPins; i++)
    {
    	class->InitAlternateFunction(FGPIO_PINx[i], FGPIO_PORTx[i], FGPIO_TYPEx[i], FGPIO_CURRENTx[i], GPIO_AF[i]);
    }
}

/*
 * Disable the UART peripheral.
 */

static void fUart_DeInit (uint32_t FUART_MODULEx)
{
#ifdef  DEBUG
    ASSERT_PARAM(FUART_ASSERT_MODULE(FUART_MODULEx));
#endif

    UARTIntUnregister(FUART_MODULEx);
    UARTDisable(FUART_MODULEx);
}

/*
 * Configure the UART peripheral.
 */

static void fUart_Config (uint32_t FUART_MODULEx, uint32_t FUART_WORD_BITSx, uint32_t FUART_STOP_BITSx, uint32_t FUART_PARITYx, uint32_t FUART_BAUDRATEx, uint32_t FUART_CLOCK_SOURCEx)
{
#ifdef  DEBUG
	ASSERT_PARAM(FUART_ASSERT_MODULE(FUART_MODULEx));
	ASSERT_PARAM(FUART_ASSERT_WORD_BITS(FUART_WORD_BITSx));
	ASSERT_PARAM(FUART_ASSERT_STOP_BITS(FUART_STOP_BITSx));
	ASSERT_PARAM(FUART_ASSERT_PARITY(FUART_PARITYx));
    ASSERT_PARAM(FUART_ASSERT_BAUDRATE(FUART_BAUDRATEx));
    ASSERT_PARAM(FUART_ASSERT_CLOCK_SOURCE(FUART_CLOCK_SOURCEx));
#endif

    uint32_t fUart_ClockFrequency;

    fUart_ClockFrequency = (FUART_CLOCK_SOURCEx == FUART_CLOCK_SOURCE_SYSTEM) ? SysCtlClockGet() : PIOSC_NOMINAL_FREQUENCY;

    UARTClockSourceSet(FUART_MODULEx, FUART_CLOCK_SOURCEx);
    UARTConfigSetExpClk(FUART_MODULEx, fUart_ClockFrequency, FUART_BAUDRATEx, FUART_PARITYx | FUART_STOP_BITSx | FUART_WORD_BITSx);
    UARTFIFODisable(FUART_MODULEx);
}

/*
 * Function to initialize and start the UARt interface specified.
 */

static void fUart_Init (fUart_InitStruct* InitStruct, fGpio_Class* GpioClass)
{
#ifdef  DEBUG
	ASSERT_PARAM(FUART_ASSERT_MODULE(InitStruct->Module));
	ASSERT_PARAM(FUART_ASSERT_WORD_BITS(InitStruct->WordBits));
	ASSERT_PARAM(FUART_ASSERT_STOP_BITS(InitStruct->StopBits));
	ASSERT_PARAM(FUART_ASSERT_PARITY(InitStruct->Parity));
    ASSERT_PARAM(FUART_ASSERT_BAUDRATE(InitStruct->BaudRate));
    ASSERT_PARAM(FUART_ASSERT_CLOCK_SOURCE(InitStruct->ClockSource));
#endif

    fUart_SysCtlStatus(InitStruct->Module, true);
    fUart_GpioInit(GpioClass, InitStruct->GpioPin, InitStruct->GpioPort, InitStruct->GpioType, InitStruct->GpioCurrent, InitStruct->GpioAlternateFunction, InitStruct->nPins);
    fUart_DeInit(InitStruct->Module);
    fUart_Config(InitStruct->Module, InitStruct->WordBits, InitStruct->StopBits, InitStruct->Parity, InitStruct->BaudRate, InitStruct->ClockSource);

    // Using UARTEnable causes the FIFO to be initialized aswell. This is not yet supported by this library.
    HWREG(InitStruct->Module + UART_O_CTL) |= (UART_CTL_UARTEN | UART_CTL_TXE | UART_CTL_RXE);
}

/*
 * Initializes the specified interrupts and sets the IRQ handler.
 *
 * NOTE: Interrupts are a logical OR and as such the function doesn't assert this parameter. It depends on the user to give correct parameters.
 * NOTE: The IntIRQ parameter isn't checked, it depends on the user to give correct parameters.
 */

static void fUart_IntInit (fUart_InitStruct* InitStruct, fGpio_Class* GpioClass, uint32_t FUART_INTx, void (*UART_IRQ) (void))
{
#ifdef  DEBUG
	ASSERT_PARAM(FUART_ASSERT_MODULE(InitStruct->Module));
	ASSERT_PARAM(FUART_ASSERT_WORD_BITS(InitStruct->WordBits));
	ASSERT_PARAM(FUART_ASSERT_STOP_BITS(InitStruct->StopBits));
	ASSERT_PARAM(FUART_ASSERT_PARITY(InitStruct->Parity));
    ASSERT_PARAM(FUART_ASSERT_BAUDRATE(InitStruct->BaudRate));
    ASSERT_PARAM(FUART_ASSERT_CLOCK_SOURCE(InitStruct->ClockSource));
#endif

    fUart_SysCtlStatus(InitStruct->Module, true);
    fUart_GpioInit(GpioClass, InitStruct->GpioPin, InitStruct->GpioPort, InitStruct->GpioType, InitStruct->GpioCurrent, InitStruct->GpioAlternateFunction, InitStruct->nPins);
    fUart_DeInit(InitStruct->Module);
    fUart_Config(InitStruct->Module, InitStruct->WordBits, InitStruct->StopBits, InitStruct->Parity, InitStruct->BaudRate, InitStruct->ClockSource);
    //IntMasterEnable();
    UARTIntRegister(InitStruct->Module, UART_IRQ);
    fUart_IntStatus(InitStruct->Module, FUART_INTx, FUART_STATUS_ON);

    // Using UARTEnable causes the FIFO to be initialized aswell. This is not yet supported by this library.
    HWREG(InitStruct->Module + UART_O_CTL) |= (UART_CTL_UARTEN | UART_CTL_TXE | UART_CTL_RXE);
}

/*
 * Tests the status of an INDIVIDUAL previously activated interrupt.
 */

static uint8_t fUart_IntTest (uint32_t FUART_MODULEx, uint32_t FUART_INTx, uint8_t OnlyMaskedInt)
{
#ifdef  DEBUG
	ASSERT_PARAM(FUART_ASSERT_MODULE(FUART_MODULEx));
	ASSERT_PARAM(FUART_ASSERT_INT(FUART_INTx));
	ASSERT_PARAM(FUART_ASSERT_STATUS(OnlyMaskedInt));
#endif

	if (UARTIntStatus(FUART_MODULEx, OnlyMaskedInt) & FUART_INTx)
		return FUART_STATUS_ON;

	return FUART_STATUS_OFF;
}

/*
 * Returns the status of the specified interrupts in a bit packed variable
 */

static uint32_t fUart_IntGet (uint32_t FUART_MODULEx, uint32_t FUART_INTx, uint8_t OnlyMaskedInt)
{
#ifdef  DEBUG
	ASSERT_PARAM(FUART_ASSERT_MODULE(FUART_MODULEx));
	ASSERT_PARAM(FUART_ASSERT_STATUS(OnlyMaskedInt));
#endif

    return (UARTIntStatus(FUART_MODULEx, OnlyMaskedInt) & FUART_INTx);
}

/*
 * Returns the status of the specified interrupts in a bit packed variable
 */

static void fUart_IntStatus (uint32_t FUART_MODULEx, uint32_t FUART_INTx, uint8_t FUART_STATUSx)
{
#ifdef  DEBUG
	ASSERT_PARAM(FUART_ASSERT_MODULE(FUART_MODULEx));
	ASSERT_PARAM(FUART_ASSERT_STATUS(FUART_STATUSx));
#endif

    if (FUART_STATUSx == FUART_STATUS_ON)
    	UARTIntEnable(FUART_MODULEx, FUART_INTx);
    else
    	UARTIntDisable(FUART_MODULEx, FUART_INTx);
}

/*
 * Clears the specified interrupts.
 */

static void fUart_IntClear (uint32_t FUART_MODULEx, uint32_t FUART_INTx)
{
#ifdef  DEBUG
	ASSERT_PARAM(FUART_ASSERT_MODULE(FUART_MODULEx));
#endif

	UARTIntClear(FUART_MODULEx, FUART_INTx);
}

/*
 * Sends a single byte.
 */

static void fUart_SendByte (uint32_t FUART_MODULEx, uint8_t byte)
{
#ifdef  DEBUG
	ASSERT_PARAM(FUART_ASSERT_MODULE(FUART_MODULEx));
#endif

	UARTCharPut(FUART_MODULEx, byte);
}

/*
 * Sends an array of bytes.
 */

static void fUart_SendBytes (uint32_t FUART_MODULEx, uint8_t* bytes, uint8_t length)
{
#ifdef  DEBUG
	ASSERT_PARAM(FUART_ASSERT_MODULE(FUART_MODULEx));
#endif
	uint8_t i = 0;

	for (i = 0; i < length; i++)
	{
		while (fUart_IntTest(FUART_MODULEx, FUART_INT_TRANSMIT, FUART_STATUS_OFF) == FUART_STATUS_OFF);
		fUart_SendByte(FUART_MODULEx, bytes[i]);
	}
}

/*
 * Reads a character.
 */

static uint8_t fUart_ReadByte (uint32_t FUART_MODULEx)
{
#ifdef  DEBUG
    ASSERT_PARAM(FUART_ASSERT_MODULE(FUART_MODULEx));
#endif

    return UARTCharGet(FUART_MODULEx);
}
