/**PROT_TI
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
#include <stdlib.h>
#include <stddef.h>

#include "ssi.h"
#include "sysctl.h"
#include "conf.h"
#include "functions_spi.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define PIOSC_NOMINAL_FREQUENCY             (16000000)

/* Private macro -------------------------------------------------------------*/
#define	FSPI_ASSERT_SPI_MODULE(x)		((x) == FSPI_MODULE_0	 || \
							 	 	 	 (x) == FSPI_MODULE_1	 || \
							 	 	 	 (x) == FSPI_MODULE_2	 || \
							 	 	 	 (x) == FSPI_MODULE_3)

#define	FSPI_ASSERT_SPI_PROTOCOL(x)		((x) == FSPI_PROTOCOL_POL0_PHA0 || \
									 	 (x) == FSPI_PROTOCOL_POL0_PHA1 || \
									 	 (x) == FSPI_PROTOCOL_POL1_PHA0 || \
									 	 (x) == FSPI_PROTOCOL_POL1_PHA1 || \
									 	 (x) == FSPI_PROTOCOL_TI 		|| \
									 	 (x) == FSPI_PROTOCOL_NMW)

#define	FSPI_ASSERT_SPI_MODE(x)			((x) == FSPI_MODE_MASTER			|| \
									 	 (x) == FSPI_MODE_SLAVE 			|| \
									 	 (x) == FSPI_MODE_SLAVE_NO_OUTPUT)

#define FSPI_ASSERT_SPI_CLOCK_SOURCE(x)  		((x) == FSPI_CLOCK_SOURCE_SYSTEM || \
                                     	 	 	 (x) == FSPI_CLOCK_SOURCE_PIOSC)

#define	FSPI_ASSERT_SPI_BITRATE(mode, bitrate)  	((mode) == FSPI_MODE_MASTER ? (bitrate) <= (SysCtlClockGet() * 2) : (bitrate) <= (SysCtlClockGet() * 12))

#define	FSPI_ASSERT_SPI_DATAWIDTH(x)				((x) >= 4 && (x) <= 16)

#define FSPI_ASSERT_SPI_INT(x)						((x) == FSPI_INT_TXFIFO_HALF_OR_LESS 	|| \
													 (x) == FSPI_INT_TXFIFO_EMPTY 			|| \
													 (x) == FSPI_INT_RXFIFO_HALF_OR_MORE	|| \
													 (x) == FSPI_INT_RX_TIMEOUT				|| \
													 (x) == FSPI_INT_RX_OVERRUN)

#define FSPI_ASSERT_SPI_STATUS(x)					((x) == FSPI_STATUS_ON    || \
													 (x) == FSPI_STATUS_OFF)

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
static void fSpi_SysCtlStatus (uint32_t FSPI_MODULEx, uint32_t FSPI_STATUSx);
static void fSpi_GpioConfig (fGpio_Class* GpioClass, uint32_t* FGPIO_PINx, uint32_t* FGPIO_PORTx, uint32_t* FGPIO_TYPEx, uint32_t* FGPIO_CURRENTx, uint32_t* GPIO_AF, uint8_t nPins);
static void fSpi_DeInit (uint32_t FSPI_MODULEx);
static void fSpi_Config (uint32_t FSPI_MODULEx, uint32_t FSPI_MODEx, uint32_t FSPI_PROTOCOLx, uint32_t FSPI_CLOCK_SOURCEx, uint8_t datawidth, uint32_t bitrate);
static void fSpi_IntInit (fSpi_Struct* InitStruct, fGpio_Class* GpioClass, uint32_t FSPI_INTx, void (*SPI_IRQ) (void));
static void fSpi_Init (fSpi_Struct* InitStruct, fGpio_Class* GpioClass);

static uint32_t fSpi_IntGet (uint32_t FSPI_MODULEx, uint8_t FSPI_STATUSx);
static void fSpi_IntClear (uint32_t FSPI_MODULEx, uint32_t FSPI_INTx);
static void fSpi_IntStatus (uint32_t FSPI_MODULEx, uint32_t FSPI_INTx, uint8_t FSPI_STATUSx);
static uint8_t fSpi_IntTest	(uint32_t FSPI_MODULEx, uint32_t FSPI_INTx, uint8_t FSPI_STATUSx);

static void fSpi_SendData (uint32_t FSPI_MODULEx, uint32_t sByte);
static void fSpi_ReadData (uint32_t FSPI_MODULEx, uint32_t* rByte);

/* Private functions ---------------------------------------------------------*/

/*
 * Creates a class.
 */

fSpi_Class* fSpi_CreateClass (void)
{
	fSpi_Class* class = malloc (sizeof(fSpi_Class));
	if (class == NULL)
		return NULL;

	class->Init = fSpi_Init;
	class->IntInit = fSpi_IntInit;

	class->IntClear = fSpi_IntClear;
	class->IntGet = fSpi_IntGet;
	class->IntStatus = fSpi_IntStatus;
	class->IntTest = fSpi_IntTest;

	class->ReadData = fSpi_ReadData;
	class->SendData = fSpi_SendData;

	return class;
}

/*
 * Destroys a class.
 */

void fSpi_DestroyClass (fSpi_Class* class)
{
	free(class);
}

/*
 * Creates an initialization structure.
 */

fSpi_Struct* fSpi_CreateInitStruct (uint8_t nPins)
{
	fSpi_Struct* InitStruct;

	InitStruct = malloc(sizeof(fSpi_Struct));
	if (InitStruct == NULL)
		return NULL;
	InitStruct->nPins = nPins;

	InitStruct->GpioPin = malloc(sizeof(uint32_t) * InitStruct->nPins);
	if (InitStruct->GpioPin == NULL)
	{
		free(InitStruct);
		return NULL;
	}
	InitStruct->GpioPort = malloc(sizeof(uint32_t) * InitStruct->nPins);
	if (InitStruct->GpioPort == NULL)
	{
		free(InitStruct->GpioPin);
		free(InitStruct);
		return NULL;
	}
	InitStruct->GpioType = malloc(sizeof(uint32_t) * InitStruct->nPins);
	if (InitStruct->GpioType == NULL)
	{
		free(InitStruct->GpioPort);
		free(InitStruct->GpioPin);
		free(InitStruct);
		return NULL;
	}
	InitStruct->GpioCurrent = malloc(sizeof(uint32_t) * InitStruct->nPins);
	if (InitStruct->GpioCurrent == NULL)
	{
		free(InitStruct->GpioType);
		free(InitStruct->GpioPort);
		free(InitStruct->GpioPin);
		free(InitStruct);
		return NULL;
	}
	InitStruct->GpioAlternateFunction = malloc(sizeof(uint32_t) * InitStruct->nPins);
	if (InitStruct->GpioAlternateFunction == NULL)
	{
		free(InitStruct->GpioCurrent);
		free(InitStruct->GpioType);
		free(InitStruct->GpioPort);
		free(InitStruct->GpioPin);
		free(InitStruct);
		return NULL;
	}

	return InitStruct;
}

/*
 * Destroys an initialization structure.
 */

void fSpi_DestroyInitStruct (fSpi_Struct* InitStruct)
{
	free(InitStruct->GpioPin);
	free(InitStruct->GpioPort);
	free(InitStruct->GpioType);
	free(InitStruct->GpioCurrent);
	free(InitStruct->GpioAlternateFunction);
	free(InitStruct);
}

/*
 * Initializes the SysCtl clock for the specified SPI module.
 */

static void fSpi_SysCtlStatus (uint32_t FSPI_MODULEx, uint32_t FSPI_STATUSx)
{
#ifdef	DEBUG
	ASSERT_PARAM(FSPI_ASSERT_SPI_MODULE(FSPI_MODULEx));
	ASSERT_PARAM(FSPI_ASSERT_SPI_STATUS(FSPI_STATUSx));
#endif

	switch (FSPI_MODULEx)
	{
	case FSPI_MODULE_0:
		(FSPI_STATUSx == FSPI_STATUS_ON) ? SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0) : SysCtlPeripheralDisable(SYSCTL_PERIPH_SSI0);
		break;
	case FSPI_MODULE_1:
		(FSPI_STATUSx == FSPI_STATUS_ON) ? SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI1) : SysCtlPeripheralDisable(SYSCTL_PERIPH_SSI1);
		break;
	case FSPI_MODULE_2:
		(FSPI_STATUSx == FSPI_STATUS_ON) ? SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI2) : SysCtlPeripheralDisable(SYSCTL_PERIPH_SSI2);
		break;
	case FSPI_MODULE_3:
		(FSPI_STATUSx == FSPI_STATUS_ON) ? SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI3) : SysCtlPeripheralDisable(SYSCTL_PERIPH_SSI3);
		break;
	default:
		break;
	}
}

/*
 * Configures the GPIO pins for the SPI peripheral.
 */

static void fSpi_GpioConfig (fGpio_Class* GpioClass, uint32_t* FGPIO_PINx, uint32_t* FGPIO_PORTx, uint32_t* FGPIO_TYPEx, uint32_t* FGPIO_CURRENTx, uint32_t* GPIO_AF, uint8_t nPins)
{
	uint8_t i;

	for (i = 0; i < nPins; i++)
	{
		GpioClass->InitAlternateFunction(FGPIO_PINx[i], FGPIO_PORTx[i], FGPIO_TYPEx[i], FGPIO_CURRENTx[i], GPIO_AF[i]);
	}
}

/*
 * Disables the SPI peripheral.
 */

static void fSpi_DeInit (uint32_t FSPI_MODULEx)
{
#ifdef	DEBUG
	ASSERT_PARAM(FSPI_ASSERT_SPI_MODULE(FSPI_MODULEx));
#endif

    //fSpi_SysCtlStatus(FSPI_MODULEx, FSPI_STATUS_OFF);
	SSIIntUnregister(FSPI_MODULEx);
	SSIDisable(FSPI_MODULEx);
}

/*
 * Configures the specified SPI peripheral.
 */

static void fSpi_Config (uint32_t FSPI_MODULEx, uint32_t FSPI_MODEx, uint32_t FSPI_PROTOCOLx, uint32_t FSPI_CLOCK_SOURCEx, uint8_t datawidth, uint32_t bitrate)
{
#ifdef	DEBUG
	ASSERT_PARAM(FSPI_ASSERT_SPI_MODULE(FSPI_MODULEx));
	ASSERT_PARAM(FSPI_ASSERT_SPI_MODE(FSPI_MODEx));
	ASSERT_PARAM(FSPI_ASSERT_SPI_PROTOCOL(FSPI_PROTOCOLx));
	ASSERT_PARAM(FSPI_ASSERT_SPI_CLOCK_SOURCE(FSPI_CLOCK_SOURCEx));
	ASSERT_PARAM(FSPI_ASSERT_SPI_DATAWIDTH(datawidth));
	ASSERT_PARAM(FSPI_ASSERT_SPI_BITRATE(FSPI_MODEx, bitrate));
#endif
	uint32_t Freq;

	SSIClockSourceSet(FSPI_MODULEx, FSPI_CLOCK_SOURCEx);

	if (FSPI_CLOCK_SOURCEx == FSPI_CLOCK_SOURCE_SYSTEM)
	    Freq = SysCtlClockGet();
	else if (FSPI_CLOCK_SOURCEx == FSPI_CLOCK_SOURCE_PIOSC)
	    Freq = PIOSC_NOMINAL_FREQUENCY;

	SSIConfigSetExpClk(FSPI_MODULEx, Freq, FSPI_PROTOCOLx, FSPI_MODEx, bitrate, datawidth);
}

/*
 * Configures the interrupts.
 */

static void fSpi_IntInit (fSpi_Struct* InitStruct, fGpio_Class* GpioClass, uint32_t FSPI_INTx, void (*SPI_IRQ) (void))
{
#ifdef DEBUG
	ASSERT_PARAM(FSPI_ASSERT_SPI_MODULE(InitStruct->Module));
	ASSERT_PARAM(FSPI_ASSERT_SPI_MODE(InitStruct->Mode));
	ASSERT_PARAM(FSPI_ASSERT_SPI_PROTOCOL(InitStruct->Protocol));
	ASSERT_PARAM(FSPI_ASSERT_SPI_CLOCK_SOURCE(InitStruct->ClockSource));
	ASSERT_PARAM(FSPI_ASSERT_SPI_DATAWIDTH(InitStruct->DataWidth));
	ASSERT_PARAM(FSPI_ASSERT_SPI_BITRATE(InitStruct->Mode, InitStruct->BitRate));
#endif

	fSpi_SysCtlStatus(InitStruct->Module, FSPI_STATUS_ON);
	fSpi_GpioConfig(GpioClass, InitStruct->GpioPin, InitStruct->GpioPort, InitStruct->GpioType, InitStruct->GpioCurrent, InitStruct->GpioAlternateFunction, InitStruct->nPins);
	fSpi_DeInit(InitStruct->Module);
	fSpi_Config(InitStruct->Module, InitStruct->Mode, InitStruct->Protocol, InitStruct->ClockSource, InitStruct->DataWidth, InitStruct->BitRate);
	SSIIntRegister(InitStruct->Module, SPI_IRQ);
	fSpi_IntStatus(InitStruct->Module, FSPI_INTx, FSPI_STATUS_ON);
	SSIEnable(InitStruct->Module);
}

/*
 * Initializes and starts the specified SPI module.
 */

static void fSpi_Init (fSpi_Struct* InitStruct, fGpio_Class* GpioClass)
{
#ifdef	DEBUG
	ASSERT_PARAM(FSPI_ASSERT_SPI_MODULE(InitStruct->Module));
	ASSERT_PARAM(FSPI_ASSERT_SPI_MODE(InitStruct->Mode));
	ASSERT_PARAM(FSPI_ASSERT_SPI_PROTOCOL(InitStruct->Protocol));
	ASSERT_PARAM(FSPI_ASSERT_SPI_CLOCK_SOURCE(InitStruct->ClockSource));
	ASSERT_PARAM(FSPI_ASSERT_SPI_DATAWIDTH(InitStruct->DataWidth));
	ASSERT_PARAM(FSPI_ASSERT_SPI_BITRATE(InitStruct->Mode, InitStruct->BitRate));
#endif

    fSpi_SysCtlStatus(InitStruct->Module, FSPI_STATUS_ON);
	fSpi_GpioConfig(GpioClass, InitStruct->GpioPin, InitStruct->GpioPort, InitStruct->GpioType, InitStruct->GpioCurrent, InitStruct->GpioAlternateFunction, InitStruct->nPins);
	fSpi_DeInit(InitStruct->Module);
	fSpi_Config(InitStruct->Module, InitStruct->Mode, InitStruct->Protocol, InitStruct->ClockSource, InitStruct->DataWidth, InitStruct->BitRate);
	SSIEnable(InitStruct->Module);
}

/*
 *
 */

static uint32_t fSpi_IntGet (uint32_t FSPI_MODULEx, uint8_t FSPI_STATUSx)
{
#ifdef	DEBUG
	ASSERT_PARAM(FSPI_ASSERT_SPI_MODULE(FSPI_MODULEx));
	ASSERT_PARAM(FSPI_ASSERT_SPI_STATUS(FSPI_STATUSx));
#endif

	return SSIIntStatus(FSPI_MODULEx, FSPI_STATUSx);
}

/*
 *
 */

static uint8_t fSpi_IntTest	(uint32_t FSPI_MODULEx, uint32_t FSPI_INTx, uint8_t FSPI_STATUSx)
{
#ifdef	DEBUG
	ASSERT_PARAM(FSPI_ASSERT_SPI_MODULE(FSPI_MODULEx));
	ASSERT_PARAM(FSPI_ASSERT_SPI_INT(FSPI_INTx));
	ASSERT_PARAM(FSPI_ASSERT_SPI_STATUS(FSPI_STATUSx));
#endif

	if (SSIIntStatus(FSPI_MODULEx, FSPI_STATUSx) & FSPI_INTx)
		return FSPI_STATUS_ON;

	return FSPI_STATUS_OFF;
}

/*
 *
 */

static void fSpi_IntClear (uint32_t FSPI_MODULEx, uint32_t FSPI_INTx)
{
#ifdef	DEBUG
	ASSERT_PARAM(FSPI_ASSERT_SPI_MODULE(FSPI_MODULEx));
#endif

	SSIIntClear(FSPI_MODULEx, FSPI_INTx);
}

/*
 *
 */

static void fSpi_IntStatus (uint32_t FSPI_MODULEx, uint32_t FSPI_INTx, uint8_t FSPI_STATUSx)
{
#ifdef	DEBUG
	ASSERT_PARAM(FSPI_ASSERT_SPI_MODULE(FSPI_MODULEx));
	ASSERT_PARAM(FSPI_ASSERT_SPI_STATUS(FSPI_STATUSx));
#endif

	if (FSPI_STATUSx == FSPI_STATUS_ON)
		SSIIntEnable(FSPI_MODULEx, FSPI_INTx);
	else
		SSIIntDisable(FSPI_MODULEx, FSPI_INTx);
}

/*
 * Sends the specified byte and returns the received byte.
 */

static void fSpi_SendData (uint32_t FSPI_MODULEx, uint32_t sByte)
{
#ifdef   DEBUG
	ASSERT_PARAM(FSPI_ASSERT_SPI_MODULE(FSPI_MODULEx));
#endif

    SSIDataPut(FSPI_MODULEx, sByte);
}

/*
 *
 */

static void fSpi_ReadData (uint32_t FSPI_MODULEx, uint32_t* rByte)
{
#ifdef   DEBUG
	ASSERT_PARAM(FSPI_ASSERT_SPI_MODULE(FSPI_MODULEx));
#endif

    SSIDataGet(FSPI_MODULEx, rByte);
}
