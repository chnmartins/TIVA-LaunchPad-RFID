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

#include "ssi.h"
#include "sysctl.h"
#include "conf.h"
#include "functions_spi.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define PIOSC_FREQUENCY             (16000000)

/* Private macro -------------------------------------------------------------*/
#define	ASSERT_SPI_MODULE(x)		((x)->Module == SPI_MOD0 || \
							 	 	 (x)->Module == SPI_MOD1 || \
							 	 	 (x)->Module == SPI_MOD2 || \
							 	 	 (x)->Module == SPI_MOD3)

#define	ASSERT_SPI_PROTOCOL(x)		((x)->Protocol == PROT_POL0_PHA0 || \
									 (x)->Protocol == PROT_POL0_PHA1 || \
									 (x)->Protocol == PROT_POL1_PHA0 || \
									 (x)->Protocol == PROT_POL1_PHA1 || \
									 (x)->Protocol == PROT_TI || \
									 (x)->Protocol == PROT_NMW)

#define	ASSERT_SPI_MODE(x)			((x)->Mode == MODE_MASTER || \
									 (x)->Mode == MODE_SLAVE || \
									 (x)->Mode == MODE_SLAVE_NO_OUTPUT)

#define ASSERT_SPI_CLOCKSOURCE(x)   ((x)->ClockSource == FSPI_CLK_PIOSC || \
                                     (x)->ClockSource == FSPI_CLK_SYSTEM)

#define	ASSERT_SPI_BITRATE(x)       ((x)->Mode == MODE_MASTER ? (x)->BitRate <= SysCtlClockGet() * 2 : (x)->BitRate <= SysCtlClockGet() * 12)

#define	ASSERT_SPI_DATAWIDTH(x)		((x)->DataWidth >= 4 && (x)->DataWidth <= 16)


/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/*
 * Initializes the SysCtl clock for the specified SPI module.
 */

void fSpi_enableSysCtl (const fSpi_Mod* Spi_Mod)
{
#ifdef	DEBUG
	ASSERT_PARAM(ASSERT_SPI_MODULE(Spi_Mod));
#endif

	switch (Spi_Mod->Module)
	{
	case SPI_MOD0:
		SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
		break;
	case SPI_MOD1:
		SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI1);
		break;
	case SPI_MOD2:
		SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI2);
		break;
	case SPI_MOD3:
		SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI3);
		break;
	default:
		break;
	}
}

/*
 * Configures the GPIO pins for the SPI peripheral.
 */

void fSpi_setGpio(const fSpi_Mod* Spi_Mod)
{
#ifdef	DEBUG
	ASSERT_PARAM(ASSERT_SPI_MODULE(Spi_Mod));
#endif

	uint8_t i = 0;

	for (i = 0; i < Spi_Mod->nPins; i++)
	{
		fGpio_enableSysCtl(*((Spi_Mod->Pins) + i));
		fGpio_unlockPin(*((Spi_Mod->Pins) + i));
		fGpio_setAlternateFunction(*((Spi_Mod->Pins) + i));
		fGpio_setDirection(*((Spi_Mod->Pins) + i));
		fGpio_setConfig(*((Spi_Mod->Pins) + i));
	}
}

/*
 * Disables the SPI peripheral.
 */

void fSpi_Disable (const fSpi_Mod* Spi_Mod)
{
#ifdef	DEBUG
	ASSERT_PARAM(ASSERT_SPI_MODULE(Spi_Mod));
#endif

	SSIDisable(Spi_Mod->Module);
}

/*
 * Configures the specified SPI peripheral.
 */

bool fSpi_Config (const fSpi_Mod* Spi_Mod)
{
#ifdef	DEBUG
	ASSERT_PARAM(ASSERT_SPI_MODULE(Spi_Mod));
    ASSERT_PARAM(ASSERT_SPI_MODE(Spi_Mod));
    ASSERT_PARAM(ASSERT_SPI_PROTOCOL(Spi_Mod));
	ASSERT_PARAM(ASSERT_SPI_CLOCKSOURCE(Spi_Mod));
	ASSERT_PARAM(ASSERT_SPI_DATAWIDTH(Spi_Mod));
#endif
	uint32_t Freq;

	SSIClockSourceSet(Spi_Mod->Module, Spi_Mod->ClockSource);

	if (Spi_Mod->ClockSource == FSPI_CLK_SYSTEM) {
	    Freq = SysCtlClockGet();
	} else if (Spi_Mod->ClockSource == FSPI_CLK_PIOSC){
	    Freq = PIOSC_FREQUENCY;
	}

	if (Spi_Mod->Mode == MODE_MASTER)
	{
	    if (Spi_Mod->BitRate > Freq * 2)
	        return false;
	} else {
	    if (Spi_Mod->BitRate > Freq * 12)
	        return false;
	}

	SSIConfigSetExpClk(Spi_Mod->Module, Freq, Spi_Mod->Protocol, Spi_Mod->Mode, Spi_Mod->BitRate, Spi_Mod->DataWidth);

	return true;
}

/*
 * Starts the SPI peripheral.
 */

void fSpi_Start (const fSpi_Mod* Spi_Mod)
{
#ifdef DEBUG
    ASSERT_PARAM(ASSERT_SPI_MODULE(Spi_Mod));
#endif

    SSIEnable(Spi_Mod->Module);
}

/*
 * Configures the interrupts.
 */

void fSpi_IntInit (const fSpi_Mod* Spi_Mod)
{
#ifdef DEBUG
    ASSERT_PARAM(ASSERT_SPI_MODULE(Spi_Mod));
#endif

    if (Spi_Mod->Int != INT_NONE)
    {
        SSIIntRegister(Spi_Mod->Module, Spi_Mod->IntIRQ);
        SSIIntEnable(Spi_Mod->Module, Spi_Mod->Int);
    }
}

/*
 * Initializes and starts the specified SPI module.
 */

void fSpi_Init (const fSpi_Mod* Spi_Mod)
{
#ifdef	DEBUG
    ASSERT_PARAM(ASSERT_SPI_MODULE(Spi_Mod));
    ASSERT_PARAM(ASSERT_SPI_MODE(Spi_Mod));
    ASSERT_PARAM(ASSERT_SPI_PROTOCOL(Spi_Mod));
    ASSERT_PARAM(ASSERT_SPI_BITRATE(Spi_Mod));
    ASSERT_PARAM(ASSERT_SPI_CLOCKSOURCE(Spi_Mod));
    ASSERT_PARAM(ASSERT_SPI_DATAWIDTH(Spi_Mod));
#endif

	fSpi_enableSysCtl(Spi_Mod);
	fSpi_setGpio(Spi_Mod);
	fSpi_Disable(Spi_Mod);
	fSpi_Config(Spi_Mod);
	fSpi_IntInit(Spi_Mod);
	fSpi_Start(Spi_Mod);
}

/*
 * Sends the specified byte and returns the received byte.
 */

void fSpi_SendReceive (const fSpi_Mod* Spi_Mod, uint32_t sByte, uint32_t* rByte)
{
#ifdef   DEBUG
    ASSERT_PARAM(ASSERT_SPI_MODULE(Spi_Mod));
#endif
    SSIDataPut(Spi_Mod->Module, sByte);
    SSIDataGet(Spi_Mod->Module, rByte);
}

