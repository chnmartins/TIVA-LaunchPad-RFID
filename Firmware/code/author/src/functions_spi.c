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

/* Private macro -------------------------------------------------------------*/
#define	ASSERT_SPI_MODULE(x)		((x) == SPI_MOD0 || \
							 	 	 (x) == SPI_MOD1 || \
							 	 	 (x) == SPI_MOD2 || \
							 	 	 (x) == SPI_MOD3)

#define	ASSERT_SPI_PROTOCOL			((x) == PROT_POL0_PHA0 || \
									 (x) == PROT_POL0_PHA1 || \
									 (x) == PROT_POL1_PHA0 || \
									 (x) == PROT_POL1_PHA1 || \
									 (x) == PROT_TI || \
									 (x) == PROT_NMW)

#define	ASSERT_SPI_MODE				((x) == MODE_MASTER || \
									 (x) == MODE_SLAVE || \
									 (x) == MODE_SLAVE_NO_OUTPUT)

#define	ASSERT_SPI_BITRATE(x)

#define	ASSERT_SPI_DATAWIDTH(x)		((x) >= 4 && (x) <= 16)


/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/*
 * Initializes the SysCtl clock for the specified SPI module.
 */

void fSpi_enableSysCtl (const fSpi_Mod* Spi_Mod)
{
#ifdef	DEBUG
	ASSERT_PARAM(ASSERT_SPI_MODULE(Spi_Mod->Module));
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
	ASSERT_PARAM(ASSERT_SPI_MODULE(Spi_Mod->Module));
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
	ASSERT_PARAM(ASSERT_SPI_MODULE(Spi_Mod->Module));
#endif

	SSIDisable(Spi_Mod->Module);
}

/*
 * Configures the specified SPI peripheral.
 */

void fSpi_Config (const fSpi_Mod* Spi_Mod)
{
#ifdef	DEBUG
	ASSERT_PARAM(ASSERT_SPI_MODULE(Spi_Mod->Module));
#endif


}

/*
 * Initializes and starts the specified SPI module.
 */

void fSpi_Init (const fSpi_Mod* Spi_Mod)
{
#ifdef	DEBUG
	ASSERT_PARAM(ASSERT_SPI_MODULE(Spi_Mod->Module));
#endif

	fSpi_enableSysCtl(Spi_Mod);
	fSpi_setGpio(Spi_Mod);
	fSpi_Disable(Spi_Mod);


}


