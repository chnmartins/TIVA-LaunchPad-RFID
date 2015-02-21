/**
  ******************************************************************************
  * @file    C_HEADER_TEMPLATE.h
  * @author  Header template author and its contact email
  * @version Version 1 Issue 1
  * @date    Date when the file was released.	(i.e. 05-March-2014)
  * @brief   Short description about the header template.
  ******************************************************************************
  * @attention
  *
  *		Copyright Information (C)
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FUNCTIONS_SPI_H
#define __FUNCTIONS_SPI_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "hw_memmap.h"
#include "ssi.h"
#include "functions_gpio.h"

/* Exported types ------------------------------------------------------------*/
typedef struct
{
	uint32_t	Module;
	uint8_t		nPins;
	fGpio_Pin**  Pins;
	uint32_t	Protocol;
	uint32_t    Mode;
	uint32_t    ClockSource;
	uint32_t    BitRate;
	uint32_t    DataWidth;

	uint32_t    Int;
	void (*IntIRQ) (void);
} fSpi_Mod;


/* Exported constants --------------------------------------------------------*/
#define	SPI_MOD0	SSI0_BASE
#define	SPI_MOD1	SSI1_BASE
#define	SPI_MOD2	SSI2_BASE
#define	SPI_MOD3	SSI3_BASE

#define PROT_POL0_PHA0		SSI_FRF_MOTO_MODE_0
#define	PROT_POL0_PHA1		SSI_FRF_MOTO_MODE_1
#define	PROT_POL1_PHA0		SSI_FRF_MOTO_MODE_2
#define	PROT_POL1_PHA1		SSI_FRF_MOTO_MODE_3
#define	PROT_TI				SSI_FRF_TI
#define	PROT_NMW			SSI_FRF_NMW

#define	MODE_MASTER				SSI_MODE_MASTER
#define	MODE_SLAVE				SSI_MODE_SLAVE
#define	MODE_SLAVE_NO_OUTPUT	SSI_MODE_SLAVE_OD

#define FSPI_CLK_SYSTEM              SSI_CLOCK_SYSTEM
#define FSPI_CLK_PIOSC               SSI_CLOCK_PIOSC

#define INT_NONE                    0
#define INT_TXFIFO_HALF_OR_LESS     SSI_TXFF
#define INT_TXFIFO_EMPTY            SSI_TXEOT
#define INT_RXFIFO_HALF_OR_MORE     SSI_RXFF
#define INT_RX_TIMEOUT              SSI_RXTO
#define INT_RX_OVERRUN              SSI_RXOR

/* Exported macro ------------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
void fSpi_enableSysCtl (const fSpi_Mod* Spi_Mod);
void fSpi_setGpio(const fSpi_Mod* Spi_Mod);
void fSpi_Disable (const fSpi_Mod* Spi_Mod);
bool fSpi_Config (const fSpi_Mod* Spi_Mod);
void fSpi_Start (const fSpi_Mod* Spi_Mod);
void fSpi_IntInit (const fSpi_Mod* Spi_Mod);
void fSpi_Init (const fSpi_Mod* Spi_Mod);
void fSpi_SendReceive (const fSpi_Mod* Spi_Mod, uint32_t sByte, uint32_t* rByte);

#ifdef __cplusplus
}
#endif

#endif /*__FUNCTIONS_SPI_H */
 
