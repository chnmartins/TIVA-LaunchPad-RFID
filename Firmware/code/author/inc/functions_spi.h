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
	uint32_t	Mode;
	uint32_t	Protocol;
	uint32_t	ClockSource;
	uint8_t		DataWidth;
	uint32_t	BitRate;

	uint8_t		nPins;
	uint32_t*	GpioPin;
	uint32_t*	GpioPort;
	uint32_t*	GpioType;
	uint32_t*	GpioCurrent;
	uint32_t*	GpioAlternateFunction;
} fSpi_Struct;

typedef struct
{
	void (*Init) (fSpi_Struct* InitStruct, fGpio_Class* GpioClass);
	void (*IntInit) (fSpi_Struct* InitStruct, fGpio_Class* GpioClass, uint32_t FSPI_INTx, void (*SPI_IRQ) (void));

	uint32_t (*IntGet) (uint32_t FSPI_MODULEx, uint8_t FSPI_STATUSx);
	void (*IntClear) (uint32_t FSPI_MODULEx, uint32_t FSPI_INTx);
	void (*IntStatus) (uint32_t FSPI_MODULEx, uint32_t FSPI_INTx, uint8_t FSPI_STATUSx);
	uint8_t (*IntTest)	(uint32_t FSPI_MODULEx, uint32_t FSPI_INTx, uint8_t FSPI_STATUSx);

	void (*SendData) (uint32_t FSPI_MODULEx, uint32_t sByte);
	void (*ReadData) (uint32_t FSPI_MODULEx, uint32_t* rByte);
} fSpi_Class;

/* Exported constants --------------------------------------------------------*/
#define	FSPI_MODULE_0	SSI0_BASE
#define	FSPI_MODULE_1	SSI1_BASE
#define	FSPI_MODULE_2	SSI2_BASE
#define	FSPI_MODULE_3	SSI3_BASE

#define FSPI_PROTOCOL_POL0_PHA0			SSI_FRF_MOTO_MODE_0
#define	FSPI_PROTOCOL_POL0_PHA1			SSI_FRF_MOTO_MODE_1
#define	FSPI_PROTOCOL_POL1_PHA0			SSI_FRF_MOTO_MODE_2
#define	FSPI_PROTOCOL_POL1_PHA1			SSI_FRF_MOTO_MODE_3
#define	FSPI_PROTOCOL_TI				SSI_FRF_TI
#define	FSPI_PROTOCOL_NMW				SSI_FRF_NMW

#define	FSPI_MODE_MASTER				SSI_MODE_MASTER
#define	FSPI_MODE_SLAVE					SSI_MODE_SLAVE
#define	FSPI_MODE_SLAVE_NO_OUTPUT		SSI_MODE_SLAVE_OD

#define FSPI_CLOCK_SOURCE_SYSTEM              SSI_CLOCK_SYSTEM
#define FSPI_CLOCK_SOURCE_PIOSC               SSI_CLOCK_PIOSC

#define FSPI_INT_TXFIFO_HALF_OR_LESS     		SSI_TXFF
#define FSPI_INT_TXFIFO_EMPTY            		SSI_TXEOT
#define FSPI_INT_RXFIFO_HALF_OR_MORE     		SSI_RXFF
#define FSPI_INT_RX_TIMEOUT              		SSI_RXTO
#define FSPI_INT_RX_OVERRUN             		SSI_RXOR

#define	FSPI_STATUS_ON							(0x01)
#define FSPI_STATUS_OFF							(0x00)

/* Exported macro ------------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
fSpi_Struct* fSpi_CreateInitStruct (uint8_t nPins);
void fSpi_DestroyInitStruct (fSpi_Struct* InitStruct);
fSpi_Class* fSpi_CreateClass (void);
void fSpi_DestroyClass (fSpi_Class* class);

#ifdef __cplusplus
}
#endif

#endif /*__FUNCTIONS_SPI_H */
 
