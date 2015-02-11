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
#ifndef __FUNCTIONS_PINS_H
#define __FUNCTIONS_PINS_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/
typedef struct
{
	uint32_t	GPIO_PIN_x;
	uint32_t	GPIO_PORTx_BASE;
} fPins_Pin;

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
uint32_t fPins_getGpioSysCtl (fPins_Pin* const Gpio_Pin);
uint32_t fPins_getGpioIntPin (fPins_Pin* const Gpio_Pin);
void fPins_unlockGpioPin (fPins_Pin* const Gpio_Pin);

#ifdef __cplusplus
}
#endif

#endif /*__FUNCTIONS_PINS_H */
 
