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
#include <stdbool.h>
#include "gpio.h"
#include "hw_memmap.h"

/* Exported types ------------------------------------------------------------*/
typedef struct
{
	uint32_t	Pin;
	uint32_t	Port;
	uint32_t    Direction;
	uint32_t	Current;
	uint32_t	Type;
	uint32_t    IntType;
	void (*IntIRQ) (void);
} fGpio_Pin;

/* Exported constants --------------------------------------------------------*/
#define PIN_0   GPIO_PIN_0
#define PIN_1   GPIO_PIN_1
#define PIN_2   GPIO_PIN_2
#define PIN_3   GPIO_PIN_3
#define PIN_4   GPIO_PIN_4
#define PIN_5   GPIO_PIN_5
#define PIN_6   GPIO_PIN_6
#define PIN_7   GPIO_PIN_7

#define PORT_A  GPIO_PORTA_BASE
#define PORT_B  GPIO_PORTB_BASE
#define PORT_C  GPIO_PORTC_BASE
#define PORT_D  GPIO_PORTD_BASE
#define PORT_E  GPIO_PORTE_BASE
#define PORT_F  GPIO_PORTF_BASE
#define PORT_G  GPIO_PORTG_BASE
#define PORT_H  GPIO_PORTH_BASE
#define PORT_J  GPIO_PORTJ_BASE
#define PORT_K  GPIO_PORTK_BASE
#define PORT_L  GPIO_PORTL_BASE
#define PORT_M  GPIO_PORTM_BASE
#define PORT_N  GPIO_PORTN_BASE
#define PORT_P  GPIO_PORTP_BASE
#define PORT_Q  GPIO_PORTQ_BASE
#define PORT_R  GPIO_PORTR_BASE
#define PORT_S  GPIO_PORTS_BASE
#define PORT_T  GPIO_PORTT_BASE

#define DIR_IN  GPIO_DIR_MODE_IN
#define DIR_OUT GPIO_DIR_MODE_OUT
#define DIR_HW  GPIO_DIR_MODE_HW

#define CURR_2MA        GPIO_STRENGTH_2MA
#define CURR_4MA        GPIO_STRENGTH_4MA
#define CURR_6MA        GPIO_STRENGTH_6MA
#define CURR_8MA        GPIO_STRENGTH_8MA
#define CURR_8MA_SC     GPIO_STRENGTH_8MA_SC
#define CURR_10MA       GPIO_STRENGTH_10MA
#define CURR_12MA       GPIO_STRENGTH_12MA

#define TYPE_PP         GPIO_PIN_TYPE_STD
#define TYPE_PP_PU      GPIO_PIN_TYPE_STD_WPU
#define TYPE_PP_PD      GPIO_PIN_TYPE_STD_WPD
#define TYPE_OD         GPIO_PIN_TYPE_OD
#define TYPE_ANALOG     GPIO_PIN_TYPE_ANALOG
#define TYPE_WAKE_HIGH  GPIO_PIN_TYPE_WAKE_HIGH
#define TYPE_WAKE_LOW   GPIO_PIN_TYPE_WAKE_LOW

#define INTTYPE_FALLING_EDGE    GPIO_FALLING_EDGE
#define INTTYPE_RISING_EDGE     GPIO_RISING_EDGE
#define INTTYPE_BOTH_EDGES      GPIO_BOTH_EDGES
#define INTTYPE_LOW_LEVEL       GPIO_LOW_LEVEL
#define INTTYPE_HIGH_LEVEL      GPIO_HIGH_LEVEL
#define INTTYPE_DISCRETE_INT    GPIO_DISCRETE_INT

/* Exported macro ------------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
void fGpio_enableSysCtl (fGpio_Pin* const Gpio_Pin);
uint32_t fGpio_getIntPin (fGpio_Pin* const Gpio_Pin);
void fGpio_unlockPin (fGpio_Pin* const Gpio_Pin);
void fGpio_setConfig (fGpio_Pin* const Gpio_Pin);
void fGpio_setDirection (fGpio_Pin* const Gpio_Pin);
void fGpio_setToggle (fGpio_Pin* const Gpio_Pin);
void fGpio_setLow (fGpio_Pin* const Gpio_Pin);
void fGpio_setHigh (fGpio_Pin* const Gpio_Pin);
bool fGpio_getLevel (fGpio_Pin* const Gpio_Pin);
void fGpio_Init (fGpio_Pin* const Gpio_Pin);
void fGpio_IntInit (fGpio_Pin* const Gpio_Pin);
uint32_t fGpio_IntGet (fGpio_Pin* const Gpio_Pin);
void fGpio_IntClear (fGpio_Pin* const Gpio_Pin);

#ifdef __cplusplus
}
#endif

#endif /*__FUNCTIONS_PINS_H */
 
