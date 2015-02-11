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
#include "functions_pins.h"
#include "conf.h"
#include "gpio.h"
#include "hw_memmap.h"
#include "sysctl.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
#define	ASSERT_GPIO_PORT_BASE(x)	((x) == GPIO_PORTA_BASE || \
									 (x) == GPIO_PORTB_BASE || \
									 (x) == GPIO_PORTC_BASE || \
									 (x) == GPIO_PORTD_BASE || \
									 (x) == GPIO_PORTE_BASE || \
									 (x) == GPIO_PORTF_BASE || \
									 (x) == GPIO_PORTG_BASE || \
									 (x) == GPIO_PORTH_BASE || \
									 (x) == GPIO_PORTJ_BASE || \
									 (x) == GPIO_PORTK_BASE || \
									 (x) == GPIO_PORTL_BASE || \
									 (x) == GPIO_PORTM_BASE || \
									 (x) == GPIO_PORTN_BASE || \
									 (x) == GPIO_PORTP_BASE || \
									 (x) == GPIO_PORTQ_BASE || \
									 (x) == GPIO_PORTR_BASE || \
									 (x) == GPIO_PORTS_BASE || \
									 (x) == GPIO_PORTT_BASE)

#define	ASSERT_GPIO_PIN(x)			((x) == GPIO_PIN_0 || \
									 (x) == GPIO_PIN_1 || \
									 (x) == GPIO_PIN_2 || \
									 (x) == GPIO_PIN_3 || \
									 (x) == GPIO_PIN_4 || \
									 (x) == GPIO_PIN_5 || \
									 (x) == GPIO_PIN_6 || \
									 (x) == GPIO_PIN_7)

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/*
 * Function that returns the SysCtl memory mapped register for the port specified.
 */

uint32_t fPins_getGpioSysCtl (const uint32_t GPIO_PORTx_BASE)
{
#ifdef	DEBUG
	ASSERT_PARAM(ASSERT_GPIO_PORT_BASE(GPIO_PORTx_BASE));
#endif

	switch (GPIO_PORTx_BASE)
	{
	case GPIO_PORTA_BASE:
		return SYSCTL_PERIPH_GPIOA;
	case GPIO_PORTB_BASE:
		return SYSCTL_PERIPH_GPIOB;
	case GPIO_PORTC_BASE:
		return SYSCTL_PERIPH_GPIOC;
	case GPIO_PORTD_BASE:
		return SYSCTL_PERIPH_GPIOD;
	case GPIO_PORTE_BASE:
		return SYSCTL_PERIPH_GPIOE;
	case GPIO_PORTF_BASE:
		return SYSCTL_PERIPH_GPIOF;
	case GPIO_PORTG_BASE:
		return SYSCTL_PERIPH_GPIOG;
	case GPIO_PORTH_BASE:
		return SYSCTL_PERIPH_GPIOH;
	case GPIO_PORTJ_BASE:
		return SYSCTL_PERIPH_GPIOJ;
	case GPIO_PORTK_BASE:
		return SYSCTL_PERIPH_GPIOK;
	case GPIO_PORTL_BASE:
		return SYSCTL_PERIPH_GPIOL;
	case GPIO_PORTM_BASE:
		return SYSCTL_PERIPH_GPIOM;
	case GPIO_PORTN_BASE:
		return SYSCTL_PERIPH_GPION;
	case GPIO_PORTP_BASE:
		return SYSCTL_PERIPH_GPIOP;
	case GPIO_PORTQ_BASE:
		return SYSCTL_PERIPH_GPIOQ;
	case GPIO_PORTR_BASE:
		return SYSCTL_PERIPH_GPIOR;
	case GPIO_PORTS_BASE:
		return SYSCTL_PERIPH_GPIOS;
	case GPIO_PORTT_BASE:
		return SYSCTL_PERIPH_GPIOT;
	default:
		return (uint32_t) 0;
	}
}

/*
 * Function that returns the GPIO interrupt pin for the pin specified.
 */

uint32_t fPins_getGpioIntPin (const uint32_t GPIO_PIN_x)
{
#ifdef	DEBUG
	ASSERT_PARAM(ASSERT_GPIO_PIN(GPIO_PIN_x));
#endif

	switch (GPIO_PIN_x)
	{
	case GPIO_PIN_0:
		return GPIO_INT_PIN_0;
	case GPIO_PIN_1:
		return GPIO_INT_PIN_1;
	case GPIO_PIN_2:
		return GPIO_INT_PIN_2;
	case GPIO_PIN_3:
		return GPIO_INT_PIN_3;
	case GPIO_PIN_4:
		return GPIO_INT_PIN_4;
	case GPIO_PIN_5:
		return GPIO_INT_PIN_5;
	case GPIO_PIN_6:
		return GPIO_INT_PIN_6;
	case GPIO_PIN_7:
		return GPIO_INT_PIN_7;
	default:
		return (uint32_t) 0;
	}
}

