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
#include <stdio.h>

#include "functions_pins.h"
#include "conf.h"
#include "gpio.h"
#include "hw_gpio.h"
#include "hw_types.h"
#include "hw_memmap.h"
#include "sysctl.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define	GPIO_UNLOCK_VALUE	(0x4C4F434B)

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

#define	ASSERT_GPIO_TYPE(x)			((x) == GPIO_PIN_TYPE_STD || \
									 (x) == GPIO_PIN_TYPE_STD_WPU || \
									 (x) == GPIO_PIN_TYPE_STD_WPD || \
									 (x) == GPIO_PIN_TYPE_OD || \
									 (x) == GPIO_PIN_TYPE_ANALOG || \
									 (x) == GPIO_PIN_TYPE_WAKE_HIGH || \
									 (x) == GPIO_PIN_TYPE_WAKE_LOW)

#define	ASSERT_GPIO_CURRENT(x)		((x) == GPIO_STRENGTH_2MA || \
									 (x) == GPIO_STRENGTH_4MA || \
									 (x) == GPIO_STRENGTH_6MA || \
									 (x) == GPIO_STRENGTH_8MA || \
									 (x) == GPIO_STRENGTH_8MA_SC || \
									 (x) == GPIO_STRENGTH_10MA || \
									 (x) == GPIO_STRENGTH_12MA)

#define ASSERT_GPIO_DIRECTION(x)    ((x) == GPIO_DIR_MODE_IN || \
                                     (x) == GPIO_DIR_MODE_OUT || \
                                     (x) == GPIO_DIR_MODE_HW)
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/*
 * Function that enables the SysCtl memory mapped register for the port specified.
 */

void fPins_enableGpioSysCtl (fPins_Pin* const Gpio_Pin)
{
#ifdef	DEBUG
	ASSERT_PARAM(ASSERT_GPIO_PORT_BASE(Gpio_Pin->GPIO_Port));
#endif

	switch (Gpio_Pin->GPIO_Port)
	{
	case GPIO_PORTA_BASE:
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
		break;
	case GPIO_PORTB_BASE:
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
        break;
	case GPIO_PORTC_BASE:
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
        break;
	case GPIO_PORTD_BASE:
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
        break;
	case GPIO_PORTE_BASE:
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
        break;
	case GPIO_PORTF_BASE:
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
        break;
	case GPIO_PORTG_BASE:
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
        break;
	case GPIO_PORTH_BASE:
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOH);
        break;
	case GPIO_PORTJ_BASE:
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);
        break;
	case GPIO_PORTK_BASE:
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
        break;
	case GPIO_PORTL_BASE:
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);
        break;
	case GPIO_PORTM_BASE:
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);
        break;
	case GPIO_PORTN_BASE:
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
        break;
	case GPIO_PORTP_BASE:
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOP);
        break;
	case GPIO_PORTQ_BASE:
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOQ);
        break;
	case GPIO_PORTR_BASE:
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOR);
        break;
	case GPIO_PORTS_BASE:
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOS);
        break;
	case GPIO_PORTT_BASE:
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOT);
        break;
	default:
        break;
	}
}

/*
 * Function that returns the GPIO interrupt pin for the pin specified.
 */

uint32_t fPins_getGpioIntPin (fPins_Pin* const Gpio_Pin)
{
#ifdef	DEBUG
	ASSERT_PARAM(ASSERT_GPIO_PIN(Gpio_Pin->GPIO_Pin));
#endif

	switch (Gpio_Pin->GPIO_Pin)
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

/*
 * Function to unlock the specified GPIO pin on the specified port.
 */

void fPins_unlockGpioPin (fPins_Pin* const Gpio_Pin)
{
#ifdef	DEBUG
	ASSERT_PARAM(ASSERT_GPIO_PORT_BASE(Gpio_Pin->GPIO_Port));
	ASSERT_PARAM(ASSERT_GPIO_PIN(Gpio_Pin->GPIO_Pin));
#endif

	HWREG(Gpio_Pin->GPIO_Port + GPIO_O_LOCK) = GPIO_UNLOCK_VALUE;
	HWREG(Gpio_Pin->GPIO_Port + GPIO_O_CR) |= Gpio_Pin->GPIO_Pin;
}

/*
 * Function to set the current limit and the pin type of the GPIO pin specified.
 */

void fPins_setGpioConfig (fPins_Pin* const Gpio_Pin)
{
#ifdef	DEBUG
	ASSERT_PARAM(ASSERT_GPIO_PORT_BASE(Gpio_Pin->GPIO_Port));
	ASSERT_PARAM(ASSERT_GPIO_PIN(Gpio_Pin->GPIO_Pin));
	ASSERT_PARAM(ASSERT_GPIO_CURRENT(Gpio_Pin->GPIO_Current));
	ASSERT_PARAM(ASSERT_GPIO_TYPE(Gpio_Pin->GPIO_Type));
#endif

	GPIOPadConfigSet(Gpio_Pin->GPIO_Port, Gpio_Pin->GPIO_Pin, Gpio_Pin->GPIO_Current, Gpio_Pin->GPIO_Type);
}

/*
 * Function to set the current limit and the pin type of the GPIO pin specified.
 */

void fPins_setGpioDirection (fPins_Pin* const Gpio_Pin)
{
#ifdef  DEBUG
    ASSERT_PARAM(ASSERT_GPIO_PORT_BASE(Gpio_Pin->GPIO_Port));
    ASSERT_PARAM(ASSERT_GPIO_PIN(Gpio_Pin->GPIO_Pin));
    ASSERT_PARAM(ASSERT_GPIO_DIRECTION(Gpio_Pin->GPIO_Direction));
#endif

    GPIODirModeSet(Gpio_Pin->GPIO_Port, Gpio_Pin->GPIO_Pin, Gpio_Pin->GPIO_Direction);
}

/*
 * Function to turn to level high the pin specified.
 */

void fPins_setGpioHigh (fPins_Pin* const Gpio_Pin)
{
#ifdef  DEBUG
    ASSERT_PARAM(ASSERT_GPIO_PORT_BASE(Gpio_Pin->GPIO_Port));
    ASSERT_PARAM(ASSERT_GPIO_PIN(Gpio_Pin->GPIO_Pin));
#endif

    GPIOPinWrite(Gpio_Pin->GPIO_Port, Gpio_Pin->GPIO_Pin, Gpio_Pin->GPIO_Pin);
}

/*
 * Function to turn to level low the pin specified.
 */

void fPins_setGpioLow (fPins_Pin* const Gpio_Pin)
{
#ifdef  DEBUG
    ASSERT_PARAM(ASSERT_GPIO_PORT_BASE(Gpio_Pin->GPIO_Port));
    ASSERT_PARAM(ASSERT_GPIO_PIN(Gpio_Pin->GPIO_Pin));
#endif

    GPIOPinWrite(Gpio_Pin->GPIO_Port, Gpio_Pin->GPIO_Pin, ~(Gpio_Pin->GPIO_Pin));
}

/*
 * Function to toggle the pin specified.
 */

void fPins_setGpioToggle (fPins_Pin* const Gpio_Pin)
{
#ifdef  DEBUG
    ASSERT_PARAM(ASSERT_GPIO_PORT_BASE(Gpio_Pin->GPIO_Port));
    ASSERT_PARAM(ASSERT_GPIO_PIN(Gpio_Pin->GPIO_Pin));
#endif

    if (fPins_getGpioLevel(Gpio_Pin))
        GPIOPinWrite(Gpio_Pin->GPIO_Port, Gpio_Pin->GPIO_Pin, ~Gpio_Pin->GPIO_Pin);
    else
        GPIOPinWrite(Gpio_Pin->GPIO_Port, Gpio_Pin->GPIO_Pin, Gpio_Pin->GPIO_Pin);
}

/*
 * Function to read the status of the GPIO pin specified.
 */

bool fPins_getGpioLevel (fPins_Pin* const Gpio_Pin)
{
#ifdef  DEBUG
    ASSERT_PARAM(ASSERT_GPIO_PORT_BASE(Gpio_Pin->GPIO_Port));
    ASSERT_PARAM(ASSERT_GPIO_PIN(Gpio_Pin->GPIO_Pin));
#endif

    if (GPIOPinRead(Gpio_Pin->GPIO_Port, Gpio_Pin->GPIO_Pin))
        return true;

    return false;
}
