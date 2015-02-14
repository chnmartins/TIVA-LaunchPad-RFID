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

#include "functions_gpio.h"
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
#define	ASSERT_PORT(x)	            ((x) == PORT_A || \
									 (x) == PORT_B || \
									 (x) == PORT_C || \
									 (x) == PORT_D || \
									 (x) == PORT_E || \
									 (x) == PORT_F || \
									 (x) == PORT_G || \
									 (x) == PORT_H || \
									 (x) == PORT_J || \
									 (x) == PORT_K || \
									 (x) == PORT_L || \
									 (x) == PORT_M || \
									 (x) == PORT_N || \
									 (x) == PORT_P || \
									 (x) == PORT_Q || \
									 (x) == PORT_R || \
									 (x) == PORT_S || \
									 (x) == PORT_T)

#define	ASSERT_PIN(x)			    ((x) == PIN_0 || \
									 (x) == PIN_1 || \
									 (x) == PIN_2 || \
									 (x) == PIN_3 || \
									 (x) == PIN_4 || \
									 (x) == PIN_5 || \
									 (x) == PIN_6 || \
									 (x) == PIN_7)

#define ASSERT_DIRECTION(x)         ((x) == DIR_IN || \
                                     (x) == DIR_OUT || \
                                     (x) == DIR_HW)

#define ASSERT_CURRENT(x)           ((x) == CURR_2MA || \
                                     (x) == CURR_4MA || \
                                     (x) == CURR_6MA || \
                                     (x) == CURR_8MA || \
                                     (x) == CURR_8MA_SC || \
                                     (x) == CURR_10MA || \
                                     (x) == CURR_12MA)

#define	ASSERT_TYPE(x)			    ((x) == TYPE_PP || \
									 (x) == TYPE_PP_PU || \
									 (x) == TYPE_PP_PD || \
									 (x) == TYPE_OD || \
									 (x) == TYPE_ANALOG || \
									 (x) == TYPE_WAKE_HIGH || \
									 (x) == TYPE_WAKE_LOW)

#define ASSERT_INTTYPE(x)           ((x) == INTTYPE_FALLING_EDGE || \
                                     (x) == INTTYPE_RISING_EDGE || \
                                     (x) == INTTYPE_BOTH_EDGES || \
                                     (x) == INTTYPE_LOW_LEVEL || \
                                     (x) == INTTYPE_HIGH_LEVEL || \
                                     (x) == INTTYPE_DISCRETE_INT)

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/*
 * Function that enables the SysCtl memory mapped register for the port specified.
 */

void fGpio_enableSysCtl (fGpio_Pin* const Gpio_Pin)
{
#ifdef	DEBUG
	ASSERT_PARAM(ASSERT_PORT(Gpio_Pin->Port));
#endif

	switch (Gpio_Pin->Port)
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

uint32_t fGpio_getIntPin (fGpio_Pin* const Gpio_Pin)
{
#ifdef	DEBUG
	ASSERT_PARAM(ASSERT_PIN(Gpio_Pin->Pin));
#endif

	switch (Gpio_Pin->Pin)
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

void fGpio_unlockPin (fGpio_Pin* const Gpio_Pin)
{
#ifdef	DEBUG
	ASSERT_PARAM(ASSERT_PORT(Gpio_Pin->Port));
	ASSERT_PARAM(ASSERT_PIN(Gpio_Pin->Pin));
#endif

	HWREG(Gpio_Pin->Port + GPIO_O_LOCK) = GPIO_UNLOCK_VALUE;
	HWREG(Gpio_Pin->Port + GPIO_O_CR) |= Gpio_Pin->Pin;
}

/*
 * Function to set the current limit and the pin type of the GPIO pin specified.
 */

void fGpio_setConfig (fGpio_Pin* const Gpio_Pin)
{
#ifdef	DEBUG
	ASSERT_PARAM(ASSERT_PORT(Gpio_Pin->Port));
	ASSERT_PARAM(ASSERT_PIN(Gpio_Pin->Pin));
	ASSERT_PARAM(ASSERT_CURRENT(Gpio_Pin->Current));
	ASSERT_PARAM(ASSERT_TYPE(Gpio_Pin->Type));
#endif

	GPIOPadConfigSet(Gpio_Pin->Port, Gpio_Pin->Pin, Gpio_Pin->Current, Gpio_Pin->Type);
}

/*
 * Function to set the current limit and the pin type of the GPIO pin specified.
 */

void fGpio_setDirection (fGpio_Pin* const Gpio_Pin)
{
#ifdef  DEBUG
    ASSERT_PARAM(ASSERT_PORT(Gpio_Pin->Port));
    ASSERT_PARAM(ASSERT_PIN(Gpio_Pin->Pin));
    ASSERT_PARAM(ASSERT_DIRECTION(Gpio_Pin->Direction));
#endif

    GPIODirModeSet(Gpio_Pin->Port, Gpio_Pin->Pin, Gpio_Pin->Direction);
}

/*
 * Function to turn to level high the pin specified.
 */

void fGpio_setHigh (fGpio_Pin* const Gpio_Pin)
{
#ifdef  DEBUG
    ASSERT_PARAM(ASSERT_PORT(Gpio_Pin->Port));
    ASSERT_PARAM(ASSERT_PIN(Gpio_Pin->Pin));
#endif

    GPIOPinWrite(Gpio_Pin->Port, Gpio_Pin->Pin, Gpio_Pin->Pin);
}

/*
 * Function to turn to level low the pin specified.
 */

void fGpio_setLow (fGpio_Pin* const Gpio_Pin)
{
#ifdef  DEBUG
    ASSERT_PARAM(ASSERT_PORT(Gpio_Pin->Port));
    ASSERT_PARAM(ASSERT_PIN(Gpio_Pin->Pin));
#endif

    GPIOPinWrite(Gpio_Pin->Port, Gpio_Pin->Pin, ~(Gpio_Pin->Pin));
}

/*
 * Function to toggle the pin specified.
 */

void fGpio_setToggle (fGpio_Pin* const Gpio_Pin)
{
#ifdef  DEBUG
    ASSERT_PARAM(ASSERT_PORT(Gpio_Pin->Port));
    ASSERT_PARAM(ASSERT_PIN(Gpio_Pin->Pin));
#endif

    if (fGpio_getLevel(Gpio_Pin))
        GPIOPinWrite(Gpio_Pin->Port, Gpio_Pin->Pin, ~Gpio_Pin->Pin);
    else
        GPIOPinWrite(Gpio_Pin->Port, Gpio_Pin->Pin, Gpio_Pin->Pin);
}

/*
 * Function to read the status of the GPIO pin specified.
 */

bool fGpio_getLevel (fGpio_Pin* const Gpio_Pin)
{
#ifdef  DEBUG
    ASSERT_PARAM(ASSERT_PORT(Gpio_Pin->Port));
    ASSERT_PARAM(ASSERT_PIN(Gpio_Pin->Pin));
#endif

    if (GPIOPinRead(Gpio_Pin->Port, Gpio_Pin->Pin))
        return true;

    return false;
}

/*
 * Completely initializes a pin.
 */

void fGpio_Init (fGpio_Pin* const Gpio_Pin)
{
#ifdef  DEBUG
    ASSERT_PARAM(ASSERT_PORT(Gpio_Pin->Port));
    ASSERT_PARAM(ASSERT_PIN(Gpio_Pin->Pin));
    ASSERT_PARAM(ASSERT_DIRECTION(Gpio_Pin->Direction));
    ASSERT_PARAM(ASSERT_CURRENT(Gpio_Pin->Current));
    ASSERT_PARAM(ASSERT_TYPE(Gpio_Pin->Type));
#endif

    fGpio_enableSysCtl(Gpio_Pin);
    fGpio_unlockPin(Gpio_Pin);
    fGpio_setDirection(Gpio_Pin);
    fGpio_setConfig(Gpio_Pin);
}

/*
 * Configures the internal interrupt on a pin, and enables it by default.
 */

void fGpio_IntInit (fGpio_Pin* const Gpio_Pin)
{
#ifdef  DEBUG
    ASSERT_PARAM(ASSERT_PORT(Gpio_Pin->Port));
    ASSERT_PARAM(ASSERT_PIN(Gpio_Pin->Pin));
    ASSERT_PARAM(ASSERT_INTTYPE(Gpio_Pin->IntType));
#endif

    GPIOIntTypeSet(Gpio_Pin->Port, Gpio_Pin->Pin, Gpio_Pin->IntType);
    GPIOIntRegister(Gpio_Pin->Port, Gpio_Pin->IntIRQ);
    GPIOIntEnable(Gpio_Pin->Port, fGpio_getIntPin(Gpio_Pin));
}

/*
 * Clears the interrupt.
 */

void fGpio_IntClear (fGpio_Pin* const Gpio_Pin)
{
#ifdef  DEBUG
    ASSERT_PARAM(ASSERT_PORT(Gpio_Pin->Port));
    ASSERT_PARAM(ASSERT_PIN(Gpio_Pin->Pin));
#endif


    GPIOIntClear(Gpio_Pin->Port, Gpio_Pin->Pin);
}

/*
 * Returns the status of the masekd external interrupts on the specified port.
 */

uint32_t fGpio_IntGet (fGpio_Pin* const Gpio_Pin)
{
#ifdef  DEBUG
    ASSERT_PARAM(ASSERT_PORT(Gpio_Pin->Port));
#endif

    return GPIOIntStatus(Gpio_Pin->Port, true);
}


