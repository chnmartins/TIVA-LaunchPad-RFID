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
#include <stdlib.h>
#include <stddef.h>

#include "functions_gpio.h"
#include "conf.h"
#include "gpio.h"
#include "hw_gpio.h"
#include "hw_types.h"
#include "hw_memmap.h"
#include "sysctl.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define	FGPIO_GPIO_UNLOCK_VALUE	            (0x4C4F434B)

/* Private macro -------------------------------------------------------------*/
#define	FGPIO_ASSERT_PORT(x)	            ((x) == FGPIO_PORT_A || \
									         (x) == FGPIO_PORT_B || \
									         (x) == FGPIO_PORT_C || \
									         (x) == FGPIO_PORT_D || \
									         (x) == FGPIO_PORT_E || \
									         (x) == FGPIO_PORT_F || \
									         (x) == FGPIO_PORT_G || \
									         (x) == FGPIO_PORT_H || \
									         (x) == FGPIO_PORT_J || \
									         (x) == FGPIO_PORT_K || \
									         (x) == FGPIO_PORT_L || \
									         (x) == FGPIO_PORT_M || \
									         (x) == FGPIO_PORT_N || \
									         (x) == FGPIO_PORT_P || \
									         (x) == FGPIO_PORT_Q || \
									         (x) == FGPIO_PORT_R || \
									         (x) == FGPIO_PORT_S || \
									         (x) == FGPIO_PORT_T)

#define	FGPIO_ASSERT_PIN(x)			        ((x) == FGPIO_PIN_0 || \
									         (x) == FGPIO_PIN_1 || \
									         (x) == FGPIO_PIN_2 || \
									         (x) == FGPIO_PIN_3 || \
									         (x) == FGPIO_PIN_4 || \
									         (x) == FGPIO_PIN_5 || \
									         (x) == FGPIO_PIN_6 || \
									         (x) == FGPIO_PIN_7)

#define FGPIO_ASSERT_DIRECTION(x)           ((x) == FGPIO_DIR_IN    || \
                                             (x) == FGPIO_DIR_OUT   || \
                                             (x) == FGPIO_DIR_HW)

#define FGPIO_ASSERT_CURRENT(x)             ((x) == FGPIO_CURRENT_2MA      || \
                                             (x) == FGPIO_CURRENT_4MA      || \
                                             (x) == FGPIO_CURRENT_6MA      || \
                                             (x) == FGPIO_CURRENT_8MA      || \
                                             (x) == FGPIO_CURRENT_8MA_SC   || \
                                             (x) == FGPIO_CURRENT_10MA     || \
                                             (x) == FGPIO_CURRENT_12MA)

#define	FGPIO_ASSERT_TYPE(x)			    ((x) == FGPIO_TYPE_PUSH_PULL            || \
									         (x) == FGPIO_TYPE_PUSH_PULL_PULLUP     || \
									         (x) == FGPIO_TYPE_PUSH_PULL_PULLDOWN   || \
									         (x) == FGPIO_TYPE_OPEN_DRAIN           || \
									         (x) == FGPIO_TYPE_ANALOG               || \
									         (x) == FGPIO_TYPE_WAKE_HIGH            || \
									         (x) == FGPIO_TYPE_WAKE_LOW)

#define FGPIO_ASSERT_INTTYPE(x)             ((x) == FGPIO_INTTYPE_FALLING_EDGE  || \
                                             (x) == FGPIO_INTTYPE_RISING_EDGE   || \
                                             (x) == FGPIO_INTTYPE_BOTH_EDGES    || \
                                             (x) == FGPIO_INTTYPE_LOW_LEVEL     || \
                                             (x) == FGPIO_INTTYPE_HIGH_LEVEL    || \
                                             (x) == FGPIO_INTTYPE_DISCRETE_INT)

#define FGPIO_ASSERT_OUTPUT(x)              ((x) == FGPIO_OUTPUT_LOW    || \
                                             (x) == FGPIO_OUTPUT_HIGH   || \
                                             (x) == FGPIO_OUTPUT_TOGGLE)

#define FGPIO_ASSERT_INT(x)                 ((x) == FGPIO_INT_ON    || \
                                             (x) == FGPIO_INT_OFF)

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

static void fGpio_InitInput (uint32_t FGPIO_PINx, uint32_t FGPIO_PORTx, uint32_t FGPIO_TYPEx, uint32_t FGPIO_CURRENTx);
static void fGpio_InitInputInt (uint32_t FGPIO_PINx, uint32_t FGPIO_PORTx, uint32_t FGPIO_TYPEx, uint32_t FGPIO_CURRENTx, uint32_t FGPIO_INTTYPEx, void (*FGPIO_INTIRQ) (void));
static void fGpio_InitOutput(uint32_t FGPIO_PINx, uint32_t FGPIO_PORTx, uint32_t FGPIO_TYPEx, uint32_t FGPIO_CURRENTx);
static void fGpio_InitAlternateFunction(uint32_t FGPIO_PINx, uint32_t FGPIO_PORTx, uint32_t FGPIO_TYPEx, uint32_t FGPIO_CURRENTx, uint32_t GPIO_AF);

static uint32_t fGpio_IntGetPin (uint32_t FGPIO_PINx);
static void fGpio_IntConfig (uint32_t FGPIO_PINx, uint32_t FGPIO_PORTx, uint32_t FGPIO_INTTYPEx, void (*FGPIO_INTIRQ) (void));
static void fGpio_IntStatus (uint32_t FGPIO_PINx, uint32_t FGPIO_PORTx, uint8_t FGPIO_INTx);
static uint8_t fGpio_IntTest (uint32_t FGPIO_PINx, uint32_t FGPIO_PORTx);
static void fGpio_IntClear (uint32_t FGPIO_PINx, uint32_t FGPIO_PORTx);
static void fGpio_IntStatus (uint32_t FGPIO_PINx, uint32_t FGPIO_PORTx, uint8_t FGPIO_INTx);

static void fGpio_UnlockPin (uint32_t FGPIO_PINx, uint32_t FGPIO_PORTx);
static void fGpio_Config (uint32_t FGPIO_PINx, uint32_t FGPIO_PORTx, uint32_t FGPIO_CURRENTx, uint32_t FGPIO_TYPEx, uint32_t GPIO_DIR_MODEx);
static void fGpio_SysCtlStatus (uint32_t FGPIO_PORTx, bool status);

static void fGpio_OutputInteract (uint32_t FGPIO_PINx, uint32_t FGPIO_PORTx, uint8_t FGPIO_OUTPUTx);
static uint8_t fGpio_InputRead(uint32_t FGPIO_PINx, uint32_t FGPIO_PORTx);

/* Private functions ---------------------------------------------------------*/

/*
 *
 */

fGpio_Class* fGpio_CreateClass (void)
{
    fGpio_Class* temp = malloc(sizeof(fGpio_Class));
    if (temp == NULL)
        return NULL;

    temp->InitAlternateFunction = fGpio_InitAlternateFunction;
    temp->InitInput = fGpio_InitInput;
    temp->InitInputInt = fGpio_InitInputInt;
    temp->InitOutput = fGpio_InitOutput;

    return temp;
}

/*
 *
 */

void fGpio_DestroyClass (fGpio_Class* class)
{
    free(class);
}

/*
 * Function that enables the SysCtl memory mapped register for the port specified.
 */

static void fGpio_SysCtlStatus (uint32_t FGPIO_PORTx, bool status)
{
#ifdef	DEBUG
	ASSERT_PARAM(FGPIO_ASSERT_PORT(FGPIO_PORTx));
	// TRACE FLOW
#endif

	switch (FGPIO_PORTx)
	{
	case FGPIO_PORT_A:
		(status) ? SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA) : SysCtlPeripheralDisable(SYSCTL_PERIPH_GPIOA);
		break;
	case FGPIO_PORT_B:
        (status) ? SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB) : SysCtlPeripheralDisable(SYSCTL_PERIPH_GPIOB);
        break;
	case FGPIO_PORT_C:
        (status) ? SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC) : SysCtlPeripheralDisable(SYSCTL_PERIPH_GPIOC);
        break;
	case FGPIO_PORT_D:
        (status) ? SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD) : SysCtlPeripheralDisable(SYSCTL_PERIPH_GPIOD);
        break;
	case FGPIO_PORT_E:
        (status) ? SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE) : SysCtlPeripheralDisable(SYSCTL_PERIPH_GPIOE);
        break;
	case FGPIO_PORT_F:
        (status) ? SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF) : SysCtlPeripheralDisable(SYSCTL_PERIPH_GPIOF);
        break;
	case FGPIO_PORT_G:
        (status) ? SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG) : SysCtlPeripheralDisable(SYSCTL_PERIPH_GPIOG);
        break;
	case FGPIO_PORT_H:
        (status) ? SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOH) : SysCtlPeripheralDisable(SYSCTL_PERIPH_GPIOH);
        break;
	case FGPIO_PORT_J:
        (status) ? SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ) : SysCtlPeripheralDisable(SYSCTL_PERIPH_GPIOJ);
        break;
	case FGPIO_PORT_K:
        (status) ? SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK) : SysCtlPeripheralDisable(SYSCTL_PERIPH_GPIOK);
        break;
	case FGPIO_PORT_L:
        (status) ? SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL) : SysCtlPeripheralDisable(SYSCTL_PERIPH_GPIOL);
        break;
	case FGPIO_PORT_M:
        (status) ? SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM) : SysCtlPeripheralDisable(SYSCTL_PERIPH_GPIOM);
        break;
	case FGPIO_PORT_N:
        (status) ? SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION) : SysCtlPeripheralDisable(SYSCTL_PERIPH_GPION);
        break;
	case FGPIO_PORT_P:
        (status) ? SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOP) : SysCtlPeripheralDisable(SYSCTL_PERIPH_GPIOP);
        break;
	case FGPIO_PORT_Q:
        (status) ? SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOQ) : SysCtlPeripheralDisable(SYSCTL_PERIPH_GPIOQ);
        break;
	case FGPIO_PORT_R:
        (status) ? SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOR) : SysCtlPeripheralDisable(SYSCTL_PERIPH_GPIOR);
        break;
	case FGPIO_PORT_S:
        (status) ? SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOS) : SysCtlPeripheralDisable(SYSCTL_PERIPH_GPIOS);
        break;
	case FGPIO_PORT_T:
        (status) ? SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOT) : SysCtlPeripheralDisable(SYSCTL_PERIPH_GPIOT);
        break;
	default:
        break;
	}
}

/*
 * Function that returns the GPIO interrupt pin for the pin specified.
 */

static uint32_t fGpio_IntGetPin (uint32_t FGPIO_PINx)
{
#ifdef	DEBUG
	ASSERT_PARAM(FGPIO_ASSERT_PIN(FGPIO_PINx));
	// TRACE FLOW
#endif

	switch (FGPIO_PINx)
	{
	case FGPIO_PIN_0:
		return GPIO_INT_PIN_0;
	case FGPIO_PIN_1:
		return GPIO_INT_PIN_1;
	case FGPIO_PIN_2:
		return GPIO_INT_PIN_2;
	case FGPIO_PIN_3:
		return GPIO_INT_PIN_3;
	case FGPIO_PIN_4:
		return GPIO_INT_PIN_4;
	case FGPIO_PIN_5:
		return GPIO_INT_PIN_5;
	case FGPIO_PIN_6:
		return GPIO_INT_PIN_6;
	case FGPIO_PIN_7:
		return GPIO_INT_PIN_7;
	default:
	    return 0;
	}
}

/*
 * Function to unlock the specified GPIO pin on the specified port.
 */

static void fGpio_UnlockPin (uint32_t FGPIO_PINx, uint32_t FGPIO_PORTx)
{
#ifdef	DEBUG
    ASSERT_PARAM(FGPIO_ASSERT_PIN(FGPIO_PINx));
	ASSERT_PARAM(FGPIO_ASSERT_PORT(FGPIO_PORTx));
	// Trace flow
#endif

	HWREG(FGPIO_PORTx + GPIO_O_LOCK) = FGPIO_GPIO_UNLOCK_VALUE;
	HWREG(FGPIO_PORTx + GPIO_O_CR) |= FGPIO_PINx;
}

/*
 * Function to set the current limit and the pin type of the GPIO pin specified.
 */

static void fGpio_Config (uint32_t FGPIO_PINx, uint32_t FGPIO_PORTx, uint32_t FGPIO_CURRENTx, uint32_t FGPIO_TYPEx, uint32_t GPIO_DIR_MODEx)
{
#ifdef	DEBUG
    ASSERT_PARAM(FGPIO_ASSERT_PIN(FGPIO_PINx));
	ASSERT_PARAM(FGPIO_ASSERT_PORT(FGPIO_PORTx));
	ASSERT_PARAM(FGPIO_ASSERT_CURRENT(FGPIO_CURRENTx));
	ASSERT_PARAM(FGPIO_ASSERT_TYPE(FGPIO_TYPEx));
	// Trace flow
#endif

	GPIOPadConfigSet(FGPIO_PORTx, FGPIO_PINx, FGPIO_CURRENTx, FGPIO_TYPEx);
	GPIODirModeSet(FGPIO_PORTx, FGPIO_PINx, GPIO_DIR_MODEx);
}

/*
 * Function to set the current limit and the pin type of the GPIO pin specified.
 */

static void fGpio_IntConfig (uint32_t FGPIO_PINx, uint32_t FGPIO_PORTx, uint32_t FGPIO_INTTYPEx, void (*FGPIO_INTIRQ) (void))
{
#ifdef  DEBUG
    ASSERT_PARAM(FGPIO_ASSERT_PIN(FGPIO_PINx));
    ASSERT_PARAM(FGPIO_ASSERT_PORT(FGPIO_PORTx));
    ASSERT_PARAM(FGPIO_ASSERT_INTTYPE(FGPIO_INTTYPEx));
    // Trace flow
#endif

    GPIOIntTypeSet(FGPIO_PORTx, FGPIO_PINx, FGPIO_INTTYPEx);
    GPIOIntRegister(FGPIO_PORTx, FGPIO_INTIRQ);
}

/*
 * Function to turn to level high the pin specified.
 */

static void fGpio_OutputInteract (uint32_t FGPIO_PINx, uint32_t FGPIO_PORTx, uint8_t FGPIO_OUTPUTx)
{
#ifdef  DEBUG
    ASSERT_PARAM(FGPIO_ASSERT_PIN(FGPIO_PINx));
    ASSERT_PARAM(FGPIO_ASSERT_PORT(FGPIO_PORTx));
    ASSERT_PARAM(FGPIO_ASSERT_OUTPUT(FGPIO_OUTPUTx));
#endif


    switch (FGPIO_OUTPUTx)
    {
    case FGPIO_OUTPUT_HIGH:
        GPIOPinWrite(FGPIO_PORTx, FGPIO_PINx, FGPIO_PINx);
        break;
    case FGPIO_OUTPUT_LOW:
        GPIOPinWrite(FGPIO_PORTx, FGPIO_PINx, ~FGPIO_PINx);
        break;
    case FGPIO_OUTPUT_TOGGLE:
        if (fGpio_InputRead(FGPIO_PINx, FGPIO_PORTx) == FGPIO_INPUT_HIGH)
            GPIOPinWrite(FGPIO_PORTx, FGPIO_PINx, ~FGPIO_PINx);
        else
            GPIOPinWrite(FGPIO_PORTx, FGPIO_PINx, FGPIO_PINx);
        break;
    }
}

/*
 * Function to read the status of the GPIO pin specified.
 */

static uint8_t fGpio_InputRead(uint32_t FGPIO_PINx, uint32_t FGPIO_PORTx)
{
#ifdef  DEBUG
    ASSERT_PARAM(FGPIO_ASSERT_PIN(FGPIO_PINx));
    ASSERT_PARAM(FGPIO_ASSERT_PORT(FGPIO_PORTx));
#endif

    if (GPIOPinRead(FGPIO_PORTx, FGPIO_PINx))
        return FGPIO_INPUT_HIGH;
    else
        return FGPIO_INPUT_LOW;
}

/*
 * Initializes an input.
 */

static void fGpio_InitInput (uint32_t FGPIO_PINx, uint32_t FGPIO_PORTx, uint32_t FGPIO_TYPEx, uint32_t FGPIO_CURRENTx)
{
#ifdef  DEBUG
    ASSERT_PARAM(FGPIO_ASSERT_PIN(FGPIO_PINx));
    ASSERT_PARAM(FGPIO_ASSERT_PORT(FGPIO_PORTx));
    ASSERT_PARAM(FGPIO_ASSERT_TYPE(FGPIO_TYPEx));
    ASSERT_PARAM(FGPIO_ASSERT_CURRENT(FGPIO_CURRENTx));
#endif

    fGpio_SysCtlStatus(FGPIO_PORTx, true);
    fGpio_UnlockPin(FGPIO_PINx, FGPIO_PORTx);
    fGpio_Config(FGPIO_PINx, FGPIO_PORTx, FGPIO_CURRENTx, FGPIO_TYPEx, GPIO_DIR_MODE_IN);
}

/*
 * Initializes an input with external interrupts capabilities.
 */

static void fGpio_InitInputInt (uint32_t FGPIO_PINx, uint32_t FGPIO_PORTx, uint32_t FGPIO_TYPEx, uint32_t FGPIO_CURRENTx, uint32_t FGPIO_INTTYPEx, void (*FGPIO_INTIRQ) (void))
{
#ifdef  DEBUG
    ASSERT_PARAM(FGPIO_ASSERT_PIN(FGPIO_PINx));
    ASSERT_PARAM(FGPIO_ASSERT_PORT(FGPIO_PORTx));
    ASSERT_PARAM(FGPIO_ASSERT_TYPE(FGPIO_TYPEx));
    ASSERT_PARAM(FGPIO_ASSERT_CURRENT(FGPIO_CURRENTx));
#endif

    fGpio_SysCtlStatus(FGPIO_PORTx, true);
    fGpio_UnlockPin(FGPIO_PINx, FGPIO_PORTx);
    fGpio_Config(FGPIO_PINx, FGPIO_PORTx, FGPIO_CURRENTx, FGPIO_TYPEx, GPIO_DIR_MODE_IN);
    fGpio_ConfigInt(FGPIO_PINx, FGPIO_PORTx, FGPIO_INTTYPEx, FGPIO_INTIRQ);
}

/*
 * Initializes an input with external interrupts capabilities.
 */

static void fGpio_InitOutput(uint32_t FGPIO_PINx, uint32_t FGPIO_PORTx, uint32_t FGPIO_TYPEx, uint32_t FGPIO_CURRENTx)
{
#ifdef  DEBUG
    ASSERT_PARAM(FGPIO_ASSERT_PIN(FGPIO_PINx));
    ASSERT_PARAM(FGPIO_ASSERT_PORT(FGPIO_PORTx));
    ASSERT_PARAM(FGPIO_ASSERT_TYPE(FGPIO_TYPEx));
    ASSERT_PARAM(FGPIO_ASSERT_CURRENT(FGPIO_CURRENTx));
#endif

    fGpio_SysCtlStatus(FGPIO_PORTx, true);
    fGpio_UnlockPin(FGPIO_PINx, FGPIO_PORTx);
    fGpio_Config(FGPIO_PINx, FGPIO_PORTx, FGPIO_CURRENTx, FGPIO_TYPEx, GPIO_DIR_MODE_OUT);
}

/*
 * Initializes an input with external interrupts capabilities.
 */

static void fGpio_InitAlternateFunction(uint32_t FGPIO_PINx, uint32_t FGPIO_PORTx, uint32_t FGPIO_TYPEx, uint32_t FGPIO_CURRENTx, uint32_t GPIO_AF)
{
#ifdef  DEBUG
    ASSERT_PARAM(FGPIO_ASSERT_PIN(FGPIO_PINx));
    ASSERT_PARAM(FGPIO_ASSERT_PORT(FGPIO_PORTx));
    ASSERT_PARAM(FGPIO_ASSERT_TYPE(FGPIO_TYPEx));
    ASSERT_PARAM(FGPIO_ASSERT_CURRENT(FGPIO_CURRENTx));
#endif

    fGpio_SysCtlStatus(FGPIO_PORTx, true);
    fGpio_UnlockPin(FGPIO_PINx, FGPIO_PORTx);
    fGpio_Config(FGPIO_PINx, FGPIO_PORTx, FGPIO_CURRENTx, FGPIO_TYPEx, GPIO_DIR_MODE_HW);
    GPIOPinConfigure(GPIO_AF);
}

/*
 * Configures the internal interrupt on a pin, and enables it by default.
 */

static void fGpio_IntStatus (uint32_t FGPIO_PINx, uint32_t FGPIO_PORTx, uint8_t FGPIO_INTx)
{
#ifdef  DEBUG
    ASSERT_PARAM(FGPIO_ASSERT_PIN(FGPIO_PINx));
    ASSERT_PARAM(FGPIO_ASSERT_PORT(FGPIO_PORTx));
    ASSERT_PARAM(FGPIO_ASSERT_INT(FGPIO_INTx));
#endif

    if (FGPIO_INTx == FGPIO_INT_ON)
        GPIOIntEnable(FGPIO_PORTx, fGpio_IntGetPin(FGPIO_PINx));
    else
        GPIOIntDisable(FGPIO_PORTx, fGpio_IntGetPin(FGPIO_PINx));
}

/*
 * Clears the interrupt.
 */

static void fGpio_IntClear (uint32_t FGPIO_PINx, uint32_t FGPIO_PORTx)
{
#ifdef  DEBUG
    ASSERT_PARAM(FGPIO_ASSERT_PIN(FGPIO_PINx));
    ASSERT_PARAM(FGPIO_ASSERT_PORT(FGPIO_PORTx));
#endif

    GPIOIntClear(FGPIO_PORTx, FGPIO_PINx);
}

/*
 * Returns the status of the masekd external interrupts on the specified port.
 */

static uint8_t fGpio_IntTest (uint32_t FGPIO_PINx, uint32_t FGPIO_PORTx)
{
#ifdef  DEBUG
    ASSERT_PARAM(FGPIO_ASSERT_PIN(FGPIO_PINx));
    ASSERT_PARAM(FGPIO_ASSERT_PORT(FGPIO_PORTx));
#endif

    if (GPIOIntStatus(FGPIO_PORTx, true) & FGPIO_PINx)
        return FGPIO_INT_ON;
    else
        return FGPIO_INT_OFF;
}
