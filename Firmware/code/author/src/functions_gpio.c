/**
  ******************************************************************************
  * @file    functions_gpio.c
  * @author  Diego Martínez García <dmartinezgarcia@outlook.com>
  * @date    08-March-2015
  * @brief   GPIO functions for TM4C microcontrollers using Tivaware.
  *
  * @details 
  * This library is written in C and is class-oriented, the structure fGpio_Class contains pointers to functions and acts like a class. You can use the following functions to create and destroy a class:
  *		
  *	<b>fGpio_CreateClass</b>: Creates and initializes the pointers to function. The class is allocated using dynamic memory allocation, and this function returns a pointer to that class from where you can access the GPIO functions.<br>
  * <b>fGpio_DestroyClass</b>: Destroys a previously allocated class.<br>
  *			
  *	Once you create a class, you can access the following initialization functions:
  *
  *	<b>InitInput</b>: Initializes the pin as an input.<br>
  *	<b>InitOutput</b>: Initializes the pin as an output.<br>
  *	<b>InitInputInt</b>: Initializes the pin as an input, and enables external interrupts on that pin.<br>
  *	<b>InitAlternateFunction</b>: Initializes the pin to be managed by other hardware peripheral other than GPIO pin.<br>
  *
  *	You can interact with the pin using the following functions:
  *
  *	<b>OutputInteract</b>: Interacts with an output pin. Current actions supported are toggle, set and reset.<br>
  *	<b>InputRead</b>: Reads the value of the specified pin. Works with output pins too.<br>
  *
  *	You can manage the external interrupts with the following functions:
  *
  *	<b>IntClear</b>: Clears the specified interrupt bit.<br>
  *	<b>IntStatus</b>: Disables or enables the external interrupt.<br>
  *	<b>IntTest</b>: Checks if the interrupt bit is enabled.<br>
  *
  *	TODO: The library is not yet complete, there should be Deinitialization functions.
  *      
  ******************************************************************************
  * @copyright
  *		
  *	functions_gpio.c, GPIO functions for TM4C microcontrollers. Copyright (C) 2015  Diego Martínez García @ dmartinezgarcia@outlook.com.
  *
  *	This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
  *
  *	This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
  *  
  * You should have received a copy of the GNU General Public License along with this program. If not, see <http://www.gnu.org/licenses/>.
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
#define	FGPIO_GPIO_UNLOCK_VALUE	            (0x4C4F434B)					/*!< Unlock value defined in the datasheet. */

/* Private macro -------------------------------------------------------------*/

/*!	Checks whether the specified port supplied as an argument is valid or not. */
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
											 
/*!	Checks whether the specified port supplied as an argument is valid or not. */
#define	FGPIO_ASSERT_PIN(x)			        ((x) == FGPIO_PIN_0 || \
									         (x) == FGPIO_PIN_1 || \
									         (x) == FGPIO_PIN_2 || \
									         (x) == FGPIO_PIN_3 || \
									         (x) == FGPIO_PIN_4 || \
									         (x) == FGPIO_PIN_5 || \
									         (x) == FGPIO_PIN_6 || \
									         (x) == FGPIO_PIN_7)

/*!	Checks whether the specified port supplied as an argument is valid or not. */									 
#define FGPIO_ASSERT_DIRECTION(x)           ((x) == FGPIO_DIR_IN    || \
                                             (x) == FGPIO_DIR_OUT   || \
                                             (x) == FGPIO_DIR_HW)
											 
/*!	Checks whether the specified port supplied as an argument is valid or not. */
#define FGPIO_ASSERT_CURRENT(x)             ((x) == FGPIO_CURRENT_2MA      || \
                                             (x) == FGPIO_CURRENT_4MA      || \
                                             (x) == FGPIO_CURRENT_6MA      || \
                                             (x) == FGPIO_CURRENT_8MA      || \
                                             (x) == FGPIO_CURRENT_8MA_SC   || \
                                             (x) == FGPIO_CURRENT_10MA     || \
                                             (x) == FGPIO_CURRENT_12MA)
											 
/*!	Checks whether the specified port supplied as an argument is valid or not. */
#define	FGPIO_ASSERT_TYPE(x)			    ((x) == FGPIO_TYPE_PUSH_PULL            || \
									         (x) == FGPIO_TYPE_PUSH_PULL_PULLUP     || \
									         (x) == FGPIO_TYPE_PUSH_PULL_PULLDOWN   || \
									         (x) == FGPIO_TYPE_OPEN_DRAIN           || \
									         (x) == FGPIO_TYPE_ANALOG               || \
									         (x) == FGPIO_TYPE_WAKE_HIGH            || \
									         (x) == FGPIO_TYPE_WAKE_LOW)

/*!	Checks whether the specified port supplied as an argument is valid or not. */											 
#define FGPIO_ASSERT_INTTYPE(x)             ((x) == FGPIO_INTTYPE_FALLING_EDGE  || \
                                             (x) == FGPIO_INTTYPE_RISING_EDGE   || \
                                             (x) == FGPIO_INTTYPE_BOTH_EDGES    || \
                                             (x) == FGPIO_INTTYPE_LOW_LEVEL     || \
                                             (x) == FGPIO_INTTYPE_HIGH_LEVEL    || \
                                             (x) == FGPIO_INTTYPE_DISCRETE_INT)

/*!	Checks whether the specified port supplied as an argument is valid or not. */										 
#define FGPIO_ASSERT_OUTPUT(x)              ((x) == FGPIO_OUTPUT_LOW    || \
                                             (x) == FGPIO_OUTPUT_HIGH   || \
                                             (x) == FGPIO_OUTPUT_TOGGLE)

/*!	Checks whether the specified port supplied as an argument is valid or not. */										 
#define FGPIO_ASSERT_INT(x)                 ((x) == FGPIO_INT_ON    || \
                                             (x) == FGPIO_INT_OFF)

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
static void fGpio_SysCtlStatus (uint32_t FGPIO_PORTx, bool status);
static uint32_t fGpio_IntGetPin (uint32_t FGPIO_PINx);
static void fGpio_UnlockPin (uint32_t FGPIO_PINx, uint32_t FGPIO_PORTx);
static void fGpio_Config (uint32_t FGPIO_PINx, uint32_t FGPIO_PORTx, uint32_t FGPIO_CURRENTx, uint32_t FGPIO_TYPEx, uint32_t GPIO_DIR_MODEx);
static void fGpio_IntConfig (uint32_t FGPIO_PINx, uint32_t FGPIO_PORTx, uint32_t FGPIO_INTTYPEx, void (*FGPIO_INTIRQ) (void));
static void fGpio_OutputInteract (uint32_t FGPIO_PINx, uint32_t FGPIO_PORTx, uint8_t FGPIO_OUTPUTx);
static uint8_t fGpio_InputRead(uint32_t FGPIO_PINx, uint32_t FGPIO_PORTx);
static void fGpio_InitInput (uint32_t FGPIO_PINx, uint32_t FGPIO_PORTx, uint32_t FGPIO_TYPEx, uint32_t FGPIO_CURRENTx);
static void fGpio_InitInputInt (uint32_t FGPIO_PINx, uint32_t FGPIO_PORTx, uint32_t FGPIO_TYPEx, uint32_t FGPIO_CURRENTx, uint32_t FGPIO_INTTYPEx, void (*FGPIO_INTIRQ) (void));
static void fGpio_InitOutput(uint32_t FGPIO_PINx, uint32_t FGPIO_PORTx, uint32_t FGPIO_TYPEx, uint32_t FGPIO_CURRENTx);
static void fGpio_InitAlternateFunction(uint32_t FGPIO_PINx, uint32_t FGPIO_PORTx, uint32_t FGPIO_TYPEx, uint32_t FGPIO_CURRENTx, uint32_t GPIO_AF);
static void fGpio_IntStatus (uint32_t FGPIO_PINx, uint32_t FGPIO_PORTx, uint8_t FGPIO_INTx);
static void fGpio_IntClear (uint32_t FGPIO_PINx, uint32_t FGPIO_PORTx);
static uint8_t fGpio_IntTest (uint32_t FGPIO_PINx, uint32_t FGPIO_PORTx);

/* Private functions ---------------------------------------------------------*/

/********************************************//**
 *  @brief 				Creates a class which allows to manage the GPIO peripheral.
 *  
 *  @return 			Pointer to the created class if succesfull, NULL if there is insufficient memory.
 *  
 *  @details 			The return value should be checked to verify that the class was created and NULL was not returned.
 ***********************************************/
fGpio_Class* fGpio_CreateClass (void)
{
    fGpio_Class* temp = malloc(sizeof(fGpio_Class));
    if (temp == NULL)
        return NULL;

    temp->InitAlternateFunction = fGpio_InitAlternateFunction;
    temp->InitInput = fGpio_InitInput;
    temp->InitInputInt = fGpio_InitInputInt;
    temp->InitOutput = fGpio_InitOutput;
    temp->OutputInteract = fGpio_OutputInteract;
    temp->InputRead = fGpio_InputRead;
    temp->IntClear = fGpio_IntClear;
    temp->IntStatus = fGpio_IntStatus;
    temp->IntTest = fGpio_IntTest;

    return temp;
}

/********************************************//**
 *  @brief 				Destroys a previously created class.
 *  
 *  @param [in] class	Pointer to a previously created class using the fGpio_CreateClass function.
 *  @return 			No return value.
 *  
 *  @details 			The function should be used only with a pointer to a previously created class as an argument, otherwise the behaviour of free is undefined and the heap might end up corrupted.
 ***********************************************/
void fGpio_DestroyClass (fGpio_Class* class)
{
    free(class);
}

/********************************************//**
 *  @brief 					Enables or disables the specified GPIO port.
 *  
 *  @param [in] FGPIO_PORTx GPIO port that will be enabled or disabled, use any of the FGPIO_PORTx macros.
 *  @param [in] status      If true, enables the specified GPIO port, if false, disables the specified GPIO port.
 *  @return 				No return value.
 *  
 *  @details 				No details.
 ***********************************************/
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

/********************************************//**
 *  @brief 					Returns the interrupt pin macro for the specified GPIO pin.
 *  
 *  @param [in] FGPIO_PINx 	GPIO pin whose associated interrupt macro will be returned, use any of the FGPIO_PINx macros.
 *  @return 				The GPIO_INT_PINx macro associated with the GPIO pin specified.
 *  
 *  @details 				No details.
 ***********************************************/
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

/********************************************//**
 *  @brief 					Unlocks the specified pin to allow modifying its settings.
 *  
 *  @param [in] FGPIO_PINx  Pin that will be unlocked, any of the FGPIO_PINx macros.
 *  @param [in] FGPIO_PORTx Port which contains the specified pin to unlock, any of the FGPIO_PORTx macros.
 *  @return 				No return value.
 *  
 *  @details 				A pin must be unlocked to modify its configuration, modifying the configuration of a locked pin will have no effect. Most of the pins are unlocked by default but some aren't.
 ***********************************************/
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

/********************************************//**
 *  @brief 						Applies the specified configuration to the specified pin.
 *  
 *  @param [in] FGPIO_PINx     	Pin to apply the specified configuration, use any of the FGPIO_PINx macros.
 *  @param [in] FGPIO_PORTx    	Port which contains the pin, use any of the FGPIO_PORTx macros.
 *  @param [in] FGPIO_CURRENTx 	Current that will go through the specified pin, use any of the FGPIO_CURRENTx macros.
 *  @param [in] FGPIO_TYPEx    	Type of the specified pin (analog, push pull, open drain...), use any of the FGPIO_TYPEx macros.
 *  @param [in] GPIO_DIR_MODEx 	Establishes whether the pin will be managed by hardware or by the GPIO peripheral as input or output, use any of the FGPIO_DIR_MODEx macros.
 *  @return 					No return value.
 *  
 *  @details 					The specified pin must not be locked for this function to apply the settings properly.
 ***********************************************/
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

/********************************************//**
 *  @brief 						Configures the external interrupt peripheral for the specified pin.
 *  
 *  @param [in] FGPIO_PINx     	Pin whose external interrupt will be enabled, use any of the FGPIO_PINx macros.
 *  @param [in] FGPIO_PORTx    	Port which contains the pin, use any of the FGPIO_PORTx macros.
 *  @param [in] FGPIO_INTTYPEx 	Establishes the event which will cause an interrupt, use any of the FGPIO_INTTYPEx macros.
 *  @param [in] FGPIO_INTIRQ   	Sets the Interrupt Service Routine for the specified interrupt.
 *  @return 					No return value.
 *  
 *  @details 					The specified pin must be configured as a digital input. Once the interrupt occurs, the execution flow will jump to the ISR function.
 ***********************************************/
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

/********************************************//**
 *  @brief 					  Function to interact with the specified digital output pin.
 *  
 *  @param [in] FGPIO_PINx    Pin to interact with, use any of the FGPIO_PINx macros.
 *  @param [in] FGPIO_PORTx   Port which contians the pin, use any of the FGPIO_PORTx macros.
 *  @param [in] FGPIO_OUTPUTx Action to perform on the pin, use any of the FGPIO_OUTPUTx macros.
 *  @return 				  No return value.
 *  
 *  @details 				  The pin must have been previously configured as a digital output pin.
 ***********************************************/
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

/********************************************//**
 *  @brief 					Reads the status of the specified pin and returns its value.
 *  
 *  @param [in] FGPIO_PINx  Pin whose status will be read, use any of the FGPIO_PINx macros.
 *  @param [in] FGPIO_PORTx Port which contains the pin, use any of the FGPIO_PORTx macros.
 *  @return 				Depending of the status of the pin it returns FGPIO_INPUT_HIGH or FGPIO_INPUT_LOW.
 *  
 *  @details 				The function works for both input or output digital pins.
 ***********************************************/
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

/********************************************//**
 *  @brief 						Initializes a pin as a digital input.
 *  
 *  @param [in] FGPIO_PINx     	Pin to initialize as a digital input, use any of the FGPIO_PINx macros.
 *  @param [in] FGPIO_PORTx    	Port which contains the pin, use any of the FGPIO_PORTx macros.
 *  @param [in] FGPIO_TYPEx    	Type of the input pin, use any of the FGPIO_TYPEx macros.
 *  @param [in] FGPIO_CURRENTx 	Sets the current that will go through to the pin, use any of the FGPIO_CURRENTx macros.
 *  @return Return_Description	No return value.
 *  
 *  @details 					Automatically makes all the steps to initialize the pin as a digital input.
 ***********************************************/
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

/********************************************//**
 *  @brief 					   Initializes the pin as a digital input with external interrupt.
 *  
 *  @param [in] FGPIO_PINx     Pin to initialize, use any of the FGPIO_PINx macros.
 *  @param [in] FGPIO_PORTx    Port which contains the pins, use any of the FGPIO_PORTx macros.
 *  @param [in] FGPIO_TYPEx    Type of the pin, use any of the FGPIO_TYPEx macros.
 *  @param [in] FGPIO_CURRENTx Current that will go through the pin, use any of the FGPIO_CURRENTx macros.
 *  @param [in] FGPIO_INTTYPEx Event that will cause the interrupt, use any of the FGPIO_INTTYPEx macros.
 *  @param [in] FGPIO_INTIRQ   Interrupt Service Routine that will be called when the specified event on the specified pin happens.
 *  @return 				   No return value.
 *  
 *  @details 				   Automatically makes all the steps to initialize the pin as a digital input with an external interrupt, keep in mind that pins on the same port share the same interrupt service routine.
 ***********************************************/
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
    fGpio_IntConfig(FGPIO_PINx, FGPIO_PORTx, FGPIO_INTTYPEx, FGPIO_INTIRQ);
}

/********************************************//**
 *  @brief 					   	Initialize the pin as a digital output.
 *  
 *  @param [in] FGPIO_PINx     	Pin to initialize, use any of the FGPIO_PINx macros.
 *  @param [in] FGPIO_PORTx    	Port which contains the pin, use any of the FGPIO_PORTx macros.
 *  @param [in] FGPIO_TYPEx    	Type of the pin, use any of the FGPIO_TYPEx macros.
 *  @param [in] FGPIO_CURRENTx 	Current that will go through the pin, use any of the FGPIO_CURRENTx macros.
 *  @return 					No return value.
 *  
 *  @details Details			Automatically initializes the specified pin as a digital output.
 ***********************************************/
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

/********************************************//**
 *  @brief 						Initializes the pin as a pin configured by another hardware peripheral other than the GPIO.
 *  
 *  @param [in] FGPIO_PINx     	Pin to initialize, use any of the FGPIO_PINx macros.
 *  @param [in] FGPIO_PORTx    	Port which contains the pin, use any of the FGPIO_PORTx macros.
 *  @param [in] FGPIO_TYPEx    	Type of the pin, use any of the FGPIO_TYPEx macros.
 *  @param [in] FGPIO_CURRENTx 	Current that will go through the pin, use any of the FGPIO_CURRENTx macros.
 *  @param [in] GPIO_AF        	Alternate function which the pin is associated to, use any of the macros in pin_map.h
 *  @return 					No return value.
 *  
 *  @details 					This functions performs all the steps to initialize a pin to be managed by a hardware peripheral other than the GPIO pin.
 ***********************************************/
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

/********************************************//**
 *  @brief 						Enables or disables the external interrupt on the specified pin.
 *  
 *  @param [in] FGPIO_PINx  	The pin to check, use any of the FGPIO_PINx macros.
 *  @param [in] FGPIO_PORTx 	The port which contains the pin, use any of the FGPIO_PORTx macros.
 *  @param [in] FGPIO_INTx  	Parameter which indicates whether the interrupt should be enabled or disabled, FGPIO_INT_ON or FGPIO_INT_OFF respectively.
 *  @return 					No return value.
 *  
 *  @details 					For this function to work as intended, the pin must have been previously configured as a digital input with external interrupt.
 ***********************************************/
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

/********************************************//**
 *  @brief 						Clears the interrupt bit for the specified pin.
 *  
 *  @param [in] FGPIO_PINx  	Pin whose interrupt bit will be cleared, use any of the FGPIO_PINx macros.
 *  @param [in] FGPIO_PORTx 	Port which contains the pin, use any of the FGPIO_PORTx macros.
 *  @return 					No return value.
 *  
 *  @details 					No details.
 ***********************************************/
static void fGpio_IntClear (uint32_t FGPIO_PINx, uint32_t FGPIO_PORTx)
{
#ifdef  DEBUG
    ASSERT_PARAM(FGPIO_ASSERT_PIN(FGPIO_PINx));
    ASSERT_PARAM(FGPIO_ASSERT_PORT(FGPIO_PORTx));
#endif

    GPIOIntClear(FGPIO_PORTx, FGPIO_PINx);
}

/********************************************//**
 *  @brief 						Checks whether the specified interrupt bit is set or reset.
 *  
 *  @param [in] FGPIO_PINx  	Pin whose interrupt bit will be checked, use any of the FGPIO_PINx macros.
 *  @param [in] FGPIO_PORTx 	Port which contains the specified pin, use any of the FGPIO_PORTx macros.
 *  @return 					Returns FGPIO_INT_ON if the interrupt bit is set and FGPIO_INT_OFF if the interrupt bit is cleared.
 *  
 *  @details 					No details.
 ***********************************************/
static uint8_t fGpio_IntTest (uint32_t FGPIO_PINx, uint32_t FGPIO_PORTx)
{
#ifdef  DEBUG
    ASSERT_PARAM(FGPIO_ASSERT_PIN(FGPIO_PINx));
    ASSERT_PARAM(FGPIO_ASSERT_PORT(FGPIO_PORTx));
#endif

    if (GPIOIntStatus(FGPIO_PORTx, true) & fGpio_IntGetPin(FGPIO_PINx))
        return FGPIO_INT_ON;
    else
        return FGPIO_INT_OFF;
}
