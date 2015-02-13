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

#include "ektm4c123gxl.h"
#include "functions_pins.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define LEDR_PIN     GPIO_PIN_1
#define LEDR_PORT    GPIO_PORTF_BASE
#define LEDB_PIN     GPIO_PIN_2
#define LEDB_PORT    GPIO_PORTF_BASE
#define LEDG_PIN     GPIO_PIN_3
#define LEDG_PORT    GPIO_PORTF_BASE

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/*
 * Initialize the LEDs.
 */

void brd_LedInit (uint8_t LEDx)
{
    fPins_Pin temp = {.Current = CURR_2MA, .Direction = DIR_OUT, .Type = TYPE_PP_PU};

    if (LEDx & LEDR)
    {
        temp.Pin = LEDR_PIN, temp.Port = LEDR_PORT;
        fPins_InitGpioPin(&temp);
    }
    if (LEDx & LEDG)
    {
        temp.Pin = LEDG_PIN, temp.Port = LEDG_PORT;
        fPins_InitGpioPin(&temp);
    }
    if (LEDx & LEDB)
    {
        temp.Pin = LEDB_PIN, temp.Port = LEDB_PORT;
        fPins_InitGpioPin(&temp);
    }
}

/*
 * Initialize the LEDs.
 */

void brd_LedInteract (uint8_t LEDx, uint8_t interact)
{
    fPins_Pin temp;
    void (*fnPtr) (fPins_Pin* const);

    switch (interact)
    {
    case LED_ON:
        fnPtr = fPins_setGpioHigh;
        break;
    case LED_OFF:
        fnPtr = fPins_setGpioLow;
        break;
    case LED_TOGGLE:
        fnPtr = fPins_setGpioToggle;
        break;
    }

    if (LEDx & LEDR)
    {
        temp.Pin = LEDR_PIN, temp.Port = LEDR_PORT;
        fnPtr(&temp);
    }
    if (LEDx & LEDG)
    {
        temp.Pin = LEDG_PIN, temp.Port = LEDG_PORT;
        fnPtr(&temp);
    }
    if (LEDx & LEDB)
    {
        temp.Pin = LEDB_PIN, temp.Port = LEDB_PORT;
        fnPtr(&temp);
    }
}
