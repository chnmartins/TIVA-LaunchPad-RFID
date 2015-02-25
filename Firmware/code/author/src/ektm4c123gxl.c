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
#include <stdlib.h>
#include <stddef.h>

#include "ektm4c123gxl.h"
#include "functions_gpio.h"
#include "conf.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define EKTM4C123GXL_LEDR_PIN     FGPIO_PIN_1
#define EKTM4C123GXL_LEDR_PORT    FGPIO_PORT_F
#define EKTM4C123GXL_LEDB_PIN     FGPIO_PIN_2
#define EKTM4C123GXL_LEDB_PORT    FGPIO_PORT_F
#define EKTM4C123GXL_LEDG_PIN     FGPIO_PIN_3
#define EKTM4C123GXL_LEDG_PORT    FGPIO_PORT_F

#define EKTM4C123GXL_PB1_PIN      FGPIO_PIN_4
#define EKTM4C123GXL_PB1_PORT     FGPIO_PORT_F
#define EKTM4C123GXL_PB2_PIN      FGPIO_PIN_0
#define EKTM4C123GXL_PB2_PORT     FGPIO_PORT_F

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
fGpio_Class* GpioClass;
/* Private function prototypes -----------------------------------------------*/
static void ektm4c123gxl_LED_Init (uint8_t LEDx);
static void ektm4c123gxl_LED_On (uint8_t LEDx);
static void ektm4c123gxl_LED_Off (uint8_t LEDx);
static void ektm4c123gxl_LED_Toggle (uint8_t LEDx);

static void ektm4c123gxl_PB_Init (uint8_t PBx);
static uint8_t ektm4c123gxl_PB_Read (uint8_t EKTM4C123GXL_PBx);
static void ektm4c123gxl_PB_IntInit (uint8_t PBx, void (*PB_INTIRQ) (void));
static uint8_t ektm4c123gxl_PB_IntTest (uint8_t EKTM4C123GXL_PBx);
static void ektm4c123gxl_PB_IntClear (uint8_t EKTM4C123GXL_PBx);
static void ektm4c123gxl_PB_IntStatus (uint8_t EKTM4C123GXL_PBx, uint8_t EKTM4C123GXL_STATUSx);

/* Private functions ---------------------------------------------------------*/

/*
 *
 */

ektm4c123gxl_Class* ektm4c123gxl_CreateClass (void)
{
    ektm4c123gxl_Class* temp = malloc(sizeof(ektm4c123gxl_Class));
    if (temp == NULL)
        return NULL;

    GpioClass = fGpio_CreateClass();
    if (GpioClass == NULL)
        return NULL;

    temp->LED_Init = ektm4c123gxl_LED_Init;
    temp->LED_Off = ektm4c123gxl_LED_Off;
    temp->LED_On = ektm4c123gxl_LED_On;
    temp->LED_Toggle = ektm4c123gxl_LED_Toggle;

    temp->PB_Init = ektm4c123gxl_PB_Init;
    temp->PB_Read = ektm4c123gxl_PB_Read;
    temp->PB_IntInit = ektm4c123gxl_PB_IntInit;
    temp->PB_IntStatus = ektm4c123gxl_PB_IntStatus;
    temp->PB_IntClear = ektm4c123gxl_PB_IntClear;
    temp->PB_IntTest = ektm4c123gxl_PB_IntTest;

    return temp;
}

/*
 *
 */

void ektm4c123gxl_DestroyClass (ektm4c123gxl_Class* class)
{
    free(class);
}

/*
 * Wrapper functions LEDs.
 */

static void ektm4c123gxl_LED_Init (uint8_t EKTM4C123GXL_LEDx)
{
    if (EKTM4C123GXL_LEDx & EKTM4C123GXL_LEDR)
        GpioClass->InitOutput(EKTM4C123GXL_LEDR_PIN, EKTM4C123GXL_LEDR_PORT, FGPIO_TYPE_PUSH_PULL_PULLDOWN, FGPIO_CURRENT_2MA);
    if (EKTM4C123GXL_LEDx & EKTM4C123GXL_LEDB)
        GpioClass->InitOutput(EKTM4C123GXL_LEDB_PIN, EKTM4C123GXL_LEDB_PORT, FGPIO_TYPE_PUSH_PULL_PULLDOWN, FGPIO_CURRENT_2MA);
    if (EKTM4C123GXL_LEDx & EKTM4C123GXL_LEDG)
        GpioClass->InitOutput(EKTM4C123GXL_LEDG_PIN, EKTM4C123GXL_LEDG_PORT, FGPIO_TYPE_PUSH_PULL_PULLDOWN, FGPIO_CURRENT_2MA);
}

/*
 * Wrapper functions LEDs.
 */

static void ektm4c123gxl_LED_On (uint8_t EKTM4C123GXL_LEDx)
{
    if (EKTM4C123GXL_LEDx & EKTM4C123GXL_LEDR)
        GpioClass->OutputInteract(EKTM4C123GXL_LEDR_PIN, EKTM4C123GXL_LEDR_PORT, FGPIO_OUTPUT_HIGH);
    if (EKTM4C123GXL_LEDx & EKTM4C123GXL_LEDB)
        GpioClass->OutputInteract(EKTM4C123GXL_LEDB_PIN, EKTM4C123GXL_LEDB_PORT, FGPIO_OUTPUT_HIGH);
    if (EKTM4C123GXL_LEDx & EKTM4C123GXL_LEDG)
        GpioClass->OutputInteract(EKTM4C123GXL_LEDG_PIN, EKTM4C123GXL_LEDG_PORT, FGPIO_OUTPUT_HIGH);
}

/*
 * Wrapper functions LEDs.
 */

static void ektm4c123gxl_LED_Off (uint8_t EKTM4C123GXL_LEDx)
{
    if (EKTM4C123GXL_LEDx & EKTM4C123GXL_LEDR)
        GpioClass->OutputInteract(EKTM4C123GXL_LEDR_PIN, EKTM4C123GXL_LEDR_PORT, FGPIO_OUTPUT_LOW);
    if (EKTM4C123GXL_LEDx & EKTM4C123GXL_LEDB)
        GpioClass->OutputInteract(EKTM4C123GXL_LEDB_PIN, EKTM4C123GXL_LEDB_PORT, FGPIO_OUTPUT_LOW);
    if (EKTM4C123GXL_LEDx & EKTM4C123GXL_LEDG)
        GpioClass->OutputInteract(EKTM4C123GXL_LEDG_PIN, EKTM4C123GXL_LEDG_PORT, FGPIO_OUTPUT_LOW);
}

/*
 * Wrapper functions LEDs.
 */

static void ektm4c123gxl_LED_Toggle (uint8_t EKTM4C123GXL_LEDx)
{
    if (EKTM4C123GXL_LEDx & EKTM4C123GXL_LEDR)
        GpioClass->OutputInteract(EKTM4C123GXL_LEDR_PIN, EKTM4C123GXL_LEDR_PORT, FGPIO_OUTPUT_TOGGLE);
    if (EKTM4C123GXL_LEDx & EKTM4C123GXL_LEDB)
        GpioClass->OutputInteract(EKTM4C123GXL_LEDB_PIN, EKTM4C123GXL_LEDB_PORT, FGPIO_OUTPUT_TOGGLE);
    if (EKTM4C123GXL_LEDx & EKTM4C123GXL_LEDG)
        GpioClass->OutputInteract(EKTM4C123GXL_LEDG_PIN, EKTM4C123GXL_LEDG_PORT, FGPIO_OUTPUT_TOGGLE);
}

/*
 * Wrapper functions PBs.
 */

static void ektm4c123gxl_PB_Init (uint8_t EKTM4C123GXL_PBx)
{
    if (EKTM4C123GXL_PBx & EKTM4C123GXL_PB1)
        GpioClass->InitInput(EKTM4C123GXL_PB1_PIN, EKTM4C123GXL_PB1_PORT, FGPIO_TYPE_PUSH_PULL_PULLUP, FGPIO_CURRENT_2MA);
    if (EKTM4C123GXL_PBx & EKTM4C123GXL_PB2)
        GpioClass->InitInput(EKTM4C123GXL_PB2_PIN, EKTM4C123GXL_PB2_PORT, FGPIO_TYPE_PUSH_PULL_PULLUP, FGPIO_CURRENT_2MA);
}

/*
 * Wrapper functions PBs.
 */

static uint8_t ektm4c123gxl_PB_Read (uint8_t EKTM4C123GXL_PBx)
{
    if (EKTM4C123GXL_PBx & EKTM4C123GXL_PB1)
    {
        if (GpioClass->InputRead(EKTM4C123GXL_PB1_PIN, EKTM4C123GXL_PB1_PORT) == FGPIO_INPUT_HIGH)
            return EKTM4C123GXL_STATUS_OFF;
    }
    else if (EKTM4C123GXL_PBx & EKTM4C123GXL_PB2)
    {
        if (GpioClass->InputRead(EKTM4C123GXL_PB2_PIN, EKTM4C123GXL_PB2_PORT) == FGPIO_INPUT_HIGH)
            return EKTM4C123GXL_STATUS_OFF;
    }

    return EKTM4C123GXL_STATUS_ON;
}

/*
 * Wrapper functions PBs.
 */

static void ektm4c123gxl_PB_IntInit (uint8_t EKTM4C123GXL_PBx, void (*EKTM4C123GXL_PB_INTIRQ) (void))
{
    if (EKTM4C123GXL_PBx & EKTM4C123GXL_PB1)
        GpioClass->InitInputInt(EKTM4C123GXL_PB1_PIN, EKTM4C123GXL_PB1_PORT, FGPIO_TYPE_PUSH_PULL_PULLUP, FGPIO_CURRENT_2MA, FGPIO_INTTYPE_FALLING_EDGE, EKTM4C123GXL_PB_INTIRQ);
    if (EKTM4C123GXL_PBx & EKTM4C123GXL_PB2)
        GpioClass->InitInputInt(EKTM4C123GXL_PB2_PIN, EKTM4C123GXL_PB2_PORT, FGPIO_TYPE_PUSH_PULL_PULLUP, FGPIO_CURRENT_2MA, FGPIO_INTTYPE_FALLING_EDGE, EKTM4C123GXL_PB_INTIRQ);
}

/*
 * Wrapper functions PBs.
 */

static uint8_t ektm4c123gxl_PB_IntTest (uint8_t EKTM4C123GXL_PBx)
{
    if (EKTM4C123GXL_PBx & EKTM4C123GXL_PB1)
    {
        if (GpioClass->IntTest(EKTM4C123GXL_PB1_PIN, EKTM4C123GXL_PB1_PORT) == FGPIO_INT_ON)
            return EKTM4C123GXL_STATUS_ON;
    }
    else if (EKTM4C123GXL_PBx & EKTM4C123GXL_PB2)
    {
        if (GpioClass->IntTest(EKTM4C123GXL_PB2_PIN, EKTM4C123GXL_PB2_PORT) == FGPIO_INT_ON)
            return EKTM4C123GXL_STATUS_ON;
    }

    return EKTM4C123GXL_STATUS_OFF;
}

/*
 * Wrapper functions PBs.
 */

static void ektm4c123gxl_PB_IntClear (uint8_t EKTM4C123GXL_PBx)
{
    if (EKTM4C123GXL_PBx & EKTM4C123GXL_PB1)
        GpioClass->IntClear(EKTM4C123GXL_PB1_PIN, EKTM4C123GXL_PB1_PORT);
    if (EKTM4C123GXL_PBx & EKTM4C123GXL_PB2)
        GpioClass->IntClear(EKTM4C123GXL_PB2_PIN, EKTM4C123GXL_PB2_PORT);
}

/*
 * Wrapper functions PBs.
 */

static void ektm4c123gxl_PB_IntStatus (uint8_t EKTM4C123GXL_PBx, uint8_t EKTM4C123GXL_STATUSx)
{
    if (EKTM4C123GXL_PBx & EKTM4C123GXL_PB1)
        (EKTM4C123GXL_STATUSx == EKTM4C123GXL_STATUS_ON) ?  GpioClass->IntStatus(EKTM4C123GXL_PB1_PIN, EKTM4C123GXL_PB1_PORT, FGPIO_INT_ON) :  GpioClass->IntStatus(EKTM4C123GXL_PB1_PIN, EKTM4C123GXL_PB1_PORT, FGPIO_INT_OFF);
    if (EKTM4C123GXL_PBx & EKTM4C123GXL_PB2)
        (EKTM4C123GXL_STATUSx == EKTM4C123GXL_STATUS_ON) ?  GpioClass->IntStatus(EKTM4C123GXL_PB2_PIN, EKTM4C123GXL_PB2_PORT, FGPIO_INT_ON) :  GpioClass->IntStatus(EKTM4C123GXL_PB2_PIN, EKTM4C123GXL_PB2_PORT, FGPIO_INT_OFF);
}
