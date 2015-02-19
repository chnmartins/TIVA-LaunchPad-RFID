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
#include "conf.h"
#include "functions_tim.h"
#include "sysctl.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define	TIM_MARGIN	(0.01)

/* Private macro -------------------------------------------------------------*/
#define ASSERT_TIM_MODULE(x)    ((x) == MOD_TIM0 || \
                                 (x) == MOD_TIM1 || \
                                 (x) == MOD_TIM2 || \
                                 (x) == MOD_TIM3 || \
                                 (x) == MOD_TIM4 || \
                                 (x) == MOD_TIM5)

#define ASSERT_TIM_TYPE(x)      ((x) == TYPE_ONE_SHOT_UP || \
                                 (x) == TYPE_ONE_SHOT_DOWN || \
                                 (x) == TYPE_PERIODIC_UP || \
                                 (x) == TYPE_PERIODIC_DOWN || \
                                 (x) == TYPE_RTC)

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/*
 *  Enable the SysCtl related to the specified timer.
 */

void fTim_enableSysCtl (const fTim_Mod* Tim_Mod)
{
#ifdef DEBUG
    ASSERT_PARAM(ASSERT_TIM_MODULE(Tim_Mod->Module));
#endif

    switch (Tim_Mod->Module)
    {
    case MOD_TIM0:
        SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER0);
        break;
    case MOD_TIM1:
        SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER1);
        break;
    case MOD_TIM2:
        SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER2);
        break;
    case MOD_TIM3:
        SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER3);
        break;
    case MOD_TIM4:
        SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER4);
        break;
    case MOD_TIM5:
        SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER5);
        break;
    default:
        break;
    }
}

/*
 * Sets the type of timer to be initialized.
 */

void fTim_setType (const fTim_Mod* Tim_Mod)
{
#ifdef DEBUG
    ASSERT_PARAM(ASSERT_TIM_MODULE(Tim_Mod->Module));
    ASSERT_PARAM(ASSERT_TIM_TYPE(Tim_Mod->Type));
#endif

    TimerConfigure(Tim_Mod->Module, Tim_Mod->Type);
}

/*
 * Calculates the prescaler and the load value for the specified time.
 *
 * Formula: Time (s) = Freq (Hz) * LoadValue | For full width timers.
 */

void fTim_setTime (const fTim_Mod* Tim_Mod)
{
#ifdef DEBUG
    ASSERT_PARAM(ASSERT_TIM_MODULE(Tim_Mod->Module));
#endif
    uint64_t LoadValue;
    double Freq = (double) SysCtlClockGet();

    LoadValue = (uint64_t) (Freq  * (Tim_Mod->sTime));

    TimerLoadSet64(Tim_Mod->Module, LoadValue);
}

/*
 * Configures and enables the interrupts.
 */

void fTim_IntInit(const fTim_Mod* Tim_Mod)
{
#ifdef DEBUG
    ASSERT_PARAM(ASSERT_TIM_MODULE(Tim_Mod->Module));
#endif

    TimerIntRegister(Tim_Mod->Module, Tim_Mod->Int, Tim_Mod->IntIRQ);
    TimerIntEnable(Tim_Mod->Module, Tim_Mod->Int);
}

/*
 * Starts the specified timer.
 */

void fTim_Start(const fTim_Mod* Tim_Mod)
{
#ifdef DEBUG
    ASSERT_PARAM(ASSERT_TIM_MODULE(Tim_Mod->Module));
#endif

    TimerEnable(Tim_Mod->Module, TIMER_A);
}

/*
 * Initializes and starts the specified timer.
 */

void fTim_Init (const fTim_Mod* Tim_Mod)
{
#ifdef DEBUG
    ASSERT_PARAM(ASSERT_TIM_MODULE(Tim_Mod->Module));
    ASSERT_PARAM(ASSERT_TIM_TYPE(Tim_Mod->Type));
#endif

    fTim_enableSysCtl(Tim_Mod);
    fTim_setType(Tim_Mod);
    fTim_setTime(Tim_Mod);
    fTim_IntInit(Tim_Mod);
    fTim_Start(Tim_Mod);
}

/*
 * Processes the IRQ interrupt.
 */

void fTim_IRQ (const fTim_Mod* Tim_Mod)
{
    uint32_t val = TimerIntStatus(Tim_Mod->Module, Tim_Mod->Int);

    if (val & INT_TIMEOUT)
    {
        TimerIntClear(Tim_Mod->Module, INT_TIMEOUT);
    }
}
