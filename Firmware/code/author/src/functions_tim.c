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

/* Private macro -------------------------------------------------------------*/
#define ASSERT_TIM_MODULE(x)    ((x) == MOD_TIM0 || \
                                 (x) == MOD_TIM1 || \
                                 (x) == MOD_TIM2 || \
                                 (x) == MOD_TIM3 || \
                                 (x) == MOD_TIM4 || \
                                 (x) == MOD_TIM5 || \
                                 (x) == MOD_TIM6)

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
        SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
        break;
    case MOD_TIM1:
        SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
        break;
    case MOD_TIM2:
        SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
        break;
    case MOD_TIM3:
        SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);
        break;
    case MOD_TIM4:
        SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER4);
        break;
    case MOD_TIM5:
        SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER5);
        break;
    case MOD_TIM6:
        SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER6);
        break;
    default:
        break;
    }
}

/*
 * Initializes the specified peripheral.
 */

void fTim_Init (const fTim_Mod* Tim_Mod)
{
#ifdef DEBUG
    ASSERT_PARAM(ASSERT_TIM_MODULE(Tim_Mod->Module));
    ASSERT_PARAM(ASSERT_TIM_TYPE(Tim_Mod->Type));
#endif

    fTim_enableSysCtl(Tim_Mod);
    TimerConfigure(Tim_Mod->Module, Tim_Mod->Type);
    TimerLoadSet()
}
