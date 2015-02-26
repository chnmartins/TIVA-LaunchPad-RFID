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
#include <stdlib.h>
#include <stddef.h>

#include "conf.h"
#include "functions_tim.h"
#include "sysctl.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define	FTIMER_TIMER_MARGIN	        (0.01)

/* Private macro -------------------------------------------------------------*/
#define FTIMER_ASSERT_TIMER_MODULE(x)      ((x) == FTIMER_MODULE_0 || \
                                            (x) == FTIMER_MODULE_1 || \
                                            (x) == FTIMER_MODULE_2 || \
                                            (x) == FTIMER_MODULE_3 || \
                                            (x) == FTIMER_MODULE_4 || \
                                            (x) == FTIMER_MODULE_5)

#define FTIMER_ASSERT_TIMER_TYPE(x)        ((x) == FTIMER_TYPE_ONE_SHOT_DOWN    || \
                                            (x) == FTIMER_TYPE_ONE_SHOT_UP      || \
                                            (x) == FTIMER_TYPE_PERIODIC_DOWN    || \
                                            (x) == FTIMER_TYPE_PERIODIC_UP      || \
                                            (x) == FTIMER_TYPE_RTC)

#define FTIMER_ASSERT_TIMER_INT(x)      ((x) == FTIMER_INT_TIMEOUT)

#define FTIMER_ASSERT_TIMER_STATUS(x)      ((x) == FTIMER_STATUS_OFF || \
                                            (x) == FTIMER_STATUS_ON)

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
static void fTim_SysCtlStatus (uint32_t FTIMER_MODULEx, uint8_t FTIMER_STATUSx);
static void fTim_Config (uint32_t FTIMER_MODULEx, uint32_t FTIMER_TYPEx);
static void fTim_SetTime (uint32_t FTIMER_MODULEx, double time);
static void fTim_IntInit (uint32_t FTIMER_MODULEx, double time, uint32_t FTIMER_TYPEx, uint32_t FTIMER_INTx, void (*TIMER_IRQ) (void));
static void fTim_Init (uint32_t FTIMER_MODULEx, double time, uint32_t FTIMER_TYPEx);
static void fTim_DeInit (uint32_t FTIMER_MODULEx);
static uint32_t fTim_IntGet (uint32_t FTIMER_MODULEx, uint8_t FTIMER_STATUSx);
static uint8_t fTim_IntTest (uint32_t FTIMER_MODULEx, uint32_t FTIMER_INTx, uint8_t FTIMER_STATUSx);
static void fTim_IntStatus (uint32_t FTIMER_MODULEx, uint32_t FTIMER_INTx, uint8_t FTIMER_STATUSx);
static void fTim_IntClear (uint32_t FTIMER_MODULEx, uint32_t FTIMER_INTx);

/* Private functions ---------------------------------------------------------*/

/*
 * Constructor.
 */

fTim_Class* fTim_CreateClass (void)
{
    fTim_Class* class = malloc(sizeof(fTim_Class));
    if (class == NULL)
        return NULL;

    class->Init = fTim_Init;
    class->IntInit = fTim_IntInit;
    class->DeInit = fTim_DeInit;
    class->IntClear = fTim_IntClear;
    class->IntGet = fTim_IntGet;
    class->IntTest = fTim_IntTest;
    class->IntStatus = fTim_IntStatus;

    return class;
}

/*
 * Destructor.
 */

void fTim_DestroyClass (fTim_Class* class)
{
    free(class);
}

/*
 *  Enable the SysCtl related to the specified timer.
 */

static void fTim_SysCtlStatus (uint32_t FTIMER_MODULEx, uint8_t FTIMER_STATUSx)
{
#ifdef DEBUG
    ASSERT_PARAM(FTIMER_ASSERT_TIMER_MODULE(FTIMER_MODULEx));
    ASSERT_PARAM(FTIMER_ASSERT_TIMER_STATUS(FTIMER_STATUSx));
#endif

    switch (FTIMER_MODULEx)
    {
    case FTIMER_MODULE_0:
        (FTIMER_STATUSx == FTIMER_STATUS_ON) ? SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER0) : SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER0);
        break;
    case FTIMER_MODULE_1:
        (FTIMER_STATUSx == FTIMER_STATUS_ON) ? SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER1) : SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER1);
        break;
    case FTIMER_MODULE_2:
        (FTIMER_STATUSx == FTIMER_STATUS_ON) ? SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER2) : SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER2);
        break;
    case FTIMER_MODULE_3:
        (FTIMER_STATUSx == FTIMER_STATUS_ON) ? SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER3) : SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER3);
        break;
    case FTIMER_MODULE_4:
        (FTIMER_STATUSx == FTIMER_STATUS_ON) ? SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER4) : SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER4);
        break;
    case FTIMER_MODULE_5:
        (FTIMER_STATUSx == FTIMER_STATUS_ON) ? SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER5) : SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER5);
        break;
    default:
        break;
    }
}

/*
 * Sets the type of timer to be initialized.
 */

static void fTim_Config (uint32_t FTIMER_MODULEx, uint32_t FTIMER_TYPEx)
{
#ifdef DEBUG
    ASSERT_PARAM(FTIMER_ASSERT_TIMER_MODULE(FTIMER_MODULEx));
    ASSERT_PARAM(FTIMER_ASSERT_TIMER_TYPE(FTIMER_TYPEx));
#endif

    TimerConfigure(FTIMER_MODULEx, FTIMER_TYPEx);
}

/*
 * Calculates the prescaler and the load value for the specified time.
 *
 * Formula: Time (s) = Freq (Hz) * LoadValue | For full width timers.
 */

static void fTim_SetTime (uint32_t FTIMER_MODULEx, double time)
{
#ifdef DEBUG
    ASSERT_PARAM(FTIMER_ASSERT_TIMER_MODULE(FTIMER_MODULEx));
#endif
    uint64_t LoadValue;
    double Freq = (double) SysCtlClockGet();

    LoadValue = (uint64_t) (Freq  * (time));

    TimerLoadSet64(FTIMER_MODULEx, LoadValue);
}

/*
 * Configures and enables the interrupts.
 */

static void fTim_IntInit (uint32_t FTIMER_MODULEx, double time, uint32_t FTIMER_TYPEx, uint32_t FTIMER_INTx, void (*TIMER_IRQ) (void))
{
#ifdef DEBUG
    ASSERT_PARAM(FTIMER_ASSERT_TIMER_MODULE(FTIMER_MODULEx));
    ASSERT_PARAM(FTIMER_ASSERT_TIMER_TYPE(FTIMER_TYPEx));
#endif

    fTim_SysCtlStatus(FTIMER_MODULEx, FTIMER_STATUS_ON);
    fTim_DeInit(FTIMER_MODULEx);
    fTim_Config(FTIMER_MODULEx, FTIMER_TYPEx);
    fTim_SetTime(FTIMER_MODULEx, time);
    TimerIntRegister(FTIMER_MODULEx, FTIMER_INTx, TIMER_IRQ);
    fTim_IntStatus(FTIMER_MODULEx, FTIMER_TYPEx, FTIMER_STATUS_ON);
    TimerEnable(FTIMER_MODULEx, TIMER_A);
}

/*
 * Initializes and starts the specified timer.
 */

static void fTim_Init (uint32_t FTIMER_MODULEx, double time, uint32_t FTIMER_TYPEx)
{
#ifdef DEBUG
    ASSERT_PARAM(FTIMER_ASSERT_TIMER_MODULE(FTIMER_MODULEx));
    ASSERT_PARAM(FTIMER_ASSERT_TIMER_TYPE(FTIMER_TYPEx));
#endif

    fTim_SysCtlStatus(FTIMER_MODULEx, FTIMER_STATUS_ON);
    fTim_DeInit(FTIMER_MODULEx);
    fTim_Config(FTIMER_MODULEx, FTIMER_TYPEx);
    fTim_SetTime(FTIMER_MODULEx, time);
    TimerEnable(FTIMER_MODULEx, TIMER_A);
}

/*
 * Deinitializes the timer.
 */

static void fTim_DeInit (uint32_t FTIMER_MODULEx)
{
#ifdef DEBUG
    ASSERT_PARAM(FTIMER_ASSERT_TIMER_MODULE(FTIMER_MODULEx));
#endif

    TimerIntUnregister(FTIMER_MODULEx, TIMER_A);
    TimerDisable(FTIMER_MODULEx, TIMER_A);
}

/*
 *
 */

static uint32_t fTim_IntGet (uint32_t FTIMER_MODULEx, uint8_t FTIMER_STATUSx)
{
#ifdef DEBUG
    ASSERT_PARAM(FTIMER_ASSERT_TIMER_MODULE(FTIMER_MODULEx));
    ASSERT_PARAM(FTIMER_ASSERT_TIMER_STATUS(FTIMER_STATUSx));
#endif

    return TimerIntStatus(FTIMER_MODULEx, FTIMER_STATUSx);
}

/*
 *
 */

static uint8_t fTim_IntTest (uint32_t FTIMER_MODULEx, uint32_t FTIMER_INTx, uint8_t FTIMER_STATUSx)
{
#ifdef DEBUG
    ASSERT_PARAM(FTIMER_ASSERT_TIMER_MODULE(FTIMER_MODULEx));
    ASSERT_PARAM(FTIMER_ASSERT_TIMER_INT(FTIMER_INTx));
    ASSERT_PARAM(FTIMER_ASSERT_TIMER_STATUS(FTIMER_STATUSx));
#endif

    if (TimerIntStatus(FTIMER_MODULEx, FTIMER_STATUSx) & FTIMER_INTx)
        return FTIMER_STATUS_ON;

    return FTIMER_STATUS_OFF;
}

/*
 *
 */

static void fTim_IntStatus (uint32_t FTIMER_MODULEx, uint32_t FTIMER_INTx, uint8_t FTIMER_STATUSx)
{
#ifdef DEBUG
    ASSERT_PARAM(FTIMER_ASSERT_TIMER_MODULE(FTIMER_MODULEx));
    ASSERT_PARAM(FTIMER_ASSERT_TIMER_STATUS(FTIMER_STATUSx));
#endif

    if (FTIMER_STATUSx == FTIMER_STATUS_ON)
        TimerIntEnable(FTIMER_MODULEx, FTIMER_INTx);
    else
        TimerIntDisable(FTIMER_MODULEx, FTIMER_INTx);
}

/*
 *
 */

static void fTim_IntClear (uint32_t FTIMER_MODULEx, uint32_t FTIMER_INTx)
{
#ifdef DEBUG
    ASSERT_PARAM(FTIMER_ASSERT_TIMER_MODULE(FTIMER_MODULEx));
#endif

    TimerIntClear(FTIMER_MODULEx, FTIMER_INTx);
}
