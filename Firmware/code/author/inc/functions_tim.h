/**
  ******************************************************************************
  * @file    C_HEADER_TEMPLATE.h
  * @author  Header template author and its contact email
  * @version Version 1 Issue 1
  * @date    Date when the file was released.	(i.e. 05-March-2014)
  * @brief   Short description about the header template.
  ******************************************************************************
  * @attention
  *
  *		Copyright Information (C)
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FUNCTIONS_TIM_H
#define __FUNCTIONS_TIM_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "hw_memmap.h"
#include "timer.h"

/* Exported types ------------------------------------------------------------*/
typedef struct
{
    uint32_t    Module;
    uint32_t    Type;
    double      sTime;
    uint32_t    Int;
    void (*IntIRQ) (void);
} fTim_Mod;

/* Exported constants --------------------------------------------------------*/
#define MOD_TIM0    WTIMER0_BASE
#define MOD_TIM1    WTIMER1_BASE
#define MOD_TIM2    WTIMER2_BASE
#define MOD_TIM3    WTIMER3_BASE
#define MOD_TIM4    WTIMER4_BASE
#define MOD_TIM5    WTIMER5_BASE

#define TYPE_ONE_SHOT_UP        TIMER_CFG_ONE_SHOT_UP
#define TYPE_ONE_SHOT_DOWN      TIMER_CFG_ONE_SHOT
#define TYPE_PERIODIC_UP        TIMER_CFG_PERIODIC_UP
#define TYPE_PERIODIC_DOWN      TIMER_CFG_PERIODIC
#define TYPE_RTC                TIMER_CFG_RTC

#define INT_NONE            0
#define INT_TIMEOUT         TIMER_TIMA_TIMEOUT
// TODO: Complete with the rest of the interrupts here.

/* Exported macro ------------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
void fTim_enableSysCtl (const fTim_Mod* Tim_Mod);
void fTim_setType (const fTim_Mod* Tim_Mod);
void fTim_setTime (const fTim_Mod* Tim_Mod);
void fTim_IntInit(const fTim_Mod* Tim_Mod);
void fTim_Start(const fTim_Mod* Tim_Mod);
void fTim_Init (const fTim_Mod* Tim_Mod);
bool fTim_IsTimeout (const fTim_Mod* Tim_Mod);

#ifdef __cplusplus
}
#endif

#endif /*__FUNCTIONS_TIM_H */
 
