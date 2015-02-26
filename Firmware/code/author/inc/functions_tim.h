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
typedef struct {
    void (*Init) (uint32_t FTIMER_MODULEx, double time, uint32_t FTIMER_TYPEx);
    void (*IntInit) (uint32_t FTIMER_MODULEx, double time, uint32_t FTIMER_TYPEx, uint32_t FTIMER_INTx, void (*TIMER_IRQ) (void));
    void (*DeInit) (uint32_t FTIMER_MODULEx);
    uint32_t (*IntGet) (uint32_t FTIMER_MODULEx, uint8_t FTIMER_STATUSx);
    uint8_t (*IntTest) (uint32_t FTIMER_MODULEx, uint32_t FTIMER_INTx, uint8_t FTIMER_STATUSx);
    void (*IntStatus) (uint32_t FTIMER_MODULEx, uint32_t FTIMER_INTx, uint8_t FTIMER_STATUSx);
    void (*IntClear) (uint32_t FTIMER_MODULEx, uint32_t FTIMER_INTx);
} fTim_Class;

/* Exported constants --------------------------------------------------------*/
#define FTIMER_MODULE_0    WTIMER0_BASE
#define FTIMER_MODULE_1    WTIMER1_BASE
#define FTIMER_MODULE_2    WTIMER2_BASE
#define FTIMER_MODULE_3    WTIMER3_BASE
#define FTIMER_MODULE_4    WTIMER4_BASE
#define FTIMER_MODULE_5    WTIMER5_BASE

#define FTIMER_TYPE_ONE_SHOT_UP        TIMER_CFG_ONE_SHOT_UP
#define FTIMER_TYPE_ONE_SHOT_DOWN      TIMER_CFG_ONE_SHOT
#define FTIMER_TYPE_PERIODIC_UP        TIMER_CFG_PERIODIC_UP
#define FTIMER_TYPE_PERIODIC_DOWN      TIMER_CFG_PERIODIC
#define FTIMER_TYPE_RTC                TIMER_CFG_RTC

#define FTIMER_INT_TIMEOUT         TIMER_TIMA_TIMEOUT
// TODO: Fill with the rest of the interrupts.

#define FTIMER_STATUS_ON            (0x01)
#define FTIMER_STATUS_OFF           (0x00)

/* Exported macro ------------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
fTim_Class* fTim_CreateClass (void);
void fTim_DestroyClass (fTim_Class* class);

#ifdef __cplusplus
}
#endif

#endif /*__FUNCTIONS_TIM_H */
 
