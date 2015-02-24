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
#ifndef __FUNCTIONS_GPIO_H
#define __FUNCTIONS_GPIO_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "gpio.h"
#include "hw_memmap.h"

/* Exported types ------------------------------------------------------------*/
typedef struct
{
    void (*InitInput) (uint32_t FGPIO_PINx, uint32_t FGPIO_PORTx, uint32_t FGPIO_TYPEx, uint32_t FGPIO_CURRENTx);
    void (*InitOutput) (uint32_t FGPIO_PINx, uint32_t FGPIO_PORTx, uint32_t FGPIO_TYPEx, uint32_t FGPIO_CURRENTx);
    void (*InitInputInt) (uint32_t FGPIO_PINx, uint32_t FGPIO_PORTx, uint32_t FGPIO_TYPEx, uint32_t FGPIO_CURRENTx, uint32_t FGPIO_INTTYPEx, void (*FGPIO_INTIRQ) (void));
    void (*InitAlternateFunction) (uint32_t FGPIO_PINx, uint32_t FGPIO_PORTx, uint32_t FGPIO_TYPEx, uint32_t FGPIO_CURRENTx, uint32_t FGPIO_AF);

} fGpio_Class;

/* Exported constants --------------------------------------------------------*/
#define FGPIO_PIN_0   GPIO_PIN_0
#define FGPIO_PIN_1   GPIO_PIN_1
#define FGPIO_PIN_2   GPIO_PIN_2
#define FGPIO_PIN_3   GPIO_PIN_3
#define FGPIO_PIN_4   GPIO_PIN_4
#define FGPIO_PIN_5   GPIO_PIN_5
#define FGPIO_PIN_6   GPIO_PIN_6
#define FGPIO_PIN_7   GPIO_PIN_7

#define FGPIO_PORT_A  GPIO_PORTA_BASE
#define FGPIO_PORT_B  GPIO_PORTB_BASE
#define FGPIO_PORT_C  GPIO_PORTC_BASE
#define FGPIO_PORT_D  GPIO_PORTD_BASE
#define FGPIO_PORT_E  GPIO_PORTE_BASE
#define FGPIO_PORT_F  GPIO_PORTF_BASE
#define FGPIO_PORT_G  GPIO_PORTG_BASE
#define FGPIO_PORT_H  GPIO_PORTH_BASE
#define FGPIO_PORT_J  GPIO_PORTJ_BASE
#define FGPIO_PORT_K  GPIO_PORTK_BASE
#define FGPIO_PORT_L  GPIO_PORTL_BASE
#define FGPIO_PORT_M  GPIO_PORTM_BASE
#define FGPIO_PORT_N  GPIO_PORTN_BASE
#define FGPIO_PORT_P  GPIO_PORTP_BASE
#define FGPIO_PORT_Q  GPIO_PORTQ_BASE
#define FGPIO_PORT_R  GPIO_PORTR_BASE
#define FGPIO_PORT_S  GPIO_PORTS_BASE
#define FGPIO_PORT_T  GPIO_PORTT_BASE

#define FGPIO_CURRENT_2MA               GPIO_STRENGTH_2MA
#define FGPIO_CURRENT_4MA               GPIO_STRENGTH_4MA
#define FGPIO_CURRENT_6MA               GPIO_STRENGTH_6MA
#define FGPIO_CURRENT_8MA               GPIO_STRENGTH_8MA
#define FGPIO_CURRENT_8MA_SC            GPIO_STRENGTH_8MA_SC
#define FGPIO_CURRENT_10MA              GPIO_STRENGTH_10MA
#define FGPIO_CURRENT_12MA              GPIO_STRENGTH_12MA

#define FGPIO_TYPE_PUSH_PULL                   GPIO_PIN_TYPE_STD
#define FGPIO_TYPE_PUSH_PULL_PULLUP            GPIO_PIN_TYPE_STD_WPU
#define FGPIO_TYPE_PUSH_PULL_PULLDOWN          GPIO_PIN_TYPE_STD_WPD
#define FGPIO_TYPE_OPEN_DRAIN                  GPIO_PIN_TYPE_OD
#define FGPIO_TYPE_ANALOG                     GPIO_PIN_TYPE_ANALOG
#define FGPIO_TYPE_WAKE_HIGH                  GPIO_PIN_TYPE_WAKE_HIGH
#define FGPIO_TYPE_WAKE_LOW                   GPIO_PIN_TYPE_WAKE_LOW

#define FGPIO_INTTYPE_FALLING_EDGE            GPIO_FALLING_EDGE
#define FGPIO_INTTYPE_RISING_EDGE             GPIO_RISING_EDGE
#define FGPIO_INTTYPE_BOTH_EDGES                GPIO_BOTH_EDGES
#define FGPIO_INTTYPE_LOW_LEVEL                       GPIO_LOW_LEVEL
#define FGPIO_INTTYPE_HIGH_LEVEL                      GPIO_HIGH_LEVEL
#define FGPIO_INTTYPE_DISCRETE_INT                    GPIO_DISCRETE_INT

#define FGPIO_OUTPUT_HIGH                       (0x00)
#define FGPIO_OUTPUT_LOW                        (0x01)
#define FGPIO_OUTPUT_TOGGLE                     (0x02)

#define FGPIO_INPUT_HIGH                        (0x00)
#define FGPIO_INPUT_LOW                         (0x01)

#define FGPIO_INT_OFF                        (0x00)
#define FGPIO_INT_ON                       (0x01)

/* Exported macro ------------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
fGpio_Class* fGpio_CreateClass (void);
void fGpio_DestroyClass (fGpio_Class* class);

#ifdef __cplusplus
}
#endif

#endif /*__FUNCTIONS_GPIO_H */
 
