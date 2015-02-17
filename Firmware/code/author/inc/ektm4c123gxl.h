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
#ifndef __EKTM4C123GXL_PINOUT_H
#define __EKTM4C123GXL_PINOUT_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
#define LEDR    0x01
#define LEDG    0x02
#define LEDB    0x04

#define LED_ON      0x01
#define LED_OFF     0x02
#define LED_TOGGLE  0x03

#define PB1     0x01
#define PB2     0x02

#define PB1_ON  0x01
#define PB2_ON  0x02

#define UARTDBG 0x01

/* Exported macro ------------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
void brd_LedInit (uint8_t LEDx);
void brd_LedInteract (uint8_t LEDx, uint8_t interact);
void brd_PushButtonInit (uint8_t PBx);
uint8_t brd_PushButtonRead (uint8_t PBx);
void brd_PushButtonInitInt (uint8_t PBx, void (*IntIRQ) (void));
uint8_t brd_PushButtonGetInt (uint8_t PBx);
void brd_UartInit (uint8_t UARTx, void (*IntIRQ) (void));
void brd_UartDbgDefIRQHandler (void);
void brd_UartDbgDefTransmit (uint8_t* string);

#ifdef __cplusplus
}
#endif

#endif /*__EKTM4C123GXL_PINOUT_H */
 
