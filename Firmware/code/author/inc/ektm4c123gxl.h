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
typedef struct
{
    void (*LED_Init) (uint8_t EKTM4C123GXL_LEDx);
    void (*LED_On) (uint8_t EKTM4C123GXL_LEDx);
    void (*LED_Off) (uint8_t EKTM4C123GXL_LEDx);
    void (*LED_Toggle) (uint8_t EKTM4C123GXL_LEDx);

    void (*PB_Init) (uint8_t EKTM4C123GXL_PBx);
    uint8_t (*PB_Read) (uint8_t EKTM4C123GXL_PBx);
    void (*PB_IntInit) (uint8_t EKTM4C123GXL_PBx, void (*EKTM4C123GXL_PB_INTIRQ) (void));
    uint8_t (*PB_IntTest) (uint8_t EKTM4C123GXL_PBx);
    void (*PB_IntClear) (uint8_t EKTM4C123GXL_PBx);
    void (*PB_IntStatus) (uint8_t EKTM4C123GXL_PBx, uint8_t EKTM4C123GXL_STATUSx);
} ektm4c123gxl_Class;

/* Exported constants --------------------------------------------------------*/
#define EKTM4C123GXL_LEDR        (0x01)
#define EKTM4C123GXL_LEDB        (0x02)
#define EKTM4C123GXL_LEDG        (0x04)

#define EKTM4C123GXL_PB1         (0x01)
#define EKTM4C123GXL_PB2         (0x02)

#define EKTM4C123GXL_STATUS_ON   (0x01)
#define EKTM4C123GXL_STATUS_OFF  (0x00)

#define UARTDBG 0x01

/* Exported macro ------------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
ektm4c123gxl_Class* ektm4c123gxl_CreateClass (void);
void ektm4c123gxl_DestroyClass (ektm4c123gxl_Class* class);

bool brd_UartInit (uint8_t UARTx);
void brd_UartDbgISR (void);
void brd_UartSend (uint8_t UARTx, const uint8_t* data);
void brd_UartParse (uint8_t UARTx);

#ifdef __cplusplus
}
#endif

#endif /*__EKTM4C123GXL_PINOUT_H */
 
