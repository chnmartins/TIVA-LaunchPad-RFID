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
#ifndef __MFRC522_H
#define __MFRC522_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
typedef struct mfrc522_Class
{
     void (*__Delay) (double seconds);
     uint8_t (*__HwInit) (void);
     void (*__ResetControl) (uint8_t status);
     void (*__ReadRegister) (uint8_t address, uint8_t* value);
     void (*__WriteRegister) (uint8_t address, uint8_t value);

     uint8_t (*Init) (struct mfrc522_Class* Class);
     uint8_t (*SelfTest) (struct mfrc522_Class* Class);
} mfrc522_Class;

/* Exported constants --------------------------------------------------------*/
#define MFRC522_STATUS_ON           (0x01)
#define MFRC522_STATUS_OFF          (0x00)
/* Exported macro ------------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
mfrc522_Class* mfrc522_CreateClass (void (*Delay) (double), uint8_t (*HwInit) (void), void (*ResetControl) (uint8_t), void (*ReadRegister) (uint8_t, uint8_t*), void (*WriteRegister) (uint8_t, uint8_t));
void mfrc522_DestroyClass (mfrc522_Class* Class);

#ifdef __cplusplus
}
#endif

#endif /*__MFRC522_H */
 
