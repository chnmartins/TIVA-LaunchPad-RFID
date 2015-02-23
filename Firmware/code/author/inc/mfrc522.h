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
 typedef struct
 {
     void (*Delay) (double seconds);
     bool (*HwInit) (void);
     void (*RstCtrl) (uint8_t status);
     void (*ReadAddress) (uint8_t address, uint8_t* value);
     void (*WriteAddress) (uint8_t address, uint8_t value);
 } mfrc522_Mod;

 typedef enum
 {
     mfrc522_ok,
     mfrc522_error,
     mfrc522_nomem,
     mfrc522_V1,
     mfrc522_V2,
 } mfrc522_result;

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
mfrc522_result mfrc522_Init (mfrc522_Mod* Dev);
void mfrc522_HardReset (mfrc522_Mod* Dev);
void mfrc522_ReadAddress (mfrc522_Mod* Dev, uint8_t address, uint8_t* value);
void mfrc522_WriteAddress (mfrc522_Mod* Dev, uint8_t address, uint8_t value);
void mfrc522_SoftReset (mfrc522_Mod* Dev);
mfrc522_result mfrc522_SelfTest (mfrc522_Mod* Dev);
void mfrc522_IBWrite (mfrc522_Mod* Dev, uint8_t* buffer);
void mfrc522_IBRead (mfrc522_Mod* Dev, uint8_t* buffer);
void mfrc522_Initialization (mfrc522_Mod* Dev);

void mfrc522_TimerConfigure (mfrc522_Mod* Dev, uint16_t options);
void mfrc522_TimerSetParams (mfrc522_Mod* Dev, uint16_t prescaler, uint16_t reloadval);
uint16_t mfrc522_TimerGetValue (mfrc522_Mod* Dev);
bool mfrc522_TimerIsFinished (mfrc522_Mod* Dev);
bool mfrc522_TimerIsRunning (mfrc522_Mod* Dev);
void mfrc522_TimerStart (mfrc522_Mod* Dev);
void mfrc522_TimerStop (mfrc522_Mod* Dev);

void mfrc522_TransmitterIntEnable (mfrc522_Mod* Dev, bool status);
bool mfrc522_TransmitterIsFinished (mfrc522_Mod* Dev);
void mfrc522_TransmitterSetBits (mfrc522_Mod* Dev, uint8_t nBits);
void mfrc522_TransmitterWaitsRF (mfrc522_Mod* Dev, bool status);
void mfrc522_TransmitterEnableCRC (mfrc522_Mod* Dev, bool status);
void mfrc522_TransmitterForce100ASK (mfrc522_Mod* Dev, bool status);

void mfrc522_AntennaSettings (mfrc522_Mod* Dev, uint8_t options);
void mfrc522_EnableParity (mfrc522_Mod* Dev, bool status);

#ifdef __cplusplus
}
#endif

#endif /*__MFRC522_H */
 
