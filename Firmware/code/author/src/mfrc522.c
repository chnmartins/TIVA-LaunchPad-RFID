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
#include "mfrc522.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define MFRC522_RST_LOW      0x00
#define MFRC522_RST_HIGH     0x01

#define MFRC522_READ_MSB    0x80
#define MFRC522_WRITE_MSB   0x00

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/*
 * Initializes the mfrc522 chip and performs a hard reset.
 */

mfrc522_result mfrc522_Init (mfrc522_Mod* Dev)
{
    mfrc522_result res = mfrc522_ok;

    if (Dev->HwInit() == false)
        res = mfrc522_error;
    else
        mfrc522_HardReset(Dev);

    return res;
}

/*
 * Reads the value at address.
 */

void mfrc522_ReadAddress (mfrc522_Mod* Dev, uint8_t address, uint8_t* value)
{
    Dev->SendByte(MFRC522_READ_MSB | (address << 1));
    Dev->ReadByte(value);
}

/*
 * Writes the value at address.
 */

void mfrc522_WriteAddress (mfrc522_Mod* Dev, uint8_t address, uint8_t value)
{
    Dev->SendByte(MFRC522_WRITE_MSB | (address << 1));
    Dev->SendByte(value);
}

/*
 * Performs a hard reset.
 */

void mfrc522_HardReset (mfrc522_Mod* Dev)
{
    Dev->RstCtrl(MFRC522_RST_LOW);
    Dev->Delay(0.1);
    Dev->RstCtrl(MFRC522_RST_HIGH);
    Dev->Delay(0.1);
}
