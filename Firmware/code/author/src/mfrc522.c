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
#include "conf.h"

/* Private typedef -----------------------------------------------------------*/
typedef enum
{
    Idle,
    Mem,
    GenerateRandomID,
    CalcCrc,
    Transmit,
    NoCmdChange,
    Receive,
    Transceive,
    MFAuthent,
    SoftReset,
} mfrc522_Command;

/* Private define ------------------------------------------------------------*/
#define MFRC522_GPIO_LOW      0x00
#define MFRC522_GPIO_HIGH     0x01

#define MFRC522_READ_MSB    0x80
#define MFRC522_WRITE_MSB   0x00


// Commands
#define MFRC522_CMD_SOFTRESET   (0x0F)

// Register Addresses
#define MFRC522_ADDR_COMMAND        (0x01)
#define MFRC522_ADDR_FIFODATA       (0x09)
#define MFRC522_ADDR_FIFOLEVEL      (0x0A)

// Bit masks
#define MFRC522_BMS_COMMAND_COMMAND_BITS                    BITS(0x0F, 0)
#define MFRC522_BMS_COMMAND_COMMAND_IDLE                    BITS(0x00, 0)
#define MFRC522_BMS_COMMAND_COMMAND_MEM                     BITS(0x01, 0)
#define MFRC522_BMS_COMMAND_COMMAND_GENRANDOMID             BITS(0x02, 0)
#define MFRC522_BMS_COMMAND_COMMAND_CALCCRC                 BITS(0x03, 0)
#define MFRC522_BMS_COMMAND_COMMAND_TRANSMIT                BITS(0x04, 0)
#define MFRC522_BMS_COMMAND_COMMAND_NOCMDCHANGE             BITS(0x07, 0)
#define MFRC522_BMS_COMMAND_COMMAND_RECEIVE                 BITS(0x08, 0)
#define MFRC522_BMS_COMMAND_COMMAND_TRANSCEIVE              BITS(0x0C, 0)
#define MFRC522_BMS_COMMAND_COMMAND_MFAUTHENT               BITS(0x0E, 0)
#define MFRC522_BMS_COMMAND_COMMAND_SOFTRESET               BITS(0x0F, 0)
#define MFRC522_BMS_COMMAND_POWERDOWN_BIT                   BIT(4)
#define MFRC522_BMS_COMMAND_RCVOFF_BIT                      BIT(5)

#define MFRC522_BMS_FIFOLEVEL_FIFOLEVEL_BITS                BITS(0x7F, 0)
#define MFRC522_BMS_FIFOLEVEL_FLUSHBUFFER_BIT               BIT(0x07)

/* Private macro -------------------------------------------------------------*/
#define BIT(n)          (1 << (n))
#define BITS(x, n)      ((x) << (n))

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
void mfrc522_CommandExecute (mfrc522_Mod* Dev, mfrc522_Command cmd);
mfrc522_Command mfrc522_GetCurrentCommand (mfrc522_Mod* Dev);
uint8_t mfrc522_FIFOGetLevel (mfrc522_Mod* Dev);
void mfrc522_FIFORead (mfrc522_Mod* Dev, uint8_t* buffer, uint8_t bytestoread);
void mfrc522_FIFOWrite (mfrc522_Mod* Dev, uint8_t* buffer, uint8_t bytestowrite);
void mfrc522_FIFOClear (mfrc522_Mod* Dev);

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

    mfrc522_SelfTest(Dev);

    return res;
}

/*
 * Reads the value at address.
 */

void mfrc522_ReadAddress (mfrc522_Mod* Dev, uint8_t address, uint8_t* value)
{
    Dev->ReadAddress(MFRC522_READ_MSB | (address << 1), value);
}

/*
 * Writes the value at address.
 */

void mfrc522_WriteAddress (mfrc522_Mod* Dev, uint8_t address, uint8_t value)
{
    Dev->WriteAddress(MFRC522_WRITE_MSB | (address << 1), value);
}

/*
 * Performs a hard reset.
 */

void mfrc522_HardReset (mfrc522_Mod* Dev)
{
    Dev->Delay(0.1);
    Dev->RstCtrl(MFRC522_GPIO_LOW);
    Dev->Delay(0.1);
    Dev->RstCtrl(MFRC522_GPIO_HIGH);
    Dev->Delay(0.1);
}

/*
 * Stops the current command and executes a soft reset.
 */

void mfrc522_SoftReset (mfrc522_Mod* Dev)
{
    mfrc522_CommandExecute(Dev, SoftReset);
    Dev->Delay(0.1);
    while (mfrc522_GetCurrentCommand(Dev) != Idle);
}

/*
 * Returns the current command being executed.
 */

mfrc522_Command mfrc522_GetCurrentCommand (mfrc522_Mod* Dev)
{
    uint8_t temp;
    mfrc522_Command cmd;

    mfrc522_ReadAddress(Dev, MFRC522_ADDR_COMMAND, &temp);

    temp &= MFRC522_BMS_COMMAND_COMMAND_BITS;

    switch (temp)
    {
    case MFRC522_BMS_COMMAND_COMMAND_IDLE:
        cmd = Idle;
        break;
    case MFRC522_BMS_COMMAND_COMMAND_MEM:
        cmd = Mem;
        break;
    case MFRC522_BMS_COMMAND_COMMAND_GENRANDOMID:
        cmd = GenerateRandomID;
        break;
    case MFRC522_BMS_COMMAND_COMMAND_CALCCRC:
        cmd = CalcCrc;
        break;
    case MFRC522_BMS_COMMAND_COMMAND_TRANSMIT:
        cmd = Transmit;
        break;
    case MFRC522_BMS_COMMAND_COMMAND_NOCMDCHANGE:
        cmd = NoCmdChange;
        break;
    case MFRC522_BMS_COMMAND_COMMAND_RECEIVE:
        cmd = Receive;
        break;
    case MFRC522_BMS_COMMAND_COMMAND_TRANSCEIVE:
        cmd = Transceive;
        break;
    case MFRC522_BMS_COMMAND_COMMAND_MFAUTHENT:
        cmd = MFAuthent;
        break;
    case MFRC522_BMS_COMMAND_COMMAND_SOFTRESET:
        cmd = SoftReset;
        break;
    default:
        break;
    }

    return cmd;
}

/*
 * Executes the specified command.
 */

void mfrc522_CommandExecute (mfrc522_Mod* Dev, mfrc522_Command cmd)
{
    uint8_t temp;

    mfrc522_ReadAddress(Dev, MFRC522_ADDR_COMMAND, &temp);

    temp &= ~(MFRC522_BMS_COMMAND_COMMAND_BITS);

    switch (cmd)
    {
    case Idle:
        temp |= MFRC522_BMS_COMMAND_COMMAND_IDLE;
        break;
    case Mem:
        temp |= MFRC522_BMS_COMMAND_COMMAND_MEM;
        break;
    case GenerateRandomID:
        temp |= MFRC522_BMS_COMMAND_COMMAND_GENRANDOMID;
        break;
    case CalcCrc:
        temp |= MFRC522_BMS_COMMAND_COMMAND_CALCCRC;
        break;
    case Transmit:
        temp |= MFRC522_BMS_COMMAND_COMMAND_TRANSMIT;
        break;
    case NoCmdChange:
        temp |= MFRC522_BMS_COMMAND_COMMAND_NOCMDCHANGE;
        break;
    case Receive:
        temp |= MFRC522_BMS_COMMAND_COMMAND_RECEIVE;
        break;
    case Transceive:
        temp |= MFRC522_BMS_COMMAND_COMMAND_TRANSCEIVE;
        break;
    case MFAuthent:
        temp |= MFRC522_BMS_COMMAND_COMMAND_MFAUTHENT;
        break;
    case SoftReset:
        temp |= MFRC522_BMS_COMMAND_COMMAND_SOFTRESET;
        break;
    default:
        break;
    }

    mfrc522_WriteAddress(Dev, MFRC522_ADDR_COMMAND, temp);
}

/*
 * Performs a self test. How to perform a self reset can be found on the datasheet (Section 16.1.1).
 */

void mfrc522_SelfTest (mfrc522_Mod* Dev)
{
    mfrc522_SoftReset(Dev);


}

/*
 * Writes the specified bytes on the FIFO. The FIFO capacity is 64 bytes.
 */

void mfrc522_FIFOWrite (mfrc522_Mod* Dev, uint8_t* buffer, uint8_t bytestowrite)
{
    uint8_t FifoLevel = 0;
    uint8_t i = 0;

    FifoLevel = mfrc522_FIFOGetLevel(Dev);

    while (FifoLevel < 64 && (i < bytestowrite))
    {
        mfrc522_WriteAddress(Dev, MFRC522_ADDR_FIFODATA, *(buffer + i));
        while (FifoLevel == mfrc522_FIFOGetLevel(Dev));
        FifoLevel = mfrc522_FIFOGetLevel(Dev);
        i++;
    }
}

/*
 * Clears the FIFO, write and read pointers become zero.
 */

void mfrc522_FIFOClear (mfrc522_Mod* Dev)
{
    uint8_t temp;

    mfrc522_ReadAddress(Dev, MFRC522_ADDR_FIFOLEVEL, &temp);

    temp |= MFRC522_BMS_FIFOLEVEL_FLUSHBUFFER_BIT;

    mfrc522_WriteAddress(Dev, MFRC522_ADDR_FIFOLEVEL, temp);

    while (mfrc522_FIFOGetLevel(Dev) > 0);
}

/*
 * Gets the current level on the FIFO.
 */

uint8_t mfrc522_FIFOGetLevel (mfrc522_Mod* Dev)
{
    uint8_t temp = 0;

    mfrc522_ReadAddress(Dev, MFRC522_ADDR_FIFOLEVEL, &temp);

    temp &= MFRC522_BMS_FIFOLEVEL_FIFOLEVEL_BITS;

    return temp;
}

/*
 * Reads data from the FIFO.
 */

void mfrc522_FIFORead (mfrc522_Mod* Dev, uint8_t* buffer, uint8_t bytestoread)
{
    uint8_t FifoLevel = mfrc522_FIFOGetLevel(Dev);
    uint8_t i = 0;

    while (FifoLevel > 0 && (i < bytestoread))
    {
        mfrc522_ReadAddress(Dev, MFRC522_ADDR_FIFODATA, buffer + i);
        while (FifoLevel == mfrc522_FIFOGetLevel(Dev));
        FifoLevel = mfrc522_FIFOGetLevel(Dev);
        i++;
    }
}
