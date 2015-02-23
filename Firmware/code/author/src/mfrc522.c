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
#include <string.h>
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

typedef enum
{
	Speed_106kBd,
	Speed_212kBd,
	Speed_424kBd,
	Speed_848kBd,
} mfrc522_Speed;

/* Private define ------------------------------------------------------------*/
#define MFRC522_GPIO_LOW      0x00
#define MFRC522_GPIO_HIGH     0x01

#define MFRC522_READ_MSB    0x80
#define MFRC522_WRITE_MSB   0x00


// Commands
#define MFRC522_CMD_SOFTRESET   (0x0F)

// Register Addresses
#define MFRC522_ADDR_COMMAND        (0x01)
#define MFRC522_ADDR_COMIEN         (0x02)
#define MFRC522_ADDR_COMIRQ         (0x04)
#define MFRC522_ADDR_STATUS1        (0x07)
#define MFRC522_ADDR_FIFODATA       (0x09)
#define MFRC522_ADDR_FIFOLEVEL      (0x0A)
#define MFRC522_ADDR_CONTROL        (0x0C)
#define	MFRC522_ADDR_BITFRAMING		(0x0D)
#define MFRC522_ADDR_MODE			(0x11)
#define	MFRC522_ADDR_TXMODE			(0x12)
#define MFRC522_ADDR_TXCONTROL		(0x14)
#define MFRC522_ADDR_TXASK			(0x15)
#define MFRC522_ADDR_DEMOD          (0x19)
#define MFRC522_ADDR_MFRX			(0x1D)
#define MFRC522_ADDR_TMODE          (0x2A)
#define MFRC522_ADDR_TPRESCALERLO   (0x2B)
#define MFRC522_ADDR_TRELOADHI      (0x2C)
#define MFRC522_ADDR_TRELOADLO      (0x2D)
#define MFRC522_ADDR_TCOUNTERVALHI  (0x2E)
#define MFRC522_ADDR_TCOUNTERVALLO  (0x2F)
#define MFRC522_ADDR_AUTOTEST       (0x36)

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

#define MFRC522_BMS_AUTOTEST_SELFTEST_BITS                  BITS(0x0F, 0)
#define MFRC522_BMS_AUTOTEST_SELFTEST_SELFTEST              BITS(0x09, 0)

#define MFRC522_BMS_TMODE_TPRESCALERHI_BITS              BITS(0x0F, 0)
#define MFRC522_BMS_TMODE_TPRESCALERHI(x)                BITS((x), 0)
#define MFRC522_BMS_TMODE_TAUTORESTART_BIT               BIT(4)
#define MFRC522_BMS_TMODE_TGATED_BITS                    BITS(0x03, 5)
#define MFRC522_BMS_TMODE_TGATED_NONGATED                BITS(0x00, 5)
#define MFRC522_BMS_TMODE_TGATED_MFIN                    BITS(0x01, 5)
#define MFRC522_BMS_TMODE_TGATED_AUX1                    BITS(0x02, 5)
#define MFRC522_BMS_TMODE_TAUTO_BIT                      BIT(7)

#define MFRC522_BMS_DEMOD_TPRESCALEVEN_BIT               BIT(4)

#define MFRC522_BMS_COMIEN_IRQINV_BIT                    BIT(7)
#define MFRC522_BMS_COMIEN_TXIEN_BIT                     BIT(6)
#define MFRC522_BMS_COMIEN_RXIEN_BIT                     BIT(5)
#define MFRC522_BMS_COMIEN_IDLEIEN_BIT                   BIT(4)
#define MFRC522_BMS_COMIEN_HIALERTIEN_BIT                BIT(3)
#define MFRC522_BMS_COMIEN_LOALERTIEN_BIT                BIT(2)
#define MFRC522_BMS_COMIEN_ERRIEN_BIT                    BIT(1)
#define MFRC522_BMS_COMIEN_TIMERIEN_BIT                  BIT(0)

#define MFRC522_BMS_COMIRQ_SET1_BIT                      BIT(7)
#define MFRC522_BMS_COMIRQ_TXIRQ_BIT                      BIT(6)
#define MFRC522_BMS_COMIRQ_RXIRQ_BIT                      BIT(5)
#define MFRC522_BMS_COMIRQ_IDLEIRQ_BIT                      BIT(4)
#define MFRC522_BMS_COMIRQ_HIALERTIRQ_BIT                      BIT(3)
#define MFRC522_BMS_COMIRQ_LOALERTIRQ_BIT                      BIT(2)
#define MFRC522_BMS_COMIRQ_ERRIRQ_BIT                      BIT(1)
#define MFRC522_BMS_COMIRQ_TIMERIRQ_BIT                      BIT(0)

#define MFRC522_BMS_CONTROL_TSTOPNOW_BIT                 BIT(7)
#define MFRC522_BMS_CONTROL_TSTARTNOW_BIT                BIT(6)
#define MFRC522_BMS_CONTROL_RXLASTBITS_BITS              BITS(0x07, 0)

#define MFRC522_BMS_STATUS1_CRCOK_BIT                        BIT(6)
#define MFRC522_BMS_STATUS1_CRCREADY_BIT                     BIT(5)
#define MFRC522_BMS_STATUS1_IRQ_BIT                          BIT(4)
#define MFRC522_BMS_STATUS1_TRUNNING_BIT                     BIT(3)
#define MFRC522_BMS_STATUS1_HIALERT_BIT                      BIT(1)
#define MFRC522_BMS_STATUS1_LOALERT_BIT                      BIT(0)

#define	MFRC522_BMS_BITFRAMING_STARTSEND_BIT				 BIT(7)
#define	MFRC522_BMS_BITFRAMING_RXALIGN_BITS				 	 BITS(0x07, 4)
#define MFRC522_BMS_BITFRAMING_RXALIGN(x)					 BITS((x), 4)
#define MFRC522_BMS_BITFRAMING_TXLASTBITS_BITS				 BITS(0x07, 0)
#define MFRC522_BMS_BITFRAMING_TXLASTBITS(x)				 BITS((x), 0)

#define MFRC522_BMS_MODE_TXWAITRF_BIT						 BIT(5)

#define MFRC522_BMS_TXMODE_TXCRCEN_BIT						 BIT(7)
#define MFRC522_BMS_TXMODE_TXSPEED_BITS						 BITS(0x07, 4)
#define MFRC522_BMS_TXMODE_TXSPEED_106KBD					 BITS(0x00, 4)
#define MFRC522_BMS_TXMODE_TXSPEED_212KBD					 BITS(0x01, 4)
#define MFRC522_BMS_TXMODE_TXSPEED_424KBD					 BITS(0x02, 4)
#define MFRC522_BMS_TXMODE_TXSPEED_848KBD					 BITS(0x03, 4)
#define MFRC522_BMS_TXMODE_INVMOD_BIT						 BIT(3)

#define MFRC522_BMS_TXCONTROL_INVTX2RFON_BIT				BIT(7)
#define MFRC522_BMS_TXCONTROL_INVTX1RFON_BIT				BIT(6)
#define MFRC522_BMS_TXCONTROL_INVTX2RFOFF_BIT				BIT(5)
#define MFRC522_BMS_TXCONTROL_INVTX1RFOFF_BIT				BIT(4)
#define MFRC522_BMS_TXCONTROL_TX2CW_BIT				BIT(3)
#define MFRC522_BMS_TXCONTROL_TX2RFEN_BIT				BIT(1)
#define MFRC522_BMS_TXCONTROL_TX1RFEN_BIT				BIT(0)

#define MFRC522_BMS_TXASK_FORCE100ASK_BIT				BIT(6)

#define MFRC522_BMS_MFRX_PARITYDISABLE_BIT				BIT(4)

// Other defines
#define MFRC522_TIMER_OPT_AUTO                           (0x01)
#define MFRC522_TIMER_OPT_GATED_MFIN                     (0x02)
#define MFRC522_TIMER_OPT_GATED_AUX1                     (0x04)
#define MFRC522_TIMER_OPT_AUTOLOAD                       (0x08)
#define MFRC522_TIMER_OPT_EVENPRESCALER                  (0x10)
#define MFRC522_TIMER_OPT_INT                            (0x20)

#define MFRC522_ANTENNA_OPT_TX1_MODULATED				 (0x01)
#define MFRC522_ANTENNA_OPT_TX2_MODULATED				 (0x02)
#define MFRC522_ANTENNA_OPT_TX2_UNMODULATED				 (0x04)
#define MFRC522_ANTENNA_OPT_TX1_INVERTED_WHEN_OFF		 (0x08)
#define MFRC522_ANTENNA_OPT_TX2_INVERTED_WHEN_OFF		 (0x10)
#define MFRC522_ANTENNA_OPT_TX1_INVERTED_WHEN_ON		 (0x20)
#define MFRC522_ANTENNA_OPT_TX2_INVERTED_WHEN_ON		 (0x40)

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

void mfrc522_TransmitterSetSpeed (mfrc522_Mod* Dev, mfrc522_Speed speed);
mfrc522_Speed mfrc522_TransmitterGetSpeed (mfrc522_Mod* Dev);

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

    mfrc522_Initialization(Dev);

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

mfrc522_result mfrc522_SelfTest (mfrc522_Mod* Dev)
{
    uint8_t* data;
    uint8_t V1[64] = {0x00, 0xC6, 0x37, 0xD5, 0x32, 0xB7, 0x57, 0x5C,
                      0xC2, 0xD8, 0x7C, 0x4D, 0xD9, 0x70, 0xC7, 0x73,
                      0x10, 0xE6, 0xD2, 0xAA, 0x5E, 0xA1, 0x3E, 0x5A,
                      0x14, 0xAF, 0x30, 0x61, 0xC9, 0x70, 0xDB, 0x2E,
                      0x64, 0x22, 0x72, 0xB5, 0xBD, 0x65, 0xF4, 0xEC,
                      0x22, 0xBC, 0xD3, 0x72, 0x35, 0xCD, 0xAA, 0x41,
                      0x1F, 0xA7, 0xF3, 0x53, 0x14, 0xDE, 0x7E, 0x02,
                      0xD9, 0x0F, 0xB5, 0x5E, 0x25, 0x1D, 0x29, 0x79};
    uint8_t V2[64] = {0x00, 0xEB, 0x66, 0xBA, 0x57, 0xBF, 0x23, 0x95,
                      0xD0, 0xE3, 0x0D, 0x3D, 0x27, 0x89, 0x5C, 0xDE,
                      0x9D, 0x3B, 0xA7, 0x00, 0x21, 0x5B, 0x89, 0x82,
                      0x51, 0x3A, 0xEB, 0x02, 0x0C, 0xA5, 0x00, 0x49,
                      0x7C, 0x84, 0x4D, 0xB3, 0xCC, 0xD2, 0x1B, 0x81,
                      0x5D, 0x48, 0x76, 0xD5, 0x71, 0x61, 0x21, 0xA9,
                      0x86, 0x96, 0x83, 0x38, 0xCF, 0x9D, 0x5B, 0x6D,
                      0xDC, 0x15, 0xBA, 0x3E, 0x7D, 0x95, 0x3B, 0x2F};
    uint8_t temp;
    mfrc522_result cmd = mfrc522_error;

    mfrc522_SoftReset(Dev);

    data = calloc(64, sizeof(uint8_t));
    if (data == NULL)
        return mfrc522_nomem;

    mfrc522_IBWrite(Dev, data);

    mfrc522_ReadAddress(Dev, MFRC522_ADDR_AUTOTEST, &temp);
    temp &= ~(MFRC522_BMS_AUTOTEST_SELFTEST_BITS);
    temp |= MFRC522_BMS_AUTOTEST_SELFTEST_SELFTEST;
    mfrc522_WriteAddress(Dev, MFRC522_ADDR_AUTOTEST, temp);

    mfrc522_FIFOWrite(Dev, data, 1);

    mfrc522_CommandExecute(Dev, CalcCrc);
    while (mfrc522_GetCurrentCommand(Dev) != Idle);

    mfrc522_FIFORead(Dev, data, 64);

    if (!(memcmp(data, V1, 64))) {
        cmd = mfrc522_V2;
    } else if (!(memcmp(data, V2, 64))) {
        cmd = mfrc522_V1;
    }

    free(data);

    return cmd;
}

/*
 * Performs the initialization part per the ISO normative for type A cards.
 */

void mfrc522_Initialization (mfrc522_Mod* Dev)
{
    mfrc522_SoftReset(Dev);

    // Set the timer managed by the protocol, 15 ms time.
    mfrc522_TimerConfigure(Dev, MFRC522_TIMER_OPT_AUTO);
    mfrc522_TimerSetParams(Dev, 3390, 30);

    // Force 100 ASK modulation on PCD to PICC.
    mfrc522_TransmitterForce100ASK(Dev, true);

    // Turn on the Antenna.
    mfrc522_AntennaSettings(Dev, MFRC522_ANTENNA_OPT_TX2_INVERTED_WHEN_ON | MFRC522_ANTENNA_OPT_TX1_MODULATED | MFRC522_ANTENNA_OPT_TX2_MODULATED);

    // Configure transmitter for short frames.
    mfrc522_EnableParity(Dev, false);
    mfrc522_TransmitterSetBits(Dev, 7);

}

/*
 * Enables or disables the transmitter interrupt on the Chip.
 */

void mfrc522_TransmitterIntEnable (mfrc522_Mod* Dev, bool status)
{
	uint8_t temp;

	mfrc522_ReadAddress(Dev, MFRC522_ADDR_COMIEN, &temp);
	if (status)
		temp |= MFRC522_BMS_COMIEN_TXIEN_BIT;
	else
		temp &= ~MFRC522_BMS_COMIEN_TXIEN_BIT;
	mfrc522_WriteAddress(Dev, MFRC522_ADDR_COMIEN, temp);
}

/*
 * Checks if the last bit of the transmitted data was sent out.
 */

bool mfrc522_TransmitterIsFinished (mfrc522_Mod* Dev)
{
	uint8_t temp;

	mfrc522_ReadAddress(Dev, MFRC522_ADDR_COMIRQ, &temp);
	if (temp & MFRC522_BMS_COMIRQ_TXIRQ_BIT)
		return true;

	return false;
}

/*
 * Defines the number of bits of the last byte that will be transmitted. Meant to be used with bit-oriented frames.
 *
 * A value of zero indicates all the bits of the last byte will be transmitted.
 */

void mfrc522_TransmitterSetBits (mfrc522_Mod* Dev, uint8_t nBits)
{
	uint8_t temp;

	mfrc522_ReadAddress(Dev, MFRC522_ADDR_BITFRAMING, &temp);
	temp &= ~(MFRC522_BMS_BITFRAMING_TXLASTBITS_BITS);
	temp |= MFRC522_BMS_BITFRAMING_TXLASTBITS((nBits & 0x07));
	mfrc522_WriteAddress(Dev, MFRC522_ADDR_BITFRAMING, temp);
}

/*
 * If enabled, transmitter can only be started if an RF field is generated.
 */

void mfrc522_TransmitterWaitsRF (mfrc522_Mod* Dev, bool status)
{
	uint8_t temp;

	mfrc522_ReadAddress(Dev, MFRC522_ADDR_MODE, &temp);
	if (status)
		temp |= MFRC522_BMS_MODE_TXWAITRF_BIT;
	else
		temp &= ~(MFRC522_BMS_MODE_TXWAITRF_BIT);
	mfrc522_WriteAddress(Dev, MFRC522_ADDR_MODE, temp);
}

/*
 * Enables or disables CRC generation during transmission.
 */

void mfrc522_TransmitterEnableCRC (mfrc522_Mod* Dev, bool status)
{
	uint8_t temp;

	mfrc522_ReadAddress(Dev, MFRC522_ADDR_TXMODE, &temp);
	if (status)
		temp |= MFRC522_BMS_TXMODE_TXCRCEN_BIT;
	else
		temp &= ~(MFRC522_BMS_TXMODE_TXCRCEN_BIT);
	mfrc522_WriteAddress(Dev, MFRC522_ADDR_TXMODE, temp);
}

/*
 * Sets the speed for the transmitter.
 */

void mfrc522_TransmitterSetSpeed (mfrc522_Mod* Dev, mfrc522_Speed speed)
{
	uint8_t temp;

	mfrc522_ReadAddress(Dev, MFRC522_ADDR_TXMODE, &temp);
	temp &= ~(MFRC522_BMS_TXMODE_TXSPEED_BITS);
	switch (speed)
	{
	case Speed_106kBd:
		temp |= MFRC522_BMS_TXMODE_TXSPEED_106KBD;
		break;
	case Speed_212kBd:
		temp |= MFRC522_BMS_TXMODE_TXSPEED_212KBD;
		break;
	case Speed_424kBd:
		temp |= MFRC522_BMS_TXMODE_TXSPEED_424KBD;
		break;
	case Speed_848kBd:
		temp |= MFRC522_BMS_TXMODE_TXSPEED_848KBD;
		break;
	default:
		break;
	}
	mfrc522_WriteAddress(Dev, MFRC522_ADDR_TXMODE, temp);
}

/*
 * Get the speed of the transmitter.
 */

mfrc522_Speed mfrc522_TransmitterGetSpeed (mfrc522_Mod* Dev)
{
	uint8_t temp;
	mfrc522_Speed speed;

	mfrc522_ReadAddress(Dev, MFRC522_ADDR_TXMODE, &temp);
	temp &= MFRC522_BMS_TXMODE_TXSPEED_BITS;
	switch (temp)
	{
	case MFRC522_BMS_TXMODE_TXSPEED_106KBD:
		speed = Speed_106kBd;
		break;
	case MFRC522_BMS_TXMODE_TXSPEED_212KBD:
		speed = Speed_212kBd;
		break;
	case MFRC522_BMS_TXMODE_TXSPEED_424KBD:
		speed = Speed_424kBd;
		break;
	case MFRC522_BMS_TXMODE_TXSPEED_848KBD:
		speed = Speed_848kBd;
		break;
	default:
		break;
	}

	return speed;
}

/*
 * Forces 100% ASK Modulation independent of other settings.
 */

void mfrc522_TransmitterForce100ASK (mfrc522_Mod* Dev, bool status)
{
	uint8_t temp;

	mfrc522_ReadAddress(Dev, MFRC522_ADDR_TXASK, &temp);
	if (status)
		temp |= MFRC522_BMS_TXASK_FORCE100ASK_BIT;
	else
		temp &= ~(MFRC522_BMS_TXASK_FORCE100ASK_BIT);
	mfrc522_WriteAddress(Dev, MFRC522_ADDR_TXMODE, temp);
}

/*
 * Controls the behaviour of the pins TX1 and TX2 that drive the antenna.
 *
 * Options is an 8 bit packed variable with the following information, from right to left:
 *
 * [0]: TxControl Tx1RFEn bit
 * [1]: TxControl Tx2RFEn bit
 * [2]: TxControl Tx2CW bit
 * [3]: TxControl InvTx1RFOff bit
 * [4]: TxControl InvTx2RFOff bit
 * [5]: TxControl InvTx1RFOn bit
 * [6]: TxControl InvTx2RFOn bit
 *
 * Use a logical OR of MFRC522_ANTENNA_OPTx defines.
 */

void mfrc522_AntennaSettings (mfrc522_Mod* Dev, uint8_t options)
{
	uint8_t temp;

	mfrc522_ReadAddress(Dev, MFRC522_ADDR_TXCONTROL, &temp);


	if (options & MFRC522_ANTENNA_OPT_TX1_MODULATED)
		temp |= MFRC522_BMS_TXCONTROL_TX1RFEN_BIT;
	else
		temp &= ~(MFRC522_BMS_TXCONTROL_TX1RFEN_BIT);

	if (options & MFRC522_ANTENNA_OPT_TX2_MODULATED)
		temp |= MFRC522_BMS_TXCONTROL_TX2RFEN_BIT;
	else
		temp &= ~(MFRC522_BMS_TXCONTROL_TX2RFEN_BIT);

	if (options & MFRC522_ANTENNA_OPT_TX2_UNMODULATED)
		temp |= MFRC522_BMS_TXCONTROL_TX2CW_BIT;
	else
		temp &= ~(MFRC522_BMS_TXCONTROL_TX2CW_BIT);

	if (options & MFRC522_ANTENNA_OPT_TX1_INVERTED_WHEN_OFF)
		temp |= MFRC522_BMS_TXCONTROL_INVTX1RFOFF_BIT;
	else
		temp &= ~(MFRC522_BMS_TXCONTROL_INVTX1RFOFF_BIT);

	if (options & MFRC522_ANTENNA_OPT_TX2_INVERTED_WHEN_OFF)
		temp |= MFRC522_BMS_TXCONTROL_INVTX2RFOFF_BIT;
	else
		temp &= ~(MFRC522_BMS_TXCONTROL_INVTX2RFOFF_BIT);

	if (options & MFRC522_ANTENNA_OPT_TX1_INVERTED_WHEN_ON)
		temp |= MFRC522_BMS_TXCONTROL_INVTX1RFON_BIT;
	else
		temp &= ~(MFRC522_BMS_TXCONTROL_INVTX1RFON_BIT);

	if (options & MFRC522_ANTENNA_OPT_TX2_INVERTED_WHEN_ON)
		temp |= MFRC522_BMS_TXCONTROL_INVTX2RFON_BIT;
	else
		temp &= ~(MFRC522_BMS_TXCONTROL_INVTX2RFON_BIT);

	mfrc522_WriteAddress(Dev, MFRC522_ADDR_TXCONTROL, temp);
}

/*
 * Sets if the parity bit for transmission and parity check for reception should be active or not.
 */

void mfrc522_EnableParity (mfrc522_Mod* Dev, bool status)
{
	uint8_t temp;

	mfrc522_ReadAddress(Dev, MFRC522_ADDR_MFRX, &temp);
	if (!(status))
		temp |= MFRC522_BMS_MFRX_PARITYDISABLE_BIT;
	else
		temp &= ~(MFRC522_BMS_MFRX_PARITYDISABLE_BIT);
	mfrc522_WriteAddress(Dev, MFRC522_ADDR_MFRX, temp);
}

/*
 * Configures the timer.
 *
 * Options is a 16 bit packed variable with the following format, from right to left:
 *
 * [0]: TMode TAuto bit.
 * [1-2]: TMode TGated bits.
 * [3]: TMode TAutoRestart.
 * [4]: Demod TPrescalEven.
 * [5]: ComIEnReg TimerIEn.
 *
 * Use MFRC522_TIMER_OPTx defines to define the options.
 */

void mfrc522_TimerConfigure (mfrc522_Mod* Dev, uint16_t options)
{
    uint8_t temp;

    mfrc522_ReadAddress(Dev, MFRC522_ADDR_TMODE, &temp);

    temp &= ~(MFRC522_BMS_TMODE_TAUTO_BIT);
    if (options & MFRC522_TIMER_OPT_AUTO)
        temp |= MFRC522_BMS_TMODE_TAUTO_BIT;

    temp &= ~(MFRC522_BMS_TMODE_TGATED_BITS);
    if (options & MFRC522_TIMER_OPT_GATED_AUX1)
        temp |= MFRC522_BMS_TMODE_TGATED_AUX1;
    else if (options & MFRC522_TIMER_OPT_GATED_MFIN)
        temp |= MFRC522_BMS_TMODE_TGATED_MFIN;

    temp &= ~(MFRC522_BMS_TMODE_TAUTORESTART_BIT);
    if (options & MFRC522_TIMER_OPT_AUTOLOAD)
        temp |= (MFRC522_BMS_TMODE_TAUTORESTART_BIT);

    mfrc522_WriteAddress(Dev, MFRC522_ADDR_TMODE, temp);

    mfrc522_ReadAddress(Dev, MFRC522_ADDR_DEMOD, &temp);

    temp &= ~(MFRC522_BMS_DEMOD_TPRESCALEVEN_BIT);
    if (options & MFRC522_TIMER_OPT_EVENPRESCALER)
        temp |= MFRC522_BMS_DEMOD_TPRESCALEVEN_BIT;

    mfrc522_WriteAddress(Dev, MFRC522_ADDR_DEMOD, temp);

    mfrc522_ReadAddress(Dev, MFRC522_ADDR_COMIEN, &temp);

    temp &= ~(MFRC522_BMS_COMIEN_TIMERIEN_BIT);
    if (options & MFRC522_TIMER_OPT_EVENPRESCALER)
        temp |= MFRC522_BMS_COMIEN_TIMERIEN_BIT;

    mfrc522_WriteAddress(Dev, MFRC522_ADDR_COMIEN, temp);
}

/*
 * Sets the prescaler and reload time on the timer.
 *
 * Prescaler: From 0 to 4095, inclusive.
 * ReloadVal: From 0 to 65535, inclusive.
 *
 * Delay in seconds, without even prescaler: ((Prescaler * 2 + 1) * (ReloadVal + 1)) / (13560000)
 * Delay in secods, with even prescaler: ((Prescaler * 2 + 2) * (ReloadVal + 1)) / (13560000)
 */

void mfrc522_TimerSetParams (mfrc522_Mod* Dev, uint16_t prescaler, uint16_t reloadval)
{
    uint8_t temp;

    mfrc522_ReadAddress(Dev, MFRC522_ADDR_TMODE, &temp);

    temp &= ~(MFRC522_BMS_TMODE_TPRESCALERHI_BITS);
    temp |= MFRC522_BMS_TMODE_TPRESCALERHI((prescaler & 0x0F00) >> 8);

    mfrc522_WriteAddress(Dev, MFRC522_ADDR_TMODE, temp);
    mfrc522_WriteAddress(Dev, MFRC522_ADDR_TPRESCALERLO, prescaler & 0x00FF);
    mfrc522_WriteAddress(Dev, MFRC522_ADDR_TRELOADHI, (reloadval & 0xFF00) >> 8);
    mfrc522_WriteAddress(Dev, MFRC522_ADDR_TRELOADLO, reloadval & 0x00FF);
}

/*
 * Gets the current value of the timer.
 */

uint16_t mfrc522_TimerGetValue (mfrc522_Mod* Dev)
{
    uint8_t Hi, Lo;
    uint16_t Count;

    mfrc522_ReadAddress(Dev, MFRC522_ADDR_TCOUNTERVALHI, &Hi);
    mfrc522_ReadAddress(Dev, MFRC522_ADDR_TCOUNTERVALLO, &Lo);

    Count = (Hi << 8) | Lo;

    return Count;
}

/*
 * Checks if the timer reached zero.
 */

bool mfrc522_TimerIsFinished (mfrc522_Mod* Dev)
{
    uint8_t temp;

    mfrc522_ReadAddress(Dev, MFRC522_ADDR_COMIRQ, &temp);

    if (temp & MFRC522_BMS_COMIRQ_TIMERIRQ_BIT)
    {
        // TODO: CLEAR THE INTERRUPT IF SET.
        return true;
    }

    return false;
}

/*
 * Checks if the timer is running
 */

bool mfrc522_TimerIsRunning (mfrc522_Mod* Dev)
{
    uint8_t temp;

    mfrc522_ReadAddress(Dev, MFRC522_ADDR_STATUS1, &temp);

    if (temp & MFRC522_BMS_STATUS1_TRUNNING_BIT)
    {
        return true;
    }

    return false;
}


/*
 * Starts the timer immediately.
 */

void mfrc522_TimerStart (mfrc522_Mod* Dev)
{
    uint8_t temp;

    mfrc522_ReadAddress(Dev, MFRC522_ADDR_CONTROL, &temp);
    temp |= MFRC522_BMS_CONTROL_TSTARTNOW_BIT;
    mfrc522_WriteAddress(Dev, MFRC522_ADDR_CONTROL, temp);
}

/*
 * Stops the timer immediately.
 */

void mfrc522_TimerStop (mfrc522_Mod* Dev)
{
    uint8_t temp;

    mfrc522_ReadAddress(Dev, MFRC522_ADDR_CONTROL, &temp);
    temp |= MFRC522_BMS_CONTROL_TSTOPNOW_BIT;
    mfrc522_WriteAddress(Dev, MFRC522_ADDR_CONTROL, temp);
}

/*
 * Writes 25 bytes from the buffer into the internal memory of the chip.
 */

void mfrc522_IBWrite (mfrc522_Mod* Dev, uint8_t* buffer)
{
    mfrc522_FIFOClear(Dev);
    mfrc522_FIFOWrite(Dev, buffer, 25);
    mfrc522_CommandExecute(Dev, Mem);
    while (mfrc522_GetCurrentCommand(Dev) != Idle);
}

/*
 * Reads the internal data from the buffer.
 */

void mfrc522_IBRead (mfrc522_Mod* Dev, uint8_t* buffer)
{
    mfrc522_FIFOClear(Dev);
    mfrc522_CommandExecute(Dev, Mem);
    while (mfrc522_GetCurrentCommand(Dev) != Idle);
    mfrc522_FIFORead(Dev, buffer, 25);
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
