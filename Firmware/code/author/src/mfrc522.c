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
#include <stddef.h>
#include <string.h>

#include "mfrc522.h"
#include "conf.h"

/* Private typedef -----------------------------------------------------------*/
typedef enum
{
    Command_Idle,
    Command_Mem,
    Command_GenerateRandomID,
    Command_CalcCrc,
    Command_Transmit,
    Command_NoCmdChange,
    Command_Receive,
    Command_Transceive,
    Command_MFAuthent,
    Command_SoftReset,
} mfrc522_Command;

typedef enum
{
	Speed_106kBd,
	Speed_212kBd,
	Speed_424kBd,
	Speed_848kBd,
} mfrc522_Speed;

typedef enum
{
    Frame_Short,
    Frame_Standard,
    Frame_Anticollision,
} mfrc522_Frame;

typedef enum
{
	Modem_Idle,
	Modem_WaitForStartSend,
	Modem_TransmitterWaitingForRFField,
	Modem_Transmitting,
	Modem_ReceiverWaitingForRFField,
	Modem_WaitForData,
	Modem_Receiving,
	Modem_Unknown,
} mfrc522_Modem;

typedef enum
{
    MFCommand_ReqA = 0x26,
    MFCommand_WupA = 0x52,
    MFCommand_SelCL1 = 0x93,
    MFCommand_SelCL2 = 0x95,
    MFCommand_SelCL3 = 0x97,
} mfrc522_MFCommand;

/* Private define ------------------------------------------------------------*/
#define MFRC522_GPIO_LOW      0x00
#define MFRC522_GPIO_HIGH     0x01

#define MFRC522_READ_MSB    0x80
#define MFRC522_WRITE_MSB   0x00

// Commands
#define MFRC522_CMD_SOFTRESET       (0x0F)

// Register Addresses
#define MFRC522_ADDR_COMMAND        (0x01)
#define MFRC522_ADDR_COMIEN         (0x02)
#define MFRC522_ADDR_DIVIEN         (0x03)
#define MFRC522_ADDR_COMIRQ         (0x04)
#define MFRC522_ADDR_DIVIRQ         (0x05)
#define MFRC522_ADDR_ERROR          (0x06)
#define MFRC522_ADDR_STATUS1        (0x07)
#define MFRC522_ADDR_STATUS2		(0x08)
#define MFRC522_ADDR_FIFODATA       (0x09)
#define MFRC522_ADDR_FIFOLEVEL      (0x0A)
#define MFRC522_ADDR_CONTROL        (0x0C)
#define	MFRC522_ADDR_BITFRAMING		(0x0D)
#define MFRC522_ADDR_COLL			(0x0E)
#define MFRC522_ADDR_MODE			(0x11)
#define	MFRC522_ADDR_TXMODE			(0x12)
#define MFRC522_ADDR_RXMODE			(0x13)
#define MFRC522_ADDR_TXCONTROL		(0x14)
#define MFRC522_ADDR_TXASK			(0x15)
#define MFRC522_ADDR_DEMOD          (0x19)
#define MFRC522_ADDR_MFRX			(0x1D)
#define MFRC522_ADDR_CRCRESULTMSB   (0x21)
#define MFRC522_ADDR_CRCRESULTLSB   (0x22)
#define MFRC522_ADDR_RFCFG			(0x26)
#define MFRC522_ADDR_TMODE          (0x2A)
#define MFRC522_ADDR_TPRESCALERLO   (0x2B)
#define MFRC522_ADDR_TRELOADHI      (0x2C)
#define MFRC522_ADDR_TRELOADLO      (0x2D)
#define MFRC522_ADDR_TCOUNTERVALHI  (0x2E)
#define MFRC522_ADDR_TCOUNTERVALLO  (0x2F)
#define MFRC522_ADDR_AUTOTEST       (0x36)
#define MFRC522_ADDR_VERSION        (0x37)

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

#define MFRC522_BMS_DIVIRQ_SET2_BIT                     BIT(7)
#define MFRC522_BMS_DIVIRQ_MFINACTIRQ_BIT               BIT(4)
#define MFRC522_BMS_DIVIRQ_CRCIRQ_BIT                   BIT(2)

#define MFRC522_BMS_DIVIEN_IRQPUSHPULL_BIT              BIT(7)
#define MFRC522_BMS_DIVIEN_MFINACTIEN_BIT               BIT(4)
#define MFRC522_BMS_DIVIEN_CRCIEN_BIT                   BIT(2)

#define MFRC522_BMS_CONTROL_TSTOPNOW_BIT                 BIT(7)
#define MFRC522_BMS_CONTROL_TSTARTNOW_BIT                BIT(6)
#define MFRC522_BMS_CONTROL_RXLASTBITS_BITS              BITS(0x07, 0)

#define MFRC522_BMS_ERROR_WRERR_BIT         BIT(7)
#define MFRC522_BMS_ERROR_TEMPERR_BIT       BIT(6)
#define MFRC522_BMS_ERROR_BUFFEROVFL_BIT    BIT(4)
#define MFRC522_BMS_ERROR_COLLERR_BIT       BIT(3)
#define MFRC522_BMS_ERROR_CRCERR_BIT        BIT(2)
#define MFRC522_BMS_ERROR_PARITYERR_BIT     BIT(1)
#define MFRC522_BMS_ERROR_PROTOCOLERR_BIT   BIT(0)

#define MFRC522_BMS_STATUS1_CRCOK_BIT                        BIT(6)
#define MFRC522_BMS_STATUS1_CRCREADY_BIT                     BIT(5)
#define MFRC522_BMS_STATUS1_IRQ_BIT                          BIT(4)
#define MFRC522_BMS_STATUS1_TRUNNING_BIT                     BIT(3)
#define MFRC522_BMS_STATUS1_HIALERT_BIT                      BIT(1)
#define MFRC522_BMS_STATUS1_LOALERT_BIT                      BIT(0)

#define MFRC522_BMS_STATUS2_MODEMSTATE_BITS					BITS(0x07, 0)
#define MFRC522_BMS_STATUS2_MODEMSTATE_IDLE					BITS(0x00, 0)
#define MFRC522_BMS_STATUS2_MODEMSTATE_WAIT_STARTSEND		BITS(0x01, 0)
#define MFRC522_BMS_STATUS2_MODEMSTATE_TX_WAIT_RF			BITS(0x02, 0)
#define MFRC522_BMS_STATUS2_MODEMSTATE_TX_TRANSMITTING		BITS(0x03, 0)
#define MFRC522_BMS_STATUS2_MODEMSTATE_RX_WAIT_RF			BITS(0x04, 0)
#define MFRC522_BMS_STATUS2_MODEMSTATE_WAIT_FOR_DATA		BITS(0x05, 0)
#define MFRC522_BMS_STATUS2_MODEMSTATE_RECEIVING			BITS(0x06, 0)

#define	MFRC522_BMS_BITFRAMING_STARTSEND_BIT				 BIT(7)
#define	MFRC522_BMS_BITFRAMING_RXALIGN_BITS				 	 BITS(0x07, 4)
#define MFRC522_BMS_BITFRAMING_RXALIGN(x)					 BITS((x), 4)
#define MFRC522_BMS_BITFRAMING_TXLASTBITS_BITS				 BITS(0x07, 0)
#define MFRC522_BMS_BITFRAMING_TXLASTBITS(x)				 BITS((x), 0)

#define MFRC522_BMS_MODE_TXWAITRF_BIT						 BIT(5)
#define MFRC522_BMS_MODE_MSBFIRST_BIT						 BIT(7)
#define MFRC522_BMS_MODE_CRCPRESET_BITS						 BITS(0x03, 0)
#define MFRC522_BMS_MODE_CRCPRESET_0000						 BITS(0x00, 0)
#define MFRC522_BMS_MODE_CRCPRESET_6363						 BITS(0x01, 0)
#define MFRC522_BMS_MODE_CRCPRESET_A671						 BITS(0x02, 0)
#define MFRC522_BMS_MODE_CRCPRESET_FFFF						 BITS(0x03, 0)

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

#define MFRC522_BMS_VERSION_VERSION1                    (0x91)
#define MFRC522_BMS_VERSION_VERSION2                    (0x92)

#define MFRC522_BMS_COLL_VALUESAFTERCOLL_BIT			BIT(7)
#define MFRC522_BMS_COLL_COLLPOSNOTVALID_BIT			BIT(5)
#define	MFRC522_BMS_COLL_COLLPOS_BITS					BITS(0x1F, 0)

#define MFRC522_BMS_RXMODE_RXCRCEN_BIT					BIT(7)
#define MFRC522_BMS_RXMODE_RXSPEED_BITS					BITS(0x07, 4)
#define MFRC522_BMS_RXMODE_RXSPEED_106KBD				BITS(0x00, 4)
#define MFRC522_BMS_RXMODE_RXSPEED_212KBD				BITS(0x01, 4)
#define MFRC522_BMS_RXMODE_RXSPEED_424KBD				BITS(0x02, 4)
#define MFRC522_BMS_RXMODE_RXSPEED_848KBD				BITS(0x03, 4)
#define MFRC522_BMS_RXMODE_RXNOERR_BIT					BIT(3)
#define MFRC522_BMS_RXMODE_RXMULTIPLE_BIT				BIT(2)

#define MFRC522_BMS_RFCFG_RXGAIN_BITS					BITS(0x07, 4)
#define MFRC522_BMS_RFCFG_RXGAIN_18DB					BITS(0x02, 4)
#define MFRC522_BMS_RFCFG_RXGAIN_23DB					BITS(0x03, 4)
#define MFRC522_BMS_RFCFG_RXGAIN_33DB					BITS(0x04, 4)
#define MFRC522_BMS_RFCFG_RXGAIN_38DB					BITS(0x05, 4)
#define MFRC522_BMS_RFCFG_RXGAIN_43DB					BITS(0x06, 4)
#define MFRC522_BMS_RFCFG_RXGAIN_48DB					BITS(0x07, 4)

// Other defines
#define MFRC522_TIMER_OPT_AUTO                           (0x01)
#define MFRC522_TIMER_OPT_GATED_MFIN                     (0x02)
#define MFRC522_TIMER_OPT_GATED_AUX1                     (0x04)
#define MFRC522_TIMER_OPT_AUTOLOAD                       (0x08)
#define MFRC522_TIMER_OPT_EVENPRESCALER                  (0x10)

#define MFRC522_ANTENNA_OPT_TX1_MODULATED				 (0x01)
#define MFRC522_ANTENNA_OPT_TX2_MODULATED				 (0x02)
#define MFRC522_ANTENNA_OPT_TX2_UNMODULATED				 (0x04)
#define MFRC522_ANTENNA_OPT_TX1_INVERTED_WHEN_OFF		 (0x08)
#define MFRC522_ANTENNA_OPT_TX2_INVERTED_WHEN_OFF		 (0x10)
#define MFRC522_ANTENNA_OPT_TX1_INVERTED_WHEN_ON		 (0x20)
#define MFRC522_ANTENNA_OPT_TX2_INVERTED_WHEN_ON		 (0x40)

#define MFRC522_INT_TX                  (0x001)
#define MFRC522_INT_RX                  (0x002)
#define MFRC522_INT_IDLE                (0x004)
#define MFRC522_INT_HIALERT             (0x008)
#define MFRC522_INT_LOALERT             (0x010)
#define MFRC522_INT_ERR                 (0x020)
#define MFRC522_INT_TIMER               (0x040)
#define MFRC522_INT_MFINACT             (0x080)
#define MFRC522_INT_CRC                 (0x100)
#define MFRC522_INT_ALLINTS             (0x1FF)

#define MFRC522_ERROR_FIFOWRITE         (0x01)
#define MFRC522_ERROR_OVERHEAT          (0x02)
#define MFRC522_ERROR_BUFFEROVERFLOW    (0x04)
#define MFRC522_ERROR_COLLISION         (0x08)
#define MFRC522_ERROR_CRC               (0x10)
#define MFRC522_ERROR_PARITY            (0x20)
#define MFRC522_ERROR_PROTOCOL          (0x40)

#define MFRC522_RXGAIN_18DB				(MFRC522_BMS_RFCFG_RXGAIN_18DB)
#define MFRC522_RXGAIN_23DB				(MFRC522_BMS_RFCFG_RXGAIN_23DB)
#define MFRC522_RXGAIN_33DB				(MFRC522_BMS_RFCFG_RXGAIN_33DB)
#define MFRC522_RXGAIN_38DB				(MFRC522_BMS_RFCFG_RXGAIN_38DB)
#define MFRC522_RXGAIN_43DB				(MFRC522_BMS_RFCFG_RXGAIN_43DB)
#define MFRC522_RXGAIN_48DB				(MFRC522_BMS_RFCFG_RXGAIN_48DB)

#define MFRC522_STATUS_CRC_OK			(0x01)
#define MFRC522_STATUS_CRC_READY		(0x02)
#define MFRC522_STATUS_FIFO_HIGH		(0x04)
#define MFRC522_STATUS_FIFO_LOW			(0x08)
#define MFRC522_STATUS_IRQ_ON			(0x10)
#define MFRC522_STATUS_TIMER_ON			(0x20)

#define MFRC522_CRC_MSBFIRST         (0x01)
#define MFRC522_CRC_PRESET_0000      (0x02)
#define MFRC522_CRC_PRESET_6363      (0x04)
#define MFRC522_CRC_PRESET_A671      (0x08)
#define MFRC522_CRC_PRESET_FFFF      (0x10)

/* Private macro -------------------------------------------------------------*/
#define BIT(n)          (1 << (n))
#define BITS(x, n)      ((x) << (n))

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
static uint8_t mfrc522_Init (mfrc522_Class* class);
static uint8_t mfrc522_SelfTest (mfrc522_Class* class);
static void mfrc522_ReadRegister (mfrc522_Class* class, uint8_t address, uint8_t* value);
static void mfrc522_WriteRegister (mfrc522_Class* class, uint8_t address, uint8_t value);
static void mfrc522_HardReset (mfrc522_Class* class);
static void mfrc522_SoftReset (mfrc522_Class* class);
static mfrc522_Command mfrc522_CommandCurrent (mfrc522_Class* class);
static void mfrc522_CommandExecute (mfrc522_Class* class, mfrc522_Command cmd);
static void mfrc522_ReceiverStatus (mfrc522_Class* class, uint8_t MFRC522_STATUSx);
static mfrc522_Modem mfrc522_ModemCheck (mfrc522_Class* class);
static void mfrc522_IntStatus (mfrc522_Class* class, uint16_t MFRC522_INTx, uint8_t MFRC522_STATUSx);
static uint16_t mfrc522_IntGet (mfrc522_Class* class);
static void mfrc522_IntClear (mfrc522_Class* class, uint16_t MFRC522_INTx);
static uint8_t mfrc522_ErrorGet (mfrc522_Class* class);
static void mfrc522_TransmitterSetBits (mfrc522_Class* class, uint8_t nBits);
static void mfrc522_TransmitterWaitForRF (mfrc522_Class* class, uint8_t MFRC522_STATUSx);
static void mfrc522_TransmitterCrcStatus (mfrc522_Class* class, uint8_t MFRC522_STATUSx);
static void mfrc522_TransmitterSpeedSet (mfrc522_Class* class, mfrc522_Speed speed);
static mfrc522_Speed mfrc522_TransmitterSpeedGet (mfrc522_Class* class);
static void mfrc522_TransmitterModulation100ASK (mfrc522_Class* class, uint8_t MFRC522_STATUSx);
static void mfrc522_TransmitterStart (mfrc522_Class* class);
static void mfrc522_AntennaConfig (mfrc522_Class* class, uint8_t MFRC522_ANTENNA_OPTx);
static void mfrc522_ParityStatus (mfrc522_Class* class, uint8_t MFRC522_STATUSx);
static void mfrc522_TimerConfig (mfrc522_Class* class, uint16_t MFRC522_TIMER_OPTx);
static void mfrc522_TimerValues (mfrc522_Class* class, uint16_t prescaler, uint16_t reloadval);
static uint16_t mfrc522_TimerCounterGet (mfrc522_Class* class);
static uint8_t mfrc522_TimerIsRunning (mfrc522_Class* class);
static void mfrc522_TimerStart (mfrc522_Class* class);
static void mfrc522_TimerStop (mfrc522_Class* class);
static void mfrc522_InternalBufferWrite (mfrc522_Class* class, uint8_t* buffer);
static void mfrc522_InternalBufferRead (mfrc522_Class* class, uint8_t* buffer);
static void mfrc522_FIFOWrite (mfrc522_Class* class, uint8_t* buffer, uint8_t bytestowrite);
static void mfrc522_FIFOClear (mfrc522_Class* class);
static uint8_t mfrc522_FIFOLevelGet (mfrc522_Class* class);
static uint8_t mfrc522_FIFORead (mfrc522_Class* class, uint8_t* buffer, uint8_t bytestoread);
static uint8_t mfrc522_FindTags (mfrc522_Class* class);
static void mfrc522_TransmitterSetFrame (mfrc522_Class* class, mfrc522_Frame frame);
static uint8_t mfrc522_CommandTransceive (mfrc522_Class* class, uint8_t* TxData, uint8_t TxDataLength, uint8_t* RxData, uint8_t* RxDataLength);
static uint8_t mfrc522_ReceiverGetBits (mfrc522_Class* class);
static uint8_t mfrc522_ReceiverGetCollision (mfrc522_Class* class);
static void mfrc522_CRCConfig (mfrc522_Class* class, uint8_t MFRC522_CRCx);
static uint8_t mfrc522_StatusGet (mfrc522_Class* class);
static void mfrc522_ReceiverCRCStatus (mfrc522_Class* class, uint8_t MFRC522_STATUSx);
static mfrc522_Speed mfrc522_ReceiverGetSpeed (mfrc522_Class* class);
static void mfrc522_ReceiverSetSpeed (mfrc522_Class* class, mfrc522_Speed speed);
static uint8_t mfrc522_ReceiverGetCollision (mfrc522_Class* class);
static uint8_t mfrc522_ReceiverGetBits (mfrc522_Class* class);
static void mfrc522_ReceiverContinous (mfrc522_Class* class, uint8_t MFRC522_STATUSx);
static void mfrc522_ReceiverIgnoreInvalid (mfrc522_Class* class, uint8_t MFRC522_STATUSx);
static void mfrc522_ReceiverAntennaGain (mfrc522_Class* class, uint8_t MFRC522_RXGAINx);
static void mfrc522_CRCCalculate (mfrc522_Class* class, uint8_t* data, uint8_t dataLength, uint8_t* recv);

/* Private functions ---------------------------------------------------------*/

/*
 * Create class.
 */

mfrc522_Class* mfrc522_CreateClass (void (*Delay) (double), uint8_t (*HwInit) (void), void (*ResetControl) (uint8_t), void (*ReadRegister) (uint8_t, uint8_t*), void (*WriteRegister) (uint8_t, uint8_t))
{
    mfrc522_Class* class = malloc(sizeof(mfrc522_Class));
    if (class == NULL)
        return NULL;

    class->Init = mfrc522_Init;
    class->SelfTest = mfrc522_SelfTest;
    class->FindTags = mfrc522_FindTags;

    class->__Delay = Delay;
    class->__HwInit = HwInit;
    class->__ResetControl = ResetControl;
    class->__ReadRegister = ReadRegister;
    class->__WriteRegister = WriteRegister;

    return class;
}


/*
 * Destroy class.
 */

void mfrc522_DestroyClass (mfrc522_Class* class)
{
    free(class);
}

/*
 * Initializes the mfrc522 chip and performs a hard reset.
 */

static uint8_t mfrc522_Init (mfrc522_Class* class)
{
    uint8_t result = MFRC522_STATUS_OFF;

    if (class->__HwInit())
    {
        mfrc522_HardReset(class);

        result = MFRC522_STATUS_ON;
    }

    return result;
}

/*
 * Reads the value at address.
 */

static void mfrc522_ReadRegister (mfrc522_Class* class, uint8_t address, uint8_t* value)
{
    class->__ReadRegister(MFRC522_READ_MSB | (address << 1), value);
}

/*
 * Writes the value at address.
 */

static void mfrc522_WriteRegister (mfrc522_Class* class, uint8_t address, uint8_t value)
{
    class->__WriteRegister(MFRC522_WRITE_MSB | (address << 1), value);
}

/*
 * Performs a hard reset.
 */

static void mfrc522_HardReset (mfrc522_Class* class)
{
    class->__Delay(0.1);
    class->__ResetControl(MFRC522_GPIO_LOW);
    class->__Delay(0.1);
    class->__ResetControl(MFRC522_GPIO_HIGH);
    class->__Delay(0.1);
}

/*
 * Stops the current command and executes a soft reset.
 */

static void mfrc522_SoftReset (mfrc522_Class* class)
{
    mfrc522_CommandExecute(class, Command_SoftReset);
    class->__Delay(0.1);
    while (mfrc522_CommandCurrent(class) != Command_Idle);
}

/*
 * Returns the current command being executed.
 */

static mfrc522_Command mfrc522_CommandCurrent (mfrc522_Class* class)
{
    uint8_t temp;
    mfrc522_Command cmd;

    mfrc522_ReadRegister(class, MFRC522_ADDR_COMMAND, &temp);

    temp &= MFRC522_BMS_COMMAND_COMMAND_BITS;

    switch (temp)
    {
    case MFRC522_BMS_COMMAND_COMMAND_IDLE:
        cmd = Command_Idle;
        break;
    case MFRC522_BMS_COMMAND_COMMAND_MEM:
        cmd = Command_Mem;
        break;
    case MFRC522_BMS_COMMAND_COMMAND_GENRANDOMID:
        cmd = Command_GenerateRandomID;
        break;
    case MFRC522_BMS_COMMAND_COMMAND_CALCCRC:
        cmd = Command_CalcCrc;
        break;
    case MFRC522_BMS_COMMAND_COMMAND_TRANSMIT:
        cmd = Command_Transmit;
        break;
    case MFRC522_BMS_COMMAND_COMMAND_NOCMDCHANGE:
        cmd = Command_NoCmdChange;
        break;
    case MFRC522_BMS_COMMAND_COMMAND_RECEIVE:
        cmd = Command_Receive;
        break;
    case MFRC522_BMS_COMMAND_COMMAND_TRANSCEIVE:
        cmd = Command_Transceive;
        break;
    case MFRC522_BMS_COMMAND_COMMAND_MFAUTHENT:
        cmd = Command_MFAuthent;
        break;
    case MFRC522_BMS_COMMAND_COMMAND_SOFTRESET:
        cmd = Command_SoftReset;
        break;
    default:
        break;
    }

    return cmd;
}

/*
 * Executes the specified command.
 */

static void mfrc522_CommandExecute (mfrc522_Class* class, mfrc522_Command cmd)
{
    uint8_t temp;

    mfrc522_ReadRegister(class, MFRC522_ADDR_COMMAND, &temp);

    temp &= ~(MFRC522_BMS_COMMAND_COMMAND_BITS);

    switch (cmd)
    {
    case Command_Idle:
        temp |= MFRC522_BMS_COMMAND_COMMAND_IDLE;
        break;
    case Command_Mem:
        temp |= MFRC522_BMS_COMMAND_COMMAND_MEM;
        break;
    case Command_GenerateRandomID:
        temp |= MFRC522_BMS_COMMAND_COMMAND_GENRANDOMID;
        break;
    case Command_CalcCrc:
        temp |= MFRC522_BMS_COMMAND_COMMAND_CALCCRC;
        break;
    case Command_Transmit:
        temp |= MFRC522_BMS_COMMAND_COMMAND_TRANSMIT;
        break;
    case Command_NoCmdChange:
        temp |= MFRC522_BMS_COMMAND_COMMAND_NOCMDCHANGE;
        break;
    case Command_Receive:
        temp |= MFRC522_BMS_COMMAND_COMMAND_RECEIVE;
        break;
    case Command_Transceive:
        temp |= MFRC522_BMS_COMMAND_COMMAND_TRANSCEIVE;
        break;
    case Command_MFAuthent:
        temp |= MFRC522_BMS_COMMAND_COMMAND_MFAUTHENT;
        break;
    case Command_SoftReset:
        temp |= MFRC522_BMS_COMMAND_COMMAND_SOFTRESET;
        break;
    default:
        break;
    }

    mfrc522_WriteRegister(class, MFRC522_ADDR_COMMAND, temp);
}

/*
 * Executes the transceive command.
 */

static uint8_t mfrc522_CommandTransceive (mfrc522_Class* class, uint8_t* TxData, uint8_t TxDataLength, uint8_t* RxData, uint8_t* RxDataLength)
{
    uint8_t error = 0, result = MFRC522_STATUS_OFF;
    uint16_t flag = 0;

    *(RxDataLength) = 0;

    mfrc522_FIFOClear(class);
    mfrc522_FIFOWrite(class, TxData, TxDataLength);
    mfrc522_ReceiverStatus(class, MFRC522_STATUS_ON);

    mfrc522_CommandExecute(class, Command_Transceive);
    mfrc522_TransmitterStart(class);

    do
    {
        error = mfrc522_ErrorGet(class);
        flag = mfrc522_IntGet(class) & (MFRC522_INT_RX | MFRC522_INT_TIMER);
    } while ((!(error)) && (!(flag)));

    if (flag & MFRC522_INT_RX)
    {
        *RxDataLength = mfrc522_FIFORead(class, RxData, 64);
        result = MFRC522_STATUS_ON;
    }

    return result;
}

/*
 * Tries to find tags in the area of operation.
 */

static uint8_t mfrc522_FindTags (mfrc522_Class* class)
{
    uint8_t data[64] = {0xFF};
    uint8_t RxDataLength = 0;
    uint8_t result;
    uint8_t RxValidBits;
    uint16_t CRCResult = 0;

    mfrc522_SoftReset(class);

    // Configure timer for auto and 50 ms.
    mfrc522_TimerConfig(class, MFRC522_TIMER_OPT_AUTO);
    mfrc522_TimerValues(class, 500, 677);

    // Configure Antenna and CRC.
    mfrc522_TransmitterModulation100ASK(class, MFRC522_STATUS_ON);
    mfrc522_CRCConfig(class, MFRC522_CRC_PRESET_6363);
    mfrc522_TransmitterCrcStatus(class, MFRC522_STATUS_OFF);
    mfrc522_ReceiverCRCStatus(class, MFRC522_STATUS_OFF);
    mfrc522_AntennaConfig(class, MFRC522_ANTENNA_OPT_TX2_INVERTED_WHEN_ON | MFRC522_ANTENNA_OPT_TX1_MODULATED | MFRC522_ANTENNA_OPT_TX2_MODULATED);

    // Configure for short frame and initialization.
    mfrc522_TransmitterSetFrame(class, Frame_Short);

    data[0] = MFCommand_ReqA;

    // REQA
    result = mfrc522_CommandTransceive(class, data, 1, data, &RxDataLength);

    if (result == MFRC522_STATUS_ON)
    {
    	// Get UIDCLn for Single Size
        data[0] = MFCommand_SelCL1;
        data[1] = 0x20;
        mfrc522_TransmitterSetFrame(class, Frame_Standard);
        result = mfrc522_CommandTransceive(class, data, 2, (data + 2), &RxDataLength);

        if (result == MFRC522_STATUS_ON)
        {
        	// Send UIDCL to get SAK.
            data[0] = MFCommand_SelCL1;
            data[1] = 0x70;
            mfrc522_CRCCalculate(class, data, 7, data + 7);
            result = mfrc522_CommandTransceive(class, data, 9, data, &RxDataLength);

            if (result == MFRC522_STATUS_ON)
            {
                data[20] = 0x00;
            }
        }
    }

    return result;
}

/*
 * Calculates the CRC.
 */

static void mfrc522_CRCCalculate (mfrc522_Class* class, uint8_t* data, uint8_t dataLength, uint8_t* recv)
{
    uint8_t temp;

    mfrc522_FIFOClear(class);
    mfrc522_FIFOWrite(class, data, dataLength);
    mfrc522_IntClear(class, MFRC522_INT_CRC);
    mfrc522_CommandExecute(class, Command_CalcCrc);
    while (!(mfrc522_IntGet(class) & MFRC522_INT_CRC));
    mfrc522_CommandExecute(class, Command_Idle);

    mfrc522_ReadRegister(class, MFRC522_ADDR_CRCRESULTLSB, &temp);
    recv[0] = temp;
    mfrc522_ReadRegister(class, MFRC522_ADDR_CRCRESULTMSB, &temp);
    recv[1] = temp;
}

/*
 * Configures the CRC.
 */

static void mfrc522_CRCConfig (mfrc522_Class* class, uint8_t MFRC522_CRCx)
{
	uint8_t temp;

	mfrc522_ReadRegister(class, MFRC522_ADDR_MODE, &temp);

	if (MFRC522_CRCx & MFRC522_CRC_MSBFIRST)
		temp |= MFRC522_BMS_MODE_MSBFIRST_BIT;
	else
		temp &= ~MFRC522_BMS_MODE_MSBFIRST_BIT;

	temp &= ~MFRC522_BMS_MODE_CRCPRESET_BITS;
	if (MFRC522_CRCx & MFRC522_CRC_PRESET_0000)
		temp |= MFRC522_BMS_MODE_CRCPRESET_0000;
	else if (MFRC522_CRCx & MFRC522_CRC_PRESET_6363)
		temp |= MFRC522_BMS_MODE_CRCPRESET_6363;
	else if (MFRC522_CRCx & MFRC522_CRC_PRESET_A671)
		temp |= MFRC522_BMS_MODE_CRCPRESET_A671;
	else if (MFRC522_CRCx & MFRC522_CRC_PRESET_FFFF)
		temp |= MFRC522_BMS_MODE_CRCPRESET_FFFF;

	mfrc522_WriteRegister(class, MFRC522_ADDR_MODE, temp);
}

/*
 * Returns the status bits.
 */

static uint8_t mfrc522_StatusGet (mfrc522_Class* class)
{
	uint8_t temp;
	uint8_t result = 0;

	mfrc522_ReadRegister(class, MFRC522_ADDR_STATUS1, &temp);
	if (temp & MFRC522_BMS_STATUS1_CRCOK_BIT)
		result |= MFRC522_STATUS_CRC_OK;
	if (temp & MFRC522_BMS_STATUS1_CRCREADY_BIT)
		result |= MFRC522_STATUS_CRC_READY;
	if (temp & MFRC522_BMS_STATUS1_IRQ_BIT)
		result |= MFRC522_STATUS_IRQ_ON;
	if (temp & MFRC522_BMS_STATUS1_TRUNNING_BIT)
		result |= MFRC522_STATUS_TIMER_ON;
	if (temp & MFRC522_BMS_STATUS1_HIALERT_BIT)
		result |= MFRC522_STATUS_FIFO_HIGH;
	if (temp & MFRC522_BMS_STATUS1_LOALERT_BIT)
		result |= MFRC522_STATUS_FIFO_LOW;

	return result;
}

/*
 * Enables or disables the CRC during reception.
 */

static void mfrc522_ReceiverCRCStatus (mfrc522_Class* class, uint8_t MFRC522_STATUSx)
{
	uint8_t temp;

	mfrc522_ReadRegister(class, MFRC522_ADDR_RXMODE, &temp);
	if (MFRC522_STATUSx == MFRC522_STATUS_ON)
		temp |= MFRC522_BMS_RXMODE_RXCRCEN_BIT;
	else
		temp &= ~MFRC522_BMS_RXMODE_RXCRCEN_BIT;
	mfrc522_WriteRegister(class, MFRC522_ADDR_RXMODE, temp);
}

/*
 *
 */

static mfrc522_Speed mfrc522_ReceiverGetSpeed (mfrc522_Class* class)
{
	uint8_t temp;
	mfrc522_Speed speed;

	mfrc522_ReadRegister(class, MFRC522_ADDR_RXMODE, &temp);

	temp &= MFRC522_BMS_RXMODE_RXSPEED_BITS;

	switch (temp)
	{
	case MFRC522_BMS_RXMODE_RXSPEED_106KBD:
		speed = Speed_106kBd;
		break;
	case MFRC522_BMS_RXMODE_RXSPEED_212KBD:
		speed = Speed_212kBd;
		break;
	case MFRC522_BMS_RXMODE_RXSPEED_424KBD:
		speed = Speed_424kBd;
		break;
	case MFRC522_BMS_RXMODE_RXSPEED_848KBD:
		speed = Speed_848kBd;
		break;
	}

	return speed;
}

/*
 *
 */

static void mfrc522_ReceiverSetSpeed (mfrc522_Class* class, mfrc522_Speed speed)
{
	uint8_t temp;

	mfrc522_ReadRegister(class, MFRC522_ADDR_RXMODE, &temp);

	temp &= ~MFRC522_BMS_RXMODE_RXSPEED_BITS;

	switch (speed)
	{
	case Speed_106kBd:
		temp |= MFRC522_BMS_RXMODE_RXSPEED_106KBD;
		break;
	case Speed_212kBd:
		temp |= MFRC522_BMS_RXMODE_RXSPEED_212KBD;
		break;
	case Speed_424kBd:
		temp |= MFRC522_BMS_RXMODE_RXSPEED_424KBD;
		break;
	case Speed_848kBd:
		temp |= MFRC522_BMS_RXMODE_RXSPEED_848KBD;
		break;
	}

	mfrc522_WriteRegister(class, MFRC522_ADDR_RXMODE, temp);
}


/*
 * Returns zero if there was no collisions, returns a positive value indicating collision at data bit index.
 */

static uint8_t mfrc522_ReceiverGetCollision (mfrc522_Class* class)
{
	uint8_t temp;

	mfrc522_ReadRegister(class, MFRC522_ADDR_COLL, &temp);
	if (!(temp & MFRC522_BMS_COLL_COLLPOSNOTVALID_BIT))
	{
		if (temp & MFRC522_BMS_COLL_COLLPOS_BITS)
			return (temp & MFRC522_BMS_COLL_COLLPOS_BITS);

		return 32;
	}

	return 0;
}

/*
 * Gets the valid bits of the last received byte.
 */

static uint8_t mfrc522_ReceiverGetBits (mfrc522_Class* class)
{
	uint8_t temp;

	mfrc522_ReadRegister(class, MFRC522_ADDR_CONTROL, &temp);
	temp &= MFRC522_BMS_CONTROL_RXLASTBITS_BITS;
	if (temp)
		return temp;

	return 8;
}

/*
 * Sets the receiver to ignore data streams with less than 4 bits received, and the receiver remains active.
 */

static void mfrc522_ReceiverContinous (mfrc522_Class* class, uint8_t MFRC522_STATUSx)
{
	uint8_t temp;

	mfrc522_ReadRegister(class, MFRC522_ADDR_RXMODE, &temp);
	if (MFRC522_STATUSx == MFRC522_STATUS_ON)
		temp |= MFRC522_BMS_RXMODE_RXMULTIPLE_BIT;
	else
		temp &= ~MFRC522_BMS_RXMODE_RXMULTIPLE_BIT;
	mfrc522_WriteRegister(class, MFRC522_ADDR_RXMODE, temp);
}

/*
 * Sets the receiver to receive data frames till user cancels it.
 */

static void mfrc522_ReceiverIgnoreInvalid (mfrc522_Class* class, uint8_t MFRC522_STATUSx)
{
	uint8_t temp;

	mfrc522_ReadRegister(class, MFRC522_ADDR_RXMODE, &temp);
	if (MFRC522_STATUSx == MFRC522_STATUS_ON)
		temp |= MFRC522_BMS_RXMODE_RXNOERR_BIT;
	else
		temp &= ~MFRC522_BMS_RXMODE_RXNOERR_BIT;
	mfrc522_WriteRegister(class, MFRC522_ADDR_RXMODE, temp);
}


/*
 * Sets the antenna gain of the receiver.
 */

static void mfrc522_ReceiverAntennaGain (mfrc522_Class* class, uint8_t MFRC522_RXGAINx)
{
	uint8_t temp;

	mfrc522_ReadRegister(class, MFRC522_ADDR_RFCFG, &temp);
	temp &= ~MFRC522_BMS_RFCFG_RXGAIN_BITS;
	temp |= MFRC522_RXGAINx;
	mfrc522_WriteRegister(class, MFRC522_ADDR_RFCFG, temp);
}

/*
 * Sets the appropiate format.
 */

static void mfrc522_TransmitterSetFrame (mfrc522_Class* class, mfrc522_Frame frame)
{
    switch (frame)
    {
    case Frame_Short:
        mfrc522_TransmitterSetBits(class, 7);
        //mfrc522_ParityStatus(class, MFRC522_STATUS_OFF);
        break;
    case Frame_Standard:
        mfrc522_TransmitterSetBits(class, 8);
        //mfrc522_ParityStatus(class, MFRC522_STATUS_ON);
        break;
    case Frame_Anticollision:
        break;
    default:
        break;
    }
}


/*
 * Performs a self test. How to perform a self reset can be found on the datasheet (Section 16.1.1).
 */

static uint8_t mfrc522_SelfTest (mfrc522_Class* class)
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
    uint8_t version = 0;
    uint8_t temp = 0;
    uint8_t result = MFRC522_STATUS_OFF;

    data = calloc(64, sizeof(uint8_t));
    if (data == NULL)
        return result;

    mfrc522_ReadRegister(class, MFRC522_ADDR_VERSION, &temp);

    if (temp == MFRC522_BMS_VERSION_VERSION1)
        version = 1;
    else if (temp == MFRC522_BMS_VERSION_VERSION2)
        version = 2;

    mfrc522_SoftReset(class);
    mfrc522_InternalBufferWrite(class, data);
    mfrc522_WriteRegister(class, MFRC522_ADDR_AUTOTEST, MFRC522_BMS_AUTOTEST_SELFTEST_SELFTEST);
    mfrc522_FIFOWrite(class, data, 1);
    mfrc522_ReceiverStatus(class, MFRC522_STATUS_ON);
    mfrc522_CommandExecute(class, Command_CalcCrc);
    while (mfrc522_CommandCurrent(class) != Command_Idle)

    mfrc522_FIFORead(class, data, 64);

    if ((version == 1) && (!(memcmp(data, V1, 64)))) {
        result = MFRC522_STATUS_ON;
    } else if ((version == 2) && (!(memcmp(data, V2, 64)))) {
        result = MFRC522_STATUS_ON;
    }

    free(data);

    return result;
}

/*
 * Deactivates or activates the analog part of the receiver.
 */

static void mfrc522_ReceiverStatus (mfrc522_Class* class, uint8_t MFRC522_STATUSx)
{
    uint8_t temp;

    mfrc522_ReadRegister(class, MFRC522_ADDR_COMMAND, &temp);
    temp &= ~MFRC522_BMS_COMMAND_COMMAND_BITS;
    temp |= MFRC522_BMS_COMMAND_COMMAND_NOCMDCHANGE; // Add CMD NO CHANGE.
    (MFRC522_STATUSx == MFRC522_STATUS_ON) ? (temp &= ~(MFRC522_BMS_COMMAND_RCVOFF_BIT)) : (temp |= MFRC522_BMS_COMMAND_RCVOFF_BIT);
    mfrc522_WriteRegister(class, MFRC522_ADDR_COMMAND, temp);
}

/*
 * Returns the current status of the receiver and transmitter state machines.
 */

static mfrc522_Modem mfrc522_ModemCheck (mfrc522_Class* class)
{
	uint8_t temp;
	mfrc522_Modem status;

	mfrc522_ReadRegister(class, MFRC522_ADDR_STATUS2, &temp);

	temp &= MFRC522_BMS_STATUS2_MODEMSTATE_BITS;

	switch (temp)
	{
	case MFRC522_BMS_STATUS2_MODEMSTATE_IDLE:
		status = Modem_Idle;
		break;
	case MFRC522_BMS_STATUS2_MODEMSTATE_RECEIVING:
		status = Modem_Receiving;
		break;
	case MFRC522_BMS_STATUS2_MODEMSTATE_RX_WAIT_RF:
		status = Modem_ReceiverWaitingForRFField;
		break;
	case MFRC522_BMS_STATUS2_MODEMSTATE_TX_TRANSMITTING:
		status = Modem_Transmitting;
		break;
	case MFRC522_BMS_STATUS2_MODEMSTATE_TX_WAIT_RF:
		status = Modem_TransmitterWaitingForRFField;
		break;
	case MFRC522_BMS_STATUS2_MODEMSTATE_WAIT_FOR_DATA:
		status = Modem_WaitForData;
		break;
	case MFRC522_BMS_STATUS2_MODEMSTATE_WAIT_STARTSEND:
		status = Modem_WaitForStartSend;
		break;
	default:
	    status = Modem_Unknown;
		break;
	}

	return status;
}

/*
 * Enables or disables the specified interrupts.
 *
 * MFRC522_INTx is the logical OR of the MFRC522_INTx defines.
 *
 * status = true enables the interrupts, status = false disables the interrupts.
 */

static void mfrc522_IntStatus (mfrc522_Class* class, uint16_t MFRC522_INTx, uint8_t MFRC522_STATUSx)
{
    uint8_t temp;

    mfrc522_ReadRegister(class, MFRC522_ADDR_COMIEN, &temp);
    if (MFRC522_INTx & MFRC522_INT_TX)
        (MFRC522_STATUSx == MFRC522_STATUS_ON) ? (temp |= MFRC522_BMS_COMIEN_TXIEN_BIT) : (temp &= ~MFRC522_BMS_COMIEN_TXIEN_BIT);
    if (MFRC522_INTx & MFRC522_INT_TIMER)
        (MFRC522_STATUSx == MFRC522_STATUS_ON) ? (temp |= MFRC522_BMS_COMIEN_TIMERIEN_BIT) : (temp &= ~MFRC522_BMS_COMIEN_TIMERIEN_BIT);
    if (MFRC522_INTx & MFRC522_INT_RX)
        (MFRC522_STATUSx == MFRC522_STATUS_ON) ? (temp |= MFRC522_BMS_COMIEN_RXIEN_BIT) : (temp &= ~MFRC522_BMS_COMIEN_RXIEN_BIT);
    if (MFRC522_INTx & MFRC522_INT_LOALERT)
        (MFRC522_STATUSx == MFRC522_STATUS_ON) ? (temp |= MFRC522_BMS_COMIEN_LOALERTIEN_BIT) : (temp &= ~MFRC522_BMS_COMIEN_LOALERTIEN_BIT);
    if (MFRC522_INTx & MFRC522_INT_HIALERT)
        (MFRC522_STATUSx == MFRC522_STATUS_ON) ? (temp |= MFRC522_BMS_COMIEN_HIALERTIEN_BIT) : (temp &= ~MFRC522_BMS_COMIEN_HIALERTIEN_BIT);
    if (MFRC522_INTx & MFRC522_INT_IDLE)
        (MFRC522_STATUSx == MFRC522_STATUS_ON) ? (temp |= MFRC522_BMS_COMIEN_IDLEIEN_BIT) : (temp &= ~MFRC522_BMS_COMIEN_IDLEIEN_BIT);
    if (MFRC522_INTx & MFRC522_INT_ERR)
        (MFRC522_STATUSx == MFRC522_STATUS_ON) ? (temp |= MFRC522_BMS_COMIEN_ERRIEN_BIT) : (temp &= ~MFRC522_BMS_COMIEN_ERRIEN_BIT);
    mfrc522_WriteRegister(class, MFRC522_ADDR_COMIEN, temp);

    mfrc522_ReadRegister(class, MFRC522_ADDR_DIVIEN, &temp);
    if (MFRC522_INTx & MFRC522_INT_MFINACT)
        (MFRC522_STATUSx == MFRC522_STATUS_ON) ? (temp |= MFRC522_BMS_DIVIEN_MFINACTIEN_BIT) : (temp &= ~MFRC522_BMS_DIVIEN_MFINACTIEN_BIT);
    if (MFRC522_INTx & MFRC522_INT_CRC)
        (MFRC522_STATUSx == MFRC522_STATUS_ON) ? (temp |= MFRC522_BMS_DIVIEN_CRCIEN_BIT) : (temp &= ~MFRC522_BMS_DIVIEN_CRCIEN_BIT);
    mfrc522_WriteRegister(class, MFRC522_ADDR_DIVIEN, temp);
}

/*
 * Returns the status of all the interrupts.
 */

static uint16_t mfrc522_IntGet (mfrc522_Class* class)
{
    uint8_t temp;
    uint16_t res = 0;

    mfrc522_ReadRegister(class, MFRC522_ADDR_COMIRQ, &temp);
    if (temp & MFRC522_BMS_COMIRQ_TXIRQ_BIT)
        res |= MFRC522_INT_TX;
    if (temp & MFRC522_BMS_COMIRQ_TIMERIRQ_BIT)
        res |= MFRC522_INT_TIMER;
    if (temp & MFRC522_BMS_COMIRQ_RXIRQ_BIT)
        res |= MFRC522_INT_RX;
    if (temp & MFRC522_BMS_COMIRQ_LOALERTIRQ_BIT)
        res |= MFRC522_INT_LOALERT;
    if (temp & MFRC522_BMS_COMIRQ_HIALERTIRQ_BIT)
        res |= MFRC522_INT_HIALERT;
    if (temp & MFRC522_BMS_COMIRQ_IDLEIRQ_BIT)
        res |= MFRC522_INT_IDLE;
    if (temp & MFRC522_BMS_COMIRQ_ERRIRQ_BIT)
        res |= MFRC522_INT_ERR;

    mfrc522_ReadRegister(class, MFRC522_ADDR_DIVIRQ, &temp);
    if (temp & MFRC522_BMS_DIVIRQ_MFINACTIRQ_BIT)
        res |= MFRC522_INT_MFINACT;
    if (temp & MFRC522_BMS_DIVIRQ_CRCIRQ_BIT)
        res |= MFRC522_INT_CRC;

    return res;
}

/*
 * Clears the specified interrupts.
 */

static void mfrc522_IntClear (mfrc522_Class* class, uint16_t MFRC522_INTx)
{
    uint8_t temp;

    mfrc522_ReadRegister(class, MFRC522_ADDR_COMIRQ, &temp);
    temp &= ~MFRC522_BMS_COMIRQ_SET1_BIT;
    if (!(MFRC522_INTx & MFRC522_INT_TX))
        temp &= ~(MFRC522_BMS_COMIRQ_TXIRQ_BIT);
    if (!(MFRC522_INTx & MFRC522_INT_TIMER))
        temp &= ~(MFRC522_BMS_COMIRQ_TIMERIRQ_BIT);
    if (!(MFRC522_INTx & MFRC522_INT_RX))
        temp &= ~(MFRC522_BMS_COMIRQ_RXIRQ_BIT);
    if (!(MFRC522_INTx & MFRC522_INT_LOALERT))
        temp &= ~(MFRC522_BMS_COMIRQ_LOALERTIRQ_BIT);
    if (!(MFRC522_INTx & MFRC522_INT_HIALERT))
        temp &= ~(MFRC522_BMS_COMIRQ_HIALERTIRQ_BIT);
    if (!(MFRC522_INTx & MFRC522_INT_IDLE))
        temp &= ~(MFRC522_BMS_COMIRQ_IDLEIRQ_BIT);
    if (!(MFRC522_INTx & MFRC522_INT_ERR))
        temp &= ~(MFRC522_BMS_COMIRQ_ERRIRQ_BIT);
    mfrc522_WriteRegister(class, MFRC522_ADDR_COMIRQ, temp);

    mfrc522_ReadRegister(class, MFRC522_ADDR_DIVIRQ, &temp);
    temp &= ~MFRC522_BMS_DIVIRQ_SET2_BIT;
    if (!(MFRC522_INTx & MFRC522_INT_MFINACT))
        temp &= ~(MFRC522_BMS_DIVIRQ_MFINACTIRQ_BIT);
    if (!(MFRC522_INTx & MFRC522_INT_CRC))
        temp &= ~(MFRC522_BMS_DIVIRQ_CRCIRQ_BIT);
    mfrc522_WriteRegister(class, MFRC522_ADDR_DIVIRQ, temp);
}

/*
 * Gets the errors indicated by the device.
 */

static uint8_t mfrc522_ErrorGet (mfrc522_Class* class)
{
    uint8_t temp;
    uint8_t result = 0;

    mfrc522_ReadRegister(class, MFRC522_ADDR_ERROR, &temp);
    if (temp & MFRC522_BMS_ERROR_WRERR_BIT)
        result |= MFRC522_ERROR_FIFOWRITE;
    if (temp & MFRC522_BMS_ERROR_TEMPERR_BIT)
        result |= MFRC522_ERROR_OVERHEAT;
    if (temp & MFRC522_BMS_ERROR_PROTOCOLERR_BIT)
        result |= MFRC522_ERROR_PROTOCOL;
    if (temp & MFRC522_BMS_ERROR_PARITYERR_BIT)
        result |= MFRC522_ERROR_PARITY;
    if (temp & MFRC522_BMS_ERROR_CRCERR_BIT)
        result |= MFRC522_ERROR_CRC;
    if (temp & MFRC522_BMS_ERROR_COLLERR_BIT)
        result |= MFRC522_ERROR_COLLISION;
    if (temp & MFRC522_BMS_ERROR_BUFFEROVFL_BIT)
        result |= MFRC522_ERROR_BUFFEROVERFLOW;

    return result;
}

/*
 * Defines the number of bits of the last byte that will be transmitted. Meant to be used with bit-oriented frames.
 *
 * A value of zero indicates all the bits of the last byte will be transmitted.
 */

static void mfrc522_TransmitterSetBits (mfrc522_Class* class, uint8_t nBits)
{
	uint8_t temp;

	mfrc522_ReadRegister(class, MFRC522_ADDR_BITFRAMING, &temp);
	temp &= ~(MFRC522_BMS_BITFRAMING_TXLASTBITS_BITS);
	if (nBits != 0x08)
	    temp |= MFRC522_BMS_BITFRAMING_TXLASTBITS((nBits & 0x07));
	mfrc522_WriteRegister(class, MFRC522_ADDR_BITFRAMING, temp);
}

/*
 * If enabled, transmitter can only be started if an RF field is generated.
 */

static void mfrc522_TransmitterWaitForRF (mfrc522_Class* class, uint8_t MFRC522_STATUSx)
{
	uint8_t temp;

	mfrc522_ReadRegister(class, MFRC522_ADDR_MODE, &temp);
	if (MFRC522_STATUSx == MFRC522_STATUS_ON)
		temp |= MFRC522_BMS_MODE_TXWAITRF_BIT;
	else
		temp &= ~(MFRC522_BMS_MODE_TXWAITRF_BIT);
	mfrc522_WriteRegister(class, MFRC522_ADDR_MODE, temp);
}

/*
 * Enables or disables CRC generation during transmission.
 */

static void mfrc522_TransmitterCrcStatus (mfrc522_Class* class, uint8_t MFRC522_STATUSx)
{
	uint8_t temp;

	mfrc522_ReadRegister(class, MFRC522_ADDR_TXMODE, &temp);
	if (MFRC522_STATUSx == MFRC522_STATUS_ON)
		temp |= MFRC522_BMS_TXMODE_TXCRCEN_BIT;
	else
		temp &= ~(MFRC522_BMS_TXMODE_TXCRCEN_BIT);
	mfrc522_WriteRegister(class, MFRC522_ADDR_TXMODE, temp);
}

/*
 * Sets the speed for the transmitter.
 */

static void mfrc522_TransmitterSpeedSet (mfrc522_Class* class, mfrc522_Speed speed)
{
	uint8_t temp;

	mfrc522_ReadRegister(class, MFRC522_ADDR_TXMODE, &temp);
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
	mfrc522_WriteRegister(class, MFRC522_ADDR_TXMODE, temp);
}

/*
 * Get the speed of the transmitter.
 */

static mfrc522_Speed mfrc522_TransmitterSpeedGet (mfrc522_Class* class)
{
	uint8_t temp;
	mfrc522_Speed speed;

	mfrc522_ReadRegister(class, MFRC522_ADDR_TXMODE, &temp);
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

static void mfrc522_TransmitterModulation100ASK (mfrc522_Class* class, uint8_t MFRC522_STATUSx)
{
	uint8_t temp;

	mfrc522_ReadRegister(class, MFRC522_ADDR_TXASK, &temp);
	if (MFRC522_STATUSx == MFRC522_STATUS_ON)
		temp |= MFRC522_BMS_TXASK_FORCE100ASK_BIT;
	else
		temp &= ~(MFRC522_BMS_TXASK_FORCE100ASK_BIT);
	mfrc522_WriteRegister(class, MFRC522_ADDR_TXASK, temp);
}

/*
 * Starts the transmission of data. Only in combination with transceive command.
 */

static void mfrc522_TransmitterStart (mfrc522_Class* class)
{
    uint8_t temp;

    mfrc522_ReadRegister(class, MFRC522_ADDR_BITFRAMING, &temp);
    temp |= MFRC522_BMS_BITFRAMING_STARTSEND_BIT;
    mfrc522_WriteRegister(class, MFRC522_ADDR_BITFRAMING, temp);
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

static void mfrc522_AntennaConfig (mfrc522_Class* class, uint8_t MFRC522_ANTENNA_OPTx)
{
	uint8_t temp;

	mfrc522_ReadRegister(class, MFRC522_ADDR_TXCONTROL, &temp);

	if (MFRC522_ANTENNA_OPTx & MFRC522_ANTENNA_OPT_TX1_MODULATED)
		temp |= MFRC522_BMS_TXCONTROL_TX1RFEN_BIT;
	else
		temp &= ~(MFRC522_BMS_TXCONTROL_TX1RFEN_BIT);

	if (MFRC522_ANTENNA_OPTx & MFRC522_ANTENNA_OPT_TX2_MODULATED)
		temp |= MFRC522_BMS_TXCONTROL_TX2RFEN_BIT;
	else
		temp &= ~(MFRC522_BMS_TXCONTROL_TX2RFEN_BIT);

	if (MFRC522_ANTENNA_OPTx & MFRC522_ANTENNA_OPT_TX2_UNMODULATED)
		temp |= MFRC522_BMS_TXCONTROL_TX2CW_BIT;
	else
		temp &= ~(MFRC522_BMS_TXCONTROL_TX2CW_BIT);

	if (MFRC522_ANTENNA_OPTx & MFRC522_ANTENNA_OPT_TX1_INVERTED_WHEN_OFF)
		temp |= MFRC522_BMS_TXCONTROL_INVTX1RFOFF_BIT;
	else
		temp &= ~(MFRC522_BMS_TXCONTROL_INVTX1RFOFF_BIT);

	if (MFRC522_ANTENNA_OPTx & MFRC522_ANTENNA_OPT_TX2_INVERTED_WHEN_OFF)
		temp |= MFRC522_BMS_TXCONTROL_INVTX2RFOFF_BIT;
	else
		temp &= ~(MFRC522_BMS_TXCONTROL_INVTX2RFOFF_BIT);

	if (MFRC522_ANTENNA_OPTx & MFRC522_ANTENNA_OPT_TX1_INVERTED_WHEN_ON)
		temp |= MFRC522_BMS_TXCONTROL_INVTX1RFON_BIT;
	else
		temp &= ~(MFRC522_BMS_TXCONTROL_INVTX1RFON_BIT);

	if (MFRC522_ANTENNA_OPTx & MFRC522_ANTENNA_OPT_TX2_INVERTED_WHEN_ON)
		temp |= MFRC522_BMS_TXCONTROL_INVTX2RFON_BIT;
	else
		temp &= ~(MFRC522_BMS_TXCONTROL_INVTX2RFON_BIT);

	mfrc522_WriteRegister(class, MFRC522_ADDR_TXCONTROL, temp);
}

/*
 * Sets if the parity bit for transmission and parity check for reception should be active or not.
 */

static void mfrc522_ParityStatus (mfrc522_Class* class, uint8_t MFRC522_STATUSx)
{
	uint8_t temp;

	mfrc522_ReadRegister(class, MFRC522_ADDR_MFRX, &temp);
	if (MFRC522_STATUSx == MFRC522_STATUS_OFF)
		temp |= MFRC522_BMS_MFRX_PARITYDISABLE_BIT;
	else
		temp &= ~(MFRC522_BMS_MFRX_PARITYDISABLE_BIT);
	mfrc522_WriteRegister(class, MFRC522_ADDR_MFRX, temp);
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

static void mfrc522_TimerConfig (mfrc522_Class* class, uint16_t MFRC522_TIMER_OPTx)
{
    uint8_t temp;

    mfrc522_ReadRegister(class, MFRC522_ADDR_TMODE, &temp);

    temp &= ~(MFRC522_BMS_TMODE_TAUTO_BIT);
    if (MFRC522_TIMER_OPTx & MFRC522_TIMER_OPT_AUTO)
        temp |= MFRC522_BMS_TMODE_TAUTO_BIT;

    temp &= ~(MFRC522_BMS_TMODE_TGATED_BITS);
    if (MFRC522_TIMER_OPTx & MFRC522_TIMER_OPT_GATED_AUX1)
        temp |= MFRC522_BMS_TMODE_TGATED_AUX1;
    else if (MFRC522_TIMER_OPTx & MFRC522_TIMER_OPT_GATED_MFIN)
        temp |= MFRC522_BMS_TMODE_TGATED_MFIN;

    temp &= ~(MFRC522_BMS_TMODE_TAUTORESTART_BIT);
    if (MFRC522_TIMER_OPTx & MFRC522_TIMER_OPT_AUTOLOAD)
        temp |= (MFRC522_BMS_TMODE_TAUTORESTART_BIT);

    mfrc522_WriteRegister(class, MFRC522_ADDR_TMODE, temp);

    mfrc522_ReadRegister(class, MFRC522_ADDR_DEMOD, &temp);

    temp &= ~(MFRC522_BMS_DEMOD_TPRESCALEVEN_BIT);
    if (MFRC522_TIMER_OPTx & MFRC522_TIMER_OPT_EVENPRESCALER)
        temp |= MFRC522_BMS_DEMOD_TPRESCALEVEN_BIT;

    mfrc522_WriteRegister(class, MFRC522_ADDR_DEMOD, temp);
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

static void mfrc522_TimerValues (mfrc522_Class* class, uint16_t prescaler, uint16_t reloadval)
{
    uint8_t temp;

    mfrc522_ReadRegister(class, MFRC522_ADDR_TMODE, &temp);

    temp &= ~(MFRC522_BMS_TMODE_TPRESCALERHI_BITS);
    temp |= MFRC522_BMS_TMODE_TPRESCALERHI((prescaler & 0x0F00) >> 8);

    mfrc522_WriteRegister(class, MFRC522_ADDR_TMODE, temp);
    mfrc522_WriteRegister(class, MFRC522_ADDR_TPRESCALERLO, prescaler & 0x00FF);
    mfrc522_WriteRegister(class, MFRC522_ADDR_TRELOADHI, (reloadval & 0xFF00) >> 8);
    mfrc522_WriteRegister(class, MFRC522_ADDR_TRELOADLO, reloadval & 0x00FF);
}

/*
 * Gets the current value of the timer.
 */

static uint16_t mfrc522_TimerCounterGet (mfrc522_Class* class)
{
    uint8_t Hi, Lo;
    uint16_t Count;

    mfrc522_ReadRegister(class, MFRC522_ADDR_TCOUNTERVALHI, &Hi);
    mfrc522_ReadRegister(class, MFRC522_ADDR_TCOUNTERVALLO, &Lo);

    Count = (Hi << 8) | Lo;

    return Count;
}

/*
 * Checks if the timer is running
 */

static uint8_t mfrc522_TimerIsRunning (mfrc522_Class* class)
{
    uint8_t temp;

    mfrc522_ReadRegister(class, MFRC522_ADDR_STATUS1, &temp);

    if (temp & MFRC522_BMS_STATUS1_TRUNNING_BIT)
        return MFRC522_STATUS_ON;

    return MFRC522_STATUS_OFF;
}

/*
 * Starts the timer immediately.
 */

static void mfrc522_TimerStart (mfrc522_Class* class)
{
    uint8_t temp;

    mfrc522_ReadRegister(class, MFRC522_ADDR_CONTROL, &temp);
    temp |= MFRC522_BMS_CONTROL_TSTARTNOW_BIT;
    mfrc522_WriteRegister(class, MFRC522_ADDR_CONTROL, temp);
}

/*
 * Stops the timer immediately.
 */

static void mfrc522_TimerStop (mfrc522_Class* class)
{
    uint8_t temp;

    mfrc522_ReadRegister(class, MFRC522_ADDR_CONTROL, &temp);
    temp |= MFRC522_BMS_CONTROL_TSTOPNOW_BIT;
    mfrc522_WriteRegister(class, MFRC522_ADDR_CONTROL, temp);
}

/*
 * Writes 25 bytes from the buffer into the internal memory of the chip.
 */

static void mfrc522_InternalBufferWrite (mfrc522_Class* class, uint8_t* buffer)
{
    mfrc522_FIFOClear(class);
    mfrc522_FIFOWrite(class, buffer, 25);
    mfrc522_CommandExecute(class, Command_Mem);
    while (mfrc522_CommandCurrent(class) != Command_Idle);
}

/*
 * Reads the internal data from the buffer.
 */

static void mfrc522_InternalBufferRead (mfrc522_Class* class, uint8_t* buffer)
{
    mfrc522_FIFOClear(class);
    mfrc522_CommandExecute(class, Command_Mem);
    while (mfrc522_CommandCurrent(class) != Command_Idle);
    mfrc522_FIFORead(class, buffer, 25);
}

/*
 * Writes the specified bytes on the FIFO. The FIFO capacity is 64 bytes.
 */

static void mfrc522_FIFOWrite (mfrc522_Class* class, uint8_t* buffer, uint8_t bytestowrite)
{
    uint8_t FifoLevel = 0;
    uint8_t i = 0;

    FifoLevel = mfrc522_FIFOLevelGet(class);

    while ((FifoLevel < 64) && (i < bytestowrite))
    {
        mfrc522_WriteRegister(class, MFRC522_ADDR_FIFODATA, *(buffer + i));
        while (FifoLevel == mfrc522_FIFOLevelGet(class));
        FifoLevel = mfrc522_FIFOLevelGet(class);
        i++;
    }
}

/*
 * Clears the FIFO, write and read pointers become zero.
 */

static void mfrc522_FIFOClear (mfrc522_Class* class)
{
    uint8_t temp;

    mfrc522_ReadRegister(class, MFRC522_ADDR_FIFOLEVEL, &temp);

    temp |= MFRC522_BMS_FIFOLEVEL_FLUSHBUFFER_BIT;

    mfrc522_WriteRegister(class, MFRC522_ADDR_FIFOLEVEL, temp);

    while (mfrc522_FIFOLevelGet(class) > 0);
}

/*
 * Gets the current level on the FIFO.
 */

static uint8_t mfrc522_FIFOLevelGet (mfrc522_Class* class)
{
    uint8_t temp = 0;

    mfrc522_ReadRegister(class, MFRC522_ADDR_FIFOLEVEL, &temp);

    temp &= MFRC522_BMS_FIFOLEVEL_FIFOLEVEL_BITS;

    return temp;
}

/*
 * Reads data from the FIFO.
 */

static uint8_t mfrc522_FIFORead (mfrc522_Class* class, uint8_t* buffer, uint8_t bytestoread)
{
    uint8_t FifoLevel = 0;
    uint8_t i = 0;

    FifoLevel = mfrc522_FIFOLevelGet(class);

    while (FifoLevel > 0 && (i < bytestoread))
    {
        mfrc522_ReadRegister(class, MFRC522_ADDR_FIFODATA, buffer + i);
        while (FifoLevel == mfrc522_FIFOLevelGet(class));
        FifoLevel = mfrc522_FIFOLevelGet(class);
        i++;
    }

    return i;
}
