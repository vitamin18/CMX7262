/**
  ******************************************************************************
  * @file    cmx7262.h
  * @brief   Файл, содержащий заголовки функций, реализующих обмен котроллера
	*						с микросхемой CMX7262
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 NTLab
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CMX7262_H
#define __CMX7262_H


#include <math.h>       /* для floor() */
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>

#include "main.h"
#include "spi.h"


#ifdef __cplusplus
 extern "C" {
#endif


#define CBUS_INTERFACE_CMX7262 (0)
 
//Частота дискретизации входного/выходного аудиосигнала, Гц
#define CMX7262_FREQ_SAMPLING						(8000)
	 
//Частота формируемого колебания в тестовом режиме, Гц	 
#define CMX7262_FREQ_SIGNAL_IN_TESTMODE	(1000)	

//Длительность вокодерного кадра, мс
#define CMX7262_FRAME_DURATION_MS				(20)

//Размер кадра звукового сигнала, отсчетов
#define CMX7262_AUDIOFRAME_SIZE_SAMPLES	(160)
	 
// Default gains
#define CMX7262_INPUT_GAIN_DEFAULT		0x0000
#define CMX7262_OUPUT_GAIN_DEFAULT		0x8000				

// Default noise gate parameters
#define CMX7262_NOISEGATE_THRESHOLD_DEFAULT		(70)
#define CMX7262_NOISEGATE_FRAMEDELAY_DEFAULT	(3)

// IRQ flag bit definitions.
#define	CMX7262_ODA								(1<<3)
#define	CMX7262_IDW								(1<<4)
#define	CMX7262_PLV								(1<<5)
#define	CMX7262_OV								(1<<10)
#define	CMX7262_UF								(1<<11)

// CMX7262 errors use the upper 16 bits.
#define	CMX7262_FI_LOAD_ERROR						(0x00010000)		// FI failed to load correctly.
#define	CMX7262_CONFIG_CLK_ERROR				(0x00020000)		// Failed to initialise the config clocks.
#define	CMX7262_ENCODE_ERROR						(0x00040000)		// Encode mode setup failed.
#define	CMX7262_DECODE_ERROR						(0x00080000)		// Decoder mode setup failed.
#define	CMX7262_IDLE_ERROR							(0x00100000)		// Idle mode setup failed.
#define	CMX7262_ODA_ERROR								(0x00200000)		// Multiple ODA flags.
#define	CMX7262_IDW_ERROR								(0x00400000)		// Multiple IDW flags.
#define	CMX7262_OV_ERROR								(0x00800000)
#define	CMX7262_UF_ERROR								(0x01000000)	 

#define	CMX7262_TRANSCODE_TIMEOUT		30//1500	// 15mS with a 10uS tick. Used when we set up codec modes.

// Регистры CMX7262
#define	FREQ_CONTROL				0x52
#define	NOISE_REDUCTION			0x53
#define	VCFG_REG						0x55
#define	SIGNAL_CONTROL			0x58
#define	AUDIO_ROUTING_REG		0x5D
#define	NOISE_GATE_REG			0x5E
#define	REG_DONE_SELECT			0x69
#define	PROG_REG						0x6A	// DSPIO_SW_WR9
#define	VCTRL_REG   				0x6B	// DSPIO_SW_WR0
#define	IRQ_ENABLE_REG			0x6C
#define	DSPIO_SW_RD0				0x70
#define	IRQ_STATUS_REG      0x7E	// DSPIO_CBUS_STATUS

// Real CBUS registers that the host can access, with special meanings
#define	CBUS_AUDIO_IN				0x49	// CBUS_TXWRITE16 - for writing 16-bit LPCM samples into CMX7262
#define	CBUS_VOCODER_IN			0x48	// CBUS_TXWRITE8 - for writing byte-wide coded data into CMX7262
#define	CBUS_TXLEVEL				0x4B
#define	CBUS_AUDIO_OUT			0x4D	// CBUS_RXREAD16 - for reading 16-bit LPCM samples from CMX7262
#define	CBUS_VOCODER_OUT		0x4C	// CBUS_RXREAD8 - for reading byte-wide coded data from CMX7262
#define	CBUS_RXLEVEL				0x4F
#define	GPIO_CONTROL				0x64
#define	PEAK_LEVEL					0x71
#define	NOISE_CTRL					0x53

#define	ANAIN_CONFIG				0xB0
#define	ANAIN_GAIN					0xB1
#define	ANAOUT_CONFIG				0xB3
#define	AOG3								0xB6		// Speaker output course gain.


// Bit references in Status (0x7E) and IRQ Enable Register (0x6C)
#define	ODA									(1UL<<3)			// Output Data Available
#define	PLV									(1UL<<5)			// Peak level sample
#define	IDW									(1UL<<8)			// Input Data Wanted
#define	OV									(1UL<<10)			// Overflow
#define	UF									(1UL<<11)			// Underflow
#define	REGDONE							(1UL<<13)			// Refister access Done
#define	PRG									(1UL<<14)			// Programming register ready
#define	IRQ									(1UL<<15)

// Bit references for configuration of vocoder (VCFG - $55)
#define	HDD									(1UL<<5)
#define	FEC									(1UL<<4)
#define	ONE_FRAME						1
#define	TWO_FRAME						2
#define	THREE_FRAME					3
#define	FOUR_FRAME					0

// Masks to extract the configuration settings above.
#define FRAME_MASK		0x0003
#define FEC_MASK		FEC

// TWELP frame size hard decision.
#define TWELP_HDD_FRAME_SIZE_BYTES		6
// TWELP frame size hard decision with FEC.
#define TWELP_FEC_HDD_FRAME_SIZE_BYTES		9

// Size of buffer to hold one packet of encoded samples (3 frame, hard decision, FEC).
#define	CMX7262_CODEC_BUFFER_SIZE	27


// Clock setup
#define	FS_DIV										78 				// Setup a 8kHz sample rate for the audio converters

#define	LOAD_FI_END	0
	 
// Definitions for the VCTRL register 6B (Main control register)
#define	CMX7262_VCTRL_IDLE				0x0000
#define	CMX7262_VCTRL_DECODE			0x0001
#define	CMX7262_VCTRL_ENCODE			0x0002
#define	CMX7262_VCTRL_ENCDEC			0x0003
#define	CMX7262_PASSTHRU					0x0004
#define	CMX7262_VCTRL_TEST				0x0007	 
	 

// CMX7262 Vocoder Modes
// These modes are required so that we can set the appropriate request
// flag in the IRQ and to prevent multiple idles. Multiple idles is a known
// issue with the CMX7262 FI.
typedef enum {
	CMX7262_INIT_MODE = 		0,
	CMX7262_IDLE_MODE = 		1,
	CMX7262_DECODE_MODE = 	2,
	CMX7262_ENCODE_MODE =		3,
	CMX7262_PASSTHRU_MODE = 4,
	CMX7262_TEST_MODE = 		5,
	CMX7262_ENCDEC_MODE = 	6
} en_cmx7262Modes;


// References for audio source and destinations in relation to
// CBUS and analog ports (Audio)..

#define	DEST_CBUS		0x1
#define	DEST_AUDIO	0x2
#define	SRC_CBUS		(0x1)<<4
#define	SRC_AUDIO		(0x2)<<4

 // Created so we can use a common function to load function images.
 typedef struct
 {
	uint16_t *db1_start_address;
 	uint16_t *db2_start_address;
 	uint16_t db1_ptr;
 	uint16_t db1_len;
 	uint16_t db1_chk_hi;
 	uint16_t db1_chk_lo;
 	uint16_t db2_ptr;
 	uint16_t db2_len;
 	uint16_t db2_chk_hi;
 	uint16_t db2_chk_lo;
 	uint16_t activate_ptr;
 	uint16_t activate_len;
 	uint16_t activation_code1;      //only needed for CMX7x3x & CMX7x4x devices
 	uint16_t activation_code2;      //only needed for CMX7x3x & CMX7x4x devices
 } cmxFI_TypeDef;

// Data type for an instance of the CMX7262.
typedef struct
{
	cmxFI_TypeDef FI;										// Function image parameters used during loading.

	en_cmx7262Modes uMode;							// Codec mode.
	uint8_t	uInterface;									// CBUS interface that the instance maps too.
	__IO uint16_t uIRQ_STATUS_REG;			// Shadow register for IRQ status reads.
	uint16_t uIRQ_ENABLE_REG;						// Shadow register for the IRQ enable.
	uint16_t uPacketSize;								// Size of vocoded speech packets in bytes written to and
																			// from the CMX7262. Packets are multiples of TWELP
																			// frames and their size depending on the type of encoding,
																			// FEC etc.
	__IO uint8_t *pDataBuffer;					// Defined as volatile because it is updated by an IRQ.
	__IO uint32_t uIRQRequest;					// Bit field for CMX7262 IRQ requests.
	__IO uint32_t uError;								// Bit field to indicate any errors.
	uint16_t uOutputGain;								// Parameter used by calls to control.
	signed int FIVersion;

	// Note the "signed int" definitions below to match up with the pointer types within the MMI definitions.
	signed int sInputGain;							// Parameter used by calls to control.

	void (*Callback_ODA)(void);
	void (*Callback_IDW)(void);
	void (*Callback_OV)(void);
	void (*Callback_UF)(void);
	void (*Callback_PLV)(void);

} CMX7262_TypeDef;	



uint8_t CMX7262_CheckModule(SPI_HandleTypeDef *hspi);

uint16_t SDR_Load_FI (cmxFI_TypeDef *pFI);

uint16_t  CMX7262_Init (CMX7262_TypeDef  *pCmx7262, SPI_HandleTypeDef *hspi);
uint16_t  CMX7262_InitHardware (CMX7262_TypeDef *pCmx7262);
void CMX7262_Config (CMX7262_TypeDef  *pCmx7262, uint16_t uConfig);
void CMX7262_Routing (CMX7262_TypeDef  *pCmx7262, uint16_t uData);
void CMX7262_AudioInputGain (CMX7262_TypeDef  *pCmx7262, uint16_t uGain);
void CMX7262_AudioOutputGain (CMX7262_TypeDef  *pCmx7262, uint16_t uGain);
void CMX7262_FlushStatusReg (CMX7262_TypeDef  *pCmx7262);
uint16_t CMX7262_ConfigClocks (CMX7262_TypeDef  *pCmx7262);
void  CMX7262_AnalogBlocks (CMX7262_TypeDef *pCmx7262);

uint16_t CMX7262_Transcode(CMX7262_TypeDef *pCmx7262, uint16_t uMode);

void CMX7262_Idle(CMX7262_TypeDef *pCmx7262);
void CMX7262_Encode(CMX7262_TypeDef *pCmx7262);
void CMX7262_Decode(CMX7262_TypeDef *pCmx7262);
void CMX7262_EncodeDecode_Audio(CMX7262_TypeDef *pCmx7262);
void CMX7262_EncodeDecode_Audio2CBUS(CMX7262_TypeDef *pCmx7262);
void CMX7262_EncodeDecode_CBUS2Audio(CMX7262_TypeDef *pCmx7262);
void CMX7262_Test_AudioOut(CMX7262_TypeDef *pCmx7262);
void CMX7262_Encode_CBUS2CBUS(CMX7262_TypeDef *pCmx7262);

void CMX7262_EnableIRQ(CMX7262_TypeDef *pCmx7262, uint16_t uIRQ);
void CMX7262_DisableIRQ(CMX7262_TypeDef *pCmx7262, uint16_t uIRQ);
void CMX7262_IRQ(CMX7262_TypeDef  *pCmx7262);

void CMX7262_EnableSquelch(CMX7262_TypeDef  *pCmx7262);
void CMX7262_DisableSquelch(CMX7262_TypeDef  *pCmx7262);
void CMX7262_EnableVOX(CMX7262_TypeDef  *pCmx7262);
void CMX7262_DisableVOX(CMX7262_TypeDef  *pCmx7262);
void CMX7262_ReadPLEVEL(CMX7262_TypeDef  *pCmx7262, uint16_t* uPlevel);

void CMX7262_RxFIFO(CMX7262_TypeDef *pCmx7262, uint8_t *pData);
void CMX7262_TxFIFO(CMX7262_TypeDef *pCmx7262, uint8_t *pData);

void CMX7262_RxFIFO_Audio(CMX7262_TypeDef *pCmx7262, uint8_t *pData, uint8_t numFrames);
void CMX7262_TxFIFO_Audio(CMX7262_TypeDef *pCmx7262, uint8_t *pData, uint8_t numFrames);

#ifndef SMART_PROTOTYPE
void CMX7262_HardwareReset(void);
#endif

//-------------------------------------- CBUS DEFINES AND FUNCTIONS ----------------------------------------------

 /* Maximum Timeout values for flags waiting loops. These timeouts are not based
    on accurate values, they just guarantee that the application will not remain
    stuck if the SPI communication is corrupted.
    You may modify these timeout values depending on CPU frequency and application
    conditions (interrupts routines ...). */
#define CBUS_TIME_OUT         ((uint32_t)0x1000)


/* Dummy Byte Send by the SPI Master device in order to generate the Clock to the Slave device */
#define CBUS_DUMMY_BYTE                 ((uint8_t)0x00)


void CBUS_Write16 (uint8_t addr, uint16_t *pTxData, uint16_t size);
void CBUS_Read16 (uint8_t addr, uint16_t *pRxData, uint16_t size);
void CBUS_Write8 (uint8_t uAddress, uint8_t *data_ptr, uint16_t uAccesses);
void CBUS_Read8 (uint8_t uAddress, uint8_t *data_ptr, uint16_t uAccesses);
uint16_t CBUS_WaitBitSet16(uint8_t Address, uint16_t uMask);
uint16_t	CBUS_WaitBitClr8(uint8_t Address, uint8_t uMask);
uint8_t CBUS_SendByte(uint8_t byte);
void CBUS_SetCSNLow(void);
void CBUS_SetCSNHigh(void);
void CMX7262_RESET(void);

	 
#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __CMX7262_H */
