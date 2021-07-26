#include "cmx7262.h"
#include "CMX7262_FI.h"

#ifdef RTOS
#include "cmsis_os.h"
#endif

SPI_HandleTypeDef *hspi_CMX7262 = &hspi3;

/**
  * @brief This function provides minimum delay (in milliseconds) based
  *        on variable incremented
  * @param Delay  specifies the delay time length, in milliseconds
  * @retval None
  */
void CMX7262_WaitMs(uint32_t ms)
{
#ifdef RTOS
	osDelay(ms);
#else
	HAL_Delay(ms);
#endif
}

/**
  * @brief Provides a tick value in millisecond
  * @retval tick value
  */
uint32_t CMX7262_GetSysTick(void)
{
#ifdef RTOS
	return osKernelSysTick();
#else
	return HAL_GetTick();
#endif
}

/**
  * @brief Checking the functionality of the module
  * @retval 1 - checking success, 0 - checking fail
  */
uint8_t CMX7262_CheckModule(SPI_HandleTypeDef *hspi)
{
	uint16_t data;
	
	//Передаем команду General Reset
	data = 0;
	CBUS_Write8(1, (uint8_t *)&data, 0);
	
	// Wait for a 300 ms.
	CMX7262_WaitMs(300);
	
	//Передаем команду запроса FIFO output level
	CBUS_Read8(0x4F, (uint8_t*)&data, 1);
	
	//Должны принять 3
	if(data != 3)
		return 0;
	
	return 1;
}


/**
  * @brief This function is to download the image file in CMX7262
  * @param Structure data for download image file
  * @retval 1 - download success, 0 - download fail
  */
uint16_t SDR_Load_FI (cmxFI_TypeDef *pFI)
{
	uint16_t	*pData;
	uint16_t 	start_code;
	uint16_t 	length;
	uint16_t	data;
	uint16_t	state;
	uint32_t tickStart;
	
	#ifdef CMX7262_IMAGE_IN_EEPROM
	uint16_t nDataWord;
	#endif

	uint16_t	uTxFIFOCount;
        
	char tempstr[5];        //used to converse the version from hex to decimal

	uTxFIFOCount = 0;
	data = 0;
	state = 0;
	// Write a general reset to the pDSP6
	CBUS_Write8(1, (uint8_t *)&data, 0);
	// Wait for a 300 ms.
	CMX7262_WaitMs(300);

	// Read the FIFO output level. It should be 3
	CBUS_Read8(0x4F, (uint8_t*)&data, 1);

	// Read the 3 device check words.
	if (data == 3)
	{
		CBUS_Read16 (0x4D, &data, 1);
		CBUS_Read16 (0x4D, &data, 1);
		CBUS_Read16 (0x4D, &data, 1);
	}
	else
	{
		// If there are no device check words, we return a failure.
		return 0;
	}
	// Initialise the data pointer to the start of the FI image.
	pData = pFI->db1_start_address;
	while (1)
	{
			switch (state)
			{
				case 0:

					// For each of the data blocks set the start codes and length from the definitions in
					// the FI header file. The address of pData is used to identify the current data block.
					if (pData == pFI->db1_start_address)
					{
						start_code = pFI->db1_ptr;
						length = pFI->db1_len;
					}
					else if (pData==pFI->db2_start_address)
					{
						start_code =  pFI->db2_ptr;
						length = pFI->db2_len;
					}
					else if (pData==LOAD_FI_END)
					{
						start_code = pFI->activate_ptr;
						length = pFI->activate_len;
					}
					else
					{
						// If the data pointer setting is not recognised then we have an error condition.
						// So return with an error code.
						return 0;
					}
					state++;
					break;

				case 1:

					CBUS_Write16(0x49, &length, 1); // Write Block N Length (DBN_len) to Audio In data word - $49 write register
					CBUS_Write16(0x49, &start_code, 1); // Write Start Block N Address (DBN_ptr) to Audio In data word - $49 write register

					// At this point, if the length is 0 then we have reached the end of the data blocks and the
					// FI should be loaded, so return 1.
					if (length == 0)
					{
						// Check the FI has programmed by checking the programming bit - protect the loop
						// with a 1 second time out. Return 1 for success or 0 for failure..

						tickStart = CMX7262_GetSysTick();

						while ((CMX7262_GetSysTick() - tickStart) < 1000)
						{
							CBUS_Read16 (0x7E, &data, 1);
							if (data & 0x4000)
							{
								//read version code
								CBUS_Read16 (0x4D, &data, 1);  //device type; ignore
								CBUS_Read16 (0x4D, &data, 1);  //version (in hex) e.g. version 1.0.0.0 will be 0x1000
								sprintf(tempstr,"%X",data);             //convert to string "1000"
								return 1;
							}
//							if (data & 0x4000 && (pFI->type==CMX7161FI))
//							{
//								//read version code
//                CBUS_Read16 (0x4D, &data, 1);
//                CBUS_Read16 (0x4D, &data, 1);
//                sprintf(tempstr,"%X",data);
//                return 1;
//							}
						}
						// Program Failure.
						return 0;
					}
					else
					{
						// Move to next state to programme the target device with
						// the function image.
						state++;
					}

	 				break;

				case 2:
					// There is a 128 deep FIFO on the CBUS interface which allows us to write
					// 128 words before checking the level register (0x4B). Check the level
					// register if the Tx FIFO count is 128. If the level does not reach zero in 1 second
					// we return an error code.
					if (uTxFIFOCount >= 128)
					{
						if (!CBUS_WaitBitClr8(0x4B, 0xFF))
							// Input FIFO level failure.
							return 0;
						uTxFIFOCount = 0;
					}
					
					#ifdef CMX7262_IMAGE_IN_EEPROM
					ReadWordFromEEPROM((uint32_t)pData,&nDataWord);
					pData++;
					CBUS_Write16(0x49,&nDataWord,1,uInterface);
					#else
					CBUS_Write16(0x49, pData++, 1);
					#endif
					length--;
					uTxFIFOCount++;

					// When the length is 0 we will be at the end of the current data block, so we
					// then work out which data block it is and set the data pointer to the next data
					// block or to END. If length is not 0 then there is still more data block to output.

					if (length == 0)
					{
						// If we are at the end of block 1 set the data pointer to the start of block 2.
						// Check if we are at the end of Block 1. If we are ....
						if (pData == (pFI->db1_start_address + pFI->db1_len))
						{
							// Read the level register but doing nothing with it.
							CBUS_Read8 (0x4F, (uint8_t *) &data, 1);
							// We are at the end of block 1, so read the checksums for the block and
							// return a failure if they are wrong.
							CBUS_Read16 (0x4D, &data, 1);
							if (data != pFI->db1_chk_hi)
								return 0;
							CBUS_Read16 (0x4D, &data, 1);
							if(data != pFI->db1_chk_lo)
								return 0;
							pData = pFI->db2_start_address;
						}
						// If we are at the end of block 2 set the data pointer to END.
						else if (pData == (pFI->db2_start_address + pFI->db2_len))
						{
							// Read the level register but doing nothing with it.
							CBUS_Read8 (0x4F, (uint8_t *)&data, 1);
							// We are at the end of block 2, so read the checksums for the block and
							// return a failure if they are wrong.
							CBUS_Read16 (0x4D, &data, 1);
							if (data != pFI->db2_chk_hi)
								return 0;
							CBUS_Read16 (0x4D, &data, 1);
							if(data != pFI->db2_chk_lo)
								return 0;
							pData = LOAD_FI_END;
						}
						state = 0;
					}
					else
						state = 2;
					break;
			}
		}

}


/**
  * @brief Initialization CMX7262
  * @param Structure data for CMX7262
  * @param Structure spi of the periphery
  * @retval 1 - download success, 0 - download fail
  */
uint16_t  CMX7262_Init(CMX7262_TypeDef *pCmx7262, SPI_HandleTypeDef *hspi)
{
	
	pCmx7262->FI.activate_len = ACTIVATE_LEN;								// Initialise to the FI load definitions above
	pCmx7262->FI.activate_ptr = ACTIVATE_PTR;
	pCmx7262->FI.db1_chk_hi = DB1_CHK_HI;
	pCmx7262->FI.db1_chk_lo = DB1_CHK_LO;
	pCmx7262->FI.db1_len = DB1_LEN;
	pCmx7262->FI.db1_ptr = DB1_PTR;
	pCmx7262->FI.db1_start_address = db1;

	pCmx7262->FI.db2_chk_hi = DB2_CHK_HI;
	pCmx7262->FI.db2_chk_lo = DB2_CHK_LO;
	pCmx7262->FI.db2_len = DB2_LEN;
	pCmx7262->FI.db2_ptr = DB2_PTR;
	pCmx7262->FI.db2_start_address = db2;

	pCmx7262->uInterface = CBUS_INTERFACE_CMX7262;
	pCmx7262->uMode = CMX7262_INIT_MODE;
	pCmx7262->uPacketSize = 0;							// How many bytes we read and write.
	pCmx7262->uIRQRequest = 0;							// Control bits, set by IRQ, MMI, SysTick
	pCmx7262->pDataBuffer = NULL;
	pCmx7262->uIRQ_STATUS_REG = 0;					// Shadow register.
	pCmx7262->uIRQ_ENABLE_REG = 0;					// Shadow register.
	pCmx7262->uOutputGain = 0;
	pCmx7262->uError = 0;										// Clear error field.

	pCmx7262->sInputGain = CMX7262_INPUT_GAIN_DEFAULT;

	if (!SDR_Load_FI(&pCmx7262->FI))
	{
#ifdef CMX7262_DEBUG
		printf("Load FI error!\n");
#endif
		pCmx7262->uError |= CMX7262_FI_LOAD_ERROR;
		return 0;
	}
#ifdef CMX7262_DEBUG
		printf("Load FI ok!\n");
#endif
	// Initialise clocks, analog blocks, input and output gains and reg done select.
	if (!CMX7262_InitHardware (pCmx7262))
	{
		pCmx7262->uError |= CMX7262_CONFIG_CLK_ERROR;
#ifdef CMX7262_DEBUG
		printf("Init hardware error!\n");
#endif
		return 0;
	}
#ifdef CMX7262_DEBUG
		printf("Init hardware ok!\n");
#endif
	// Set up packet length, hard decision decoding, FEC disabled.
	//CMX7262_Config(pCmx7262, THREE_FRAME | HDD | FEC);
	CMX7262_Config(pCmx7262, ONE_FRAME);
		
	// Clear any bits set in the status register and align the shadow register.
	CMX7262_FlushStatusReg(pCmx7262);
	
	return 1;
}

/**
 * @brief Initialization mode Idle
 * @param Structure data for CMX7262
 * @retval No retval
 */
void CMX7262_Idle(CMX7262_TypeDef *pCmx7262)
{
	// Setting the codec to IDLE when it is already IDLE sometimes causes a problem with the
	// the CMX7262 FI. This has been verified as a known issue and the problem found.

	if (pCmx7262->uMode != CMX7262_IDLE_MODE)
	{
		// Disable CBUS IRQs before putting the codec into into Idle mode.
		pCmx7262->uIRQ_ENABLE_REG &= ~(IRQ+ODA+IDW+UF);
		CBUS_Write16(IRQ_ENABLE_REG, &pCmx7262->uIRQ_ENABLE_REG, 1);

		if (!CMX7262_Transcode(pCmx7262,CMX7262_VCTRL_IDLE)){
			pCmx7262->uError |= CMX7262_IDLE_ERROR;
#ifdef CMX7262_DEBUG
			printf("CMX7262_Idle error!\n");
#endif
		}
		else{
			pCmx7262->uMode = CMX7262_IDLE_MODE;
#ifdef CMX7262_DEBUG
			printf("CMX7262_Idle ok!\n");
#endif
		}
	}
	// The codec could have underflowed or set any of the IRQ flags before the Idle mode change took effect,
	// As we confirm the mode change we read the status register and pick up any other flags such as the
	// underflow flag (which is valid for a starved decoder). Therefore after the idle we clear the flags
	// in the shadow status register. As good measure I have added this to all calls to Idle - rrespective
	// of wether the hardware is already idle or not.
	pCmx7262->uIRQ_STATUS_REG &= ~(IRQ+ODA+IDW+OV+UF);

}

/**
 * @brief Initialization mode Encode
 * @param Structure data for CMX7262
 * @retval No retval
 */
void CMX7262_Encode (CMX7262_TypeDef *pCmx7262)
{
	// PCM samples in through audio port and TWELP out through CBUS - in relation to the CMX7262
	CMX7262_Routing(pCmx7262, SRC_AUDIO | DEST_CBUS);
	
	// The encoder is started, there will be a packet delay before  we are requested to service it..
	if (!CMX7262_Transcode (pCmx7262,CMX7262_VCTRL_ENCODE)){
		pCmx7262->uError |= CMX7262_ENCODE_ERROR;
#ifdef CMX7262_DEBUG
		printf("CMX7262_Encode error!\n");
#endif
	}
	else
	{
		// Set the soft copy of the mode before we enable the IRQ because this is used by the IRQ
		// to set the appropriate request flags.
		pCmx7262->uMode = CMX7262_ENCODE_MODE;
		CMX7262_EnableIRQ(pCmx7262, IRQ+ODA+PLV);
#ifdef CMX7262_DEBUG
		printf("CMX7262_Encode ok!\n");
#endif
	}

}

/**
 * @brief Initialization mode Decode
 * @param Structure data for CMX7262
 * @retval No retval
 */
void CMX7262_Decode (CMX7262_TypeDef *pCmx7262)
{
	// PCM samples out through audio port and TWELP in through CBUS - in relation to the CMX7262.
	CMX7262_Routing(pCmx7262, SRC_CBUS | DEST_AUDIO);
	// So this routine is taking a long time to execute. About 10mS.
	if(!CMX7262_Transcode(pCmx7262,CMX7262_VCTRL_DECODE)){
		pCmx7262->uError |= CMX7262_DECODE_ERROR;
#ifdef CMX7262_DEBUG
		printf("CMX7262_Decode error!\n");
#endif
	}
	else
	{
		pCmx7262->uMode = CMX7262_DECODE_MODE;
		CMX7262_EnableIRQ(pCmx7262, IRQ+UF+IDW);        //enable underflow irq so we know when the call is over.
#ifdef CMX7262_DEBUG
		printf("CMX7262_Decode ok!\n");
#endif
	}
}

/**
 * @brief Initialization mode EncodeDecode. Input audio data, output audio data
 * @param Structure data for CMX7262
 * @retval No retval
 */
void CMX7262_EncodeDecode_Audio (CMX7262_TypeDef *pCmx7262)
{
	
	// PCM samples in through audio port, encode, decode and out through audio port - in relation to the CMX7262
	CMX7262_Routing(pCmx7262, SRC_AUDIO | DEST_AUDIO);
	// The encoder+decoder is started, there will be a delay before we are requested to service it..
	if (!CMX7262_Transcode (pCmx7262,CMX7262_VCTRL_ENCDEC)){
		pCmx7262->uError |= CMX7262_ENCODE_ERROR;
#ifdef CMX7262_DEBUG
		printf("CMX7262_EncodeDecode_Audio error!\n");
#endif
	}
	else
	{
		// Set the soft copy of the mode before we enable the IRQ because this is used by the IRQ
		// to set the appropriate request flags.
		pCmx7262->uMode = CMX7262_ENCDEC_MODE;
#ifdef CMX7262_DEBUG
		printf("CMX7262_EncodeDecode_Audio ok!\n");
#endif
	}
}

/**
 * @brief Initialization mode EncodeDecode. Input audio data, encoded data output via CBUS
 * @param Structure data for CMX7262
 * @retval No retval
 */
void CMX7262_EncodeDecode_Audio2CBUS (CMX7262_TypeDef *pCmx7262)
{
	// PCM samples in through audio port, encode, decode and out through cbus port - in relation to the CMX7262
	CMX7262_Routing(pCmx7262, SRC_AUDIO | DEST_CBUS);
	// The encoder+decoder is started, there will be a delay before we are requested to service it..
	if (!CMX7262_Transcode (pCmx7262,CMX7262_VCTRL_ENCDEC)){
		pCmx7262->uError |= CMX7262_ENCODE_ERROR;
#ifdef CMX7262_DEBUG
		printf("CMX7262_EncodeDecode_Audio2CBUS error!\n");
#endif
	}
	else
	{
		// Set the soft copy of the mode before we enable the IRQ because this is used by the IRQ
		// to set the appropriate request flags.
		pCmx7262->uMode = CMX7262_ENCDEC_MODE;
#ifdef CMX7262_DEBUG
		printf("CMX7262_EncodeDecode_Audio2CBUS ok!\n");
#endif
		CMX7262_EnableIRQ(pCmx7262, IRQ+ODA+OV);		// ожидаем прерываний по следующим событиям:
																								// "Output Data Available" (доступны новые выходные данные)
																								// "Overflow" (переполнение выходного буфера ввиду того, что хост не успел вычитать данные)
	}	
}

/**
 * @brief Initialization mode EncodeDecode. Input encoded data via CBUS, output audio data
 * @param Structure data for CMX7262
 * @retval No retval
 */
void CMX7262_EncodeDecode_CBUS2Audio (CMX7262_TypeDef *pCmx7262)
{
	// PCM samples in through cbus port, encode, decode and out through audio port - in relation to the CMX7262
	CMX7262_Routing(pCmx7262, SRC_CBUS | DEST_AUDIO);
	// The encoder+decoder is started, there will be a delay before we are requested to service it..
	if (!CMX7262_Transcode (pCmx7262,CMX7262_VCTRL_ENCDEC)){
		pCmx7262->uError |= CMX7262_ENCODE_ERROR;
#ifdef CMX7262_DEBUG
		printf("CMX7262_EncodeDecode_CBUS2Audio error!\n");
#endif
	}
	else
	{
		// Set the soft copy of the mode before we enable the IRQ because this is used by the IRQ
		// to set the appropriate request flags.
		pCmx7262->uMode = CMX7262_ENCDEC_MODE;
		CMX7262_EnableIRQ(pCmx7262, IRQ+IDW+UF);    // ожидаем прерываний по следующим событиям:
																								// "Input Data Wanted" (устройство готово принять новые данные)
																								// "Underflow" (устройству не хватает данных для работы в непрерывном режиме)
#ifdef CMX7262_DEBUG
		printf("CMX7262_EncodeDecode_CBUS2Audio ok!\n");
#endif
	}	
}

/**
 * @brief Initialization mode Encode. Input PCM data via CBUS, output encoded data via CBUS
 * @param Structure data for CMX7262
 * @retval No retval
 */
void CMX7262_Encode_CBUS2CBUS (CMX7262_TypeDef *pCmx7262)
{
	// PCM samples in through CBUS and TWELP out through CBUS - in relation to the CMX7262
	CMX7262_Routing(pCmx7262, SRC_CBUS | DEST_CBUS);
	
	// The encoder is started, there will be a packet delay before  we are requested to service it..
	if (!CMX7262_Transcode (pCmx7262,CMX7262_VCTRL_ENCODE)){
		pCmx7262->uError |= CMX7262_ENCODE_ERROR;
#ifdef CMX7262_DEBUG
		printf("CMX7262_Encode_CBUS2CBUS ok!\n");
#endif
	}
	else
	{
		// Set the soft copy of the mode before we enable the IRQ because this is used by the IRQ
		// to set the appropriate request flags.
		pCmx7262->uMode = CMX7262_ENCODE_MODE;
		CMX7262_EnableIRQ(pCmx7262, IRQ+ODA);
#ifdef CMX7262_DEBUG
		printf("CMX7262_Encode_CBUS2CBUS ok!\n");
#endif
	}
}

/**
 * @brief Initialization mode Test. Input test signal via CBUS, output audio signal
 * @param Structure data for CMX7262
 * @retval No retval
 */
void CMX7262_Test_AudioOut (CMX7262_TypeDef *pCmx7262)
{
	uint16_t uData;
	uint16_t nFreq = CMX7262_FREQ_SIGNAL_IN_TESTMODE; 	//Частота формируемого колебания, Гц
	
	//Источник сигнала (в режиме теста звукового выхода должен игнорироваться) - CBUS,
	//адресат сигнала - аудиовыход
	CMX7262_Routing(pCmx7262, SRC_CBUS | DEST_AUDIO);
	
	//Код частоты NCO
	//Расчет кода частоты - п.8.1.9 Frequency Control документа D/7262_FI-1.x/4 August 2013 (datasheet)
	uData = floor(((double)nFreq * USHRT_MAX)/CMX7262_FREQ_SAMPLING);
	CBUS_Write16(FREQ_CONTROL, &uData, 1);
	
	// The test mode is started, there will be a delay before we are requested to service it..
	if (!CMX7262_Transcode (pCmx7262,CMX7262_VCTRL_TEST)){
		pCmx7262->uError |= CMX7262_ENCODE_ERROR;
		printf("Error: CMX7262_TEST_MODE!\n");
	}
	else
	{
		// Set the soft copy of the mode before we enable the IRQ because this is used by the IRQ
		// to set the appropriate request flags.
		pCmx7262->uMode = CMX7262_TEST_MODE;
		CMX7262_EnableIRQ(pCmx7262, IRQ+ODA);	//не знаю, какие прерывания нужны в этом режиме
		printf("Mode: CMX7262_TEST_MODE!\n");
	}
}


/**
 * @brief Set mode
 * @param Structure data for CMX7262
 * @retval 1 - mode set success, mode set fail
 */
uint16_t CMX7262_Transcode(CMX7262_TypeDef *pCmx7262, uint16_t uMode)
{
	uint32_t tickstart;
	uint16_t uData;

	// Set the codec mode.
	CBUS_Write16(VCTRL_REG, (uint16_t *)&uMode, 1);
	
	tickstart = CMX7262_GetSysTick();
	// Wait until we have confirmation of the mode being set.
	while ((CMX7262_GetSysTick() - tickstart) < CMX7262_TRANSCODE_TIMEOUT)
	{
		CBUS_Read16 (IRQ_STATUS_REG, &uData, 1);
		pCmx7262->uIRQ_STATUS_REG |= uData;
		if ((pCmx7262->uIRQ_STATUS_REG & REGDONE) == REGDONE)
		{
			// Clear the REGDONE bit in the shadow regsiter.
			pCmx7262->uIRQ_STATUS_REG &= (uint16_t)(~REGDONE);
			return 1;
		}
	}
	
	// If we get here we have timed out and the mode selection was not successful,
	// so return 0.
	return 0;
}

/**
 * @brief Default initialization hardware
 * @param Structure data for CMX7262
 * @retval 1 - hardware set success, hardware set fail
 */
uint16_t CMX7262_InitHardware(CMX7262_TypeDef *pCmx7262)
{
	uint16_t uData;

	// Configure the clocks
	if(!CMX7262_ConfigClocks(pCmx7262))
		return 0;

	// Configure analog blocks
	CMX7262_AnalogBlocks(pCmx7262);
	// Setup the input and output gains.
	CMX7262_AudioInputGain(pCmx7262,CMX7262_INPUT_GAIN_DEFAULT);
	CMX7262_AudioOutputGain(pCmx7262,CMX7262_OUPUT_GAIN_DEFAULT);
	
	// Enable register write confirmation for VCTRL
	uData = 0x0008;
	CBUS_Write16(REG_DONE_SELECT, &uData, 1);

	return 1;
}

/**
 * @brief Set routing
 * @param Structure data for CMX7262
 * @param Audio source and Audio destination
 * @retval No retval
 */
void CMX7262_Routing(CMX7262_TypeDef  *pCmx7262, uint16_t uData)
{
	CBUS_Write16(AUDIO_ROUTING_REG, (uint16_t *)&uData, 1);
}

/**
 * @brief Set configuration frames per packet
 * @param Structure data for CMX7262
 * @param Frames per packet.
 * @retval No retval
 */
void CMX7262_Config(CMX7262_TypeDef  *pCmx7262, uint16_t uConfig)
{
	if((uConfig & FEC_MASK)==FEC)
	{
		// Calculate the size of a packet transfer based on the size of a frame and the number of
		// frames per packet.
		pCmx7262->uPacketSize = TWELP_FEC_HDD_FRAME_SIZE_BYTES * (FRAME_MASK & uConfig);
	}
	else
	{
		// Calculate the size of a packet transfer based on the size of a frame and the number of
		// frames per packet.
		pCmx7262->uPacketSize = TWELP_HDD_FRAME_SIZE_BYTES * (FRAME_MASK & uConfig);
	}
	CBUS_Write16(VCFG_REG, &uConfig, 1);

}

/**
 * @brief Set configuration clocks
 * @param Structure data for CMX7262
 * @retval 1 - configuration clocks success 0 - configuration clocks fail
 */
uint16_t CMX7262_ConfigClocks(CMX7262_TypeDef  *pCmx7262)
{
	uint16_t data;

	// Flush by reading the status register.
	CBUS_Read16 (IRQ_STATUS_REG, &data, 1);

	data = 0x110;	// Select program block 1.1
	CBUS_Write16 (VCTRL_REG, &data, 1);
	//P1.1
	data = 0x41;		// Idle Internal Clock Divide
	CBUS_Write16 (PROG_REG, &data, 1);
	if(!CBUS_WaitBitSet16 (IRQ_STATUS_REG, PRG))
		return 0;		// Program fail.
	//P1.2
	data = 40;			// Set ref clk Divide in Rx or Tx Mode
	CBUS_Write16 (PROG_REG, &data, 1);
	if(!CBUS_WaitBitSet16 (IRQ_STATUS_REG, PRG))
	{
		return 0;		// Program fail.
	}
	//P1.3
	data = 208;		// Set PLL clk Divide in Rx or Tx Mode
	CBUS_Write16 (PROG_REG, &data, 1);
	if(!CBUS_WaitBitSet16 (IRQ_STATUS_REG, PRG))
		return 0;		// Program fail.
	//P1.4
	data = 0x41;		// Set Tx/Rx Internal Clock Divide
	CBUS_Write16 (PROG_REG, &data, 1);
	if(!CBUS_WaitBitSet16 (IRQ_STATUS_REG, PRG))
		return 0;		// Program fail.
	//P1.5
	data = 78;		// Set I/Q IO Clock Divide
	CBUS_Write16 (PROG_REG, &data, 1);
	if(!CBUS_WaitBitSet16 (IRQ_STATUS_REG, PRG))
		return 0;		// Program fail
	//P1.6
	data = 0x20D9;		// MAINPLLCON0
	CBUS_Write16 (PROG_REG,&data, 1);
	if(!CBUS_WaitBitSet16 (IRQ_STATUS_REG, PRG))
		return 0;		// Program fail
	//Set P1.14
	data = 0xE10;	// Select program block 1.14
	CBUS_Write16(VCTRL_REG, &data, 1);
	//P1.14
	data = 1;				// XTAL Driver Enable
	CBUS_Write16(PROG_REG, &data, 1);
	if(!CBUS_WaitBitSet16(IRQ_STATUS_REG, PRG))
		return 0;		// Program fail.

	return 1;			// Configuration was a success.

}

/**
 * @brief Set analog outputs
 * @param Structure data for CMX7262
 * @retval No retval
 */
void CMX7262_AnalogBlocks(CMX7262_TypeDef *pCmx7262)
{
	uint16_t uData;

	// Power up the appropriate analog blocks - Start
	// DAC Pwr, OP Bias, SPKR1/SPKR2, Enable DrvPwr 1&2
	#ifndef CMX7262_SPKR1_OUT
	//uData = 0x086A;
	//uData = 0x0820;
	uData = 0x08A0;
	#else
	uData = 0x88A0;
	#endif	
	CBUS_Write16(ANAOUT_CONFIG, &uData, 1);
	// Single ended uses ANAIN2, differential uses ANAIN1.
	// J24 Pins 1 to 2, 3 to 4 and 7 to 8, 9 to 10 need shorting.
	//uData = 0x0802; 	// ANAIN1 - Differential input
	// ADC Pwr, ANA Sw, ANAIN2 Pwr
	uData = 0x0A08;
	CBUS_Write16(ANAIN_CONFIG, &uData, 1);
	// Power up the appropriate analog blocks - End

}


/**
* @brief Set audio input gain
* @param Structure data for CMX7262
* @param Value register AIG
* @retval No retval
*/
void CMX7262_AudioInputGain (CMX7262_TypeDef  *pCmx7262, uint16_t uGain)
{
	uint16_t uData;
	#ifdef DEBUG_CMX7262_MIC_MAXGAIN
	uData = (uint16_t)pCmx7262->sInputGain;
	uData = 7;
	#endif
	// Position the gain to ANAIN2
	uData = uGain << 8;
	CBUS_Write16(ANAIN_GAIN, &uData, 1);
}

/**
* @brief Sets speaker output course gain
* @param Structure data for CMX7262
* @param Value register AOG3
* @retval No retval
*/
void CMX7262_AudioOutputGain (CMX7262_TypeDef  *pCmx7262, uint16_t uGain)
{
	CBUS_Write16(AOG3, &uGain, 1);
}

/**
* @brief Clear status register and write in the shadow register
* @param Structure data for CMX7262
* @param Value register AOG3
* @retval No retval
*/
void CMX7262_FlushStatusReg (CMX7262_TypeDef  *pCmx7262)
{
	uint16_t uData;
	// Flush by reading the status register.
	CBUS_Read16 (IRQ_STATUS_REG, &uData, 1);
	// Align the shadow register to the hardware status register.
	CBUS_Read16 (IRQ_STATUS_REG, (uint16_t *)(&pCmx7262->uIRQ_STATUS_REG), 1);
}


/**
* @brief Enable IRQ
* @param Structure data for CMX7262
* @param Interrupt
* @retval No retval
*/
void CMX7262_EnableIRQ (CMX7262_TypeDef *pCmx7262, uint16_t uIRQ)
{
	// Set the bits in the interrupt enable shadow register, then write to the cbus
	// register in hardware.
	pCmx7262->uIRQ_ENABLE_REG |= uIRQ;
	CBUS_Write16 (IRQ_ENABLE_REG, &pCmx7262->uIRQ_ENABLE_REG, 1);
}

/**
* @brief Disable IRQ
* @param Structure data for CMX7262
* @param Interrupt
* @retval No retval
*/
void CMX7262_DisableIRQ (CMX7262_TypeDef *pCmx7262, uint16_t uIRQ)
{
	// Clear the mask bits in the interrupt enable shadow register, then write to the
	// cbus register in hardware.
	pCmx7262->uIRQ_ENABLE_REG &= ~uIRQ;
	CBUS_Write16(IRQ_ENABLE_REG, &pCmx7262->uIRQ_ENABLE_REG, 1);
	pCmx7262->uIRQ_STATUS_REG &= ~uIRQ;
}

/**
* @brief Enable squelch
* @param Structure data for CMX7262
* @retval No retval
*/
void CMX7262_EnableSquelch(CMX7262_TypeDef  *pCmx7262)
{
	uint16_t uData;

	uData = (CMX7262_NOISEGATE_FRAMEDELAY_DEFAULT<<12) | CMX7262_NOISEGATE_THRESHOLD_DEFAULT;
	CBUS_Write16(NOISE_GATE_REG, &uData, 1);
	uData = 1;	//Noise Supression -20dB
		//uint16_t uData = 1;	//Noise Supression -20dB (most aggressive noise suppression)
	CBUS_Write16(NOISE_REDUCTION, &uData, 1);
}

/**
* @brief Disable squelch
* @param Structure data for CMX7262
* @retval No retval
*/
void CMX7262_DisableSquelch(CMX7262_TypeDef  *pCmx7262)
{
	uint16_t uData = 0;

	CBUS_Write16(NOISE_GATE_REG, &uData, 1);
}

/**
* @brief Enable VOX
* @param Structure data for CMX7262
* @retval No retval
*/
void CMX7262_EnableVOX(CMX7262_TypeDef  *pCmx7262)
{
	CMX7262_EnableIRQ (pCmx7262, PLV);
}

/**
* @brief Disable VOX
* @param Structure data for CMX7262
* @retval No retval
*/
void CMX7262_DisableVOX(CMX7262_TypeDef  *pCmx7262)
{
	pCmx7262->uIRQ_ENABLE_REG &= ~(uint16_t)PLV;
	CBUS_Write16 (IRQ_ENABLE_REG, &pCmx7262->uIRQ_ENABLE_REG, 1);
}

/**
* @brief Read peak frame level
* @param Structure data for CMX7262
* @param Variable pointer to store the value
* @retval No retval
*/
void CMX7262_ReadPLEVEL(CMX7262_TypeDef  *pCmx7262, uint16_t* uPlevel)
{
	CBUS_Read16(PEAK_LEVEL, uPlevel, 1);
}


// These functions provide a clear interface to the DMR application calling them. They simply read and write
// data to and from the codec FIFOs and extract the interface and packet size from the Cmx7262 data structure.
/**
* @brief Rx packet
* @param Structure data for CMX7262
* @param Variable pointer to store the packet
* @retval No retval
*/
void CMX7262_RxFIFO (CMX7262_TypeDef  *pCmx7262, uint8_t *pData)
{

	CBUS_Read8(CBUS_VOCODER_OUT, pData, pCmx7262->uPacketSize);
}

void CMX7262_TxFIFO (CMX7262_TypeDef  *pCmx7262, uint8_t *pData)
{
	CBUS_Write8(CBUS_VOCODER_IN, pData, pCmx7262->uPacketSize);
}

/**
* @brief Tx packet
* @param Structure data for CMX7262
* @param Variable pointer to store the packet
* @param Number packets
* @retval No retval
*/
void CMX7262_RxFIFO_Audio (CMX7262_TypeDef  *pCmx7262, uint8_t *pData, uint8_t numFrames)
{
	CBUS_Read8(CBUS_AUDIO_OUT, pData, sizeof(uint16_t)*numFrames*CMX7262_AUDIOFRAME_SIZE_SAMPLES);
}

void CMX7262_TxFIFO_Audio (CMX7262_TypeDef  *pCmx7262, uint8_t *pData, uint8_t numFrames)
{
	CBUS_Write8(CBUS_AUDIO_IN, pData, sizeof(uint16_t)*numFrames*CMX7262_AUDIOFRAME_SIZE_SAMPLES);
}

/**
* @brief Process irq handler
* @param Structure data for CMX7262
* @retval No retval
*/
void CMX7262_ProcessIRQ(CMX7262_TypeDef * pCmx7262)
{
//	CMX7262_IRQ(&g_CMX7262Struct);

	uint16_t status = 0;

	CBUS_Read16(IRQ_STATUS_REG, &status, 1);

	if((status & CMX7262_IDW) == CMX7262_IDW && pCmx7262->uIRQ_ENABLE_REG & IDW){
		if(pCmx7262->Callback_IDW != NULL){
			pCmx7262->Callback_IDW();
		}
	}

	if((status & CMX7262_ODA) == CMX7262_ODA && pCmx7262->uIRQ_ENABLE_REG & ODA){
		if(pCmx7262->Callback_ODA != NULL){
			pCmx7262->Callback_ODA();
		}
	}

	if((status & CMX7262_UF) == CMX7262_UF && pCmx7262->uIRQ_ENABLE_REG & UF){
		if(pCmx7262->Callback_UF != NULL){
			pCmx7262->Callback_UF();
		}
	}

	if((status & CMX7262_OV) == CMX7262_OV && pCmx7262->uIRQ_ENABLE_REG & OV){
		if(pCmx7262->Callback_OV != NULL){
			pCmx7262->Callback_OV();
		}
	}

	if((status & CMX7262_PLV) == CMX7262_PLV && pCmx7262->uIRQ_ENABLE_REG & PLV){
			if(pCmx7262->Callback_PLV != NULL){
				pCmx7262->Callback_PLV();
			}
		}
}



/**
* @brief Hardware reset
* @retval No retval
*/
void CMX7262_HardwareReset(void)
{
	HAL_GPIO_WritePin(CMX7262_RESET_GPIO_Port, CMX7262_RESET_Pin, GPIO_PIN_RESET);
	CMX7262_WaitMs(50);
	HAL_GPIO_WritePin(CMX7262_RESET_GPIO_Port, CMX7262_RESET_Pin, GPIO_PIN_SET);
	CMX7262_WaitMs(100);
}

//-------------------------------------------- CBUS FUNCTIONS --------------------------------------------------------


__IO uint32_t  CBUSTimeout = CBUS_TIME_OUT;

/**
  * @brief Sends Byte through the SPI interface and return the Byte received
  * @param hspi : Handle of SPI Interface
  *				 nByteForTX: variable to send byte
  *				 nByteForRX: pointer to receive byte
  * @retval Result of Transmission (HAL Status)
  */
HAL_StatusTypeDef SPI_TransmitReceiveByte(SPI_HandleTypeDef *hspi, uint8_t nByteForTX, uint8_t *nByteForRX)
{
	HAL_StatusTypeDef status = HAL_OK; // начальное состояние HAL.

	status = HAL_SPI_TransmitReceive(hspi, &nByteForTX, nByteForRX, 1, 10);

	return(status);
}

/**
  * @brief  Sends a number of Bytes through the SPI interface and return the number of Bytes received
	* @param  hspi : Handle of SPI Interface
	* 				pTxData: pointer to transmission data buffer
	*					pRxData: pointer to reception data buffer to be
  *					Size: amount of data to be sent

  * @retval Result of Transmission (HAL Status)
  */
HAL_StatusTypeDef SPI_TransmitRecieve(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size)
{
	HAL_StatusTypeDef status = HAL_OK; // начальное состояние HAL.

	status = HAL_SPI_TransmitReceive(hspi, pTxData, pRxData, Size, 100);

	return(status);
}

/**
  * @brief  Sends a number of uint16_t through the CBUS
  * @param  addr : CBUS's internal address to write to
  * 				pTxData : pointer to transmission data buffer
  * 				size: Number of bytes to write.
  * @retval None
  */
void CBUS_Write16(uint8_t addr, uint16_t *pTxData, uint16_t size)
{
	uint16_t sizeCnt;

	/* Set chip select Low at the start of the transmission */
	CBUS_SetCSNLow();

	/* Send the Address of the register */
	CBUS_SendByte(addr);

	for (sizeCnt = 0; sizeCnt < size; sizeCnt++)
	{
		/* Send the data that will be written into the device (MSB First) */
		CBUS_SendByte((*(pTxData + sizeCnt) & 0xFF00)>>8);
		CBUS_SendByte(*(pTxData + sizeCnt) & 0x00FF);
	}

	/* Set chip select High at the end of the transmission */
	CBUS_SetCSNHigh();
}

/**
  * @brief  Read a number of uint16_t through the CBUS
  * @param  addr : CBUS's internal address to write to
  * 				pRxData : pointer to receive data buffer
  * 				size: Number of bytes to read.
  * @retval None
  */
void CBUS_Read16(uint8_t addr, uint16_t *pRxData, uint16_t size)
{
	uint16_t sizeCnt;

	*pRxData=0;

	/* Set chip select Low at the start of the transmission */

	CBUS_SetCSNLow();

	// Setup the CBUS address to read from.
	CBUS_SendByte(addr);

	for (sizeCnt = 0; sizeCnt < size; sizeCnt++)
	{
		*pRxData = 0;
		 // Send dummy byte (0x00) to generate the SPI clock to CBUS (Slave device).
		 // This reads the first byte for 2 byte accesses.
		 *pRxData = (CBUS_SendByte(CBUS_DUMMY_BYTE) << 8);

		 // Send dummy byte (0x00) to generate the SPI clock to CBUS (Slave device).
		 // This reads the second byte (for 2 byte accesses) or the first byte for single byte
		 // reads.
		 *pRxData |= CBUS_SendByte(CBUS_DUMMY_BYTE);
		 pRxData++;
	}

	 /* Set chip select High at the end of the transmission */
	CBUS_SetCSNHigh();
}


/**
  * @brief  Send a number of Bytes through the CBUS
  * @param  addr : CBUS's internal address to write to
  * 				pTxData : pointer to transmission data buffer
  * 				size: Number of bytes to read.
  * @retval None
  */
void CBUS_Write8(uint8_t addr, uint8_t *pTxData, uint16_t size)
{
	uint16_t sizeCnt;

	/* Set chip select Low at the start of the transmission */
	CBUS_SetCSNLow();

	/* Send the Address of the register */
	CBUS_SendByte(addr);

	for (sizeCnt = 0; sizeCnt < size; sizeCnt++)
	{
		/* Send the data that will be written into the device  */
		CBUS_SendByte(*(pTxData + sizeCnt));
	}

	/* Set chip select High at the end of the transmission */
	CBUS_SetCSNHigh();
}


/**
  * @brief  Read a number of Bytes through the CBUS
  * @param  addr : CBUS's internal address to write to
  * 				pRxData : pointer to receive data buffer
  * 				size: Number of bytes to read.
  * @retval None
  */
void CBUS_Read8(uint8_t uAddress, uint8_t *pRxData, uint16_t size)
{
	uint16_t sizeCnt;

	*pRxData=0;

	/* Set chip select Low at the start of the transmission */
	CBUS_SetCSNLow();
	
	// Setup the CBUS address to read from.
	CBUS_SendByte(uAddress);

	for (sizeCnt = 0; sizeCnt < size; sizeCnt++)
	{
		// Send dummy byte (0x00) to generate the SPI clock to CBUS (Slave device).
		// This reads the first byte for single byte  reads.
		*pRxData = CBUS_SendByte(CBUS_DUMMY_BYTE);
		pRxData++;
	}

	 /* Set chip select High at the end of the transmission */
	CBUS_SetCSNHigh();
}


/**
  * @brief  Continually read a CBUS register until the bits in the mask are set or the 1 second
  * timeout expires. Returns 1 when the mask is matched. Otherwise 0 for a timeout.
  * @param  addr : CBUS's internal address to write to
  * 				bitMask : bit mask
  * 				size: Number of bytes to read.
  * @retval 0 - bit not set, value bitMask - bit set
  */
uint16_t	CBUS_WaitBitSet16 (uint8_t addr, uint16_t bitMask)
{
	uint16_t data;
	uint32_t tickStart;

	tickStart = CMX7262_GetSysTick();

	while ((CMX7262_GetSysTick() - tickStart) < 1000)
	{
		data = 0;
		CBUS_Read16(addr, &data, 1);
		if (bitMask == (data & bitMask))
		{
			return data;
		}
	}
	return 0;
}


// Continually read a CBUS register until the bits in the mask are clear or the 1 second
// timeout expires. Returns 1 when the bits in the mask are clear. Otherwise 0 for a timeout.
/**
  * @brief Continually read a CBUS register until the bits in the mask are clear or the 1 second
  * timeout expires. Returns 1 when the bits in the mask are clear. Otherwise 0 for a timeout.
  * @param  addr : CBUS's internal address to write to
  * 				bitMask : bit mask
  * 				size: Number of bytes to read.
  * @retval 0 - bit not clear, 1 - bit clear
  */
uint16_t	CBUS_WaitBitClr8(uint8_t addr, uint8_t bitMask)
{
	uint8_t data;
	uint32_t tickStart;

	tickStart = CMX7262_GetSysTick();

	while ((CMX7262_GetSysTick() - tickStart) < 1000)
	{
		data = 0;
		CBUS_Read8(addr, &data, 1);
		if ((data & ~bitMask)==0)
		{
			return 1;
		}
	}
	return 0;
}

/**
  * @brief  Sends a Byte through the SPI interface and return the Byte received
  *         from the SPI bus.
  * @param  Byte : Byte send.
  * @retval The received byte value
  */
uint8_t CBUS_SendByte(uint8_t byte)
{
	uint8_t nByteForRX;
	
  SPI_TransmitReceiveByte(hspi_CMX7262, byte, &nByteForRX);
	
	return nByteForRX;
}

/**
  * @brief  Set the CSN bit low based on the mask
  * @param  No retval
  * @retval No retval
  */
void CBUS_SetCSNLow(void)
{
	HAL_GPIO_WritePin(CMX7262_CS_GPIO_Port, CMX7262_CS_Pin, GPIO_PIN_RESET);
}

/**
  * @brief  Set the CSN bit high based on the mask
  * @param  No retval
  * @retval No retval
  */
void CBUS_SetCSNHigh(void)
{
	HAL_GPIO_WritePin(CMX7262_CS_GPIO_Port, CMX7262_CS_Pin, GPIO_PIN_SET);
}
