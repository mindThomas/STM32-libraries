/* Copyright (C) 2018-2019 Thomas Jespersen, TKJ Electronics. All rights reserved.
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the MIT License
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the MIT License for further details.
 *
 * Contact information
 * ------------------------------------------
 * Thomas Jespersen, TKJ Electronics
 * Web      :  http://www.tkjelectronics.dk
 * e-mail   :  thomasj@tkjelectronics.dk
 * ------------------------------------------
 */
 
#include "EEPROM.h"
#include "stm32h7xx_hal.h"
#include "Debug.h"

EEPROM::EEPROM()
{
	resourceSemaphore_ = xSemaphoreCreateBinary();
	if (resourceSemaphore_ == NULL) {
		ERROR("Could not create EEPROM resource semaphore");
		return;
	}
	vQueueAddToRegistry(resourceSemaphore_, "EEPROM Resource");
	xSemaphoreGive( resourceSemaphore_ ); // give the resource the first time

	virtAddrTable_ = new std::vector<uint16_t>;
	if (virtAddrTable_ == NULL) {
		ERROR("Could not create EEPROM address table");
		return;
	}

	//uint16_t * sectionsEnum = (uint16_t *)&sections.internal;
	//EnableSection(sections.internal, sectionsEnum[1]-sectionsEnum[0]); // initialize EEPROM library section for reset state detection - sectionsEnum[1] gives the address of the next element after the internal
	EnableSection(sections.internal, sizeof(internal_state_t)); // initialize EEPROM library section for reset state detection
}

EEPROM::~EEPROM()
{
	if (resourceSemaphore_) {
		vQueueUnregisterQueue(resourceSemaphore_);
		vSemaphoreDelete(resourceSemaphore_);
	}
}

bool EEPROM::CheckForAssignedStructureChange(void)
{
	internal_state_t internalState;
	ReadData(sections.internal, (uint8_t *)&internalState, sizeof(internalState));

	// Check the state validation flag
	if (internalState.validationFlag != STATE_VALIDATION_FLAG) {
		return true; // validation flag does not seem to exist (be correct)
	}

	// Check the sections table checksum, to detect if the table has been changed in code/firmware
	if (internalState.sectionsChecksum != CalculateSectionsTableChecksum()) {
		return true; // section table has been changed
	}

	return false; // internal settings seems correct = not changed
}

uint32_t EEPROM::CalculateSectionsTableChecksum(void)
{
	uint32_t checksum = 0;
	uint16_t * sectionsEnum = (uint16_t *)&sections;
	int elements = sizeof(sections) / sizeof(uint16_t);
	for (int i = 0; i < elements; i++) {
		checksum += sectionsEnum[i];
	}

	return checksum;
}

void EEPROM::InitializeInternalState(void)
{
	internal_state_t internalState;
	internalState.validationFlag = STATE_VALIDATION_FLAG;
	internalState.sectionsChecksum = CalculateSectionsTableChecksum();
	WriteData(sections.internal, (uint8_t *)&internalState, sizeof(internalState));
}


bool EEPROM::EnableSection(uint16_t address, uint16_t sectionSize)
{
	bool SectionAlreadyInUse = false;

	if ((address % 2) != 0) return false; // address not aligned - section start addresses has to be aligned

	xSemaphoreTake( resourceSemaphore_, ( TickType_t ) portMAX_DELAY); // take hardware resource

	// sectionSize is given in bytes
	uint16_t virtAddress = address / 2;
	uint16_t sectionSize16 = sectionSize / 2 + (sectionSize % 2);

	// OBS. This function should have a check of whether the section is occupied already
	int16_t i;
	for (i = 0; i < sectionSize16; i++) {
		if (IsSectionInUse(virtAddress+i)) {
			SectionAlreadyInUse = true;
			break;
		}
		virtAddrTable_->push_back((uint16_t)(virtAddress+i));
	}
	if (SectionAlreadyInUse) { // we need to remove recently inserted (now occupied) addresses
		while (i > 0) {
			virtAddrTable_->pop_back();
			i--;
		}
	}

	/*if (!SectionAlreadyInUse) {
		HAL_FLASH_Unlock();
		Init(); // initialize (or reinitialize) EEPROM to include this newly added section
		HAL_FLASH_Lock();
	}*/

	xSemaphoreGive( resourceSemaphore_ ); // give hardware resource back

	return SectionAlreadyInUse;
}

bool EEPROM::IsSectionInUse(uint16_t address)
{
	uint16_t * table = virtAddrTable_->data();
	for (size_t i = 0; i < virtAddrTable_->size(); i++)
		if (table[i] == address)
			return true; // section is already in use

	return false; // section not in use
}

/* Call this after enabling all necessary sections */
void EEPROM::Initialize(void)
{
	HAL_FLASH_Unlock();
	Init();

	if (CheckForAssignedStructureChange()) {
		Format();
		Init();
		InitializeInternalState();
		WasFormattedAtBoot_ = true;
	} else {
		WasFormattedAtBoot_ = false;
	}

	HAL_FLASH_Lock();
}

bool EEPROM::WasFormattedAtBoot(void)
{
	return WasFormattedAtBoot_;
}

void EEPROM::Write8(uint16_t address, uint8_t value)
{
	xSemaphoreTake( resourceSemaphore_, ( TickType_t ) portMAX_DELAY); // take hardware resource

	HAL_FLASH_Unlock();

	uint16_t virtAddr = address / 2;
	bool LSB = ((address % 2) == 0); // little endian format, so LSB byte is at address

	uint16_t read;
	if (ReadVariable(virtAddr, &read) == EEPROM_FLASH_COMPLETE) {
		uint16_t tmp;
		if (LSB)
			tmp = value | (read & 0xFF00);
		else
			tmp = ((uint16_t)value) << 8 | (read & 0xFF);

		WriteVariable(virtAddr, tmp);
	}

	HAL_FLASH_Lock();

	xSemaphoreGive( resourceSemaphore_ ); // give hardware resource back
}

void EEPROM::Write16(uint16_t address, uint16_t value)
{
	xSemaphoreTake( resourceSemaphore_, ( TickType_t ) portMAX_DELAY); // take hardware resource

	HAL_FLASH_Unlock();

	bool aligned = ((address % 2) == 0);

	if (aligned) {
		uint16_t virtAddr = address / 2;
		WriteVariable(virtAddr, value);
	}
	else {
		uint16_t virtAddr1 = address / 2;  // Need to place LSB from value into MSB of this address, while still keeping LSB
		uint16_t virtAddr2 = virtAddr1 + 1;  // Need to place MSB from value into LSB of this address, while still keeping MSB

		uint16_t read1, read2;
		uint16_t write1, write2;
		if (ReadVariable(virtAddr1, &read1) == EEPROM_FLASH_COMPLETE) {
			if (ReadVariable(virtAddr2, &read2) == EEPROM_FLASH_COMPLETE) {
				write1 = ((value & 0xFF) << 8) | (read1 & 0xFF);
				write2 = ((value >> 8) & 0xFF) | (read2 & 0xFF00);
				if (WriteVariable(virtAddr1, write1) == EEPROM_FLASH_COMPLETE) {
					WriteVariable(virtAddr2, write2);
				}
			}
		}
	}

	HAL_FLASH_Lock();

	xSemaphoreGive( resourceSemaphore_ ); // give hardware resource back
}

void EEPROM::Write32(uint16_t address, uint32_t value)
{
	xSemaphoreTake( resourceSemaphore_, ( TickType_t ) portMAX_DELAY); // take hardware resource

	HAL_FLASH_Unlock();

	bool aligned = ((address % 2) == 0);

	if (aligned) {
		uint16_t virtAddr = address / 2;
		uint16_t value_lsb = value & 0xFFFF;
		uint16_t value_msb = (value >> 16) & 0xFFFF;
		WriteVariable(virtAddr, value_lsb);
		WriteVariable(virtAddr+1, value_msb);
	}
	else {
		uint16_t virtAddr1 = address / 2;  // Need to place 8 LSB bits from value into 8 MSB bits of this address, while still keeping LSB
		uint16_t virtAddr2 = virtAddr1 + 1;  // Need to place the middle bits from value into this address
		uint16_t virtAddr3 = virtAddr2 + 1;  // Need to place 8 MSB bits from value into 8 LSB bits of this address, while still keeping MSB

		uint16_t read1, read2, read3;
		uint16_t write1, write2, write3;
		if (ReadVariable(virtAddr1, &read1) == EEPROM_FLASH_COMPLETE) {
			if (ReadVariable(virtAddr2, &read2) == EEPROM_FLASH_COMPLETE) {
				if (ReadVariable(virtAddr3, &read3) == EEPROM_FLASH_COMPLETE) {
					write1 = ((value & 0xFF) << 8) | (read1 & 0xFF);
					write2 = ((value >> 8) & 0xFF) | (read2 & 0xFF00);
					if (WriteVariable(virtAddr1, write1) == EEPROM_FLASH_COMPLETE) {
						if (WriteVariable(virtAddr2, write2) == EEPROM_FLASH_COMPLETE) {
							WriteVariable(virtAddr3, write3);
						}
					}
				}
			}
		}
	}

	HAL_FLASH_Lock();

	xSemaphoreGive( resourceSemaphore_ ); // give hardware resource back
}

uint8_t EEPROM::Read8(uint16_t address)
{
	uint8_t value = 0xFF;
	xSemaphoreTake( resourceSemaphore_, ( TickType_t ) portMAX_DELAY); // take hardware resource

	uint16_t virtAddr = address / 2;
	bool LSB = ((address % 2) == 0); // little endian format, so LSB byte is at address

	uint16_t tmp;
	if (ReadVariable(virtAddr, &tmp) == EEPROM_FLASH_COMPLETE) {
		if (LSB)
			value = tmp & 0xFF;
		else
			value = (tmp >> 8) & 0xFF;
	}

	xSemaphoreGive( resourceSemaphore_ ); // give hardware resource back

	return value;
}

uint16_t EEPROM::Read16(uint16_t address)
{
	uint16_t value = 0xFFFF;
	xSemaphoreTake( resourceSemaphore_, ( TickType_t ) portMAX_DELAY); // take hardware resource

	bool aligned = ((address % 2) == 0);

	if (aligned) {
		uint16_t virtAddr = address / 2;
		uint16_t tmp;
		if (ReadVariable(virtAddr, &tmp) == EEPROM_FLASH_COMPLETE)
			value = tmp;
	}
	else {
		uint16_t virtAddr1 = address / 2; // need to take MSB from this address and put into LSB of value
		uint16_t virtAddr2 = virtAddr1 + 1; // need to take LSB from this address and put into MSB of value
		uint16_t tmp1, tmp2;
		if (ReadVariable(virtAddr1, &tmp1) == EEPROM_FLASH_COMPLETE)
			if (ReadVariable(virtAddr2, &tmp2) == EEPROM_FLASH_COMPLETE)
				value = ((tmp1 >> 8) & 0xFF) | ((tmp2 & 0xFF) << 8);
	}

	xSemaphoreGive( resourceSemaphore_ ); // give hardware resource back

	return value;
}

uint32_t EEPROM::Read32(uint16_t address)
{
	uint32_t value = 0xFFFFFFFF;
	xSemaphoreTake( resourceSemaphore_, ( TickType_t ) portMAX_DELAY); // take hardware resource

	bool aligned = ((address % 2) == 0);

	if (aligned) {
		uint16_t virtAddr = address / 2;
		uint16_t tmp_lo, tmp_hi;
		if (ReadVariable(virtAddr, &tmp_lo) == EEPROM_FLASH_COMPLETE)		 // little endian format
			if (ReadVariable(virtAddr+1, &tmp_hi) == EEPROM_FLASH_COMPLETE)
				value = (((uint32_t)tmp_hi) << 16) | (uint32_t)tmp_lo;
	}
	else {
		uint16_t virtAddr1 = address / 2; // need to take 8 MSB bits from this address and put into 8 LSB bits of value
		uint16_t virtAddr2 = virtAddr1 + 1; // need to take all 16 bits from this address and put in middle of value
		uint16_t virtAddr3 = virtAddr2 + 1; // need to take 8 LSB bits from this address and put into 8 MSB bits of value
		uint16_t tmp1, tmp2, tmp3;
		if (ReadVariable(virtAddr1, &tmp1) == EEPROM_FLASH_COMPLETE)		 // little endian format
			if (ReadVariable(virtAddr2, &tmp2) == EEPROM_FLASH_COMPLETE)
				if (ReadVariable(virtAddr3, &tmp3) == EEPROM_FLASH_COMPLETE)
					value = ((tmp1 >> 8) & 0xFF) | (((uint32_t)tmp2) << 8) | (((uint32_t)(tmp3 & 0xFF)) << 24);
	}

	xSemaphoreGive( resourceSemaphore_ ); // give hardware resource back

	return value;
}

EEPROM::errorCode_t EEPROM::WriteData(uint16_t address, uint8_t * data, uint16_t dataLength)
{
	errorCode_t status = EEPROM_ERROR;

	if ((address % 2) != 0) return EEPROM_ERROR; // address not aligned - it has to be aligned for multi-writing

	xSemaphoreTake( resourceSemaphore_, ( TickType_t ) portMAX_DELAY); // take hardware resource

	HAL_FLASH_Unlock();

	uint16_t virtAddress = address / 2;
	uint16_t idx = 0;
	while (idx < dataLength) {
		uint16_t value = (uint16_t)data[idx] | (((uint16_t)data[idx+1]) << 8); // little endian
		if ((status = WriteVariable(virtAddress, value)) != EEPROM_FLASH_COMPLETE) break;
		virtAddress++;
		idx += 2;
	}

	HAL_FLASH_Lock();

	xSemaphoreGive( resourceSemaphore_ ); // give hardware resource back

	return status;
}

EEPROM::errorCode_t EEPROM::ReadData(uint16_t address, uint8_t * data, uint16_t dataLength)
{
	errorCode_t status = EEPROM_ERROR;

	if ((address % 2) != 0) return EEPROM_ERROR; // address not aligned - it has to be aligned for multi-writing

	xSemaphoreTake( resourceSemaphore_, ( TickType_t ) portMAX_DELAY); // take hardware resource

	uint16_t virtAddress = address / 2;
	uint16_t idx = 0;
	while (idx < dataLength) {
		uint16_t value;
		if (ReadVariable(virtAddress, &value) != EEPROM_FLASH_COMPLETE) break;
		data[idx] = value & 0xFF; // little endian, hence LSB corresponds to first address
		data[idx+1] = (value >> 8) & 0xFF;
		virtAddress++;
		idx += 2;
	}

	xSemaphoreGive( resourceSemaphore_ ); // give hardware resource back

	return status;
}

/**
  ******************************************************************************
  * @file    EEPROM/EEPROM_Emulation/src/eeprom.c
  * @author  MCD Application Team
  * @brief   This file provides all the EEPROM emulation firmware functions.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V.
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other
  *    contributors to this software may be used to endorse or promote products
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under
  *    this license is void and will automatically terminate your rights under
  *    this license.
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/**
  * @brief  Restore the pages to a known good state in case of page's status
  *   corruption after a power loss.
  * @param  None.
  * @retval - Flash error code: on write Flash error
  *         - FLASH_COMPLETE: on success
  */

uint16_t EEPROM::Init(void)
{
  uint16_t PageStatus0 = 6, PageStatus1 = 6;
  uint16_t VarIdx = 0;
  uint16_t EepromStatus = 0;
  errorCode_t ReadStatus = EEPROM_FLASH_COMPLETE;
  int16_t x = -1;
  HAL_StatusTypeDef  FlashStatus;
  uint32_t SectorError = 0;
  FLASH_EraseInitTypeDef pEraseInit;
  uint64_t valid[4] = {0x0000};

  /* Get Page0 status */
  PageStatus0 = (*(__IO uint16_t*)PAGE0_BASE_ADDRESS);
  /* Get Page1 status */
  PageStatus1 = (*(__IO uint16_t*)PAGE1_BASE_ADDRESS);

  pEraseInit.TypeErase = TYPEERASE_SECTORS;
  pEraseInit.Banks = FLASH_BANK_2 ;
  pEraseInit.Sector = PAGE0_ID;
  pEraseInit.NbSectors = 1;
  pEraseInit.VoltageRange = VOLTAGE_RANGE;

  /* Check for invalid header states and repair if necessary */
  switch (PageStatus0)
  {
	case ERASED:
	  if (PageStatus1 == VALID_PAGE) /* Page0 erased, Page1 valid */
	  {
		  /* Erase Page0 */
		if(!VerifyPageFullyErased(PAGE0_BASE_ADDRESS))
		{
		  FlashStatus = HAL_FLASHEx_Erase(&pEraseInit, &SectorError);
		  /* If erase operation was failed, a Flash error code is returned */
		  if (FlashStatus != HAL_OK)
		  {
			return FlashStatus;
		  }
		   SCB_CleanInvalidateDCache_by_Addr((uint32_t*)PAGE0_BASE_ADDRESS,PAGE_SIZE);
		}
	  }
	  else if (PageStatus1 == RECEIVE_DATA) /* Page0 erased, Page1 receive */
	  {
		/* Erase Page0 */
		if(!VerifyPageFullyErased(PAGE0_BASE_ADDRESS))
		{
		  FlashStatus = HAL_FLASHEx_Erase(&pEraseInit, &SectorError);
		  /* If erase operation was failed, a Flash error code is returned */
		  if (FlashStatus != HAL_OK)
		  {
			return FlashStatus;
		  }
		  SCB_CleanInvalidateDCache_by_Addr((uint32_t*)PAGE0_BASE_ADDRESS,PAGE_SIZE);
		}
		/* Mark Page1 as valid */
		FlashStatus = HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, PAGE1_BASE_ADDRESS, (uint64_t)((uint32_t)valid));

		/* If program operation was failed, a Flash error code is returned */
		if (FlashStatus != HAL_OK)
		{
		  return FlashStatus;
		}
	  }
	  else /* First EEPROM access (Page0&1 are erased) or invalid state -> format EEPROM */
	  {
		/* Erase both Page0 and Page1 and set Page0 as valid page */
		FlashStatus = Format();
		/* If erase/program operation was failed, a Flash error code is returned */
		if (FlashStatus != HAL_OK)
		{
		  return FlashStatus;
		}
	  }
	  break;

	case RECEIVE_DATA:
	  if (PageStatus1 == VALID_PAGE) /* Page0 receive, Page1 valid */
	  {
		/* Transfer data from Page1 to Page0 */
		for (VarIdx = 0; VarIdx < virtAddrTable_->size(); VarIdx++)
		{
		  if (( *(__IO uint16_t*)(PAGE0_BASE_ADDRESS + 6)) == virtAddrTable_->at(VarIdx))
		  {
			x = VarIdx;
		  }
		  if (VarIdx != x)
		  {
			/* Read the last variables' updates */
			ReadStatus = ReadVariable(virtAddrTable_->at(VarIdx), &DataVar);
			/* In case variable corresponding to the virtual address was found */
			if (ReadStatus != 0x1)
			{
			  /* Transfer the variable to the Page0 */
			  EepromStatus = VerifyPageFullWriteVariable(virtAddrTable_->at(VarIdx), DataVar);
			  /* If program operation was failed, a Flash error code is returned */
			  if (EepromStatus != HAL_OK)
			  {
				return EepromStatus;
			  }
			}
		  }
		}
		/* Mark Page0 as valid */
		FlashStatus = HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, PAGE0_BASE_ADDRESS, (uint64_t)((uint32_t)valid));

		/* If program operation was failed, a Flash error code is returned */
		if (FlashStatus != HAL_OK)
		{
		  return FlashStatus;
		}
		pEraseInit.Sector = PAGE1_ID;
		pEraseInit.Banks = FLASH_BANK_2 ;
		pEraseInit.NbSectors = 1;
		pEraseInit.VoltageRange = VOLTAGE_RANGE;
		/* Erase Page1 */
		if(!VerifyPageFullyErased(PAGE1_BASE_ADDRESS))
		{
		  FlashStatus = HAL_FLASHEx_Erase(&pEraseInit, &SectorError);
		  /* If erase operation was failed, a Flash error code is returned */
		  if (FlashStatus != HAL_OK)
		  {
			return FlashStatus;
		  }
		  SCB_CleanInvalidateDCache_by_Addr((uint32_t*)PAGE1_BASE_ADDRESS,PAGE_SIZE);
		}
	  }
	  else if (PageStatus1 == ERASED) /* Page0 receive, Page1 erased */
	  {
		pEraseInit.Sector = PAGE1_ID;
		pEraseInit.Banks = FLASH_BANK_1 ;
		pEraseInit.NbSectors = 1;
		pEraseInit.VoltageRange = VOLTAGE_RANGE;
		/* Erase Page1 */
		if(!VerifyPageFullyErased(PAGE1_BASE_ADDRESS))
		{
		  FlashStatus = HAL_FLASHEx_Erase(&pEraseInit, &SectorError);
		  /* If erase operation was failed, a Flash error code is returned */
		  if (FlashStatus != HAL_OK)
		  {
			return FlashStatus;
		  }
		  SCB_CleanInvalidateDCache_by_Addr((uint32_t*)PAGE1_BASE_ADDRESS,PAGE_SIZE);
		}
		/* Mark Page0 as valid */
		FlashStatus = HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, PAGE0_BASE_ADDRESS, (uint64_t)((uint32_t)valid));
		/* If program operation was failed, a Flash error code is returned */
		if (FlashStatus != HAL_OK)
		{
		  return FlashStatus;
		}
	  }
	  else /* Invalid state -> format eeprom */
	  {
		/* Erase both Page0 and Page1 and set Page0 as valid page */
		FlashStatus = Format();
		/* If erase/program operation was failed, a Flash error code is returned */
		if (FlashStatus != HAL_OK)
		{
		  return FlashStatus;
		}
	  }
	  break;

	case VALID_PAGE:
	  if (PageStatus1 == VALID_PAGE) /* Invalid state -> format eeprom */
	  {
		/* Erase both Page0 and Page1 and set Page0 as valid page */
		FlashStatus = Format();
		/* If erase/program operation was failed, a Flash error code is returned */
		if (FlashStatus != HAL_OK)
		{
		  return FlashStatus;
		}
	  }
	  else if (PageStatus1 == ERASED) /* Page0 valid, Page1 erased */
	  {
		pEraseInit.Sector = PAGE1_ID;
		pEraseInit.Banks = FLASH_BANK_2 ;
		pEraseInit.NbSectors = 1;
		pEraseInit.VoltageRange = VOLTAGE_RANGE;
		/* Erase Page1 */
		if(!VerifyPageFullyErased(PAGE1_BASE_ADDRESS))
		{
		  FlashStatus = HAL_FLASHEx_Erase(&pEraseInit, &SectorError);
		  /* If erase operation was failed, a Flash error code is returned */
		  if (FlashStatus != HAL_OK)
		  {
			return FlashStatus;
		  }
		  SCB_CleanInvalidateDCache_by_Addr((uint32_t*)PAGE1_BASE_ADDRESS,PAGE_SIZE);
		}
	  }
	  else /* Page0 valid, Page1 receive */
	  {
		/* Transfer data from Page0 to Page1 */
		for (VarIdx = 0; VarIdx < virtAddrTable_->size(); VarIdx++)
		{
		  if ((*(__IO uint16_t*)(PAGE1_BASE_ADDRESS + 6)) == virtAddrTable_->at(VarIdx))
		  {
			x = VarIdx;
		  }
		  if (VarIdx != x)
		  {
			/* Read the last variables' updates */
			ReadStatus = ReadVariable(virtAddrTable_->at(VarIdx), &DataVar);
			/* In case variable corresponding to the virtual address was found */
			if (ReadStatus != 0x1)
			{
			  /* Transfer the variable to the Page1 */
			  EepromStatus = VerifyPageFullWriteVariable(virtAddrTable_->at(VarIdx), DataVar);
			  /* If program operation was failed, a Flash error code is returned */
			  if (EepromStatus != HAL_OK)
			  {
				return EepromStatus;
			  }
			}
		  }
		}
		/* Mark Page1 as valid */
		FlashStatus = HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, PAGE1_BASE_ADDRESS, (uint64_t)((uint32_t)valid));

		/* If program operation was failed, a Flash error code is returned */
		if (FlashStatus != HAL_OK)
		{
		  return FlashStatus;
		}
		pEraseInit.Sector = PAGE0_ID;
		pEraseInit.Banks = FLASH_BANK_2 ;
		pEraseInit.NbSectors = 1;
		pEraseInit.VoltageRange = VOLTAGE_RANGE;
		/* Erase Page0 */
		if(!VerifyPageFullyErased(PAGE0_BASE_ADDRESS))
		{
		  FlashStatus = HAL_FLASHEx_Erase(&pEraseInit, &SectorError);
		  /* If erase operation was failed, a Flash error code is returned */
		  if (FlashStatus != HAL_OK)
		  {
			return FlashStatus;
		  }
		  SCB_CleanInvalidateDCache_by_Addr((uint32_t*)PAGE0_BASE_ADDRESS,PAGE_SIZE);
		}
	  }
	  break;

	default:  /* Any other state -> format eeprom */
	  /* Erase both Page0 and Page1 and set Page0 as valid page */
	  FlashStatus = Format();
	  /* If erase/program operation was failed, a Flash error code is returned */
	  if (FlashStatus != HAL_OK)
	  {
		return FlashStatus;
	  }
	  break;
  }

  return HAL_OK;
}

/**
  * @brief  Verify if specified page is fully erased.
  * @param  Address: page address
  *   This parameter can be one of the following values:
  *     @arg PAGE0_BASE_ADDRESS: Page0 base address
  *     @arg PAGE1_BASE_ADDRESS: Page1 base address
  * @retval page fully erased status:
  *           - 0: if Page not erased
  *           - 1: if Page erased
  */
uint16_t EEPROM::VerifyPageFullyErased(uint32_t Address)
{
  uint32_t ReadStatus = 1;
  uint16_t AddressValue = 0x5555;

  /* Check each active page address starting from end */
  while (Address <= PAGE0_END_ADDRESS)
  {
	/* Get the current location content to be compared with virtual address */
	AddressValue = (*(__IO uint16_t*)Address);

	/* Compare the read address with the virtual address */
	if (AddressValue != ERASED)
	{

	  /* In case variable value is read, reset ReadStatus flag */
	  ReadStatus = 0;

	  break;
	}
	/* Next address location */
	Address = Address + 4;
  }

  /* Return ReadStatus value: (0: Page not erased, 1: Sector erased) */
  return ReadStatus;
}

/**
  * @brief  Returns the last stored variable data, if found, which correspond to
  *   the passed virtual address
  * @param  VirtAddress: Variable virtual address
  * @param  Data: Global variable contains the read variable value
  * @retval Success or error status:
  *           - 0: if variable was found
  *           - 1: if the variable was not found
  *           - NO_VALID_PAGE: if no valid page was found.
  */
EEPROM::errorCode_t EEPROM::ReadVariable(uint16_t VirtAddress, uint16_t* Data)
{
  uint16_t ValidPage = PAGE0;
  uint16_t AddressValue = 0x5555;
  errorCode_t ReadStatus = EEPROM_ERROR;
  uint32_t Address = EEPROM_START_ADDRESS, PageStartAddress = EEPROM_START_ADDRESS;

  /* Get active Page for read operation */
  ValidPage = FindValidPage(READ_FROM_VALID_PAGE);

  /* Check if there is no valid page */
  if (ValidPage == EEPROM_NO_VALID_PAGE)
  {
	return  EEPROM_NO_VALID_PAGE;
  }

  /* Get the valid Page start Address */
  PageStartAddress = (uint32_t)(EEPROM_START_ADDRESS + (uint32_t)(ValidPage * PAGE_SIZE));

  /* Get the valid Page end Address */
  Address = (uint32_t)((EEPROM_START_ADDRESS - 32) + (uint32_t)((1 + ValidPage) * PAGE_SIZE));

  /* Check each active page address starting from end */
  while (Address > (PageStartAddress + 32))
  {
	/* Get the current location content to be compared with virtual address */
	AddressValue = (*(__IO uint16_t*)Address);

	/* Compare the read address with the virtual address */
	if (AddressValue == VirtAddress)
	{
	  /* Get content of Address-2 which is variable value */
	  *Data = (*(__IO uint16_t*)(Address - 32));

	  /* In case variable value is read, reset ReadStatus flag */
	  ReadStatus = EEPROM_FLASH_COMPLETE;

	  break;
	}
	else
	{
	  /* Next address location */
	  Address = Address - 64;
	}
  }

  /* Return ReadStatus value: (0: variable exist, 1: variable doesn't exist) */
  return ReadStatus;
}

/**
  * @brief  Writes/updates variable data in EEPROM.
  * @param  VirtAddress: Variable virtual address
  * @param  Data: 16 bit data to be written
  * @retval Success or error status:
  *           - FLASH_COMPLETE: on success
  *           - PAGE_FULL: if valid page is full
  *           - NO_VALID_PAGE: if no valid page was found
  *           - Flash error code: on write Flash error
  */
EEPROM::errorCode_t EEPROM::WriteVariable(uint16_t VirtAddress, uint16_t Data)
{
  EEPROM::errorCode_t Status = EEPROM_FLASH_COMPLETE;

  /* Write the variable virtual address and value in the EEPROM */
  Status = VerifyPageFullWriteVariable(VirtAddress, Data);

  /* In case the EEPROM active page is full */
  if (Status == EEPROM_PAGE_FULL)
  {
	/* Perform Page transfer */
	Status = PageTransfer(VirtAddress, Data);
  }

  /* Return last operation status */
  return Status;
}

/**
  * @brief  Erases PAGE and PAGE1 and writes VALID_PAGE header to PAGE
  * @param  None
  * @retval Status of the last operation (Flash write or erase) done during
  *         EEPROM formating
  */
HAL_StatusTypeDef EEPROM::Format(void)
{
  HAL_StatusTypeDef FlashStatus = HAL_OK;
  uint32_t SectorError = 0;
  FLASH_EraseInitTypeDef pEraseInit;
  uint64_t valid[4] = {0x0000};

  pEraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
  pEraseInit.Banks = FLASH_BANK_2 ;
  pEraseInit.Sector = PAGE0_ID;
  pEraseInit.NbSectors = 1;
  pEraseInit.VoltageRange = VOLTAGE_RANGE;

  /* Erase Page0 */
  if(!VerifyPageFullyErased(PAGE0_BASE_ADDRESS))
  {
	FlashStatus = HAL_FLASHEx_Erase(&pEraseInit, &SectorError);
	/* If erase operation was failed, a Flash error code is returned */
	if (FlashStatus != HAL_OK)
	{
	  return FlashStatus;
	}
	SCB_CleanInvalidateDCache_by_Addr((uint32_t*)PAGE0_BASE_ADDRESS,PAGE_SIZE);
  }
  /* Set Page0 as valid page: Write VALID_PAGE at Page0 base address */

	FlashStatus = HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, PAGE0_BASE_ADDRESS,(uint64_t)((uint32_t)valid));

  /* If program operation was failed, a Flash error code is returned */
  if (FlashStatus != HAL_OK)
  {
	return FlashStatus;
  }

  pEraseInit.Sector = PAGE1_ID;
  /* Erase Page1 */
  if(!VerifyPageFullyErased(PAGE1_BASE_ADDRESS))
  {
	FlashStatus = HAL_FLASHEx_Erase(&pEraseInit, &SectorError);
	/* If erase operation was failed, a Flash error code is returned */
	if (FlashStatus != HAL_OK)
	{
	  return FlashStatus;
	}
	SCB_CleanInvalidateDCache_by_Addr((uint32_t*)PAGE1_BASE_ADDRESS,PAGE_SIZE);
  }

  return HAL_OK;
}

/**
  * @brief  Find valid Page for write or read operation
  * @param  Operation: operation to achieve on the valid page.
  *   This parameter can be one of the following values:
  *     @arg READ_FROM_VALID_PAGE: read operation from valid page
  *     @arg WRITE_IN_VALID_PAGE: write operation from valid page
  * @retval Valid page number (PAGE or PAGE1) or NO_VALID_PAGE in case
  *   of no valid page was found
  */
uint16_t EEPROM::FindValidPage(uint8_t Operation)
{
  uint16_t PageStatus0 = 6, PageStatus1 = 6;

  /* Get Page0 actual status */
  PageStatus0 = (*(__IO uint16_t*)PAGE0_BASE_ADDRESS);

  /* Get Page1 actual status */
  PageStatus1 = (*(__IO uint16_t*)PAGE1_BASE_ADDRESS);

  /* Write or read operation */
  switch (Operation)
  {
	case WRITE_IN_VALID_PAGE:   /* ---- Write operation ---- */
	  if (PageStatus1 == VALID_PAGE)
	  {
		/* Page0 receiving data */
		if (PageStatus0 == RECEIVE_DATA)
		{
		  return PAGE0;         /* Page0 valid */
		}
		else
		{
		  return PAGE1;         /* Page1 valid */
		}
	  }
	  else if (PageStatus0 == VALID_PAGE)
	  {
		/* Page1 receiving data */
		if (PageStatus1 == RECEIVE_DATA)
		{
		  return PAGE1;         /* Page1 valid */
		}
		else
		{
		  return PAGE0;         /* Page0 valid */
		}
	  }
	  else
	  {
		return EEPROM_NO_VALID_PAGE;   /* No valid Page */
	  }

	case READ_FROM_VALID_PAGE:  /* ---- Read operation ---- */
	  if (PageStatus0 == VALID_PAGE)
	  {
		return PAGE0;           /* Page0 valid */
	  }
	  else if (PageStatus1 == VALID_PAGE)
	  {
		return PAGE1;           /* Page1 valid */
	  }
	  else
	  {
		return EEPROM_NO_VALID_PAGE ;  /* No valid Page */
	  }

	default:
	  return PAGE0;             /* Page0 valid */
  }
}

/**
  * @brief  Verify if active page is full and Writes variable in EEPROM.
  * @param  VirtAddress: 16 bit virtual address of the variable
  * @param  Data: 16 bit data to be written as variable value
  * @retval Success or error status:
  *           - FLASH_COMPLETE: on success
  *           - PAGE_FULL: if valid page is full
  *           - NO_VALID_PAGE: if no valid page was found
  *           - Flash error code: on write Flash error
  */
EEPROM::errorCode_t EEPROM::VerifyPageFullWriteVariable(uint16_t VirtAddress, uint16_t Data)
{
  HAL_StatusTypeDef FlashStatus = HAL_OK;
  uint16_t ValidPage = PAGE0;
  uint32_t Address = EEPROM_START_ADDRESS, PageEndAddress = EEPROM_START_ADDRESS+PAGE_SIZE;

  uint64_t data64[4] = {Data};
  uint64_t VirtAddress1[4] = {VirtAddress};
  /* Get valid Page for write operation */
  ValidPage = FindValidPage(WRITE_IN_VALID_PAGE);

  /* Check if there is no valid page */
  if (ValidPage == EEPROM_NO_VALID_PAGE)
  {
	return  EEPROM_NO_VALID_PAGE;
  }

  /* Get the valid Page start Address */
  Address = (uint32_t)(EEPROM_START_ADDRESS + (uint32_t)(ValidPage * PAGE_SIZE));

  /* Get the valid Page end Address */
  PageEndAddress = (uint32_t)((EEPROM_START_ADDRESS - 1) + (uint32_t)((ValidPage + 1) * PAGE_SIZE));

  /* Check each active page address starting from begining */
  while (Address < PageEndAddress)
  {
	/* Verify if Address and Address+2 contents are 0xFFFFFFFF */
	if ((*(__IO uint32_t*)Address) == 0xFFFFFFFF)
	{
	  /* Set variable data */
	  FlashStatus = HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, Address, (uint64_t)((uint32_t)data64));

	  /* If program operation was failed, a Flash error code is returned */
	  if (FlashStatus != HAL_OK)
	  {
		return (errorCode_t)FlashStatus;
	  }
	  /* Set variable virtual address */
	 FlashStatus = HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, Address + 32,(uint64_t)((uint32_t)VirtAddress1));

	  /* Return program operation status */
	 return (errorCode_t)FlashStatus;
	}
	else
	{
	  /* Next address location */
	  Address = Address + 64;
	}
  }

  /* Return PAGE_FULL in case the valid page is full */
  return EEPROM_PAGE_FULL;
}

/**
  * @brief  Transfers last updated variables data from the full Page to
  *   an empty one.
  * @param  VirtAddress: 16 bit virtual address of the variable
  * @param  Data: 16 bit data to be written as variable value
  * @retval Success or error status:
  *           - FLASH_COMPLETE: on success
  *           - PAGE_FULL: if valid page is full
  *           - NO_VALID_PAGE: if no valid page was found
  *           - Flash error code: on write Flash error
  */
EEPROM::errorCode_t EEPROM::PageTransfer(uint16_t VirtAddress, uint16_t Data)
{
  HAL_StatusTypeDef FlashStatus = HAL_OK;
  uint32_t NewPageAddress = EEPROM_START_ADDRESS;
  uint32_t OldPageAddress = 0;
  uint16_t OldPageId=0;
  uint16_t ValidPage = PAGE0, VarIdx = 0;
  errorCode_t EepromStatus = EEPROM_FLASH_COMPLETE, ReadStatus = EEPROM_FLASH_COMPLETE;
  uint32_t SectorError = 0;
  FLASH_EraseInitTypeDef pEraseInit;
  uint64_t valid[4] = {0x0000};
  uint64_t receive[4] = {0xEEEE};
  /* Get active Page for read operation */
  ValidPage = FindValidPage(READ_FROM_VALID_PAGE);

  if (ValidPage == PAGE1)       /* Page1 valid */
  {
	/* New page address where variable will be moved to */
	NewPageAddress = PAGE0_BASE_ADDRESS;

	/* Old page address  where variable will be moved from */
	OldPageAddress = PAGE1_BASE_ADDRESS;

	/* Old page ID where variable will be taken from */
	OldPageId = PAGE1_ID;
  }
  else if (ValidPage == PAGE0)  /* Page0 valid */
  {
	/* New page address  where variable will be moved to */
	NewPageAddress = PAGE1_BASE_ADDRESS;

	/* Old page address  where variable will be moved from */
	OldPageAddress = PAGE0_BASE_ADDRESS;

	/* Old page ID where variable will be taken from */
	OldPageId = PAGE0_ID;
  }
  else
  {
	return EEPROM_NO_VALID_PAGE;       /* No valid Page */
  }

  /* Set the new Page status to RECEIVE_DATA status */

  FlashStatus = HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, NewPageAddress, (uint64_t)((uint32_t)receive));

  /* If program operation was failed, a Flash error code is returned */
  if (FlashStatus != HAL_OK)
  {
	return (errorCode_t)FlashStatus;
  }

  /* Write the variable passed as parameter in the new active page */
  EepromStatus = VerifyPageFullWriteVariable(VirtAddress, Data);
  /* If program operation was failed, a Flash error code is returned */
  if ((HAL_StatusTypeDef)EepromStatus != HAL_OK)
  {
	return EepromStatus;
  }

  /* Transfer process: transfer variables from old to the new active page */
  for (VarIdx = 0; VarIdx < virtAddrTable_->size(); VarIdx++)
  {
	if (virtAddrTable_->at(VarIdx) != VirtAddress)  /* Check each variable except the one passed as parameter */
	{
	  /* Read the other last variable updates */
	  ReadStatus = ReadVariable(virtAddrTable_->at(VarIdx), &DataVar);
	  /* In case variable corresponding to the virtual address was found */
	  if (ReadStatus != 0x1)
	  {
		/* Transfer the variable to the new active page */
		EepromStatus = VerifyPageFullWriteVariable(virtAddrTable_->at(VarIdx), DataVar);
		/* If program operation was failed, a Flash error code is returned */
		if ((HAL_StatusTypeDef)EepromStatus != HAL_OK)
		{
		  return EepromStatus;
		}
	  }
	}
  }

  HAL_FLASH_Unlock();

  /* Clear pending flags (if any) */
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
						 FLASH_FLAG_PGSERR | FLASH_FLAG_WRPERR);

  pEraseInit.TypeErase = TYPEERASE_SECTORS;
  pEraseInit.Banks = FLASH_BANK_2 ;
  pEraseInit.Sector = OldPageId;
  pEraseInit.NbSectors = 1;
  pEraseInit.VoltageRange = VOLTAGE_RANGE;

  /* Erase the old Page: Set old Page status to ERASED status */
  FlashStatus = HAL_FLASHEx_Erase(&pEraseInit, &SectorError);
  /* If erase operation was failed, a Flash error code is returned */
  if (FlashStatus != HAL_OK)
  {
	return (errorCode_t)FlashStatus;
  }
  SCB_CleanInvalidateDCache_by_Addr((uint32_t*)OldPageAddress,PAGE_SIZE);

  HAL_FLASH_Lock();

  /* Set new Page status to VALID_PAGE status */
  FlashStatus = HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, NewPageAddress, (uint64_t)((uint32_t)valid));
  /* If program operation was failed, a Flash error code is returned */
  if (FlashStatus != HAL_OK)
  {
	return (errorCode_t)FlashStatus;
  }

  /* Return last operation flash status */
  return (errorCode_t)FlashStatus;
}

/**
  * @}
  */

/******************* (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
