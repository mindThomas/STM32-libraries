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
 
#ifndef PERIPHIRALS_EEPROM_H
#define PERIPHIRALS_EEPROM_H

#include "stm32h7xx_hal.h"
#include "cmsis_os.h" // for semaphore support
#include <string.h> // for memcpy

/* EEPROM Flash locations */
#define FLASH_BASE_ADDR      (uint32_t)(FLASH_BASE)
#define FLASH_END_ADDR       (uint32_t)(0x081FFFFF)

/* Base address of the Flash sectors Bank 1 */
#define ADDR_FLASH_SECTOR_0_BANK1     ((uint32_t)0x08000000) /* Base @ of Sector 0, 128 Kbytes */
#define ADDR_FLASH_SECTOR_1_BANK1     ((uint32_t)0x08020000) /* Base @ of Sector 1, 128 Kbytes */
#define ADDR_FLASH_SECTOR_2_BANK1     ((uint32_t)0x08040000) /* Base @ of Sector 2, 128 Kbytes */
#define ADDR_FLASH_SECTOR_3_BANK1     ((uint32_t)0x08060000) /* Base @ of Sector 3, 128 Kbytes */
#define ADDR_FLASH_SECTOR_4_BANK1     ((uint32_t)0x08080000) /* Base @ of Sector 4, 128 Kbytes */
#define ADDR_FLASH_SECTOR_5_BANK1     ((uint32_t)0x080A0000) /* Base @ of Sector 5, 128 Kbytes */
#define ADDR_FLASH_SECTOR_6_BANK1     ((uint32_t)0x080C0000) /* Base @ of Sector 6, 128 Kbytes */
#define ADDR_FLASH_SECTOR_7_BANK1     ((uint32_t)0x080E0000) /* Base @ of Sector 7, 128 Kbytes */

/* Base address of the Flash sectors Bank 2 */
#define ADDR_FLASH_SECTOR_0_BANK2     ((uint32_t)0x08100000) /* Base @ of Sector 0, 128 Kbytes */
#define ADDR_FLASH_SECTOR_1_BANK2     ((uint32_t)0x08120000) /* Base @ of Sector 1, 128 Kbytes */
#define ADDR_FLASH_SECTOR_2_BANK2     ((uint32_t)0x08140000) /* Base @ of Sector 2, 128 Kbytes */
#define ADDR_FLASH_SECTOR_3_BANK2     ((uint32_t)0x08160000) /* Base @ of Sector 3, 128 Kbytes */
#define ADDR_FLASH_SECTOR_4_BANK2     ((uint32_t)0x08180000) /* Base @ of Sector 4, 128 Kbytes */
#define ADDR_FLASH_SECTOR_5_BANK2     ((uint32_t)0x081A0000) /* Base @ of Sector 5, 128 Kbytes */
#define ADDR_FLASH_SECTOR_6_BANK2     ((uint32_t)0x081C0000) /* Base @ of Sector 6, 128 Kbytes */
#define ADDR_FLASH_SECTOR_7_BANK2     ((uint32_t)0x081E0000) /* Base @ of Sector 7, 128 Kbytes */

/* Exported constants --------------------------------------------------------*/
/* EEPROM emulation firmware error codes */
#define EE_OK      (uint32_t)HAL_OK
#define EE_ERROR   (uint32_t)HAL_ERROR
#define EE_BUSY    (uint32_t)HAL_BUSY
#define EE_TIMEOUT (uint32_t)HAL_TIMEOUT

/* Define the size of the sectors to be used */

  #define PAGE_SIZE               (uint32_t)0x20000  /* Page size = 128KByte */


/* Device voltage range supposed to be [2.7V to 3.6V], the operation will
   be done by word  */
#define VOLTAGE_RANGE           (uint8_t)VOLTAGE_RANGE_3

/* EEPROM emulation start address in Flash */

  #define EEPROM_START_ADDRESS  ADDR_FLASH_SECTOR_6_BANK1 /* sector6 of bank 2 */


/* Pages 0 and 1 base and end addresses */
#define PAGE0_BASE_ADDRESS    ((uint32_t)(EEPROM_START_ADDRESS + 0x0000))
#define PAGE0_END_ADDRESS     ((uint32_t)(EEPROM_START_ADDRESS + (PAGE_SIZE - 1)))

  #define PAGE0_ID               FLASH_SECTOR_6


#define PAGE1_BASE_ADDRESS    ((uint32_t)(EEPROM_START_ADDRESS + PAGE_SIZE))
#define PAGE1_END_ADDRESS     ((uint32_t)(EEPROM_START_ADDRESS + (2 * PAGE_SIZE - 1)))

  #define PAGE1_ID               FLASH_SECTOR_7


/* Used Flash pages for EEPROM emulation */
#define PAGE0                 ((uint16_t)0x0000)
#define PAGE1                 ((uint16_t)0x0001) /* Page nb between PAGE0_BASE_ADDRESS & PAGE1_BASE_ADDRESS*/

/* Page status definitions */
#define ERASED                ((uint16_t)0xFFFF)     /* Page is empty */
#define RECEIVE_DATA          ((uint16_t)0xEEEE)     /* Page is marked to receive data */
#define VALID_PAGE            ((uint16_t)0x0000)     /* Page containing valid data */

/* Valid pages in read and write defines */
#define READ_FROM_VALID_PAGE  ((uint8_t)0x00)
#define WRITE_IN_VALID_PAGE   ((uint8_t)0x01)

#ifdef __cplusplus

#include <vector>

class EEPROM
{
	public:
		/* The struct below contains a list of addresses of assigned EEPROM sections (address) */
		/* Important not to use address 0x000 */
		const struct {
			uint16_t internal = 0x002;
			uint16_t sys_info = 0x050;
			uint16_t imu_calibration = 0x100;
			uint16_t parameters = 0x200;
		} sections;

	public:
		typedef enum errorCode_t : uint16_t
		{
			EEPROM_FLASH_COMPLETE = 0x0000, /* HAL_OK */
			EEPROM_ERROR    = 0x01,
			EEPROM_BUSY     = 0x02,
			EEPROM_TIMEOUT  = 0x03,
			EEPROM_PAGE_FULL = 0x0080, 	 /* Page full define */
			EEPROM_NO_VALID_PAGE = 0x00AB   /* No valid page define */
		} errorCode_t;

	public:
		EEPROM();
		~EEPROM();

		void Initialize(void);

		void Write8(uint16_t address, uint8_t value);
		void Write16(uint16_t address, uint16_t value);
		void Write32(uint16_t address, uint32_t value);
		uint8_t Read8(uint16_t address);
		uint16_t Read16(uint16_t address);
		uint32_t Read32(uint16_t address);

		errorCode_t WriteData(uint16_t address, uint8_t * data, uint16_t dataLength);
		errorCode_t ReadData(uint16_t address, uint8_t * data, uint16_t dataLength);

		bool EnableSection(uint16_t address, uint16_t sectionSize);
		bool WasFormattedAtBoot(void);

	private:
		uint16_t Init(void);
		bool CheckForAssignedStructureChange(void);
		void InitializeInternalState(void);
		bool IsSectionInUse(uint16_t address);
		HAL_StatusTypeDef Format(void);

		uint16_t FindValidPage(uint8_t Operation);
		errorCode_t VerifyPageFullWriteVariable(uint16_t VirtAddress, uint16_t Data);
		errorCode_t PageTransfer(uint16_t VirtAddress, uint16_t Data);
		uint16_t VerifyPageFullyErased(uint32_t Address);

		errorCode_t ReadVariable(uint16_t VirtAddress, uint16_t* Data);
		errorCode_t WriteVariable(uint16_t VirtAddress, uint16_t Data);

		uint32_t CalculateSectionsTableChecksum(void);

	private:
		SemaphoreHandle_t resourceSemaphore_;
		std::vector<uint16_t> * virtAddrTable_;

		uint16_t DataVar = 0;
		uint32_t Address = 0;

		bool WasFormattedAtBoot_;

		const uint32_t STATE_VALIDATION_FLAG = 0x55555555;

	private:
			typedef struct internal_state_t {
				uint32_t validationFlag;
				uint32_t sectionsChecksum;
			} internal_state_t;
};
	
#endif
	
#endif
