/*
 *  $Author: Ho Hai Dang
*/

#include "Flash_NVM.h"

/* ****************************************************************************
 * API public: FlashNVM_Erase
 *
 * Description:
 * This API shall execute erase pages in flash memory.
 *
 * Arguments:
 *    PageAddress_u32[in]: Address of page. Any address within the page
 *                              (ODD or EVEN) -> That page still be erased.
 *    Size_u32[in]: Size of byte to be erased.
 *
 * Return:
 *    HAL_StatusTypeDef: Status of erasing process
 *
 * Usage guide & Scheduling:
 *    Event-Driven.
 *
 * Remarks:
 *    Flash memory just can erase all data in page. Cannot erase in byte.
 *    After erasing, memory data should be 0xFF.
 * ****************************************************************************/
HAL_StatusTypeDef FlashNVM_Erase(const uint32_t PageAddress_u32, const uint32_t Size_u32)
{
    HAL_StatusTypeDef StatRet_en = HAL_ERROR;
    uint8_t NumOfPageErase = 0u;

    if (Size_u32 > 0u)
    {
        /*Compute Number of page to be erased base on Size_u32*/
        NumOfPageErase = ((Size_u32 % 2048u) != 0) ? ((Size_u32 >> 11u) + 1u) : (Size_u32 >> 11u);
        StatRet_en = FlashNVM_ErasePage(PageAddress_u32, NumOfPageErase);
    }
    return StatRet_en;
}
/* ****************************************************************************
 * API public: FlashNVM_ErasePage
 *
 * Description:
 * This API shall execute erase pages in flash memory.
 *
 * Arguments:
 *    PageStartAddress_u32[in]: Address of page. Any address within the page
 *                              (ODD or EVEN) -> That page still be erased.
 *    NumOfPage_u8[in]: Number of Page to be erased.
 *
 * Return:
 *    HAL_StatusTypeDef: Status of erasing process
 *
 * Usage guide & Scheduling:
 *    Event-Driven.
 *
 * Remarks:
 *    After erasing, memory data should be 0xFF.
 * ****************************************************************************/
HAL_StatusTypeDef FlashNVM_ErasePage(uint32_t const PageStartAddress_u32, uint8_t const NumOfPage_u8)
{
    HAL_StatusTypeDef HAL_Status_en = HAL_ERROR;
    FLASH_EraseInitTypeDef EraseInitStruct_st; /* Parameter collect information to erase flash memory.*/
    uint32_t PageError_u32;

    /* Check Input
     * Page Address Valid?
     * Number of page out of range?*/
    if ((PageStartAddress_u32 < MIN_ADDR_PAGE) || (PageStartAddress_u32 > MAX_ADDR_PAGE) ||
        (NumOfPage_u8 > MAX_NUMOFPAGE) || (NumOfPage_u8 < 0u))
    {
        return HAL_ERROR;
    }
    /*Unlock Flash memory*/
    HAL_Status_en = HAL_FLASH_Unlock();
    if (HAL_OK == HAL_Status_en)
    {
        /*Clear Status Register*/
        __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_WRPERR | FLASH_FLAG_PGERR);
        EraseInitStruct_st.TypeErase = FLASH_TYPEERASE_PAGES;
        EraseInitStruct_st.PageAddress = PageStartAddress_u32;
        EraseInitStruct_st.NbPages = NumOfPage_u8;

        HAL_Status_en = HAL_BUSY;
        /*Start Erase Page*/
        while(HAL_BUSY == HAL_Status_en)
        {
            HAL_Status_en = HAL_FLASHEx_Erase(&EraseInitStruct_st, &PageError_u32);
        }
    }

    /*Lock Flash Memory*/
    HAL_FLASH_Lock();
    return HAL_Status_en;
}


/* ****************************************************************************
 * API public: FlashNVM_ErasePage
 *
 * Description:
 * This API shall execute read
 *
 * Arguments:
 *    StartAddress_u32[in]: Address shall be read
 *    *DataOut_pu8[out]: Buffer data shall be copied to
 *    Size_u32: Size of data to read in byte
 *
 * Return:
 *    HAL_StatusTypeDef: Status of read process
 *
 * Usage guide & Scheduling:
 *    Event-Driven.
 *
 * Remarks:
 *    None
 * ****************************************************************************/
HAL_StatusTypeDef FlashNVM_Read(uint32_t StartAddress_u32, uint8_t *DataOut_pu8, uint32_t const Size_u32)
{
    uint32_t SizeCounter_u32 = 0u;

    /* Check input
     * Address is flash address?
     * DataIn is NULL Pointer?
     * Size of data write is valid? */
    if (!IS_FLASH_ADDRESS(StartAddress_u32) || (((void *)0) == DataOut_pu8) || (Size_u32 > MAX_SIZEOFFLASH))
    {
        return HAL_ERROR;
    }

    /* Start read */
    while (SizeCounter_u32 < Size_u32)
    {
        DataOut_pu8[SizeCounter_u32++] = (*(__IO uint32_t*)StartAddress_u32);
        StartAddress_u32++;
    }
    return HAL_OK;
}


/* ****************************************************************************
 * API public: FlashNVM_Write
 *
 * Description:
 * This API shall execute write flash memory.
 *
 * Arguments:
 *    StartAddress_u32[in]: Start address (Shall be EVEN)
 *    DataIn_pu16[in]: Data used to be written
 *    Size_u32[in]: Size of data in byte
 *
 * Return:
 *    HAL_StatusTypeDef: HAL status
 *
 * Usage guide & Scheduling:
 *    Event-Driven.
 *
 * Remarks:
 *    Before writing, all data in that pages will be erased to 0xFF no matter writing 1 byte or more bytes.
 * ****************************************************************************/
HAL_StatusTypeDef FlashNVM_Write(uint32_t const StartAddress_u32, uint16_t const * DataIn_pu16, uint32_t Size_u32)
{
    HAL_StatusTypeDef HAL_Status_en = HAL_ERROR;
    uint32_t LoopIndex_u32 = 0u;
   // uint8_t NumOfPageErase = 0u;
    uint32_t Temp_u32 = 0u;

    /* Check Input
     * Address is flash address?
     * DataIn is NULL Pointer?
     * Size of data write is valid?
     * Start Address is ODD?*/
    if((!IS_FLASH_ADDRESS(StartAddress_u32)) || (((void *)0) == DataIn_pu16) ||
       (Size_u32 > MAX_SIZEOFFLASH) || (0u == Size_u32) || (IS_DATA_ODD(StartAddress_u32)))
    {
        return HAL_ERROR;
    }

    /*Erase page before write*/
//    NumOfPageErase = ((Size_u32 % 2048u) != 0) ? ((Size_u32 >> 11u) + 1u) : (Size_u32 >> 11u);
//    FlashNVM_ErasePage(StartAddress_u32, NumOfPageErase);

    /*Unlock Flash memory*/
    HAL_Status_en = HAL_FLASH_Unlock();
    if (HAL_OK == HAL_Status_en)
    {
        /*Clear Status Register*/
        __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_WRPERR | FLASH_FLAG_PGERR);

        /*Add one if size is odd. Because of write 2 bytes at one*/
        Size_u32 = IS_DATA_ODD(Size_u32) ? (Size_u32 + 1u) : (Size_u32);
        Temp_u32 = (Size_u32 >> 1u); /*Write 2 bytes per one then LoopIndex has to divide by 2*/
        /*Write data*/
        for (LoopIndex_u32 = 0u; LoopIndex_u32 < Temp_u32; ++LoopIndex_u32)
        {
            HAL_Status_en = HAL_BUSY;
            while (HAL_BUSY == HAL_Status_en)
            {
                HAL_Status_en = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, (StartAddress_u32 + (2u*LoopIndex_u32)), DataIn_pu16[LoopIndex_u32]);
            }
            if (HAL_OK != HAL_Status_en)
            {
                /*Write memory error.*/
                break;
            }
        }
    }
    /*Lock Flash Memory*/
    HAL_FLASH_Lock();
    return HAL_Status_en;
}
//
//
///**
//  * @brief  Count Sectors data size
//  * @param  start_sector: start FLASH sector
//  * @param  last_sector: last FLASH area sector
//  * @retval overall data size in bytes
//  */
//uint32_t FlashNVM_GetSectorSize(uint8_t start_sector, uint8_t last_sector)
//{
//	uint32_t size = 0;
//	uint8_t sect;
//
//	for (sect = start_sector; sect <= last_sector; sect++)
//	{
//#ifndef STM32F4
//		size += 4 * 1024; //4 Kbytes For STM32F0xx devices
//#else
//		switch (sect) {
//			case FLASH_SECTOR_0:
//			case FLASH_SECTOR_1:
//			case FLASH_SECTOR_2:
//			case FLASH_SECTOR_3:
//				size += 16 * 1024; // 16 Kbytes
//				break;
//
//			case FLASH_SECTOR_4:
//				size += 64 * 1024; // 64 Kbytes
//				break;
//
//			default:
//				size += 128 * 1024; // 128 Kbytes
//				break;
//		}
//#endif
//	}
//	return size;
//}
//
//
//
///**
//  * @brief  Count selected data bank size
//  * @param  fl_bank: flash area (bootloader/ application or its copy bank)
//  * @retval overall data size in bytes
//  */
//uint32_t FlashNVM_GetBankSize(FLASH_BANK fl_bank)
//{
//
//	if (fl_bank == FLASH_BANK_BOOTLOADER) {
//			return FlashNVM_GetSectorSize(FLASH_BANKB_START_SECTOR, FLASH_BANKB_START_SECTOR + FLASH_BANKB_SECTORS - 1);
//	}
//
//	if (fl_bank == FLASH_BANK_APPLICATION) {
//			return FlashNVM_GetSectorSize(FLASH_BANKA_START_SECTOR, FLASH_BANKA_SECTORS + FLASH_BANKB_SECTORS - 1);
//	}
//
//	if (fl_bank == FLASH_BANK_COPY) {
//			return FlashNVM_GetSectorSize(FLASH_BANKC_START_SECTOR, FLASH_BANKC_SECTORS + FLASH_BANKB_SECTORS - 1);
//	}
//
//	return 0;
//}
//
//
///**
//  * @brief  Count selected data bank start address
//  * @param  fl_bank: flash area (bootloader/ application or its copy bank)
//  * @retval start address value in FLASH
//  */
//uint32_t FlashNVM_GetBankStartAddress(FLASH_BANK fl_bank)
//{
//	if (fl_bank == FLASH_BANK_BOOTLOADER) {
//		return FLASH_BASE + FlashNVM_GetSectorSize(FLASH_SECTOR_0, FLASH_BANKB_START_SECTOR) - FlashNVM_GetSectorSize(FLASH_BANKB_START_SECTOR, FLASH_BANKB_START_SECTOR);
//	}
//
//	if (fl_bank == FLASH_BANK_APPLICATION) {
//		return FLASH_BASE + FlashNVM_GetSectorSize(FLASH_SECTOR_0, FLASH_BANKA_START_SECTOR) - FlashNVM_GetSectorSize(FLASH_BANKA_START_SECTOR, FLASH_BANKA_START_SECTOR);
//	}
//
//	if (fl_bank == FLASH_BANK_COPY) {
//		return FLASH_BASE + FlashNVM_GetSectorSize(FLASH_SECTOR_0, FLASH_BANKC_START_SECTOR) - FlashNVM_GetSectorSize(FLASH_BANKC_START_SECTOR, FLASH_BANKC_START_SECTOR);
//	}
//	return 0;
//}
