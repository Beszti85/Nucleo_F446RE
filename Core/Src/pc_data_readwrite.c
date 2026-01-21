/*
 * pc_data_readwrite.c
 *
 *  Created on: Sep 7, 2025
 *      Author: drCsabesz
 */

#include "pc_data_readwrite.h"
#include "flash.h"
#include "ds1307.h"

extern FLASH_Handler_t FlashHandler;
extern float ADC_Voltage[5u];

// Static module variables
uint32_t PcExtFlashReadAddress = 0u;
uint32_t PcExtFlashReadLength = 0u;

// Data read handler
uint8_t PC_ReadDataHandler( uint8_t readId, uint8_t* ptrTxBuffer )
{
  // return value is the length of the response
  uint8_t retval = 0u;

  switch (readId)
  {
    // Read board name
    case PC_RD_BOARD_ID:
      memcpy(ptrTxBuffer, "NucleoF446RE", sizeof("NucleoF446RE"));
      retval = sizeof("NucleoF446RE");
      break;
    // Read ADC data
    case PC_RD_ADC_PHY_VALUES:
      memcpy(ptrTxBuffer, ADC_Voltage, sizeof(ADC_Voltage));
      retval = 20u;
      break;
    // DS1307 time read
    case PC_RD_DS1307_READ_TIME:
      // TODO
      break;
    // Read Flash data
    case PC_RD_FLASH_READ:
      FLASH_Read(&FlashHandler, PcExtFlashReadAddress, ptrTxBuffer, PcExtFlashReadLength);
      retval = PcExtFlashReadLength;
      break;
    default:
      break;
  }

  return retval;
}

// Data write handler
void PC_WriteDataHandler( uint8_t* ptrTxBuffer )
{
  uint8_t writeId = ptrTxBuffer[0];
  switch (writeId)
  {
    // Set flash read address
    case PC_WR_FLASH_CFG_WRITE:
      memcpy(&PcExtFlashReadAddress, ptrTxBuffer, sizeof(PcExtFlashReadAddress));
      memcpy(&PcExtFlashReadLength, ptrTxBuffer + sizeof(PcExtFlashReadAddress), sizeof(PcExtFlashReadLength));
      break;
    default:
      break;
  }
}