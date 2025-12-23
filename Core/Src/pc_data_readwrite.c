/*
 * pc_data_readwrite.c
 *
 *  Created on: Sep 7, 2025
 *      Author: drCsabesz
 */

#include "pc_data_readwrite.h"
#include "bme280.h"
#include "flash.h"
#include "ds1307.h"

extern BME280_PhysValues_t BME280_PhysicalValues;
extern FLASH_Handler_t FlashHandler;
extern float ADC_Voltage[5u];

uint8_t PC_ReadDataHandler( uint8_t readId, uint8_t* ptrTxBuffer )
{
  // return value is the length of the response
  uint8_t retval = 0u;

  switch (readId)
  {
    // Read board name
    case BOARD_ID:
      memcpy(ptrTxBuffer, "NucleoF446RE", sizeof("NucleoF446RE"));
      retval = sizeof("NucleoF446RE");
      break;
    // Read ADC data
    case ADC_PHY_VALUES:
      memcpy(ptrTxBuffer, ADC_Voltage, sizeof(ADC_Voltage));
      retval = 20u;
      break;
    // DS1307 time read
    case DS1307_READ_TIME:
      // TODO
      break;
    default:
      break;
  }

  return retval;
}
