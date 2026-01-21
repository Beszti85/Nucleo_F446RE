/*
 * pc_data_readwrite.h
 *
 *  Created on: Sep 1, 2025
 *      Author: drCsabesz
 */

#ifndef INC_PC_DATA_READWRITE_H_
#define INC_PC_DATA_READWRITE_H_

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#define PC_RD_BOARD_ID                 0u
#define PC_RD_BME280_PHYSICAL_VALUES   1u
#define PC_RD_ADC_PHY_VALUES           2u
#define PC_RD_FLASH_ID                 3u
#define PC_RD_DS1307_READ_TIME         4u
#define PC_RD_FLASH_READ               5u

#define PC_WR_FLASH_CFG_WRITE          1u

uint8_t PC_ReadDataHandler( uint8_t readId, uint8_t* ptrTxBuffer );
void PC_WriteDataHandler( uint8_t* ptrTxBuffer );

#endif /* INC_PC_DATA_READWRITE_H_ */
