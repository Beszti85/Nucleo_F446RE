/*
 * pc_act_cmd.c
 *
 *  Created on: Sep 7, 2025
 *      Author: drCsabesz
 */

#include "pc_act_cmd.h"
#include "ds1307.h"

extern DS1307_Handler_t DS1307_Handle;

void PC_ExecCmdHandler( uint8_t* ptrRxBuffer, uint8_t* ptrTxBuffer )
{
  uint8_t cmd = ptrRxBuffer[0];
  // Cehck first byte
  switch (cmd)
  {
    // Read external flash
    case PC_CMD_READ_EXT_FLASH:
      //FLASH_Read()
      break;
    // Write external flash
    case PC_CMD_WRITE_EXT_FLASH:
      //FLASH_Write()
      break;
    // Erase external flash
    case PC_CMD_ERASE_EXT_FLASH:
      //FLASH_Erase()
      break;
    // Start DS1307
    case PC_CMD_DS1307_START:
      //DS1307_Start()
      break;
    case PC_CMD_DS1307_CTRL_SQW:
      DS1307_SquareWaveOutput(&DS1307_Handle, (DS1307_Frequency_e)ptrRxBuffer[0]);
      break;
  }
}
