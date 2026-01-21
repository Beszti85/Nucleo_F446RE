/*
 * pc_act_cmd.h
 *
 *  Created on: Sep 1, 2025
 *      Author: drCsabesz
 */

#ifndef INC_PC_ACT_CMD_H_
#define INC_PC_ACT_CMD_H_

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#define PC_CMD_READ_EXT_FLASH       0x01
#define PC_CMD_WRITE_EXT_FLASH      0x02
#define PC_CMD_ERASE_EXT_FLASH      0x03
#define PC_CMD_DS1307_START         0x10
#define PC_CMD_DS1307_CTRL_SQW      0x11

void PC_ExecCmdHandler( uint8_t* ptrRxBuffer, uint8_t* ptrTxBuffer );

#endif /* INC_PC_ACT_CMD_H_ */
