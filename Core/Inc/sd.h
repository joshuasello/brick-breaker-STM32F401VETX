/*
 * sd.h
 *
 *  Created on: Sep 15, 2020
 *      Author: lvisagie
 */

#ifndef INC_SD_H_
#define INC_SD_H_

uint8_t SD_Init();
uint8_t SD_Read(uint8_t* rxbuffer, uint32_t address, uint32_t numblocks);
uint8_t SD_Write(uint8_t* txbuffer, uint32_t address, uint32_t numblocks);
uint8_t SD_RxDataBlock(uint8_t* buff, uint16_t len);
uint8_t SD_SendCommand(uint8_t cmd, uint32_t args);
uint8_t SD_ReceiveByte();
void SD_Select();
void SD_Deselect();

#endif /* INC_SD_H_ */
