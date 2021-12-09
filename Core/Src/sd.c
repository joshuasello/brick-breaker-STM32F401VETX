/*
 * sd.c
 *
 *  Created on: Sep 15, 2020
 *      Author: lvisagie
 */
#include "main.h"

extern SPI_HandleTypeDef hspi1;

void Select()
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 0);
	HAL_Delay(1);
}

void Deselect()
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 1);
	HAL_Delay(1);
}

void SD_Select()
{
	Select();
}
void SD_Deselect()
{
	Deselect();
}

uint8_t SD_ReceiveByte()
{
	uint8_t dummy = 0xff;
	uint8_t data;
	HAL_SPI_TransmitReceive(&hspi1, &dummy, &data, 1, 10);
	return data;
}

uint8_t SD_SendCommand(uint8_t cmd, uint32_t args)
{
	uint8_t res;
	uint32_t tickstart = HAL_GetTick();
	do
	{
		res = SD_ReceiveByte();
	} while (((HAL_GetTick() - tickstart) < 50) && (res != 0xff));

	uint8_t cmdbuffer[6];
	cmdbuffer[0] = 0x40 | cmd;
	cmdbuffer[1] = (uint8_t)(args >> 24);
	cmdbuffer[2] = (uint8_t)(args >> 16);
	cmdbuffer[3] = (uint8_t)(args >> 8);
	cmdbuffer[4] = (uint8_t)(args);
	if (cmd == 0)
		cmdbuffer[5] = 0x95;
	else if (cmd == 8)
		cmdbuffer[5] = 0x87;
	else
		cmdbuffer[5] = 1;

	HAL_SPI_Transmit(&hspi1, cmdbuffer, 6, 10);

	// read response code
	uint8_t n = 10;
	do
	{
		res = SD_ReceiveByte();
		n--;
	} while (((res & 0x80) == 0x80) && (n > 0));

	return res;
}



uint8_t SD_Init()
{
	Deselect();

	uint8_t dummy = 0xff;
	for (int i = 0; i < 10; i++)
	{
		HAL_SPI_Transmit(&hspi1, &dummy, 1, 10);
	}

	Select();

	// send command 0
	if (SD_SendCommand(0, 0) != 1)
		return 0;

	// send command 8
	if (SD_SendCommand(8, 0x1aa) != 1)
		return 0;

	// read R7 response after cmd8
	uint8_t cmdreturn[4];
	for (int i = 0; i < 4; i++)
		cmdreturn[i] = SD_ReceiveByte();

	if ((cmdreturn[2] != 1) && (cmdreturn[3] != 0xaa))
		return 0;

	uint32_t tickstart = HAL_GetTick();
	do
	{
		if (SD_SendCommand(55, 0) <= 1)
		{
			if (SD_SendCommand(41, 1 << 30) == 0)
			{
				// ok!
				break;
			}
		}
	} while ((HAL_GetTick() - tickstart) < 1000);

	Deselect();

	return 1;
}


uint8_t SD_RxDataBlock(uint8_t* buff, uint16_t len)
{
	uint8_t token;

	uint32_t ticksstart = HAL_GetTick();

	// loop until response received or timeout
	do
	{
		token = SD_ReceiveByte();
	} while ((token == 0xff) && (HAL_GetTick() - ticksstart < 20));

	// check for invalid response
	if (token != 0xfe)
		return 0;

	// receive data
	for (int i = 0; i < len; i++)
	{
		buff[i] = SD_ReceiveByte();
	}

	// read and discard CRC
	SD_ReceiveByte();
	SD_ReceiveByte();

	// return success
	return 1;
}

uint8_t SD_Read(uint8_t* rxbuffer, uint32_t address, uint32_t numblocks)
{
	Select();

	if (numblocks == 1)
	{
		// read single block
		if (SD_SendCommand(17, address) != 0)
			return 0;

		SD_RxDataBlock(rxbuffer, 512);

		Deselect();

		return 1;
	}
	else
	{
		// read multiple blocks
		if (SD_SendCommand(18, address) != 0)
			return 0;

		for (int i = 0; i < numblocks; i++)
		{
			if (!SD_RxDataBlock(rxbuffer, 512))
				break;

			rxbuffer += 512;
		}

		// stop receiving
		SD_SendCommand(12, 0);

		Deselect();

		return 1;
	}
}

uint8_t SD_TxDataBlock(uint8_t* buff, uint8_t token)
{
	uint8_t crc[2] = {0, 0};
	uint8_t resp;

	// transmit token
	HAL_SPI_Transmit(&hspi1, &token, 1, 10);

	// transmit data bytes
	HAL_SPI_Transmit(&hspi1, buff, 512, 10);

	// transmit dummy crc
	HAL_SPI_Transmit(&hspi1, crc, 2, 10);

	// wait for data response token
	for (int i = 0; i < 64; i++)
	{
		resp = SD_ReceiveByte();

		if ((resp & 0x1f) == 0x05)
			break;
	}

	// now wait for programming to finish. TODO: add a timeout here...
	while (SD_ReceiveByte() == 0);

	// return success if data was accepted
	if ((resp & 0x1f) == 0x05)
		return 1;

	return 0;
}

uint8_t SD_Write(uint8_t* txbuffer, uint32_t address, uint32_t numblocks)
{
	Select();

	if (numblocks == 1)
	{
		if (SD_SendCommand(24, address) == 0)
		{
			SD_TxDataBlock(txbuffer, 0xfe);
		}
	}
	else
	{
		if (SD_SendCommand(25, address) == 0)
		{
			for (int i = 0; i < numblocks; i++)
			{
				if (!SD_TxDataBlock(txbuffer, 0xfc))
					break;

				txbuffer += 512;
			}

			uint8_t token = 0xfd;
			HAL_SPI_Transmit(&hspi1, &token, 1, 10);
		}
	}

	Deselect();

	return 1;
}
