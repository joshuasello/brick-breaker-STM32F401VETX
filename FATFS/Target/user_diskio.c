/* USER CODE BEGIN Header */
/**
 ******************************************************************************
  * @file    user_diskio.c
  * @brief   This file includes a diskio driver skeleton to be completed by the user.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
 /* USER CODE END Header */

#ifdef USE_OBSOLETE_USER_CODE_SECTION_0
/* 
 * Warning: the user section 0 is no more in use (starting from CubeMx version 4.16.0)
 * To be suppressed in the future. 
 * Kept to ensure backward compatibility with previous CubeMx versions when 
 * migrating projects. 
 * User code previously added there should be copied in the new user sections before 
 * the section contents can be deleted.
 */
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */
#endif

/* USER CODE BEGIN DECL */

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "ff_gen_drv.h"

#include "sd.h"

uint8_t csd[16];

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
/* Disk status */
static volatile DSTATUS Stat = STA_NOINIT;

/* USER CODE END DECL */

/* Private function prototypes -----------------------------------------------*/
DSTATUS USER_initialize (BYTE pdrv);
DSTATUS USER_status (BYTE pdrv);
DRESULT USER_read (BYTE pdrv, BYTE *buff, DWORD sector, UINT count);
#if _USE_WRITE == 1
  DRESULT USER_write (BYTE pdrv, const BYTE *buff, DWORD sector, UINT count);  
#endif /* _USE_WRITE == 1 */
#if _USE_IOCTL == 1
  DRESULT USER_ioctl (BYTE pdrv, BYTE cmd, void *buff);
#endif /* _USE_IOCTL == 1 */

Diskio_drvTypeDef  USER_Driver =
{
  USER_initialize,
  USER_status,
  USER_read, 
#if  _USE_WRITE
  USER_write,
#endif  /* _USE_WRITE == 1 */  
#if  _USE_IOCTL == 1
  USER_ioctl,
#endif /* _USE_IOCTL == 1 */
};

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initializes a Drive
  * @param  pdrv: Physical drive number (0..)
  * @retval DSTATUS: Operation status
  */
DSTATUS USER_initialize (
	BYTE pdrv           /* Physical drive nmuber to identify the drive */
)
{
  /* USER CODE BEGIN INIT */
	if (SD_Init() == 1)
		Stat = 0;
	else
		Stat = STA_NOINIT;

    return Stat;
  /* USER CODE END INIT */
}
 
/**
  * @brief  Gets Disk Status 
  * @param  pdrv: Physical drive number (0..)
  * @retval DSTATUS: Operation status
  */
DSTATUS USER_status (
	BYTE pdrv       /* Physical drive number to identify the drive */
)
{
  /* USER CODE BEGIN STATUS */
    return Stat;
  /* USER CODE END STATUS */
}

/**
  * @brief  Reads Sector(s) 
  * @param  pdrv: Physical drive number (0..)
  * @param  *buff: Data buffer to store read data
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to read (1..128)
  * @retval DRESULT: Operation result
  */
DRESULT USER_read (
	BYTE pdrv,      /* Physical drive nmuber to identify the drive */
	BYTE *buff,     /* Data buffer to store read data */
	DWORD sector,   /* Sector address in LBA */
	UINT count      /* Number of sectors to read */
)
{
  /* USER CODE BEGIN READ */
	if (SD_Read(buff, sector << 9, count) == 1)
		return RES_OK;
	else
		return RES_ERROR;
  /* USER CODE END READ */
}

/**
  * @brief  Writes Sector(s)  
  * @param  pdrv: Physical drive number (0..)
  * @param  *buff: Data to be written
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to write (1..128)
  * @retval DRESULT: Operation result
  */
#if _USE_WRITE == 1
DRESULT USER_write (
	BYTE pdrv,          /* Physical drive nmuber to identify the drive */
	const BYTE *buff,   /* Data to be written */
	DWORD sector,       /* Sector address in LBA */
	UINT count          /* Number of sectors to write */
)
{ 
  /* USER CODE BEGIN WRITE */
  /* USER CODE HERE */
	if (SD_Write((unsigned char*)buff, sector << 9, count) == 1)
		return RES_OK;
	else
		return RES_ERROR;
  /* USER CODE END WRITE */
}
#endif /* _USE_WRITE == 1 */

/**
  * @brief  I/O control operation  
  * @param  pdrv: Physical drive number (0..)
  * @param  cmd: Control code
  * @param  *buff: Buffer to send/receive control data
  * @retval DRESULT: Operation result
  */
#if _USE_IOCTL == 1
DRESULT USER_ioctl (
	BYTE pdrv,      /* Physical drive nmuber (0..) */
	BYTE cmd,       /* Control code */
	void *buff      /* Buffer to send/receive control data */
)
{
  /* USER CODE BEGIN IOCTL */
    DRESULT res = RES_ERROR;
    uint8_t* ptr = buff;
    uint32_t csize;
    uint16_t mult;
    uint16_t read_bl_len;

    SD_Select();

    switch (cmd)
    {
    	case CTRL_SYNC:/* Complete pending write process (needed at _FS_READONLY == 0) */
    		res = RES_OK;
    		break;
    	case GET_SECTOR_COUNT:	/* Get media size (needed at _USE_MKFS == 1) */
    		if ((SD_SendCommand(9, 0) == 0) && SD_RxDataBlock((unsigned char*)csd, 16)) res = RES_OK;
    		csize = (((uint32_t)csd[6] & 0x3) << 10) + ((uint32_t) csd[7] << 2) + (((uint32_t) csd[8] & 0xc0) >> 6) + 1;
    		read_bl_len = csd[5] & 0x0f;
    		mult = ((csd[9] & 0x3) << 1) + ((csd[10] & 0x80) >> 7);
    		*(unsigned long*) buff = (unsigned long) csize << (2+mult+read_bl_len-9);
    		break;
    	case GET_SECTOR_SIZE:	/* Get sector size (needed at _MAX_SS != _MIN_SS) */
    		*(unsigned short*)buff = 512;
    		res = RES_OK;
    		break;
    	case GET_BLOCK_SIZE: /* Get erase block size (needed at _USE_MKFS == 1) */
    		*(unsigned short*)buff = 512;
    		res = RES_OK;
    		break;
    	case CTRL_TRIM:/* Inform device that the data on the block of sectors is no longer used (needed at _USE_TRIM == 1) */
    		res = RES_OK;
    		break;

    	/* Generic command (Not used by FatFs) */
    	case CTRL_POWER:	/* Get/Set power status */
    		res = RES_OK;
    		break;
    	case CTRL_LOCK:	/* Lock/Unlock media removal */
    		res = RES_OK;
    		break;
    	case CTRL_EJECT:	/* Eject media */
    		res = RES_OK;
    		break;
    	case CTRL_FORMAT:	/* Create physical format on the media */
    		res = RES_OK;
    		break;

    	/* MMC/SDC specific ioctl command */
    	case MMC_GET_TYPE:	/* Get card type */
    		break;
    	case MMC_GET_CSD:	/* Get CSD */
    		if ((SD_SendCommand(9, 0) == 0) && SD_RxDataBlock((unsigned char*)buff, 16)) res = RES_OK;
    		break;
    	case MMC_GET_CID:	/* Get CID */
    		if ((SD_SendCommand(10, 0) == 0) && SD_RxDataBlock((unsigned char*)buff, 16)) res = RES_OK;
    		break;
    	case MMC_GET_OCR:	/* Get OCR */
    		if (SD_SendCommand(58, 0) == 0)
    		{
    			for (int i = 0; i < 4; i++)
    			{
    				*ptr++ = SD_ReceiveByte();
    			}
    			res = RES_OK;
    		}
    		break;
    	case MMC_GET_SDSTAT:	/* Get SD status */
    		*(unsigned char*)buff = Stat;
    		break;
    }

    SD_Deselect();

    return res;
  /* USER CODE END IOCTL */
}
#endif /* _USE_IOCTL == 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
