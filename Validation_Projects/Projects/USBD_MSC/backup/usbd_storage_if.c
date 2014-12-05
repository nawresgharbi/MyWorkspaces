/**
  ******************************************************************************
  * @file           : usbd_storage_if.c
  * @author         : MCD Application Team
  * @version        : V1.1.0
  * @date           : 19-March-2012
  * @brief          : Memory management layer
  ******************************************************************************
  * COPYRIGHT(c) 2014 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  * 1. Redistributions of source code must retain the above copyright notice,
  * this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  * this list of conditions and the following disclaimer in the documentation
  * and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of its contributors
  * may be used to endorse or promote products derived from this software
  * without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "usbd_storage_if.h"

/* USER CODE BEGIN xx */ 

/* Memory to be used */
/* USE_FLASH: flash memory will be used as mass storage memory */
/* USE_RAM: RAM memory will be used as mass storage memory */
#define USE_FLASH
#define ADDR_FLASH_SECTOR_11    ((uint32_t)0x080E0000)
/* Memory size to be used.. be careful to use only empty memory*/
#define memory_size 0x10000

/* Memory end address for flash */
#ifdef USE_FLASH
uint32_t memory_end = 0x080F0000;
uint8_t buffer[0x10000];
#endif

/* Memory end address for RAM */
#ifdef USE_RAM
uint32_t memory_end = 0x2002FFE0;
#endif

/* USER CODE END xx */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* USB handler declaration */
/* Handle for USB Full Speed IP */
USBD_HandleTypeDef  *hUsbDevice_0;

extern USBD_HandleTypeDef hUsbDeviceFS;

/* Private function prototypes -----------------------------------------------*/
/* Extern function prototypes ------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

#define STORAGE_LUN_NBR                  1  
#define STORAGE_BLK_NBR                  0x10000  
#define STORAGE_BLK_SIZ                  0x200

static int8_t STORAGE_Init_FS (uint8_t lun);
static int8_t STORAGE_GetCapacity_FS (uint8_t lun, 
                           uint32_t *block_num, 
                           uint16_t *block_size);
static int8_t  STORAGE_IsReady_FS (uint8_t lun);
static int8_t  STORAGE_IsWriteProtected_FS (uint8_t lun);
static int8_t STORAGE_Read_FS (uint8_t lun, 
                        uint8_t *buf, 
                        uint32_t blk_addr,
                        uint16_t blk_len);
static int8_t STORAGE_Write_FS (uint8_t lun, 
                        uint8_t *buf, 
                        uint32_t blk_addr,
                        uint16_t blk_len);
static int8_t STORAGE_GetMaxLun_FS (void);

/* USER CODE BEGIN 0 */ 
/* USB Mass storage Standard Inquiry Data */
const int8_t  STORAGE_Inquirydata_FS[] = {//36
  
  /* LUN 0 */
  0x00,		
  0x80,		
  0x02,		
  0x02,
  (STANDARD_INQUIRY_DATA_LEN - 5),
  0x00,
  0x00,	
  0x00,
  'S', 'T', 'M', ' ', ' ', ' ', ' ', ' ', /* Manufacturer : 8 bytes */
  'P', 'r', 'o', 'd', 'u', 'c', 't', ' ', /* Product      : 16 Bytes */
  ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ',
  '0', '.', '0' ,'1',                     /* Version      : 4 Bytes */
}; 
/* USER CODE END 0 */ 

USBD_StorageTypeDef USBD_Storage_Interface_fops_FS =
{
  STORAGE_Init_FS,
  STORAGE_GetCapacity_FS,
  STORAGE_IsReady_FS,
  STORAGE_IsWriteProtected_FS,
  STORAGE_Read_FS,
  STORAGE_Write_FS,
  STORAGE_GetMaxLun_FS,
  (int8_t *)STORAGE_Inquirydata_FS,
};

/*******************************************************************************
* Function Name  : STORAGE_Init_FS
* Description    : 
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
int8_t STORAGE_Init_FS (uint8_t lun)
{
  /* USER CODE BEGIN 2 */ 
  return (USBD_OK);
  /* USER CODE END 2 */ 
}

/*******************************************************************************
* Function Name  : STORAGE_GetCapacity_FS
* Description    : 
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
int8_t STORAGE_GetCapacity_FS (uint8_t lun, uint32_t *block_num, uint16_t *block_size)
{
  /* USER CODE BEGIN 3 */   
  *block_num  = (memory_size)/STORAGE_BLK_SIZ - 1;
  *block_size = STORAGE_BLK_SIZ;
  return (USBD_OK);
  /* USER CODE END 3 */ 
}

/*******************************************************************************
* Function Name  : STORAGE_IsReady_FS
* Description    : 
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
int8_t  STORAGE_IsReady_FS (uint8_t lun)
{
  /* USER CODE BEGIN 4 */ 
  return (USBD_OK);
  /* USER CODE END 4 */ 
}

/*******************************************************************************
* Function Name  : STORAGE_IsWriteProtected_FS
* Description    : 
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
int8_t  STORAGE_IsWriteProtected_FS (uint8_t lun)
{
  /* USER CODE BEGIN 5 */ 
  return (USBD_OK);
  /* USER CODE END 5 */ 
}

/*******************************************************************************
* Function Name  : STORAGE_Read_FS
* Description    : 
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
int8_t STORAGE_Read_FS (uint8_t lun, 
                        uint8_t *buf, 
                        uint32_t blk_addr,                       
                        uint16_t blk_len)
{
  /* USER CODE BEGIN 6 */ 
  uint8_t* StorageBuffer = (uint8_t*) ((memory_end - memory_size) + blk_addr * STORAGE_BLK_SIZ) ;
  memcpy(buf, StorageBuffer, sizeof(uint8_t)*STORAGE_BLK_SIZ * blk_len);
  return (USBD_OK);
  /* USER CODE END 6 */ 
}

/*******************************************************************************
* Function Name  : STORAGE_Write_FS
* Description    :
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
int8_t STORAGE_Write_FS (uint8_t lun, 
                         uint8_t *buf, 
                         uint32_t blk_addr,
                         uint16_t blk_len)
{
  /* USER CODE BEGIN 7 */ 
  
  #ifdef USE_RAM
  uint8_t* Rambuf = (uint8_t*) ((memory_end - memory_size) + blk_addr * STORAGE_BLK_SIZ) ;
  memcpy(Rambuf, buf, sizeof(uint8_t)*STORAGE_BLK_SIZ * blk_len);
  
  return (USBD_OK);
#endif
  
#ifdef USE_FLASH
  
  uint32_t Address = 0;
  uint8_t* pBuffer = buffer;
  uint32_t SectorError = 0;
  
  uint8_t* Flashbuf = (uint8_t*) ((memory_end - memory_size) );
  FLASH_EraseInitTypeDef EraseInitStruct;
  
  memset(&EraseInitStruct, 0, sizeof(EraseInitStruct));
  memcpy(buffer, Flashbuf, memory_size);
  memcpy(buffer + blk_addr * STORAGE_BLK_SIZ, buf, sizeof(uint8_t)*STORAGE_BLK_SIZ * blk_len);
  
  HAL_FLASH_Unlock();
  
  EraseInitStruct.TypeErase = TYPEERASE_SECTORS;
  EraseInitStruct.VoltageRange = VOLTAGE_RANGE_3;
  EraseInitStruct.Sector = FLASH_SECTOR_11;
  EraseInitStruct.NbSectors = 1;
  HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError);
  
  Address = ADDR_FLASH_SECTOR_11;
  while (Address < (ADDR_FLASH_SECTOR_11+0x10000))
  {
    HAL_FLASH_Program(TYPEPROGRAM_WORD, Address, *(uint32_t*)pBuffer);
    pBuffer+=4;
    Address += 4;
  }  
    return (USBD_OK);
#endif
  /* USER CODE END 7 */ 
}

/*******************************************************************************
* Function Name  : STORAGE_GetMaxLun_FS
* Description    : 
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
int8_t STORAGE_GetMaxLun_FS (void)
{
  /* USER CODE BEGIN 8 */ 
  
  return (STORAGE_LUN_NBR - 1);
  /* USER CODE END 8 */ 
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
