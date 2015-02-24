/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @date    20/02/2015 18:03:59
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
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
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"

/* USER CODE BEGIN 0 */
int usb_status = 0;
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern HCD_HandleTypeDef hhcd_USB_OTG_FS;
extern I2C_HandleTypeDef hi2c1;
/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles EXTI Line[15:10] interrupts.
*/
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */
  
  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
  
  
    /* USER CODE BEGIN EXTI15_10_IRQn 1 */

/* LSB and MSB of status and direction registers*/
uint8_t data_LSB =0;
uint8_t data_MSB =0;
uint8_t data2_LSB =0;
uint8_t data2_MSB =0;
uint8_t ID_MSB=0;
uint8_t ID_LSB=0;
  
  /* activate the power switch*/
  if (usb_status == 0)
  {
    
  usb_status++;
  
  /* Check the device ID*/
  HAL_I2C_Mem_Read(&hi2c1, 0x84/*io exp adress*/, 0x0/*reg addr 0x0080*/, I2C_MEMADD_SIZE_8BIT, &ID_LSB, 0x1/*length*/,  1000/*timout*/);
  HAL_I2C_Mem_Read(&hi2c1, 0x84/*io exp adress*/, 0x1/*reg addr 0x0080*/, I2C_MEMADD_SIZE_8BIT, &ID_MSB, 0x1/*length*/,  1000/*timout*/);
  

  /* Read the status and direction registers*/  
  uint8_t LSB = HAL_I2C_Mem_Read(&hi2c1, 0x84/*io exp adress*/, 0x12/*reg addr 0x0080*/, I2C_MEMADD_SIZE_8BIT, &data_LSB, 0x1/*length*/,  1000/*timout*/);
  uint8_t MSB = HAL_I2C_Mem_Read(&hi2c1, 0x84/*io exp adress*/, 0x13/*reg addr 0x0080*/, I2C_MEMADD_SIZE_8BIT, &data_MSB, 0x1/*length*/,  1000/*timout*/);
  
  HAL_I2C_Mem_Read(&hi2c1, 0x84/*io exp adress*/, 0x14/*reg addr 0x0080*/, I2C_MEMADD_SIZE_8BIT, &data2_LSB, 0x1/*length*/,  1000/*timout*/);
  HAL_I2C_Mem_Read(&hi2c1, 0x84/*io exp adress*/, 0x15/*reg addr 0x0080*/, I2C_MEMADD_SIZE_8BIT, &data2_MSB, 0x1/*length*/,  1000/*timout*/);
  
  /*Value to be set in the LSB to set the IO7 in output mode and high state*/
  LSB|= 0x80;
  

  
  /*configure IO7 as output*/
  HAL_I2C_Mem_Write(&hi2c1, 0x84/*io exp adress*/, 0x14/*reg addr 0x0080*/, I2C_MEMADD_SIZE_8BIT, &LSB/*data*/, 0x1/*length*/, 100/*timout*/);
  HAL_I2C_Mem_Write(&hi2c1, 0x84/*io exp adress*/, 0x15/*reg addr 0x0080*/, I2C_MEMADD_SIZE_8BIT, &MSB/*data*/, 0x1/*length*/, 100/*timout*/);
  
  /*configure IO7 in haigh state*/
  HAL_I2C_Mem_Write(&hi2c1, 0x84/*io exp adress*/, 0x12/*reg addr 0x0080*/, I2C_MEMADD_SIZE_8BIT, &LSB/*data*/, 0x1/*length*/, 100/*timout*/);
  HAL_I2C_Mem_Write(&hi2c1, 0x84/*io exp adress*/, 0x13/*reg addr 0x0080*/, I2C_MEMADD_SIZE_8BIT, &MSB/*data*/, 0x1/*length*/, 100/*timout*/);


  /* Registers read to check if the write operation was successful*/
  HAL_I2C_Mem_Read(&hi2c1, 0x84/*io exp adress*/, 0x12/*reg addr 0x0080*/, I2C_MEMADD_SIZE_8BIT, &data_LSB, 0x1/*length*/,  1000/*timout*/);
  HAL_I2C_Mem_Read(&hi2c1, 0x84/*io exp adress*/, 0x13/*reg addr 0x0080*/, I2C_MEMADD_SIZE_8BIT, &data_MSB, 0x1/*length*/,  1000/*timout*/); 
  HAL_I2C_Mem_Read(&hi2c1, 0x84/*io exp adress*/, 0x0/*reg addr 0x0080*/, I2C_MEMADD_SIZE_8BIT, &ID_LSB, 0x1/*length*/,  1000/*timout*/);
  HAL_I2C_Mem_Read(&hi2c1, 0x84/*io exp adress*/, 0x1/*reg addr 0x0080*/, I2C_MEMADD_SIZE_8BIT, &ID_MSB, 0x1/*length*/,  1000/*timout*/);
  HAL_I2C_Mem_Read(&hi2c1, 0x84/*io exp adress*/, 0x14/*reg addr 0x0080*/, I2C_MEMADD_SIZE_8BIT, &data2_LSB, 0x1/*length*/,  1000/*timout*/);
  HAL_I2C_Mem_Read(&hi2c1, 0x84/*io exp adress*/, 0x15/*reg addr 0x0080*/, I2C_MEMADD_SIZE_8BIT, &data2_MSB, 0x1/*length*/,  1000/*timout*/);
  
  }
  else
  {
    usb_status--;
    uint8_t LSB = 0;
    uint8_t MSB = 0;
    

    /*IO Direction reset*/
    HAL_I2C_Mem_Write(&hi2c1, 0x84/*io exp adress*/, 0x14/*reg addr 0x0080*/, I2C_MEMADD_SIZE_8BIT, &LSB/*data*/, 0x1/*length*/, 100/*timout*/);
    HAL_I2C_Mem_Write(&hi2c1, 0x84/*io exp adress*/, 0x15/*reg addr 0x0080*/, I2C_MEMADD_SIZE_8BIT, &MSB/*data*/, 0x1/*length*/, 100/*timout*/);
    
    /*IO status reset*/
    HAL_I2C_Mem_Write(&hi2c1, 0x84/*io exp adress*/, 0x12/*reg addr 0x0080*/, I2C_MEMADD_SIZE_8BIT, &LSB/*data*/, 0x1/*length*/, 100/*timout*/);
    HAL_I2C_Mem_Write(&hi2c1, 0x84/*io exp adress*/, 0x13/*reg addr 0x0080*/, I2C_MEMADD_SIZE_8BIT, &MSB/*data*/, 0x1/*length*/, 100/*timout*/);
    
    /* Registers read to check if the write operation was successful*/
    HAL_I2C_Mem_Read(&hi2c1, 0x84/*io exp adress*/, 0x12/*reg addr 0x0080*/, I2C_MEMADD_SIZE_8BIT, &data_LSB, 0x1/*length*/,  1000/*timout*/);
    HAL_I2C_Mem_Read(&hi2c1, 0x84/*io exp adress*/, 0x13/*reg addr 0x0080*/, I2C_MEMADD_SIZE_8BIT, &data_MSB, 0x1/*length*/,  1000/*timout*/); 
    HAL_I2C_Mem_Read(&hi2c1, 0x84/*io exp adress*/, 0x0/*reg addr 0x0080*/, I2C_MEMADD_SIZE_8BIT, &ID_LSB, 0x1/*length*/,  1000/*timout*/);
    HAL_I2C_Mem_Read(&hi2c1, 0x84/*io exp adress*/, 0x1/*reg addr 0x0080*/, I2C_MEMADD_SIZE_8BIT, &ID_MSB, 0x1/*length*/,  1000/*timout*/);
    HAL_I2C_Mem_Read(&hi2c1, 0x84/*io exp adress*/, 0x14/*reg addr 0x0080*/, I2C_MEMADD_SIZE_8BIT, &data2_LSB, 0x1/*length*/,  1000/*timout*/);
    HAL_I2C_Mem_Read(&hi2c1, 0x84/*io exp adress*/, 0x15/*reg addr 0x0080*/, I2C_MEMADD_SIZE_8BIT, &data2_MSB, 0x1/*length*/,  1000/*timout*/);
  }
    
    /* USER CODE END EXTI15_10_IRQn 1 */
}

/**
* @brief This function handles USB On The Go FS global interrupt.
*/
void OTG_FS_IRQHandler(void)
{
  /* USER CODE BEGIN OTG_FS_IRQn 0 */

  /* USER CODE END OTG_FS_IRQn 0 */
  HAL_HCD_IRQHandler(&hhcd_USB_OTG_FS);
  /* USER CODE BEGIN OTG_FS_IRQn 1 */

  /* USER CODE END OTG_FS_IRQn 1 */
}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
