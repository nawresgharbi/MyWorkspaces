/**
  ******************************************************************************
  * File Name          : main.c
  * Date               : 29/10/2014 18:30:23
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2014 STMicroelectronics
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

/* USER CODE BEGIN Includes */
#include "stm324x9i_eval.h"
#include "stm324x9i_eval_io.h"
#include "stm324x9i_eval_lcd.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */
uint8_t buffer_HZ[1] = {0};
uint8_t buffer_KHZ[1] = {0};
uint8_t buffer_MHZ[1] = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);

/* USER CODE BEGIN PFP */
void GetFrequency(void);
void TimerUpdate(int frequency);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();

  /* USER CODE BEGIN 2 */
  BSP_JOY_Init(JOY_MODE_GPIO);
  BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);
  
  BSP_LCD_Init();
  BSP_LCD_DisplayOn();
  BSP_LCD_DisplayStringAtLine(1, buffer_HZ);
  BSP_LCD_DisplayStringAtLine(2, buffer_KHZ);
  BSP_LCD_DisplayStringAtLine(3, buffer_MHZ);
  
  
  /* USER CODE END 2 */

  /* USER CODE BEGIN 3 */
  /* Infinite loop */
  while (1)
  {

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
                              |RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_PLLCLK, RCC_MCODIV_2);

  HAL_RCC_MCOConfig(RCC_MCO2, RCC_MCO2SOURCE_HSE, RCC_MCODIV_1);

}

/* TIM1 init function */
void MX_TIM1_Init(void)
{


  TIM_OC_InitTypeDef sConfigOC;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0x0;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  HAL_TIM_PWM_Init(&htim1);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0x0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);

}

/** Configure pins
     PC9   ------> RCC_MCO_2
     PA8   ------> RCC_MCO_1
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOC_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOH_CLK_ENABLE();
  __GPIOE_CLK_ENABLE();

  /*Configure GPIO pin : PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void GetFrequency(void)
{
  int counter = 0;
  int frequency = 0;
  BSP_LCD_DisplayStringAtLine(5, "HZ");
    
  switch(BSP_JOY_GetState())
  {
  
  case JOY_SEL:
  frequency = buffer_HZ[1] + (buffer_KHZ[1]*1000) + (buffer_MHZ[1]*1000000);
  TimerUpdate(frequency);
  break;
  
  case JOY_LEFT:
    counter++;
    if( counter ==0 )
    {
      BSP_LCD_ClearStringLine(5);
      BSP_LCD_DisplayStringAtLine(5, "HZ");
    }
    
    else if( counter ==1)
    {
      BSP_LCD_ClearStringLine(5);
      BSP_LCD_DisplayStringAtLine(5, "KHZ");
    }
    
    else if( counter ==2)
    {
      BSP_LCD_ClearStringLine(5);
      BSP_LCD_DisplayStringAtLine(5, "MHZ");
    }
    else if (counter == 3)
    {
      BSP_LCD_ClearStringLine(5);
      counter = 0;
      BSP_LCD_DisplayStringAtLine(5, "HZ");
    }
  break;  
    
  
  
  case JOY_RIGHT:
    counter--;
    if( counter ==0 )
    {
      BSP_LCD_ClearStringLine(5);
      BSP_LCD_DisplayStringAtLine(5, "HZ");
    }
    
    else if( counter ==1)
    {
      BSP_LCD_ClearStringLine(5);
      BSP_LCD_DisplayStringAtLine(5, "KHZ");
    }
    
    else if( counter ==2)
    {
      BSP_LCD_ClearStringLine(5);
      BSP_LCD_DisplayStringAtLine(5, "MHZ");
    }
    else if (counter == -1)
    {
      BSP_LCD_ClearStringLine(5);
      counter = 2;
      BSP_LCD_DisplayStringAtLine(5, "MHZ");
    }
    break;
    
  case JOY_DOWN:
    if (counter ==0)
    {
      buffer_HZ[1] = buffer_HZ[1] -1;
      BSP_LCD_ClearStringLine(1);
      BSP_LCD_DisplayStringAtLine(1, buffer_HZ);
    }
    else if (counter ==1)
    {
      buffer_KHZ[1] = buffer_KHZ[1] -1;
      BSP_LCD_ClearStringLine(2);
      BSP_LCD_DisplayStringAtLine(2, buffer_KHZ);
    }
    else if (counter ==2)
    {
      buffer_MHZ[1] = buffer_MHZ[1] -1;
      BSP_LCD_ClearStringLine(3);
      BSP_LCD_DisplayStringAtLine(3, buffer_MHZ);
    }
    


  break;
    
  case JOY_UP:
    if (counter ==0)
    {
      buffer_HZ[1] = buffer_HZ[1] +1;
      BSP_LCD_ClearStringLine(1);
      BSP_LCD_DisplayStringAtLine(1, buffer_HZ);
    }
    else if (counter ==1)
    {
      buffer_KHZ[1] = buffer_KHZ[1] +1;
      BSP_LCD_ClearStringLine(2);
      BSP_LCD_DisplayStringAtLine(2, buffer_KHZ);
    }
    else if (counter ==2)
    {
      buffer_MHZ[1] = buffer_MHZ[1] +1;
      BSP_LCD_ClearStringLine(3);
      BSP_LCD_DisplayStringAtLine(3, buffer_MHZ);
    }
    

  break;
    
  default:
    break;
    
    
    
    
  }

}


/** void timerupdate
   * @brief 
*/
void TimerUpdate(int frequency)
{
  int prescaler = 0;
  int TimFrequency = 86000000;
  
  while (((TimFrequency/(prescaler+1))/(frequency)) >= 0xFFFF)
  {
    prescaler ++;
  }
  
  TIM1->PSC = prescaler;
  TIM1->ARR = ((TimFrequency/(prescaler+1))/(frequency))-1;
  TIM1->CCR1 = (TIM1->ARR)/2;
}
/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
