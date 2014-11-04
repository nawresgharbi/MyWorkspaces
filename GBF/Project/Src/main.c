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
#include "stm324x9i_eval_ts.h"
#include "lcd_log.h"
#include "stdlib.h"
#include "background.h"
#include "string.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */
char buffer_HZ[4] = "000";
char buffer_KHZ[4] = "000";
char buffer_MHZ[4] = "000";
int counter = 0;
int multiplier = 1;
int frequency = 0;
TS_StateTypeDef cord;
uint16_t y ;
uint16_t x;
int counter_start = 0;
char frequency_c[20];
char frequency_display[20];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);

/* USER CODE BEGIN PFP */
void GetFrequency(void);
void TimerUpdate(int frequency);
void frequency_displayer(int frequency);
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
  BSP_JOY_Init(JOY_MODE_EXTI);
  BSP_PB_Init(BUTTON_TAMPER, BUTTON_MODE_EXTI);
  
  BSP_IO_Init();
  BSP_LCD_Init();
  BSP_TS_Init(BSP_LCD_GetXSize(), BSP_LCD_GetYSize());
  BSP_TS_ITConfig();
  
  
  BSP_LCD_LayerDefaultInit(0, LCD_FB_START_ADDRESS); 
  BSP_LCD_SelectLayer(0);
  
  BSP_LCD_DisplayOn();
  LCD_LOG_Init();
  
  BSP_LCD_Clear(LCD_COLOR_BLACK);
  
  BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
  BSP_LCD_SetBackColor(LCD_COLOR_RED);
  BSP_LCD_DrawBitmap(0, 0, (uint8_t *)background);
  BSP_LCD_SetFont(&Font24);
  BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
  BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
  
  frequency = 10000;
  TimerUpdate(frequency);
  sprintf(frequency_c,"%d",frequency);
  strcat(frequency_c, "  Hz");
  BSP_LCD_DisplayStringAtLine(4, (uint8_t *)frequency_c);
  BSP_LCD_DisplayStringAtLine(7, (uint8_t *)frequency_c);

  
  
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
  /* GPIO Ports Clock Enable */
  __GPIOC_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOH_CLK_ENABLE();
  __GPIOE_CLK_ENABLE();
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t gpio)
{
  if (gpio == GPIO_PIN_8)
  {
  GetFrequency();
  }
}


void GetFrequency(void)
{
  

BSP_TS_GetState(&cord);
  
     y = cord.y;
     x = cord.x;

if (cord.TouchDetected == 1)
{
  if ((y>=228) && (y<=260))
  {
    if ((x>=1) && (x<=46))
    {
      multiplier = 10000000;
    }
        if ((x>=46) && (x<=72))
    {
      multiplier = 1000000;
    }
        if ((x>=72) && (x<=108))
    {
      multiplier = 100000;
    }
        if ((x>=108) && (x<=140))
    {
      multiplier = 10000;
    }
        if ((x>=140) && (x<=174))
    {
      multiplier = 1000;
    }
        if ((x>=174) && (x<=207))
    {
      multiplier = 100;
    }
        if ((x>=207) && (x<=232))
    {
      multiplier = 10;
    }
        if ((x>=232) && (x<=261))
    {
      multiplier = 1;
    }
    
    if ((x>=261) && (x <= 430))
    {
      if (counter_start == 0)
      {
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
        counter_start++;
      }
      else
      {
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
        counter_start--;
      }
    }
  }
  
  if ((x>=418) && (x<=460))
  {
    if ((y>=38) && (y<=84))
    {
      frequency = frequency + multiplier;
//      sprintf(frequency_c,"%d",frequency);
//      strcat(frequency_c, "  Hz");
      frequency_displayer(frequency);
      BSP_LCD_DisplayStringAtLine(7, (uint8_t *)frequency_display);
      
    }
    if ((y>=84) && (y<=142))
    {
      frequency = frequency - multiplier;
      if (frequency < 0)
      {
        frequency = 0;
      }
//      sprintf(frequency_c,"%d",frequency);
//      strcat(frequency_c, "  Hz");
      frequency_displayer( frequency);
      BSP_LCD_DisplayStringAtLine(7, (uint8_t *)frequency_display);
    }
    
    if ((y>=142) && (y<=180))
    {
      TimerUpdate(frequency);
      //      sprintf(frequency_c,"%d",frequency);
      //      strcat(frequency_c, "  Hz");
      frequency_displayer( frequency);
      BSP_LCD_DisplayStringAtLine(4, (uint8_t *)frequency_display);
    }
  }

    
}

#ifdef USE_JOYSTICK
  switch(BSP_JOY_GetState())
  {
  
  case JOY_SEL:
  
  TimerUpdate(frequency);
  break;
  
  case JOY_LEFT:
          counter++;

    if( counter ==0 )
    {
      multiplier = 1;
    }
    
    else if( counter ==1)
    {
      multiplier = 10;
    }
    
    else if( counter ==2)
    {
      multiplier = 100;
    }
    else if( counter ==3)
    {
      multiplier = 1000;
    }
    
    else if( counter ==4)
    {
      multiplier = 10000;
    }
    else if( counter ==5)
    {
      multiplier = 100000;
    }
    
    else if( counter ==6)
    {
      multiplier = 1000000;
    }
    
    else if( counter ==7)
    {
      multiplier = 10000000;
    }
    
    else if( counter >7)
    {
      multiplier = 1;
      counter = 0;
      
    }

  break;  
    
  
  
  case JOY_RIGHT:
    
    counter--;
    if( counter ==0 )
    {
      multiplier = 1;
    }
    
    else if( counter ==1)
    {
      multiplier = 10;
    }
    
    else if( counter ==2)
    {
      multiplier = 100;
    }
    else if( counter ==3)
    {
      multiplier = 1000;
    }
    
    else if( counter ==4)
    {
      multiplier = 10000;
    }
    else if( counter ==5)
    {
      multiplier = 100000;
    }
    
    else if( counter ==6)
    {
      multiplier = 1000000;
    }
    
    else if( counter ==7)
    {
      multiplier = 10000000;
    }
    
    else if( counter < 0)
    {
      multiplier = 10000000;
      counter = 7;
    }
    break;
    
  case JOY_DOWN:

      frequency = frequency - multiplier;
  break;
    
  case JOY_UP:
    frequency = frequency + multiplier;
  break;
    
  default:
    break;
    
  }
#endif
}


/** void timerupdate
   * @brief 
*/
void TimerUpdate(int frequency)
{
  int prescaler = 0;
  int TimFrequency = 168000000;
  
  while (((TimFrequency/(prescaler+1))/(frequency)) >= 0xFFFF)
  {
    prescaler ++;
  }
  
  TIM1->PSC = (prescaler);
  TIM1->ARR = ((TimFrequency/(prescaler+1))/(frequency))-1;
  TIM1->CCR1 = (TIM1->ARR)/2;
}


void frequency_displayer(int frequency)
{

  int divider = 1;
  while ((frequency / divider)>=1)
  {
    divider = divider * 10;
  }
  
  divider = divider / 10;
  
  if (divider >= 1000000)
  {
    sprintf(frequency_display,"%d",frequency/1000000);
    strcat(frequency_display, "  MHz");
  }
  
  else if (divider >= 1000)
  {
    sprintf(frequency_display,"%d",frequency/1000);
    strcat(frequency_display, "  KHz");
  }
  
  else
  {
    sprintf(frequency_display,"%d",frequency);
    strcat(frequency_display, "  Hz");
  }
  

  
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
