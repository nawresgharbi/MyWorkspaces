/**
  ******************************************************************************
  * @file    USB_Device/HID_Application
  * @author  MCD Tools Team
  * @version V0.0.1
  * @date    22/10/2014 
  * @brief   USB device HID Application
  ******************************************************************************
*/

/* Boards define--------------------------------------------------------------*/
//#define stm32f4xx_eval
//#define stm32f2xx_eval




/* BSP Includes --------------------------------------------------------------*/
//#ifdef stm32f4xx_eval
//  #include "stm324x9i_eval.c" 
//  #include "stm324x9i_eval_io.c" 
//  #include "stm324x9i_eval.h" 
//  #include "stm324x9i_eval_io.h" 
//  #include "stmpe1600.c"
//  #include "../../Drivers\STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_i2c.h"
//  #include "../../Drivers\STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c.c"
//  #include "../../Drivers\STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c_ex.c"
//  #include "../../Drivers\STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_uart.h"
//  #include "../../Drivers\STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c"
//#endif
//
//#ifdef stm32f2xx_eval
//  #include "stm322xg_eval.c" 
//  #include "stm322xg_eval_io.c" 
//  #include "stm322xg_eval.h" 
//  #include "stm322xg_eval_io.h" 
//  #include "stmpe811.c"
//  #include "../../Drivers/STM32F2xx_HAL_Driver/Inc/stm32f2xx_hal_i2c.h"
//  #include "../../Drivers\STM32F2xx_HAL_Driver/Src/stm32f2xx_hal_i2c.c"
//  #include "../../Drivers\STM32F2xx_HAL_Driver/Src/stm32f2xx_hal_i2c_ex.c"
//  #include "../../Drivers\STM32F2xx_HAL_Driver/Inc/stm32f2xx_hal_uart.h"
//  #include "../../Drivers\STM32F2xx_HAL_Driver/Src/stm32f2xx_hal_uart.c"
//#endif


/* Includes ------------------------------------------------------------------*/
#include "HID_Application.h"
#include "usb_device.h" 
#include "usbd_hid.h" 
/* Private typedef -----------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
extern USBD_HandleTypeDef hUsbDeviceFS;
extern uint32_t HID_Transfert;
/* Private define ------------------------------------------------------------*/
#define CURSOR_STEP     5
uint32_t HID_Transfert = 0;
uint32_t counter_x = 0;
uint32_t counter_y = 0;
int i,j = 0;

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t HID_Buffer[4];
/* Private function prototypes -----------------------------------------------*/
void Toggle_Leds(void);
void HIDApplication_Start(void);
static void GetPointerDatax(uint8_t *pbuf);
static void GetPointerDatay(uint8_t *pbuf);
/* Private functions ---------------------------------------------------------*/

void HIDApplication_Start()
{
  /* begin transfert of HID packets to the host */
  HID_Transfert = 1;
}

/* Get joystick state---------------------------------------------------------*/
/**
  * @brief  Gets Pointer Data.
  * @param  pbuf: Pointer to report
  * @retval None
  */
static void GetPointerDatax(uint8_t *pbuf)
{
  static int8_t cnt = 0;
  int8_t  x = 0, y = 0 ;
  
  if(cnt++ > 0)
  {
    x = CURSOR_STEP;
    y = CURSOR_STEP;
  }
  else
  {
    x = -CURSOR_STEP;
    y = -CURSOR_STEP;
  }
  
  pbuf[0] = 0;
  pbuf[1] = x;
  pbuf[2] = y;
  pbuf[3] = 0;
}

static void GetPointerDatay(uint8_t *pbuf)
{
  static int8_t cnt = 0;
  int8_t  x = 0, y = 0 ;
  
  if(cnt++ > 0)
  {
    y = CURSOR_STEP;
  }
  else
  {
    y = -CURSOR_STEP;
  }
  
  pbuf[0] = 0;
  pbuf[1] = x;
  pbuf[2] = y;
  pbuf[3] = 0;
}

/* HID send command with systick callback-------------------------------------*/
void HAL_SYSTICK_Callback(void)
{
uint8_t HID_Buffer[4];
  static __IO uint32_t counter=0;

  
  /* check Joystick state every 10ms */
  if (counter++ == 10)
  {  
    GetPointerDatax(HID_Buffer);
    
    
    /* send data though IN endpoint*/
    if((HID_Buffer[1] != 0) || (HID_Buffer[2] != 0))
    {
      USBD_HID_SendReport(&hUsbDeviceFS, HID_Buffer, 4);
    }
    
    GetPointerDatay(HID_Buffer);
    
    /* send data though IN endpoint*/
    if((HID_Buffer[1] != 0) || (HID_Buffer[2] != 0))
    {
      USBD_HID_SendReport(&hUsbDeviceFS, HID_Buffer, 4);
    }    
    
    counter =0;
  }

}

/* ---------------------------------------------------------------------------*/
