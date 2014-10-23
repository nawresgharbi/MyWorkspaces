/**
  ******************************************************************************
  * @file    USB_Device/HID_Application
  * @author  MCD Tools Team
  * @version V0.0.1
  * @date    22/10/2014 
  * @brief   USB device HID Application
  ******************************************************************************
*/




/* Includes ------------------------------------------------------------------*/
#include "HID_Application.h"
#include "usb_device.h" 
#include "usbd_hid.h" 
#include "usbd_conf.h" 


/* Private typedef -----------------------------------------------------------*/
#define USE_USB_FS
/* import handles for USB FS */
#ifdef USE_USB_FS
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
extern USBD_HandleTypeDef hUsbDeviceFS;
#endif

/* import handles for USB HS and FS in HS */
#ifdef USE_USB_HS 
extern PCD_HandleTypeDef hpcd_USB_OTG_HS;
extern USBD_HandleTypeDef hUsbDeviceHS;
#endif


/* Private define ------------------------------------------------------------*/
#define CURSOR_STEP     1
uint32_t HID_Transfert = 0;
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void GetPointerData(uint8_t *pbuf);
/* Private functions ---------------------------------------------------------*/
/* Get joystick state---------------------------------------------------------*/






/* Application ---------------------------------------------------------------*/
/**
  * @brief  Gets Pointer Data.
  * @param  pbuf: Pointer to report
  * @retval None
  */
static void GetPointerData(uint8_t *pbuf)
{
  
  static int8_t x = 0;
  static int8_t y = 0;
  static int8_t Sens = 0;
  static int8_t Pas = 0;
  
  if (Pas == 20)
  {
    Pas=0;
    Sens++;
  }
  
  if(Sens == 0)
  {
    x=Pas++;
    y=0;       
  } 
  if(Sens == 1)
  {
    y=Pas++;
    x=0;   
  } 
  if (Sens == 2)
  {
    x=256-Pas++;
    y=0;
  } 
  if (Sens == 3)
  {
    y=256-Pas++;
    x=0;
  } 
  if (Sens == 4)
  { 
    Sens=0;
    x=0;
    y=0; 
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
    GetPointerData(HID_Buffer);
    
    
    /* send data though IN endpoint*/
    if((HID_Buffer[1] != 0) || (HID_Buffer[2] != 0))
    {
      #ifdef USE_USB_FS
      USBD_HID_SendReport(&hUsbDeviceFS, HID_Buffer, 4);
      #endif
      
      #ifdef USE_USB_HS
      USBD_HID_SendReport(&hUsbDeviceHS, HID_Buffer, 4);
      #endif
    }    
    counter =0;
  }

}
/* ---------------------------------------------------------------------------*/
