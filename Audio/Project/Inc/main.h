/**
  ******************************************************************************
  * @file    Audio_playback_and_record/inc/main.h
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    26-June-2014
  * @brief   Header for main.c module.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */ 
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
//#include "stm32f4xx_hal.h"
//#include "stm32f4_discovery.h"
//#include "stm32f4_discovery_audio.h"
//#include "stm32f4_discovery_accelerometer.h"
//#include <stdio.h>
//#include "stm32f4xx_it.h"
//#include "waveplayer.h"
//
//#include "waverecorder.h"
//#include "ff.h"
//#include "ff_gen_drv.h"
//#include "usbh_diskio.h"
 
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Uncomment this define to disable repeat feature */
/* #define PLAY_REPEAT_DISABLED */

typedef enum
{
  APPLICATION_IDLE = 0,  
  APPLICATION_START,    
  APPLICATION_RUNNING,
}
MSC_ApplicationTypeDef;
/* You can change the Wave file name as you need, but do not exceed 11 characters */
#define WAVE_NAME "0:audio.wav"

  
/* State Machine for the USBH_USR_ApplicationState */
#define USBH_USR_FS_INIT    ((uint8_t)0x00)
#define USBH_USR_AUDIO      ((uint8_t)0x01)

/* Defines for the Audio used commands */
#define CMD_PLAY           ((uint32_t)0x00)
#define CMD_RECORD         ((uint32_t)0x01)
#define CMD_STOP           ((uint32_t)0x02)



/* Defines for the Audio playing process */
#define PAUSE_STATUS     ((uint32_t)0x00) /* Audio Player in Pause Status */
#define RESUME_STATUS    ((uint32_t)0x01) /* Audio Player in Resume Status */
#define IDLE_STATUS      ((uint32_t)0x02) /* Audio Player in Idle Status */

#define REPEAT_ON        ((uint32_t)0x00) /* Replay Status in ON */
#define REPEAT_OFF       ((uint32_t)0x01) /* Replay Status in OFF */


                                                                                    
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void Error_Handler(void);

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
