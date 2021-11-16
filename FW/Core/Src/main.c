/***********************************************************************************************************************
*                                                                                                                      *
* HackerspaceSG WS2812 Lighting Controller                                                                             *
*                                                                                                                      *
* Description: Main Program File                                                                                       *
* Author: Joyce Ng                                                                                                     *
*                                                                                                                      *
* Copyright (c) 2021 Joyce Ng, HackerspaceSG                                                                           *
*                                                                                                                      *
* Redistribution and use in source and binary forms, with or without modification, are permitted provided that the     *
* following conditions are met:                                                                                        *
*                                                                                                                      *
*    * Redistributions of source code must retain the above copyright notice, this list of conditions, and the         *
*      following disclaimer.                                                                                           *
*                                                                                                                      *
*    * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the       *
*      following disclaimer in the documentation and/or other materials provided with the distribution.                *
*                                                                                                                      *
*    * Neither the name of the author nor the names of any contributors may be used to endorse or promote products     *
*      derived from this software without specific prior written permission.                                           *
*                                                                                                                      *
* THIS SOFTWARE IS PROVIDED BY THE AUTHORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED   *
* TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL *
* THE AUTHORS BE HELD LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES        *
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR       *
* BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT *
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE       *
* POSSIBILITY OF SUCH DAMAGE.                                                                                          *
*                                                                                                                      *
***********************************************************************************************************************/

#include "main.h"
#include "cmsis_os.h"
#include "fatfs.h"
#include "lwip.h"
#include "usb_host.h"
#include "lcd_log.h"
#include "hw_init.h"

// Include drivers
#include "ws2812.h"

// Function Prototypes
void StartThread(void *argument);
void WatchdogThread(void *argument);

// Definition of Threads
osThreadId_t defaultTaskHandle;
osThreadId_t watchdogTaskHandle;

const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 4096 * 4
};

const osThreadAttr_t watchdogTask_attributes = {
    .name = "watchdogTask",
    .priority = (osPriority_t) osPriorityNormal,
    .stack_size = 128
  };

int main(void)
{
  // Reset of all peripherals, Initializes the Flash interface and the Systick.
  HAL_Init();

  // Initialize System Clock and hardware peripherals
  hw_init();

  // Initialize RTOS Kernel
  osKernelInitialize();

  // Definition of Threads
  defaultTaskHandle = osThreadNew(StartThread, NULL, &defaultTask_attributes);
  watchdogTaskHandle = osThreadNew(WatchdogThread, NULL, &watchdogTask_attributes);

  // Start RTOS Kernel
  osKernelStart();

  // We should never get here as control is now taken by the scheduler
  for( ;; );

}

// Default Start Thread
void StartThread(void *argument)
{
   // Initialize LCD
   BSP_LCD_Init();
   BSP_LCD_LayerDefaultInit(1, LCD_FB_START_ADDRESS);

   // Set LCD Foreground Layer
   BSP_LCD_SelectLayer(1);

   // Set LCD Default Font
   BSP_LCD_SetFont(&LCD_DEFAULT_FONT);

   // Initialize LCD Log module
   LCD_LOG_Init();

   LCD_LOG_SetHeader((uint8_t *)"HackerspaceSG Lighting Controller");
   LCD_LOG_SetFooter((uint8_t *)"STM32746G-DISCO board");

   LCD_UsrLog ((char *)"  State: Ethernet Initialization ...\n");

   // Initialize USB host and lwIP stack
   MX_USB_HOST_Init();
   MX_LWIP_Init();


   osThreadTerminate(NULL);
}

// Thread to feed the watchdog
void WatchdogThread(void *argument)
{
   while (1)
   {
	   HAL_IWDG_Refresh(&hiwdg);
	   osDelay(300);
   }
}

// Timer Callback for Timer 6
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
 if (htim->Instance == TIM6) {
   HAL_IncTick();
 }
}

// Error Handler in case of failure
void Error_Handler(void)
{
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{

}
#endif


