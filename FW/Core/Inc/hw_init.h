/***********************************************************************************************************************
*                                                                                                                      *
* HackerspaceSG WS2812 Lighting Controller                                                                             *
*                                                                                                                      *
* Description: Files handling hardware peripheral initialization                                                       *
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

#ifndef HW_INIT_H
#define HW_INIT_H

#include "main.h" // Pin Definitions contained here

ADC_HandleTypeDef hadc3;
CRC_HandleTypeDef hcrc;
DCMI_HandleTypeDef hdcmi;
DMA_HandleTypeDef hdma_tim1_ch1;
DMA2D_HandleTypeDef hdma2d;
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;
IWDG_HandleTypeDef hiwdg;
LTDC_HandleTypeDef hltdc;
QSPI_HandleTypeDef hqspi;
RTC_HandleTypeDef hrtc;
SAI_HandleTypeDef hsai_BlockA2;
SAI_HandleTypeDef hsai_BlockB2;
SD_HandleTypeDef hsd1;
SPDIFRX_HandleTypeDef hspdif;
SPI_HandleTypeDef hspi2;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim12;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart6;
SDRAM_HandleTypeDef hsdram1;

// Function Prototypes
void hw_init();
void SystemClock_Config(void);
void GPIO_Init(void);
void ADC3_Init(void);
void CRC_Init(void);
void DCMI_Init(void);
void DMA_Init(void);
void DMA2D_Init(void);
void FMC_Init(void);
void I2C1_Init(void);
void I2C3_Init(void);
void LTDC_Init(void);
void QUADSPI_Init(void);
void RTC_Init(void);
void SAI2_Init(void);
void SDMMC1_SD_Init(void);
void SPDIFRX_Init(void);
void SPI2_Init(void);
void TIM1_Init(void);
void TIM2_Init(void);
void TIM3_Init(void);
void TIM5_Init(void);
void TIM8_Init(void);
void TIM12_Init(void);
void USART1_UART_Init(void);
void USART6_UART_Init(void);
void IWDG_Init(void);

#endif
