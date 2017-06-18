/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define ERRLED_Pin GPIO_PIN_13
#define ERRLED_GPIO_Port GPIOC
#define AD0_Pin GPIO_PIN_0
#define AD0_GPIO_Port GPIOA
#define AD1_Pin GPIO_PIN_4
#define AD1_GPIO_Port GPIOA
#define SPI1SCK_Pin GPIO_PIN_5
#define SPI1SCK_GPIO_Port GPIOA
#define SPI1MISO_Pin GPIO_PIN_6
#define SPI1MISO_GPIO_Port GPIOA
#define SPI1MOSI_Pin GPIO_PIN_0
#define SPI1MOSI_GPIO_Port GPIOB
#define ENC_CS1_Pin GPIO_PIN_1
#define ENC_CS1_GPIO_Port GPIOB
#define ENC_CS2_Pin GPIO_PIN_2
#define ENC_CS2_GPIO_Port GPIOB
#define AD2_Pin GPIO_PIN_8
#define AD2_GPIO_Port GPIOE
#define AD3_Pin GPIO_PIN_9
#define AD3_GPIO_Port GPIOE
#define SPI2MISO_Pin GPIO_PIN_14
#define SPI2MISO_GPIO_Port GPIOB
#define SPI2MOSI_Pin GPIO_PIN_15
#define SPI2MOSI_GPIO_Port GPIOB
#define SPI2SCK_Pin GPIO_PIN_8
#define SPI2SCK_GPIO_Port GPIOD
#define SD_CS_Pin GPIO_PIN_8
#define SD_CS_GPIO_Port GPIOA
#define UART1TX_Pin GPIO_PIN_9
#define UART1TX_GPIO_Port GPIOA
#define UART1RX_Pin GPIO_PIN_10
#define UART1RX_GPIO_Port GPIOA
#define PTSW_Pin GPIO_PIN_11
#define PTSW_GPIO_Port GPIOA
#define MDSW_Pin GPIO_PIN_12
#define MDSW_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define LSRCTR_Pin GPIO_PIN_6
#define LSRCTR_GPIO_Port GPIOF
#define STBY_Pin GPIO_PIN_7
#define STBY_GPIO_Port GPIOF
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define PWMA_Pin GPIO_PIN_15
#define PWMA_GPIO_Port GPIOA
#define PWMB_Pin GPIO_PIN_3
#define PWMB_GPIO_Port GPIOB
#define A1_Pin GPIO_PIN_4
#define A1_GPIO_Port GPIOB
#define A2_Pin GPIO_PIN_5
#define A2_GPIO_Port GPIOB
#define B1_Pin GPIO_PIN_6
#define B1_GPIO_Port GPIOB
#define B2_Pin GPIO_PIN_7
#define B2_GPIO_Port GPIOB
#define I2C1SCL_Pin GPIO_PIN_8
#define I2C1SCL_GPIO_Port GPIOB
#define I2C1SDA_Pin GPIO_PIN_9
#define I2C1SDA_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_dma.h"
#include "stm32f3xx_hal_adc.h"
#include "stm32f3xx_hal_spi.h"
#include "stm32f3xx_hal_i2c.h"
#include "stm32f3xx_hal_uart.h"
#include "stm32f3xx_hal_rtc.h"
#include "stm32f3xx_hal_tim.h"
#include "stm32f3xx_hal_tsc.h"
#include "stm32f3xx_hal_sdadc.h"
//#include "ff.h"

extern ADC_HandleTypeDef hadc1;
extern I2C_HandleTypeDef hi2c1;
extern RTC_HandleTypeDef hrtc;
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern UART_HandleTypeDef huart1;
//UART_HandleTypeDef huart2;
extern TSC_HandleTypeDef htsc;
extern TIM_HandleTypeDef htim19;
extern SDADC_HandleTypeDef hsdadc2;

void Error_Handler(void);

/* USER CODE END Private defines */

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
