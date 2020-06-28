/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define red_led_Pin GPIO_PIN_13
#define red_led_GPIO_Port GPIOC
#define green_led_Pin GPIO_PIN_14
#define green_led_GPIO_Port GPIOC
#define yellow_led_Pin GPIO_PIN_15
#define yellow_led_GPIO_Port GPIOC
#define aux_out_Pin GPIO_PIN_1
#define aux_out_GPIO_Port GPIOA
#define az_encoder_cs_Pin GPIO_PIN_0
#define az_encoder_cs_GPIO_Port GPIOB
#define el_encoder_cs_Pin GPIO_PIN_1
#define el_encoder_cs_GPIO_Port GPIOB
#define UNUSED_Pin GPIO_PIN_2
#define UNUSED_GPIO_Port GPIOB
#define SPI2_SS1_Pin GPIO_PIN_10
#define SPI2_SS1_GPIO_Port GPIOB
#define SPI2_SS2_Pin GPIO_PIN_11
#define SPI2_SS2_GPIO_Port GPIOB
#define az_enable_Pin GPIO_PIN_12
#define az_enable_GPIO_Port GPIOB
#define SPI2_SCK_Pin GPIO_PIN_13
#define SPI2_SCK_GPIO_Port GPIOB
#define SPI2_MISO_Pin GPIO_PIN_14
#define SPI2_MISO_GPIO_Port GPIOB
#define az_step_Pin GPIO_PIN_8
#define az_step_GPIO_Port GPIOA
#define az_dir_Pin GPIO_PIN_11
#define az_dir_GPIO_Port GPIOA
#define RTS_Pin GPIO_PIN_12
#define RTS_GPIO_Port GPIOA
#define el_enable_Pin GPIO_PIN_5
#define el_enable_GPIO_Port GPIOB
#define el_dir_Pin GPIO_PIN_6
#define el_dir_GPIO_Port GPIOB
#define el_step_Pin GPIO_PIN_7
#define el_step_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
