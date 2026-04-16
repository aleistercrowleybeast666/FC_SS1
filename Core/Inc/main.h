/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32f4xx_hal.h"

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
#define RADIO_DIO1_Pin GPIO_PIN_2
#define RADIO_DIO1_GPIO_Port GPIOE
#define RADIO_DIO1_EXTI_IRQn EXTI2_IRQn
#define RADIO_DIO2_Pin GPIO_PIN_3
#define RADIO_DIO2_GPIO_Port GPIOE
#define RADIO_DIO3_Pin GPIO_PIN_4
#define RADIO_DIO3_GPIO_Port GPIOE
#define RADIO_BUSY_Pin GPIO_PIN_1
#define RADIO_BUSY_GPIO_Port GPIOC
#define RADIO_RST_Pin GPIO_PIN_2
#define RADIO_RST_GPIO_Port GPIOC
#define SPI_IMU_MOSI_Pin GPIO_PIN_3
#define SPI_IMU_MOSI_GPIO_Port GPIOC
#define KEY_Pin GPIO_PIN_0
#define KEY_GPIO_Port GPIOA
#define LED_Pin GPIO_PIN_1
#define LED_GPIO_Port GPIOA
#define UART_GNSS_MT_Pin GPIO_PIN_2
#define UART_GNSS_MT_GPIO_Port GPIOA
#define UART_GNSS_MR_Pin GPIO_PIN_3
#define UART_GNSS_MR_GPIO_Port GPIOA
#define SPI_RADIO_NSS_Pin GPIO_PIN_4
#define SPI_RADIO_NSS_GPIO_Port GPIOA
#define SPI_RADIO_SCK_Pin GPIO_PIN_5
#define SPI_RADIO_SCK_GPIO_Port GPIOA
#define SPI_RADIO_MISO_Pin GPIO_PIN_6
#define SPI_RADIO_MISO_GPIO_Port GPIOA
#define SPI_RADIO_MOSI_Pin GPIO_PIN_7
#define SPI_RADIO_MOSI_GPIO_Port GPIOA
#define IMU1_INT1_Pin GPIO_PIN_7
#define IMU1_INT1_GPIO_Port GPIOE
#define IMU1_INT1_EXTI_IRQn EXTI9_5_IRQn
#define IMU1_INT2_Pin GPIO_PIN_8
#define IMU1_INT2_GPIO_Port GPIOE
#define IMU1_INT2_EXTI_IRQn EXTI9_5_IRQn
#define IMU1_INT3_Pin GPIO_PIN_9
#define IMU1_INT3_GPIO_Port GPIOE
#define IMU1_INT3_EXTI_IRQn EXTI9_5_IRQn
#define IMU1_INT4_Pin GPIO_PIN_10
#define IMU1_INT4_GPIO_Port GPIOE
#define IMU1_INT4_EXTI_IRQn EXTI15_10_IRQn
#define IMU2_INT1_Pin GPIO_PIN_11
#define IMU2_INT1_GPIO_Port GPIOE
#define IMU2_INT1_EXTI_IRQn EXTI15_10_IRQn
#define IMU2_INT2_Pin GPIO_PIN_12
#define IMU2_INT2_GPIO_Port GPIOE
#define IMU2_INT2_EXTI_IRQn EXTI15_10_IRQn
#define SPI_IMU_NSS1A_Pin GPIO_PIN_13
#define SPI_IMU_NSS1A_GPIO_Port GPIOE
#define SPI_IMU_NSS1G_Pin GPIO_PIN_14
#define SPI_IMU_NSS1G_GPIO_Port GPIOE
#define SPI_IMU_NSS2_Pin GPIO_PIN_15
#define SPI_IMU_NSS2_GPIO_Port GPIOE
#define I2C_SENSOR_SCL_Pin GPIO_PIN_10
#define I2C_SENSOR_SCL_GPIO_Port GPIOB
#define I2C_SENSOR_SDA_Pin GPIO_PIN_11
#define I2C_SENSOR_SDA_GPIO_Port GPIOB
#define SPI_IMU_SCK_Pin GPIO_PIN_13
#define SPI_IMU_SCK_GPIO_Port GPIOB
#define SPI_IMU_MISO_Pin GPIO_PIN_14
#define SPI_IMU_MISO_GPIO_Port GPIOB
#define P_CONTROL1_Pin GPIO_PIN_8
#define P_CONTROL1_GPIO_Port GPIOD
#define P_CONTROL2_Pin GPIO_PIN_9
#define P_CONTROL2_GPIO_Port GPIOD
#define S_CONTROL1_Pin GPIO_PIN_10
#define S_CONTROL1_GPIO_Port GPIOD
#define S_CONTROL2_Pin GPIO_PIN_11
#define S_CONTROL2_GPIO_Port GPIOD
#define S_CONTROL3_Pin GPIO_PIN_12
#define S_CONTROL3_GPIO_Port GPIOD
#define S_CONTROL4_Pin GPIO_PIN_13
#define S_CONTROL4_GPIO_Port GPIOD
#define S_CONTROL5_Pin GPIO_PIN_14
#define S_CONTROL5_GPIO_Port GPIOD
#define PRESSURE_INT_Pin GPIO_PIN_6
#define PRESSURE_INT_GPIO_Port GPIOC
#define PRESSURE_INT_EXTI_IRQn EXTI9_5_IRQn
#define UART_EXT_MT_Pin GPIO_PIN_9
#define UART_EXT_MT_GPIO_Port GPIOA
#define UART_EXT_MR_Pin GPIO_PIN_10
#define UART_EXT_MR_GPIO_Port GPIOA
#define GNSS_EXTINT_Pin GPIO_PIN_5
#define GNSS_EXTINT_GPIO_Port GPIOB
#define GNSS_EXTINT_EXTI_IRQn EXTI9_5_IRQn
#define GNSS_TIMEPAUSE_Pin GPIO_PIN_7
#define GNSS_TIMEPAUSE_GPIO_Port GPIOB
#define GNSS_RST_Pin GPIO_PIN_9
#define GNSS_RST_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
