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
#include "stm32l0xx_hal.h"

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
void IWDG_Reset(void);

void __attribute__((optimize("O0"))) ShutDown_with_Power_Off(void);
void __attribute__((optimize("O0"))) Enable_INV(void);
void __attribute__((optimize("O0"))) ShutDown_with_Power_On(void);

void LED_Blink(GPIO_TypeDef* sh_port, uint16_t sh_pin,uint16_t sh_delay);
void LED_Blink_X(GPIO_TypeDef* sh_port, uint16_t sh_pin,uint8_t sh_cnt,uint16_t sh_delay);

uint8_t CheckStamp(uint32_t time_stamp, uint32_t time_base);

void Calc_Temp(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CURRENT_Pin GPIO_PIN_0
#define CURRENT_GPIO_Port GPIOA
#define AB_Pin GPIO_PIN_1
#define AB_GPIO_Port GPIOA
#define TEMP_ADC_Pin GPIO_PIN_2
#define TEMP_ADC_GPIO_Port GPIOA
#define KLAPAN_COMP_Pin GPIO_PIN_3
#define KLAPAN_COMP_GPIO_Port GPIOA
#define SLOW_CURRENT_Pin GPIO_PIN_4
#define SLOW_CURRENT_GPIO_Port GPIOA
#define COOLER_Pin GPIO_PIN_5
#define COOLER_GPIO_Port GPIOA
#define LED_Pin GPIO_PIN_6
#define LED_GPIO_Port GPIOA
#define LED_2_Pin GPIO_PIN_7
#define LED_2_GPIO_Port GPIOA
#define BLOCK_POWER_Pin GPIO_PIN_0
#define BLOCK_POWER_GPIO_Port GPIOB
#define SD_Pin GPIO_PIN_1
#define SD_GPIO_Port GPIOB
#define SD_EXTI_IRQn EXTI0_1_IRQn
#define BLOCK_PORT_2_Pin GPIO_PIN_2
#define BLOCK_PORT_2_GPIO_Port GPIOB
#define BLOCK_PORT_Pin GPIO_PIN_10
#define BLOCK_PORT_GPIO_Port GPIOB
#define BUZZER_Pin GPIO_PIN_13
#define BUZZER_GPIO_Port GPIOB
#define DIO_Pin GPIO_PIN_13
#define DIO_GPIO_Port GPIOA
#define CLK_Pin GPIO_PIN_14
#define CLK_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */
#define  COOLER_PRESCALER	(uint32_t)(0)  /* PRESCALER Value  */
#define  COOLER_PERIOD_VALUE       (uint32_t)(31999)  /* Period Value  */
#define  PULSE100_VALUE       (uint32_t)(COOLER_PERIOD_VALUE)        /* 100% */
#define  PULSE70_VALUE       (uint32_t)(COOLER_PERIOD_VALUE*70/100)        /* 70% */
#define  PULSE40_VALUE       (uint32_t)(COOLER_PERIOD_VALUE*30/100) /* 40%  */
#define  PULSE20_VALUE       (uint32_t)(COOLER_PERIOD_VALUE*20/100)        /* 20% */

#define TEMP_MAX 75 // celsium
#define TEMP_COLRSTART 50
#define TEMP_ROLLBACK 45

// !!! WORK VALUE
#define RESTART_MAX_LENGTH 7200 //sec
#define RESTART_MAX_LENGTH_LONG 7200 //sec

/// !!!! TEST VALUE
//#define RESTART_MAX_LENGTH 5 //sec
//#define RESTART_MAX_LENGTH_LONG 20 //sec

#define BUZZER_MAX_LENGTH 240 // 4 min
#define COOLER_MAX_LENGTH 30 //sec
#define AB_MAX_LENGTH 120 //2 min
#define TEMP_DELAY_LENGTH 120 // 5 min
#define TEMP_ROLLBACK_DELAY_LENGTH 10 //sec


#define AB_LOW ((uint16_t) 460)
#define AB_MAX ((uint16_t) 645)
#define AB_COLDRUN_FROM_MAX ((uint16_t) 475)

#define LIFEPO4

#ifndef LIFEPO4
	// AGM/GEL
	#define AB_ROLLBACK ((uint16_t) 550)
	#define AB_COLDRUN ((uint16_t) 525)
	#define BUZZER_OPORA 500 // 1.0V - 0.2V
	#define BUZZER_OPORA_MIN ((uint16_t) 497)
	#define BUZZER_OPORA_MAX ((uint16_t) 503)
#endif

#ifdef LIFEPO4
	// LiFe  Challenger
	#define AB_ROLLBACK ((uint16_t) 515)
	#define AB_COLDRUN ((uint16_t) 475)
	#define BUZZER_OPORA 470 // 1.0V - 0.2V
	#define BUZZER_OPORA_MIN ((uint16_t) 467)
	#define BUZZER_OPORA_MAX ((uint16_t) 473)
#endif


#define ADC_CONVERTED_DATA_BUFFER_SIZE   ((uint32_t)  4)   /* Size of array aADCxConvertedData[] */
#define ADC_LENGTH_SAMPLING_BASE	((uint8_t)  10) //10msec
#define ADC_LENGTH_SAMPLING_AVG  	((uint16_t)  1000) //1sec

extern volatile uint32_t adc_base_stamp;
extern volatile uint32_t adc_avg_stamp;

extern volatile uint32_t Global_AB_BASE;
extern volatile uint32_t Global_AB_BASE_CNT;
extern volatile uint64_t Global_AB_BASE_SUMMATOR;

extern volatile uint32_t Global_TEMP_AVG;
extern volatile uint32_t Global_TEMP_AVG_CNT;
extern volatile uint64_t Global_TEMP_AVG_SUMMATOR;

extern volatile uint32_t Global_SLOWCUR_AVG;
extern volatile uint32_t Global_SLOWCUR_AVG_CNT;
extern volatile uint64_t Global_SLOWCUR_AVG_SUMMATOR;

extern volatile uint32_t Global_CUR_AVG;
extern volatile uint32_t Global_CUR_AVG_CNT;
extern volatile uint64_t Global_CUR_AVG_SUMMATOR;

extern volatile uint32_t Global_CUR_BASE;
extern volatile uint32_t Global_CUR_BASE_CNT;
extern volatile uint64_t Global_CUR_BASE_SUMMATOR;

extern volatile uint8_t Ready_AVG_CUR;
extern volatile uint8_t Ready_AVG_BASE;
extern volatile uint8_t Ready_AVG_TEMP;
extern volatile uint16_t Power_Percent;
extern volatile uint16_t Power_Percent_Base;


// resistance at 25 degrees C
#define THERMISTORNOMINAL 10000
// temp. for nominal resistance (almost always 25 C)
#define TEMPERATURENOMINAL 25
// The beta coefficient of the thermistor (usually 3000-4000)
#define BCOEFFICIENT 3950
// the value of the 'other' resistor
#define SERIESRESISTOR 10000

#define TEMP_BASE 3

extern volatile float TEMP_C;

#define ADC_KLAPAN_1_8_x 1560 // 1.8x

extern volatile uint16_t   aADCxConvertedData[ADC_CONVERTED_DATA_BUFFER_SIZE]; /* Variable containing ADC conversions data */
extern volatile uint16_t Global_AB;
extern volatile uint16_t Global_TEMP;
extern volatile uint16_t Global_SLOWCURR;
extern volatile uint16_t Global_CURR;

#define POLKA_100 ((uint16_t)  (100))
#define POLKA_130 ((uint16_t)  (130))
#define POLKA_95 ((uint16_t)  (95))

extern volatile uint8_t INV_STATE;

extern volatile uint8_t Blocked_by_AB;
extern volatile uint8_t Blocked_by_Perek;
extern volatile uint8_t Blocked_by_PVD;
extern volatile uint8_t Blocked_by_TEMP;

#define KLAPAN_CNT 5
extern volatile uint8_t Blocked_by_Klapan;
extern volatile uint8_t Blocked_by_150;
extern volatile uint8_t Blocked_by_Klapan_CNT;

extern volatile uint8_t KLAPAN_SIGN;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
