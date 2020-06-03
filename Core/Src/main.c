/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "comp.h"
#include "dma.h"
#include "iwdg.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEBUG_IWDG
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint8_t INV_STATE=0; // 0-off

volatile uint8_t Blocked_by_AB=1;
volatile uint8_t Blocked_by_Perek=1;
volatile uint8_t Blocked_by_TEMP=1;
volatile uint8_t Blocked_by_PVD=1;

volatile uint8_t Blocked_by_Klapan=0;
volatile uint8_t Blocked_by_Klapan_CNT=0;
volatile uint8_t Blocked_by_150=0;

volatile uint8_t KLAPAN_SIGN=0;

volatile uint16_t aADCxConvertedData[ADC_CONVERTED_DATA_BUFFER_SIZE];
volatile uint16_t Global_AB=0;
volatile uint16_t Global_TEMP=0;
volatile uint16_t Global_SLOWCURR=0;
volatile uint16_t Global_CURR=0;

// ADC Stamp
volatile uint32_t adc_base_stamp=0;
volatile uint32_t adc_avg_stamp=0;

// AB AVERAGE ADC
volatile uint32_t Global_AB_BASE=0;
volatile uint32_t Global_AB_BASE_CNT=0;
volatile uint64_t Global_AB_BASE_SUMMATOR=0;

volatile uint32_t Global_TEMP_AVG=0;
volatile uint32_t Global_TEMP_AVG_CNT=0;
volatile uint64_t Global_TEMP_AVG_SUMMATOR=0;

volatile uint32_t Global_SLOWCUR_AVG=0;
volatile uint32_t Global_SLOWCUR_AVG_CNT=0;
volatile uint64_t Global_SLOWCUR_AVG_SUMMATOR=0;

volatile uint32_t Global_CUR_AVG=0;
volatile uint32_t Global_CUR_AVG_CNT=0;
volatile uint64_t Global_CUR_AVG_SUMMATOR=0;

volatile uint32_t Global_CUR_BASE=0;
volatile uint32_t Global_CUR_BASE_CNT=0;
volatile uint64_t Global_CUR_BASE_SUMMATOR=0;

volatile uint8_t Ready_AVG_CUR=0;
volatile uint8_t Ready_AVG_BASE=0;
volatile uint8_t Ready_AVG_TEMP=0;

volatile uint16_t Power_Percent=0;
volatile uint16_t Power_Percent_Base=0;

volatile float TEMP_C=TEMP_MAX;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
void Enable_SH_DEBUG(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Calc_Temp(void) {
	if (Ready_AVG_TEMP==1) {
		Ready_AVG_TEMP=0;

	    // convert the value to resistance
		double average=Global_TEMP_AVG;
	    average = 4095/average-1;
	    average = SERIESRESISTOR * average;

	    TEMP_C = average / THERMISTORNOMINAL;     // (R/Ro)
	    TEMP_C = log(TEMP_C);                  // ln(R/Ro)
	    TEMP_C /= BCOEFFICIENT;                   // 1/B * ln(R/Ro)
	    TEMP_C += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
	    TEMP_C = 1.0 / TEMP_C;                 // Invert
	    TEMP_C -= 273.15;                         // convert to C
	    TEMP_C=TEMP_C+TEMP_BASE;
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	//if(GPIO_Pin== SD_Pin) {
		if (HAL_GPIO_ReadPin(SD_GPIO_Port,SD_Pin)==GPIO_PIN_RESET) {
			Blocked_by_Perek=1;
		} else {
			Blocked_by_Perek=0;
		}
	//}
}

void HAL_PWR_PVDCallback(void)
{
	if __HAL_PWR_GET_FLAG(PWR_FLAG_PVDO) {
		Blocked_by_PVD=1;
	} else {
		Blocked_by_PVD=0;
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc1)
{
	Global_CURR=aADCxConvertedData[0];
	Global_AB=aADCxConvertedData[1];
	Global_TEMP=aADCxConvertedData[2];
	Global_SLOWCURR=aADCxConvertedData[3];

	if (Global_CURR>=ADC_KLAPAN_1_8_x) {
		Blocked_by_150=1;
	}

	Global_CUR_BASE_SUMMATOR=Global_CURR+Global_CUR_BASE_SUMMATOR;
	Global_CUR_BASE_CNT++;
	Global_CUR_AVG_SUMMATOR=Global_CUR_AVG_SUMMATOR+Global_CURR;
	Global_CUR_AVG_CNT++;

	Global_AB_BASE_CNT++;
	Global_AB_BASE_SUMMATOR=Global_AB+Global_AB_BASE_SUMMATOR;

	Global_TEMP_AVG_CNT++;
	Global_TEMP_AVG_SUMMATOR=Global_TEMP_AVG_SUMMATOR+Global_TEMP;

	Global_SLOWCUR_AVG_CNT++;
	Global_SLOWCUR_AVG_SUMMATOR=Global_SLOWCUR_AVG_SUMMATOR+Global_SLOWCURR;

	if (xTaskGetTickCount()-adc_base_stamp>=ADC_LENGTH_SAMPLING_BASE) {
		if (Global_AB_BASE_CNT==0) {
			Global_AB_BASE_CNT=1;
		}

		if (Global_CUR_BASE_CNT==0) {
			Global_CUR_BASE_CNT=1;
		}

		Global_CUR_BASE=(uint32_t) (Global_CUR_BASE_SUMMATOR/Global_CUR_BASE_CNT);
		Global_CUR_BASE_CNT=0;
		Global_CUR_BASE_SUMMATOR=0;

		Global_AB_BASE=(uint32_t) (Global_AB_BASE_SUMMATOR/Global_AB_BASE_CNT);
		Global_AB_BASE_CNT=0;
		Global_AB_BASE_SUMMATOR=0;

		if (Global_SLOWCUR_AVG!=0) {
			Power_Percent_Base=(100*Global_CUR_BASE)/Global_SLOWCUR_AVG;
		} else {
			Power_Percent_Base=1;
		}

		Ready_AVG_BASE=1;

		adc_base_stamp=xTaskGetTickCount();
	}

	if (xTaskGetTickCount()-adc_avg_stamp>=ADC_LENGTH_SAMPLING_AVG) {
			if (Global_TEMP_AVG_CNT==0) {
				Global_TEMP_AVG_CNT=1;
			}

			if (Global_CUR_AVG_CNT==0) {
				Global_CUR_AVG_CNT=1;
			}

			if (Global_SLOWCUR_AVG_CNT==0) {
				Global_SLOWCUR_AVG_CNT=1;
			}

			Global_CUR_AVG=(uint32_t) (Global_CUR_AVG_SUMMATOR/Global_CUR_AVG_CNT);
			Global_CUR_AVG_CNT=0;
			Global_CUR_AVG_SUMMATOR=0;

			Global_TEMP_AVG=(uint32_t) (Global_TEMP_AVG_SUMMATOR/Global_TEMP_AVG_CNT);
			Global_TEMP_AVG_CNT=0;
			Global_TEMP_AVG_SUMMATOR=0;

			Global_SLOWCUR_AVG=(uint32_t) (Global_SLOWCUR_AVG_SUMMATOR/Global_SLOWCUR_AVG_CNT);
			Global_SLOWCUR_AVG_CNT=0;
			Global_SLOWCUR_AVG_SUMMATOR=0;

			adc_avg_stamp=xTaskGetTickCount();

			Ready_AVG_CUR=1;
			Ready_AVG_TEMP=1;

			if (Global_CUR_AVG!=0) {
				Power_Percent=(100*Global_CUR_AVG)/Global_SLOWCUR_AVG;
			} else {
				Power_Percent=1;
			}
		}
}

/*void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc1)
{
	Global_CURR=aADCxConvertedData[0];
	Global_AB=aADCxConvertedData[1];
}*/

uint8_t CheckStamp(uint32_t time_stamp, uint32_t time_base) {
	if (xTaskGetTickCount()-time_stamp>time_base*1000) {
		return 1;
	} else {
		return 0;
	}
}

void LED_Blink(GPIO_TypeDef* sh_port, uint16_t sh_pin,uint16_t sh_delay) {
	HAL_GPIO_WritePin(sh_port, sh_pin, GPIO_PIN_SET); // STAT
	osDelay(sh_delay);
	HAL_GPIO_WritePin(sh_port, sh_pin, GPIO_PIN_RESET); // STAT
}

void LED_Blink_X(GPIO_TypeDef* sh_port, uint16_t sh_pin,uint8_t sh_cnt,uint16_t sh_delay) {
	for (uint16_t i=0;i<sh_cnt;i++) {
		LED_Blink(sh_port,sh_pin,sh_delay);
		osDelay(100);
	}
}

void __attribute__((optimize("O0"))) ShutDown_with_Power_On(void) {
	/*HAL_GPIO_WritePin(GPIOA, BLOCK_PORT_1_Pin, GPIO_PIN_RESET); // SET BLOCK 8V FLAG and block all PWMs
	HAL_GPIO_WritePin(GPIOA, BLOCK_PORT_2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(BLOCK_POWER_GPIO_Port, BLOCK_POWER_Pin, GPIO_PIN_SET);   // POWER ON
	*/

	GPIOB->BRR  = GPIO_BRR_BR_10;
	GPIOB->BSRR  = GPIO_BSRR_BS_2;
	GPIOB->BSRR  = GPIO_BSRR_BS_0;
}

void __attribute__((optimize("O0"))) Enable_INV(void) {

	GPIOB->BRR  = GPIO_BRR_BR_10;
	GPIOB->BSRR  = GPIO_BSRR_BS_2;
	GPIOB->BSRR  = GPIO_BSRR_BS_0;
	GPIOB->BSRR  = GPIO_BSRR_BS_10;
	GPIOB->BRR  = GPIO_BRR_BR_2;


	/*HAL_GPIO_WritePin(GPIOA, BLOCK_PORT_1_Pin, GPIO_PIN_RESET); // SET BLOCK 8V FLAG and block all PWMs
	HAL_GPIO_WritePin(GPIOA, BLOCK_PORT_2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(BLOCK_POWER_GPIO_Port, BLOCK_POWER_Pin, GPIO_PIN_SET); // POWER ON
	HAL_GPIO_WritePin(GPIOA, BLOCK_PORT_1_Pin, GPIO_PIN_SET); //  UNBLOCK 8V FLAG and UNBLOCK all PWMs
	HAL_GPIO_WritePin(GPIOA, BLOCK_PORT_2_Pin, GPIO_PIN_RESET);*/
}

void __attribute__((optimize("O0"))) ShutDown_with_Power_Off(void) {
	GPIOB->BRR  = GPIO_BRR_BR_10; // Set 8V Block
	GPIOB->BSRR  = GPIO_BSRR_BS_2; // Block PWM
	GPIOB->BRR  = GPIO_BRR_BR_0; // BLOCK POWER

	/*HAL_GPIO_WritePin(GPIOA, BLOCK_PORT_1_Pin, GPIO_PIN_RESET); // SET BLOCK 8V FLAG and block all PWMs
	HAL_GPIO_WritePin(GPIOA, BLOCK_PORT_2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(BLOCK_POWER_GPIO_Port, BLOCK_POWER_Pin, GPIO_PIN_RESET);   // POWER OFF
	*/
}

void IWDG_Reset(void) {
	HAL_IWDG_Refresh(&hiwdg);
}

void Enable_SH_DEBUG(void) {
			__HAL_RCC_DBGMCU_CLK_ENABLE();
		    DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_WWDG_STOP;
		    DBGMCU->APB2FZ = 0xFFFFFFFF;
		    DBGMCU->APB1FZ = 0xFFFFFFFF;
		    DBGMCU->CR |=DBGMCU_CR_DBG_STOP;
		    __HAL_DBGMCU_FREEZE_IWDG();
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
	  /* USER CODE BEGIN 1 */

		// Enable_SH_DEBUG
		#ifdef DEBUG_IWDG
			Enable_SH_DEBUG();
		#endif
	  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC_Init();
  MX_COMP2_Init();
  MX_IWDG_Init();
  MX_TIM2_Init();
  MX_TIM21_Init();
  MX_TIM22_Init();
  /* USER CODE BEGIN 2 */
  IWDG_Reset();
  /* USER CODE END 2 */

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init(); 
  /* Start scheduler */
  osKernelStart();
 
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
