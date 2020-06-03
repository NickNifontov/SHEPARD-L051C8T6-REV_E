/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32l0xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32l0xx_it.h"
#include "FreeRTOS.h"
#include "task.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc;
extern ADC_HandleTypeDef hadc;
extern COMP_HandleTypeDef hcomp2;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M0+ Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
#if (INCLUDE_xTaskGetSchedulerState == 1 )
  if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
  {
#endif /* INCLUDE_xTaskGetSchedulerState */
  xPortSysTickHandler();
#if (INCLUDE_xTaskGetSchedulerState == 1 )
  }
#endif /* INCLUDE_xTaskGetSchedulerState */
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32L0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32l0xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles PVD interrupt through EXTI line 16.
  */
void PVD_IRQHandler(void)
{
  /* USER CODE BEGIN PVD_IRQn 0 */

  /* USER CODE END PVD_IRQn 0 */
  HAL_PWR_PVD_IRQHandler();
  /* USER CODE BEGIN PVD_IRQn 1 */

  /* USER CODE END PVD_IRQn 1 */
}

/**
  * @brief This function handles EXTI line 0 and line 1 interrupts.
  */
void EXTI0_1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_1_IRQn 0 */

  /* USER CODE END EXTI0_1_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(SD_Pin);
  /* USER CODE BEGIN EXTI0_1_IRQn 1 */

  /* USER CODE END EXTI0_1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel 1 interrupt.
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */

  /* USER CODE END DMA1_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc);
  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
  * @brief This function handles ADC, COMP1 and COMP2 interrupts (COMP interrupts through EXTI lines 21 and 22).
  */
void ADC1_COMP_IRQHandler(void)
{
  /* USER CODE BEGIN ADC1_COMP_IRQn 0 */

  /* USER CODE END ADC1_COMP_IRQn 0 */
  //HAL_COMP_IRQHandler(&hcomp2);
  //HAL_ADC_IRQHandler(&hadc);


	  /*uint32_t exti_line = COMP_GET_EXTI_LINE(hcomp->Instance);

	  if(READ_BIT(EXTI->PR, exti_line) != RESET)
	  {
	    if(READ_BIT(COMP12_COMMON->CSR, COMP_CSR_WINMODE) != 0)
	    {
	      WRITE_REG(EXTI->PR, (COMP_EXTI_LINE_COMP1 | COMP_EXTI_LINE_COMP2));
	    }
	    else
	    {
	      WRITE_REG(EXTI->PR, exti_line);
	    }*/

	 // uint32_t exti_line = COMP_GET_EXTI_LINE(hcomp->Instance);
	  if (READ_BIT(EXTI->PR, COMP_EXTI_LINE_COMP2) != RESET) {
		  GPIOB->BRR  = GPIO_BRR_BR_10;
		  GPIOB->BSRR  = GPIO_BSRR_BS_2;
		  GPIOB->BRR  = GPIO_BRR_BR_0;

		  //turn off LED
		  GPIOA->BRR  = GPIO_BRR_BR_7;

		  //turn off buzzer
		  GPIOB->BRR  = GPIO_BRR_BR_13;

		  //turn LED
		  GPIOA->BSRR  = GPIO_BSRR_BS_7;

		  //turn buzzer
		  GPIOB->BSRR  = GPIO_BSRR_BS_13;

		if (Blocked_by_Klapan_CNT>=KLAPAN_CNT) {
			GPIOB->BRR  = GPIO_BRR_BR_10;
		    GPIOB->BSRR  = GPIO_BSRR_BS_2;
			GPIOB->BRR  = GPIO_BRR_BR_0;
			Blocked_by_Klapan_CNT=KLAPAN_CNT+1;
			Blocked_by_Klapan=1;
		} else {
			Blocked_by_Klapan_CNT++;

			//GPIOA->BRR  = GPIO_BRR_BR_6;
			//GPIOA->BSRR  = GPIO_BSRR_BS_7;
			//GPIOB->BRR  = GPIO_BRR_BR_1;

			//uint16_t microsec_pause=Blocked_by_Klapan_CNT*500;

			//40msec
			//volatile uint16_t sec_delay=500;

			//10msec
			volatile uint16_t sec_delay=100+(Blocked_by_Klapan_CNT-1)*1000;
			//volatile uint16_t sec_delay=250;

			for (uint16_t i=0; i<sec_delay; ++i) {
						// 1 microsec
						for (int j = 0; j < 32; ++j) {
							__asm__ __volatile__("nop\n\t":::"memory");
						}
			}

			GPIOB->BSRR  = GPIO_BSRR_BS_10;
			GPIOB->BRR  = GPIO_BRR_BR_2;

			GPIOB->BSRR  = GPIO_BSRR_BS_0;
		}

		/*GPIOA->BRR  = GPIO_BRR_BR_6;
	    GPIOA->BSRR  = GPIO_BSRR_BS_7;
	    GPIOB->BRR  = GPIO_BRR_BR_1;
	    Blocked_by_Klapan=1;*/

	    //HAL_COMP_IRQHandler(&hcomp2);
	    WRITE_REG(EXTI->PR, COMP_EXTI_LINE_COMP2);
	  }

	  //else {
	  //	  HAL_ADC_IRQHandler(&hadc);
	  //}

	  //if (Blocked_by_Klapan_CNT>0) {
	  //	  	  HAL_ADC_IRQHandler(&hadc);
	  //}

	  /*if(LL_ADC_IsActiveFlag_AWD1(ADC1) != 0) {
	      GPIOA->BRR  = GPIO_BRR_BR_6;
	      GPIOA->BSRR  = GPIO_BSRR_BS_7;
	      GPIOB->BRR  = GPIO_BRR_BR_1;
	      Blocked_by_150=1;
	  }*/



  /* USER CODE BEGIN ADC1_COMP_IRQn 1 */

  /* USER CODE END ADC1_COMP_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
