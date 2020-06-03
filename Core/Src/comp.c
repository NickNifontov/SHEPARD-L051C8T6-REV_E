/**
  ******************************************************************************
  * File Name          : COMP.c
  * Description        : This file provides code for the configuration
  *                      of the COMP instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "comp.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

COMP_HandleTypeDef hcomp2;

/* COMP2 init function */
void MX_COMP2_Init(void)
{

  hcomp2.Instance = COMP2;
  hcomp2.Init.InvertingInput = COMP_INPUT_MINUS_VREFINT;
  hcomp2.Init.NonInvertingInput = COMP_INPUT_PLUS_IO1;
  hcomp2.Init.LPTIMConnection = COMP_LPTIMCONNECTION_DISABLED;
  hcomp2.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  hcomp2.Init.Mode = COMP_POWERMODE_MEDIUMSPEED;
  hcomp2.Init.WindowMode = COMP_WINDOWMODE_DISABLE;
  hcomp2.Init.TriggerMode = COMP_TRIGGERMODE_IT_RISING_FALLING;
  if (HAL_COMP_Init(&hcomp2) != HAL_OK)
  {
    Error_Handler();
  }

}

void HAL_COMP_MspInit(COMP_HandleTypeDef* compHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(compHandle->Instance==COMP2)
  {
  /* USER CODE BEGIN COMP2_MspInit 0 */

  /* USER CODE END COMP2_MspInit 0 */
  
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**COMP2 GPIO Configuration    
    PA3     ------> COMP2_INP 
    */
    GPIO_InitStruct.Pin = KLAPAN_COMP_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(KLAPAN_COMP_GPIO_Port, &GPIO_InitStruct);

    /* COMP2 interrupt Init */
    HAL_NVIC_SetPriority(ADC1_COMP_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(ADC1_COMP_IRQn);
  /* USER CODE BEGIN COMP2_MspInit 1 */

  /* USER CODE END COMP2_MspInit 1 */
  }
}

void HAL_COMP_MspDeInit(COMP_HandleTypeDef* compHandle)
{

  if(compHandle->Instance==COMP2)
  {
  /* USER CODE BEGIN COMP2_MspDeInit 0 */

  /* USER CODE END COMP2_MspDeInit 0 */
  
    /**COMP2 GPIO Configuration    
    PA3     ------> COMP2_INP 
    */
    HAL_GPIO_DeInit(KLAPAN_COMP_GPIO_Port, KLAPAN_COMP_Pin);

    /* COMP2 interrupt Deinit */
  /* USER CODE BEGIN COMP2:ADC1_COMP_IRQn disable */
    /**
    * Uncomment the line below to disable the "ADC1_COMP_IRQn" interrupt
    * Be aware, disabling shared interrupt may affect other IPs
    */
    /* HAL_NVIC_DisableIRQ(ADC1_COMP_IRQn); */
  /* USER CODE END COMP2:ADC1_COMP_IRQn disable */

  /* USER CODE BEGIN COMP2_MspDeInit 1 */

  /* USER CODE END COMP2_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
