/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */     
#include "adc.h"
#include "comp.h"
#include "tim.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId Loop_TaskHandle;
osThreadId LED_TaskHandle;
osThreadId Buzzer_TaskHandle;
osThreadId Cooler_TaskHandle;
osThreadId AB_TaskHandle;
osThreadId TEMP_TaskHandle;
osThreadId CUR_TaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
   
/* USER CODE END FunctionPrototypes */

void StartLoop_Task(void const * argument);
void StartLED_Task(void const * argument);
void StartBuzzer_Task(void const * argument);
void StartCooler_Task(void const * argument);
void StartAB_Task(void const * argument);
void StartTEMP_Task(void const * argument);
void StartCUR_Task(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];
  
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}                   
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of Loop_Task */
  osThreadDef(Loop_Task, StartLoop_Task, osPriorityNormal, 0, 128);
  Loop_TaskHandle = osThreadCreate(osThread(Loop_Task), NULL);

  /* definition and creation of LED_Task */
  osThreadDef(LED_Task, StartLED_Task, osPriorityNormal, 0, 128);
  LED_TaskHandle = osThreadCreate(osThread(LED_Task), NULL);

  /* definition and creation of Buzzer_Task */
  osThreadDef(Buzzer_Task, StartBuzzer_Task, osPriorityNormal, 0, 128);
  Buzzer_TaskHandle = osThreadCreate(osThread(Buzzer_Task), NULL);

  /* definition and creation of Cooler_Task */
  osThreadDef(Cooler_Task, StartCooler_Task, osPriorityNormal, 0, 128);
  Cooler_TaskHandle = osThreadCreate(osThread(Cooler_Task), NULL);

  /* definition and creation of AB_Task */
  osThreadDef(AB_Task, StartAB_Task, osPriorityNormal, 0, 128);
  AB_TaskHandle = osThreadCreate(osThread(AB_Task), NULL);

  /* definition and creation of TEMP_Task */
  osThreadDef(TEMP_Task, StartTEMP_Task, osPriorityNormal, 0, 128);
  TEMP_TaskHandle = osThreadCreate(osThread(TEMP_Task), NULL);

  /* definition and creation of CUR_Task */
  osThreadDef(CUR_Task, StartCUR_Task, osPriorityNormal, 0, 128);
  CUR_TaskHandle = osThreadCreate(osThread(CUR_Task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartLoop_Task */
/**
  * @brief  Function implementing the Loop_Task thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartLoop_Task */
void StartLoop_Task(void const * argument)
{

	 // uint32_t restart_stamp=0;
	  uint8_t restart_flag=0;
	  uint32_t restart_flag_stamp=0;


	  INV_STATE=0;
	  ShutDown_with_Power_Off();


	  /* ###  - Start conversion in DMA mode ################################# */
	  HAL_ADC_Start_DMA(&hadc,(uint32_t *)aADCxConvertedData,ADC_CONVERTED_DATA_BUFFER_SIZE);

	  // Check Perek
	  HAL_GPIO_EXTI_Callback(SD_Pin);

	  //check PVD
	  HAL_PWR_PVDCallback();

	  /* ###  - Start COMP ################################# */
	  HAL_COMP_Start(&hcomp2);

	  osDelay(1000);

	  for(;;)
	  {
		if ((Blocked_by_AB==0) && (Blocked_by_Perek==0) && (Blocked_by_PVD==0) && (Blocked_by_TEMP==0)
				&& (Blocked_by_Klapan==0) && (Blocked_by_150==0) ) {
			if (INV_STATE!=1) {
				INV_STATE=1;
				Enable_INV();
			}
			//Enable_INV();
			KLAPAN_SIGN=0;
			//restart_stamp=0;
			restart_flag=0;
			restart_flag_stamp=0;
		} else {
			if (INV_STATE!=0) {
				INV_STATE=0;
				//restart_stamp=0;
				restart_flag=0;
				restart_flag_stamp=0;
				ShutDown_with_Power_On();
						if ((Blocked_by_Klapan==1) && (Blocked_by_150==1)) {
							//HAL_GPIO_WritePin(BLOCK_POWER_GPIO_Port, BLOCK_POWER_Pin, GPIO_PIN_SET);
							GPIOB->BSRR  = GPIO_BSRR_BS_0; //// UNBLOCK POWER
						}
			}
			if ( (Blocked_by_Klapan==1) && (KLAPAN_SIGN==0)) {
				KLAPAN_SIGN=1;
			    LED_Blink_X(BUZZER_GPIO_Port,BUZZER_Pin,10,300);
			}
			if ( (Blocked_by_150==1) && (KLAPAN_SIGN==0)) {
				KLAPAN_SIGN=1;
				LED_Blink_X(BUZZER_GPIO_Port,BUZZER_Pin,10,1);
			}


			// new code
			restart_flag=1;
			if ((Blocked_by_AB==0) && (Blocked_by_PVD==0) && (Blocked_by_TEMP==0)
								&& (Blocked_by_Perek==1) ) {

				if (restart_flag==0) {

					if (CheckStamp(restart_flag_stamp,RESTART_MAX_LENGTH)==1) {
						// RESTART CODE BEGIN

							//SHUTDOWN PWM
							GPIOB->BSRR  = GPIO_BSRR_BS_10;
							for (uint16_t i=0; i<500; ++i) {
								// 1 microsec
								for (int j = 0; j < 32; ++j) {
									__asm__ __volatile__("nop\n\t":::"memory");
								}
							}

							//SHUTDOWN AND UP INV
							GPIOB->BRR  = GPIO_BRR_BR_0;

							Blocked_by_Klapan=0;
							Blocked_by_150=0;
							Blocked_by_Klapan_CNT=0;


							for (uint32_t i=0; i<500; ++i) {
								// 1 microsec
								for (int j = 0; j < 32; ++j) {
									__asm__ __volatile__("nop\n\t":::"memory");
								}
							}
							GPIOB->BSRR  = GPIO_BSRR_BS_0;

						// RESTART CODE END

						restart_flag=1;
						restart_flag_stamp=0;
					}
				}
				else {
					if (restart_flag_stamp==0) {
											restart_flag_stamp=xTaskGetTickCount();
										}
					if (CheckStamp(restart_flag_stamp,RESTART_MAX_LENGTH_LONG)==1) {
						restart_flag=0;
						restart_flag_stamp=0;
					}
				}

			} else {
				restart_flag=0;
				restart_flag_stamp=0;
			}
		}

		taskYIELD();
	  }
}

/* USER CODE BEGIN Header_StartLED_Task */
/**
* @brief Function implementing the LED_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLED_Task */
void StartLED_Task(void const * argument)
{
	  for(;;)
	  {
		// RESET IWDG
		IWDG_Reset();

		//turn off LED
		GPIOA->BRR  = GPIO_BRR_BR_7;

		/*if ((Blocked_by_PVD==1) || (Blocked_by_TEMP==1)) {
					LED_Blink(LED_GPIO_Port,LED_Pin,10);
					osDelay(100);
		} else {*/
							if (Global_AB_BASE<BUZZER_OPORA) {
								HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET); // AB ON
							} else {
								HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET); // AB ON
							}
							osDelay(500);
		//}
	  }
}

/* USER CODE BEGIN Header_StartBuzzer_Task */
/**
* @brief Function implementing the Buzzer_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBuzzer_Task */
void StartBuzzer_Task(void const * argument)
{
      uint32_t buzzer_stamp=xTaskGetTickCount();

	  while ((Global_AB_BASE<AB_COLDRUN) && (Blocked_by_Perek==0))  {
		  if (xTaskGetTickCount()-buzzer_stamp<=BUZZER_MAX_LENGTH*1000) {
			  LED_Blink(BUZZER_GPIO_Port,BUZZER_Pin,1);
		  }
		  osDelay(3000);
	  }

	  LED_Blink(BUZZER_GPIO_Port,BUZZER_Pin,100);

	  buzzer_stamp=0;

	  for(;;)
	  {
		  		  //turn off buzzer
		  		  GPIOB->BRR  = GPIO_BRR_BR_13;

				if (  ((Blocked_by_Perek==0) && ((Global_AB_BASE>=AB_LOW) && ((Global_AB_BASE<=BUZZER_OPORA_MAX) || (Global_AB_BASE>=AB_MAX)) ) )
						|| ((Blocked_by_Perek==1) &&  (Global_AB_BASE>=AB_MAX) )
						|| (Blocked_by_TEMP==1) ) {
					if (buzzer_stamp==0) {
						buzzer_stamp=xTaskGetTickCount();
					}
					if (CheckStamp(buzzer_stamp,BUZZER_MAX_LENGTH)==0) {
						LED_Blink_X(BUZZER_GPIO_Port,BUZZER_Pin,3,300);
					}
					osDelay(2500);
				}
				if ((buzzer_stamp!=0) && (Global_AB_BASE>=AB_COLDRUN) &&  (Global_AB_BASE<AB_COLDRUN_FROM_MAX)) {
					LED_Blink_X(BUZZER_GPIO_Port,BUZZER_Pin,6,100);
					buzzer_stamp=0;
				}

				osDelay(1);

	  }
}

/* USER CODE BEGIN Header_StartCooler_Task */
/**
* @brief Function implementing the Cooler_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCooler_Task */
void StartCooler_Task(void const * argument)
{
	  uint32_t cooler_stamp=0;

	  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) ;
	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, PULSE40_VALUE);
	  osDelay(3000);
	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);

	  for(;;)
	  {
		if (Blocked_by_TEMP==1) {
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, PULSE100_VALUE);
			cooler_stamp=0;
			osDelay(10000);
		} else {
			if (Blocked_by_Perek==1) {
				if ((TEMP_C>TEMP_ROLLBACK) && (TEMP_C<TEMP_COLRSTART)) {
					__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, PULSE20_VALUE);
					cooler_stamp=0;
				}
				if (TEMP_C>=TEMP_COLRSTART) {
					__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, PULSE40_VALUE);
					cooler_stamp=0;
				}
				if ((TEMP_C<=TEMP_ROLLBACK) && (__HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_1)!=0)) {
					if (cooler_stamp==0) {
						cooler_stamp=xTaskGetTickCount();
					}
					if (CheckStamp(cooler_stamp,COOLER_MAX_LENGTH)==1)  {
						__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
						cooler_stamp=0;
						LED_Blink_X(BUZZER_GPIO_Port,BUZZER_Pin,4,100);
					}
				}
				osDelay(1000);
			} else {
				if ((Power_Percent>=40) || (TEMP_C>TEMP_COLRSTART)) {
					__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, PULSE100_VALUE);
					cooler_stamp=0;
				} else {
					if ( ((Power_Percent>=15) && (Power_Percent<40)) || (TEMP_C>TEMP_ROLLBACK)) {
						__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, PULSE70_VALUE);
						cooler_stamp=0;
					} else {
						__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, PULSE40_VALUE);
						cooler_stamp=0;
					}
				}
				osDelay(3000);
			}
		}
		osDelay(1000);
	  }
}

/* USER CODE BEGIN Header_StartAB_Task */
/**
* @brief Function implementing the AB_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartAB_Task */
void StartAB_Task(void const * argument)
{
	  uint32_t ab_stamp=0;
		//while ( (Global_AB_BASE<AB_COLDRUN) && (Blocked_by_Perek==0)) {
		while ( (Global_AB_BASE<AB_COLDRUN)) {
			  Blocked_by_AB=1;
			  osDelay(1);
		  }

	  Blocked_by_AB=0;

	  for(;;)
	  {
		if ( (Blocked_by_AB==0) && ((Global_AB_BASE<=AB_LOW)  || (Global_AB_BASE>=AB_MAX) ) ) {
			Blocked_by_AB=1;
			ab_stamp=0;
			LED_Blink_X(BUZZER_GPIO_Port,BUZZER_Pin,4,100);
			osDelay(3000);
		}
		if ( (Blocked_by_AB==1) && (Global_AB_BASE>=AB_ROLLBACK) &&   (Global_AB_BASE>=AB_COLDRUN_FROM_MAX) ) {
			if (ab_stamp==0) {
				ab_stamp=xTaskGetTickCount();
			}
			if ((CheckStamp(ab_stamp,AB_MAX_LENGTH)==1) || (Blocked_by_Perek==1)) {
				 Blocked_by_AB=0;
				 ab_stamp=0;
				 LED_Blink_X(BUZZER_GPIO_Port,BUZZER_Pin,4,100);
			}
		} else {
			ab_stamp=0;
		}

		osDelay(1);
	  }
}


/* USER CODE BEGIN Header_StartTEMP_Task */
/**
* @brief Function implementing the TEMP_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTEMP_Task */
void StartTEMP_Task(void const * argument)
{
	  uint32_t temp_stamp=0;

	  Calc_Temp();

	  while (TEMP_C>=TEMP_COLRSTART) {
		      Calc_Temp();
			  Blocked_by_TEMP=1;
			  osDelay(1000);
	  }

	  Blocked_by_TEMP=0;

	  for(;;)
	  {
		    Calc_Temp();
		    //
		    if (TEMP_C<TEMP_MAX)  {
			  if ((Blocked_by_TEMP==1) && (TEMP_C<=TEMP_ROLLBACK)) {
				  if (temp_stamp==0) {
					  temp_stamp=xTaskGetTickCount();
				  }
				  if (CheckStamp(temp_stamp,TEMP_DELAY_LENGTH)==1) {
					  Blocked_by_TEMP=0;
					  temp_stamp=0;
				 }
			  } else {
				  if (Blocked_by_TEMP!=1) {
					  Blocked_by_TEMP=0;
				  }
				  temp_stamp=0;
			  }
			  osDelay(500);
		    } else {
		  	  Blocked_by_TEMP=1;
		  	  temp_stamp=0;
		  	  osDelay(TEMP_ROLLBACK_DELAY_LENGTH*1000); //sec
		    }
	  }
}

/* USER CODE BEGIN Header_StartCUR_Task */
/**
* @brief Function implementing the CUR_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCUR_Task */
void StartCUR_Task(void const * argument)
{
	 for(;;) {

		 if (Ready_AVG_CUR==1) {
			 Ready_AVG_CUR=0; //received AVG value and now perform it

	  	  	  	  // CUR LED
	  			  if ( (Blocked_by_Klapan==1) || (Blocked_by_150==1)) {
	  				  //LED_Blink(LED_2_GPIO_Port,LED_2_Pin,500);
	  				  //osDelay(500);
	  				  HAL_GPIO_TogglePin(LED_2_GPIO_Port,LED_2_Pin);
	  			  } else {
	  				 // check 120%
	  				  if (Power_Percent>=POLKA_100) {
	  					Blocked_by_150=1;
	  				  } else {
	  					  	  if( (Power_Percent<POLKA_95) && (Blocked_by_Klapan_CNT<=KLAPAN_CNT) && (Blocked_by_Klapan_CNT>0) ) {
	  							  Blocked_by_Klapan_CNT=0;
	  						  }
	  				  }

	  				  if (Power_Percent<50) {
	  					  //HAL_GPIO_WritePin(LED_2_GPIO_Port,LED_2_Pin, GPIO_PIN_RESET);
	  					  GPIOA->BRR  = GPIO_BRR_BR_7;

	  				  }
	  				  if (Power_Percent>=50) {
	  					  GPIOA->BSRR  = GPIO_BSRR_BS_7;
	  				  }
	  			  }


		 }
		 //
		 if (Ready_AVG_BASE==1) {
			 Ready_AVG_BASE=0;

			 if (Power_Percent>=POLKA_130) {
				 Blocked_by_150=1;
			 }

		 }
		 //
		 osDelay(1);
	 }
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
