/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    stm32f4xx_it.c
 * @brief   Interrupt Service Routines.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "fatfs.h"
#include "sdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
RTC_TimeTypeDef sTime;
RTC_DateTypeDef sDate;
RTC_HandleTypeDef hrtc;
FIL myFile;
SD_HandleTypeDef hsd;
FATFS myFATFS;
UINT testByte;
/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
char FileName[] = "Measure.txt";//Bestands naam voor externe sensor
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
extern SD_HandleTypeDef hsd;
extern TIM_HandleTypeDef htim2;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern UART_HandleTypeDef huart6;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  HAL_RCC_NMI_IRQHandler();
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
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line1 interrupt.
  */
void EXTI1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI1_IRQn 0 */
	if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_1) != RESET)
	{
		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_1);
		static int State = 1;
		static int Reading;
		static int Previous = 0;
		static int VorigSec = 0;
		static int b = 0;
		static int Calc[10];
		char Tekst[] = "Regen input 1:\r\n";
		char Tekst1[] = "mm/h\r\n";
		char Tekst2[] = "Tijd:\t";
		char Tekst3[] = ":";
		char Tekst4[] = "\r\n\r\n";

		Reading++;

		if(Reading == 1 && Previous == 0)
		{
			if(State == 1)
			{
				State = 0;
				GPIOC -> ODR ^= GPIO_PIN_11;
				HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
				HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
				int HuidigSec = sTime.Seconds;
				int x = 0;
				int Sum = 0;
				float Avg = 0;

				if(VorigSec < HuidigSec)
				{
					x = HuidigSec - VorigSec;
				}
				else if(VorigSec > HuidigSec)
				{
					x = (HuidigSec + 60) - VorigSec;
				}

				VorigSec = HuidigSec;

				if(x >= 1)
				{
					Calc[b] = (3600/x) * 0.1;
					b++;
				}

				if(b == 10)
				{
					b = 0;
					for(int i = 0; i <= 9; i++)
					{
						Sum += Calc[i];
					}
					Avg = Sum/10;
					char DataH[3];
					itoa(Avg, DataH, 10);
					if(f_mount(&myFATFS, SDPath, 1) == FR_OK)
					{
						GPIOC -> ODR &= ~GPIO_PIN_9;
						f_open(&myFile, FileName, FA_WRITE | FA_OPEN_APPEND);
						f_write(&myFile, &Tekst, sizeof(Tekst), &testByte);
						f_write(&myFile, &DataH, sizeof(DataH), &testByte);
						f_write(&myFile, &Tekst1, sizeof(Tekst1), &testByte);
						f_write(&myFile, &Tekst2, sizeof(Tekst2), &testByte);
						uint8_t h = sTime.Hours;
						char DataHours[2];
						itoa(h, DataHours, 10);
						f_write(&myFile, &DataHours, sizeof(DataHours), &testByte);
						f_write(&myFile, &Tekst3, sizeof(Tekst3), &testByte);
						uint8_t m = sTime.Minutes;
						char DataM[2];
						itoa(m, DataM, 10);
						f_write(&myFile, &DataM, sizeof(DataM), &testByte);
						f_write(&myFile, &Tekst3, sizeof(Tekst3), &testByte);
						uint8_t s = sTime.Seconds;
						char DataS[2];
						itoa(s, DataS, 10);
						f_write(&myFile, &DataS, sizeof(DataS), &testByte);
						f_write(&myFile, &Tekst4, sizeof(Tekst4), &testByte);
						f_close(&myFile);
						GPIOC -> ODR |= GPIO_PIN_9;
					}
				}
				Reading = 0;
			}
			else
			{
				State = 1;
				Reading = 0;
			}
		}
		for(uint32_t i = 0; i<=100000; i++);
	}
  /* USER CODE END EXTI1_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
  /* USER CODE BEGIN EXTI1_IRQn 1 */

  /* USER CODE END EXTI1_IRQn 1 */
}

/**
  * @brief This function handles EXTI line2 interrupt.
  */
void EXTI2_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI2_IRQn 0 */
	if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_2) != RESET)
	{
		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_2);
		static int State = 1;
		static int Reading;
		static int Previous = 0;
		static int VorigSec = 0;
		static int b = 0;
		static int Calc[10];
		char Tekst[] = "Regen input 2:\r\n";
		char Tekst1[] = "mm/h\r\n";
		char Tekst2[] = "Tijd:\t";
		char Tekst3[] = ":";
		char Tekst4[] = "\r\n\r\n";

		Reading++;

		if(Reading == 1 && Previous == 0)
		{
			if(State == 1)
			{
				State = 0;
				GPIOC -> ODR ^= GPIO_PIN_11;
				HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
				HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
				int HuidigSec = sTime.Seconds;
				int x = 0;
				int Sum = 0;
				float Avg = 0;

				if(VorigSec < HuidigSec)
				{
					x = HuidigSec - VorigSec;
				}
				else if(VorigSec > HuidigSec)
				{
					x = (HuidigSec + 60) - VorigSec;
				}

				VorigSec = HuidigSec;

				if(x >= 1)
				{
					Calc[b] = (3600/x) * 0.1;
					b++;
				}

				if(b == 10)
				{
					b = 0;
					for(int i = 0; i <= 9; i++)
					{
						Sum += Calc[i];
					}
					Avg = Sum/10;
					char DataH[3];
					itoa(Avg, DataH, 10);
					if(f_mount(&myFATFS, SDPath, 1) == FR_OK)
					{
						f_open(&myFile, FileName, FA_WRITE | FA_OPEN_APPEND);
						f_write(&myFile, &Tekst, sizeof(Tekst), &testByte);
						f_write(&myFile, &DataH, sizeof(DataH), &testByte);
						f_write(&myFile, &Tekst1, sizeof(Tekst1), &testByte);
						f_write(&myFile, &Tekst2, sizeof(Tekst2), &testByte);
						uint8_t h = sTime.Hours;
						char DataHours[2];
						itoa(h, DataHours, 10);
						f_write(&myFile, &DataHours, sizeof(DataHours), &testByte);
						f_write(&myFile, &Tekst3, sizeof(Tekst3), &testByte);
						uint8_t m = sTime.Minutes;
						char DataM[2];
						itoa(m, DataM, 10);
						f_write(&myFile, &DataM, sizeof(DataM), &testByte);
						f_write(&myFile, &Tekst3, sizeof(Tekst3), &testByte);
						uint8_t s = sTime.Seconds;
						char DataS[2];
						itoa(s, DataS, 10);
						f_write(&myFile, &DataS, sizeof(DataS), &testByte);
						f_write(&myFile, &Tekst4, sizeof(Tekst4), &testByte);
						f_close(&myFile);
					}
				}
				Reading = 0;
			}
			else
			{
				State = 1;
				Reading = 0;
			}
		}
		for(uint32_t i = 0; i<=100000; i++);
	}
  /* USER CODE END EXTI2_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
  /* USER CODE BEGIN EXTI2_IRQn 1 */

  /* USER CODE END EXTI2_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream5 global interrupt.
  */
void DMA1_Stream5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream5_IRQn 0 */

  /* USER CODE END DMA1_Stream5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_rx);
  /* USER CODE BEGIN DMA1_Stream5_IRQn 1 */

  /* USER CODE END DMA1_Stream5_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles SDIO global interrupt.
  */
void SDIO_IRQHandler(void)
{
  /* USER CODE BEGIN SDIO_IRQn 0 */

  /* USER CODE END SDIO_IRQn 0 */
  HAL_SD_IRQHandler(&hsd);
  /* USER CODE BEGIN SDIO_IRQn 1 */

  /* USER CODE END SDIO_IRQn 1 */
}

/**
  * @brief This function handles USART6 global interrupt.
  */
void USART6_IRQHandler(void)
{
  /* USER CODE BEGIN USART6_IRQn 0 */

  /* USER CODE END USART6_IRQn 0 */
  HAL_UART_IRQHandler(&huart6);
  /* USER CODE BEGIN USART6_IRQn 1 */

  /* USER CODE END USART6_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
