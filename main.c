/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
#include "dma.h"
#include "fatfs.h"
#include "i2c.h"
#include "rtc.h"
#include "sdio.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "MY_FLASH.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
FIL myFile;
SD_HandleTypeDef hsd;
FATFS myFATFS;
UINT testByte;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;
RTC_HandleTypeDef hrtc;
RTC_TimeTypeDef sTime;
RTC_DateTypeDef sDate;
TIM_HandleTypeDef htim2;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
char rxData[40]; //Buffer
char FileName_Internal[] = "Diag.txt"; //Bestands naam voor interne sensor
char FileName_Measure[] = "Measure.txt";//Bestands naam voor externe sensor
char FileName_Test[] = "TEST.txt"; //Data overdracht test
uint8_t Timer = 0; //Timer voor de interrupt
uint8_t LED = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void ToggleRGB(char Colour, int Mode);
void ToggleRelay(int Relay, int Toggle);
void TogglePower(int side, int Toggle);
void MeasureLogInternal(void);
uint8_t MeasureRH(void);
uint8_t MeasureT(void);
void CreateFile(char FileName[]);
void CreateFileNew(char FileName[]);
void SaveRH(char FileName[]);
void SaveT(char FileName[]);
void SaveRS(char FileName[], uint8_t RS, uint8_t Sensor);
void SaveSDi(char FileName[], uint8_t Sensor);
void WriteRS(uint8_t port, char Message[]);
void ReadRS(uint8_t RS, uint8_t port, uint16_t TimeOut);
void WriteSDi(char Message[], uint16_t TimeOut);
void ReadSDi(uint16_t TimeOut);
void GPIO_Reset(void);
void Start_Up(void);
uint8_t Create_File_Func(void);
void EmptyBuffer(void);
void SetTime(void);
void RemountSD(void);
void Flash_Write(uint32_t Flash_Address, uint32_t Flash_Data);
uint32_t Flash_Read(uint32_t Flash_Address);
void Flash_Erase_SectorSeven(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/* USER CODE BEGIN 1 */
	uint8_t File = 0;
	uint8_t Timer2 = 0;

	char Poort_Switch[2][10] = {" CLOSED",
			                    " OPEN  "};
	char Poort_Protocol_RS[3][10] = {" RS232 ",
			                         " RS422 ",
			                         " RS485 "};
	uint8_t MM = 0; //Main Menu optie
	uint8_t Sp = 0;
	uint8_t Opt_Menu = 0; //Onthouden welke menu de gebruiker in zit
	uint8_t Opt_One = 0; //Open/Closed Seriele poort optie
	uint8_t Save_One = 0;
	uint8_t Opt_Two = 0; //RS-232/RS-422/RS-485 Seriele poort optie
	uint8_t Save_Two = 0;
	uint8_t Opt_Three = 0; //Open/Closed SDi-12 poort optie
	uint8_t Save_Three = 0;
	uint8_t Opt_Four = 0; //Open/Closed Power switch poort 1 optie
	uint8_t Save_Four = 0;
	uint8_t Opt_Five = 0; //Open/Closed Power switch poort 2 optie
	uint8_t Save_Five = 0;
	uint8_t Opt_Six = 0; //Open/Closed Relay poort 1 optie
	uint8_t Save_Six = 0;
	uint8_t Opt_Seven = 0; //Open/Closed Relay poort 2 optie
	uint8_t Save_Seven = 0;
	uint8_t Relay = 0;
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
	MX_I2C1_Init();
	MX_RTC_Init();
	MX_SDIO_SD_Init();
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	MX_USART6_UART_Init();
	MX_FATFS_Init();
	MX_DMA_Init();
	MX_TIM2_Init();
	/* USER CODE BEGIN 2 */
	//Reset GPIO pinnen
	GPIO_Reset();
	LED = 1;

	sDate.Date = 7;
	sDate.Month = RTC_MONTH_JANUARY;
	sDate.WeekDay = RTC_WEEKDAY_TUESDAY;
	sDate.Year = 20;
	HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

	HAL_TIM_Base_Start_IT(&htim2);
	Timer = 0; //Timer reset.

	Save_One = Flash_Read(0x08060000); //Open/Closed Seriele poort optie variable is opgeslagen in sector 7 0x08060000
	Save_Two = Flash_Read(0x08060010); //RS-232/RS-422/RS-485 Seriele poort optie variable opgeslagen in 0x08060010
	Save_Three = Flash_Read(0x08060020); //
	Save_Four = Flash_Read(0x08060030);
	Save_Five = Flash_Read(0x08060040);
	Save_Six = Flash_Read(0x08060050);
	Save_Seven = Flash_Read(0x08060060);

	WriteRS(1, "\x1b[1J"); //Clear screen
	WriteRS(1, "\x1b[f"); //Move cursor to upper left corner

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		ToggleRGB('R', 0);
		EmptyBuffer();
		ReadRS(1, 1, 5);

		/* Terminal Menu */
		if(MM == 0) //Main Menu
		{
			Opt_Menu = 0;
			WriteRS(1, "\x1b[2J"); //Clear screen
			WriteRS(1, "\x1b[f"); //Move cursor to upper left corner
			WriteRS(1, "MENU\r\n");
			WriteRS(1, "1) Seriele Poorten\r\n");
			WriteRS(1, "2) Sensor Voeding switch\r\n");
			MM = 1;
		}
		if(Sp == 1) // Seriele poort optie menu
		{
			Opt_Menu = 1;
			WriteRS(1, "\x1b[2J"); //Clear screen
			WriteRS(1, "\x1b[f"); //Move cursor to upper left corner
			WriteRS(1, "Seriele Poort\r\n");
			WriteRS(1, "   1)");
			if(Save_One >= 100)
			{
				Save_One = 0;
			}
			WriteRS(1, Poort_Switch[Save_One]);
			WriteRS(1, "\r\n");
			WriteRS(1, "   2)");
			if(Save_Two >= 100)
			{
				Save_Two = 0;
			}
			WriteRS(1, Poort_Protocol_RS[Save_Two]);
			WriteRS(1, "\r\n");
			WriteRS(1, "   3) OMC-160-3\r\n");
			WriteRS(1, "SDi-12\r\n");
			WriteRS(1, "   4)");
			if(Save_Three >= 100)
			{
				Save_Three = 0;
			}
			WriteRS(1, Poort_Switch[Save_Three]);
			WriteRS(1, "\r\n");
			WriteRS(1, "   5) PT12\r\n");
			Sp = 0;
		}
		if(Sp == 2) // Power Switch optie Menu
		{
			Opt_Menu = 2;
			WriteRS(1, "\x1b[2J"); //Clear screen
			WriteRS(1, "\x1b[f"); //Move cursor to upper left corner
			WriteRS(1, "Power Switch\r\n");
			WriteRS(1, "   1) Poort 1:");
			if(Save_Four >= 100)
			{
				Save_Four = 0;
			}
			WriteRS(1, Poort_Switch[Save_Four]);
			WriteRS(1, "\r\n");
			WriteRS(1, "   2) Poort 2:");
			if(Save_Five >= 100)
			{
				Save_Five = 0;
			}
			WriteRS(1, Poort_Switch[Save_Five]);
			WriteRS(1, "\r\n");
			WriteRS(1, "Relay Switch\r\n");
			WriteRS(1, "   3) Poort 1:");
			if(Save_Six >= 100)
			{
				Save_Six = 0;
			}
			WriteRS(1, Poort_Switch[Save_Six]);
			WriteRS(1, "\r\n");
			WriteRS(1, "   4) Poort 2:");
			if(Save_Seven >= 100)
			{
				Save_Seven = 0;
			}
			WriteRS(1, Poort_Switch[Save_Seven]);
			WriteRS(1, "\r\n");
			Sp = 0;
		}

		switch(rxData[0])
		{
		case 'e':
			MM = 0;
			break;
		case '1':
			if(Opt_Menu == 0)
			{
				Sp = 1;
			}
			else if(Opt_Menu == 1)
			{
				Sp = 1;
				Opt_One++;
				if(Opt_One >= 2)
				{
					Opt_One = 0;
				}
				Flash_Erase_SectorSeven();
				Flash_Write(0x08060000, Opt_One);
				Save_One = Flash_Read(0x08060000);
				Flash_Write(0x08060010, Save_Two);
				Flash_Write(0x08060020, Save_Three);
				Flash_Write(0x08060030, Save_Four);
				Flash_Write(0x08060040, Save_Five);
				Flash_Write(0x08060050, Save_Six);
				Flash_Write(0x08060060, Save_Seven);
			}
			else if(Opt_Menu == 2)
			{
				Sp = 2;
				Opt_Four++;
				if(Opt_Four >= 2)
				{
					Opt_Four = 0;
				}
				Flash_Erase_SectorSeven();
				Flash_Write(0x08060030, Opt_Four);
				Save_Four = Flash_Read(0x08060030);
				Flash_Write(0x08060010, Save_Two);
				Flash_Write(0x08060020, Save_Three);
				Flash_Write(0x08060000, Save_One);
				Flash_Write(0x08060040, Save_Five);
				Flash_Write(0x08060050, Save_Six);
				Flash_Write(0x08060060, Save_Seven);
			}
			break;
		case '2':
			if(Opt_Menu == 0)
			{
				Sp = 2;
			}
			else if(Opt_Menu == 1)
			{
				Sp = 1;
				Opt_Two++;
				if(Opt_Two >= 3)
				{
					Opt_Two = 0;
				}
				Flash_Erase_SectorSeven();
				Flash_Write(0x08060010, Opt_Two);
				Save_Two = Flash_Read(0x08060010);
				Flash_Write(0x08060030, Save_Four);
				Flash_Write(0x08060020, Save_Three);
				Flash_Write(0x08060000, Save_One);
				Flash_Write(0x08060040, Save_Five);
				Flash_Write(0x08060050, Save_Six);
				Flash_Write(0x08060060, Save_Seven);
			}
			else if(Opt_Menu == 2)
			{
				Sp = 2;
				Opt_Five++;
				if(Opt_Five >= 2)
				{
					Opt_Five = 0;
				}
				Flash_Erase_SectorSeven();
				Flash_Write(0x08060040, Opt_Five);
				Save_Five = Flash_Read(0x08060040);
				Flash_Write(0x08060010, Save_Two);
				Flash_Write(0x08060020, Save_Three);
				Flash_Write(0x08060000, Save_One);
				Flash_Write(0x08060030, Save_Four);
				Flash_Write(0x08060050, Save_Six);
				Flash_Write(0x08060060, Save_Seven);
			}
			break;
		case '3':
			if(Opt_Menu == 2)
			{
				Sp = 2;
				Relay = 1;
				Opt_Six++;
				if(Opt_Six >= 2)
				{
					Opt_Six = 0;
				}
				Flash_Erase_SectorSeven();
				Flash_Write(0x08060050, Opt_Six);
				Save_Six = Flash_Read(0x08060050);
				Flash_Write(0x08060010, Save_Two);
				Flash_Write(0x08060020, Save_Three);
				Flash_Write(0x08060000, Save_One);
				Flash_Write(0x08060040, Save_Five);
				Flash_Write(0x08060030, Save_Four);
				Flash_Write(0x08060060, Save_Seven);
			}
			break;
		case '4':
			if(Opt_Menu == 1)
			{
				Sp = 1;
				Opt_Three++;
				if(Opt_Three >= 2)
				{
					Opt_Three = 0;
				}
				Flash_Erase_SectorSeven();
				Flash_Write(0x08060020, Opt_Three);
				Save_Three = Flash_Read(0x08060020);
				Flash_Write(0x08060010, Save_Two);
				Flash_Write(0x08060030, Save_Four);
				Flash_Write(0x08060000, Save_One);
				Flash_Write(0x08060040, Save_Five);
				Flash_Write(0x08060050, Save_Six);
				Flash_Write(0x08060060, Save_Seven);
			}
			else if(Opt_Menu == 2)
			{
				Sp = 2;
				Opt_Seven++;
				Relay = 2;
				if(Opt_Seven >= 2)
				{
					Opt_Seven = 0;
				}
				Flash_Erase_SectorSeven();
				Flash_Write(0x08060060, Opt_Seven);
				Save_Seven = Flash_Read(0x08060060);
				Flash_Write(0x08060010, Save_Two);
				Flash_Write(0x08060020, Save_Three);
				Flash_Write(0x08060000, Save_One);
				Flash_Write(0x08060040, Save_Five);
				Flash_Write(0x08060050, Save_Six);
				Flash_Write(0x08060030, Save_Four);
			}
			break;
		}

		/* Power switch keuze voor poort 1 */
		if(Save_Four == 0)
		{
			GPIOA -> ODR &= ~GPIO_PIN_1;
		}
		else if(Save_Four == 1)
		{
			GPIOA -> ODR |= GPIO_PIN_1;
		}
		/* Power switch keuze voor poort 2 */
		if(Save_Five == 0)
		{
			GPIOA -> ODR &= ~GPIO_PIN_0;
		}
		else if(Save_Five == 1)
		{
			GPIOA -> ODR |= GPIO_PIN_0;
		}

		if(Relay == 1)
		{
			Relay = 0;
			/* Poort 1 relay switch keuze */
			if(Save_Six == 0)
			{
				//Relay Reset
				GPIOC -> ODR |= GPIO_PIN_2;
				HAL_Delay(5);
				GPIOC -> ODR &= ~GPIO_PIN_2;
			}
			else if(Save_Six == 1)
			{
				//Relay set
				GPIOC -> ODR |= GPIO_PIN_1;
				HAL_Delay(5);
				GPIOC -> ODR &= ~GPIO_PIN_1;
			}
		}
		else if(Relay == 2)
		{
			Relay = 0;
			/* Poort 2 relay switch keuze */
			if(Save_Seven == 0)
			{
				//Relay Reset
				GPIOC -> ODR |= GPIO_PIN_4;
				HAL_Delay(5);
				GPIOC -> ODR &= ~GPIO_PIN_4;
			}
			else if(Save_Seven == 1)
			{
				//Relay set
				GPIOC -> ODR |= GPIO_PIN_0;
				HAL_Delay(5);
				GPIOC -> ODR &= ~GPIO_PIN_0;
			}
		}

		RemountSD();

		if(Timer == 1 && Timer2 == 0) //Internal meusurement
		{
			MeasureLogInternal();
			Timer2 = 1;
		}
		else if(Timer == 2)//external meusurement
		{
			if(Save_One == 1)
			{
				SaveRS(FileName_Measure, Save_Two, 2);
			}

			if(Save_Three == 1)
			{
				SaveSDi(FileName_Measure, 1);
			}

			Timer = 0;
			Timer2 = 0;
		}
		else if(Timer > 2)
		{
			Timer = 0;
		}
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
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 100;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
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
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
	{
		Error_Handler();
	}
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
	PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Enables the Clock Security System
	 */
	HAL_RCC_EnableCSS();
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* Prevent unused argument(s) compilation warning */
	UNUSED(htim);

	/* NOTE : This function should not be modified, when the callback is needed,
            the HAL_TIM_PeriodElapsedCallback could be implemented in the user file
	 */

	if(LED == 0)
	{
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9);
	}
	Timer++;
}

void MeasureLogInternal(void)
{
	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR0, sTime.Hours);
	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, sTime.Minutes);
	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR2, sTime.Seconds);
	SaveRH(FileName_Internal);
	HAL_Delay(10);
	SaveT(FileName_Internal);
}

void GPIO_Reset(void)
{
	//RGB uit
	ToggleRGB('R', 0);
	ToggleRGB('G', 0);
	ToggleRGB('B', 0);
	//3.3VDCb aan
	GPIOB -> ODR |= GPIO_PIN_0;
	//Protocol tranceiver aan (Nu een Userinterface in RS232 mode)
	GPIOB -> ODR |= GPIO_PIN_12;
}

uint8_t Create_File_Func(void)
{
	uint8_t RS232 = 1;
	uint8_t File = 0;
	WriteRS(1, "Wil je de oude bestanden verwijderen of erover heen schrijven?\r\n");
	HAL_Delay(200);
	WriteRS(1, "type '1' voor verwijderen\r\n");
	HAL_Delay(200);
	WriteRS(1, "type '2' voor overheen schrijven\r\n");
	do
	{
		EmptyBuffer();
		ReadRS(RS232, 1, 1);
		WriteRS(1, rxData);
		switch(rxData[0])
		{
		case '1':
			File = 1;
			CreateFileNew(FileName_Internal);
			CreateFileNew(FileName_Measure);
			break;
		case '2':
			File = 2;
			CreateFile(FileName_Internal);
			CreateFile(FileName_Measure);
			break;
		}
	}while(rxData[0] != '\r' || File >= 3 || File == 0);
	return File;
}

void ToggleRGB(char Colour, int Mode)
{
	if(Mode == 1)
	{
		if(Colour == 'R')
		{
			GPIOC -> ODR &= ~GPIO_PIN_9;
		}
		if(Colour == 'G')
		{
			GPIOC -> ODR &= ~GPIO_PIN_10;
		}
		if(Colour == 'B')
		{
			GPIOC -> ODR &= ~GPIO_PIN_11;
		}
	}
	else if(Mode == 0)
	{
		if(Colour == 'R')
		{
			GPIOC -> ODR |= GPIO_PIN_9;
		}
		if(Colour == 'G')
		{
			GPIOC -> ODR |= GPIO_PIN_10;
		}
		if(Colour == 'B')
		{
			GPIOC -> ODR |= GPIO_PIN_11;
		}
	}
}

void ToggleRelay(int Relay, int Toggle)
{
	if(Relay == 1)
	{
		if(Toggle == 1)
		{
			GPIOC -> ODR |= GPIO_PIN_1;
			HAL_Delay(10);
			GPIOC -> ODR &= ~GPIO_PIN_1;
		}
		else if(Toggle == 0)
		{
			GPIOC -> ODR |= GPIO_PIN_2;
			HAL_Delay(10);
			GPIOC -> ODR &= ~GPIO_PIN_2;
		}
	}
	else if(Relay == 2)
	{
		if(Toggle == 1)
		{
			GPIOC -> ODR |= GPIO_PIN_0;
			HAL_Delay(10);
			GPIOC -> ODR &= ~GPIO_PIN_0;
		}
		else if(Toggle == 0)
		{
			GPIOC -> ODR |= GPIO_PIN_4;
			HAL_Delay(10);
			GPIOC -> ODR &= ~GPIO_PIN_4;
		}
	}
}

void TogglePower(int side, int Toggle)
{
	if(side == 1)
	{
		if(Toggle == 1)
		{
			GPIOA -> ODR |= GPIO_PIN_1;
		}
		if(Toggle == 0)
		{
			GPIOA -> ODR &= ~GPIO_PIN_1;
		}
	}
	if(side == 2)
	{
		if(Toggle == 1)
		{
			GPIOA -> ODR |= GPIO_PIN_0;
		}
		if(Toggle == 0)
		{
			GPIOA -> ODR &= ~GPIO_PIN_0;
		}
	}
}

uint8_t MeasureRH(void)
{
	unsigned char buffer[5];
	ToggleRGB('G', 1);
	uint8_t MRH = 0xE5;
	HAL_I2C_Master_Transmit(&hi2c1, 0x40<<1, &MRH, 1, 100);
	HAL_Delay(5);
	HAL_I2C_Master_Receive(&hi2c1, 0x40<<1, buffer, 2, 100);
	uint16_t rawH = buffer[0]<<8 | buffer[1];
	uint8_t rv = (float)((125*rawH/65536)-6);
	ToggleRGB('G', 0);
	return rv;
}

void SaveRH(char FileName[])
{
	if(f_mount(&myFATFS, SDPath, 1) == FR_OK)
	{
		char Tekst[] = "Interne RH:";
		char Tekst1[] = "%\r\n";

		f_open(&myFile, FileName, FA_WRITE | FA_OPEN_APPEND);
		f_write(&myFile, &Tekst, sizeof(Tekst), &testByte);
		f_close(&myFile);
		HAL_Delay(5);

		uint8_t h = MeasureRH();
		char Data[2];
		itoa(h, Data, 10);
		ToggleRGB('R', 1);
		f_open(&myFile, FileName, FA_WRITE | FA_OPEN_APPEND);
		f_write(&myFile, &Data, sizeof(Data), &testByte);
		f_close(&myFile);
		HAL_Delay(5);

		f_open(&myFile, FileName, FA_WRITE | FA_OPEN_APPEND);
		f_write(&myFile, &Tekst1, sizeof(Tekst1), &testByte);
		f_close(&myFile);
		HAL_Delay(5);
		ToggleRGB('R', 0);
	}
}

void SaveT(char FileName[])
{
	if(f_mount(&myFATFS, SDPath, 1) == FR_OK)
	{
		char Tekst[] = "Interne T:";
		char Tekst1[] = "Graden\r";
		char Tekst2[] = "Tijd:";
		char Tekst3[] = "\r\n";
		char Tekst4[] = ":";

		f_open(&myFile, FileName, FA_WRITE | FA_OPEN_APPEND);
		f_write(&myFile, &Tekst, sizeof(Tekst), &testByte);
		f_close(&myFile);

		uint8_t t = MeasureT();
		char Data[2];
		itoa(t, Data, 10);
		ToggleRGB('R', 1);
		f_open(&myFile, FileName, FA_WRITE | FA_OPEN_APPEND);
		f_write(&myFile, &Data, sizeof(Data), &testByte);
		f_close(&myFile);

		f_open(&myFile, FileName, FA_WRITE | FA_OPEN_APPEND);
		f_write(&myFile, &Tekst1, sizeof(Tekst1), &testByte);
		f_close(&myFile);

		f_open(&myFile, FileName, FA_WRITE | FA_OPEN_APPEND);
		f_write(&myFile, &Tekst2, sizeof(Tekst2), &testByte);
		f_close(&myFile);

		uint8_t h = sTime.Hours;
		char DataH[2];
		itoa(h, DataH, 10);
		f_open(&myFile, FileName, FA_WRITE | FA_OPEN_APPEND);
		f_write(&myFile, &DataH, sizeof(DataH), &testByte);
		f_close(&myFile);

		f_open(&myFile, FileName, FA_WRITE | FA_OPEN_APPEND);
		f_write(&myFile, &Tekst4, sizeof(Tekst4), &testByte);
		f_close(&myFile);

		uint8_t m = sTime.Minutes;
		char DataM[2];
		itoa(m, DataM, 10);
		f_open(&myFile, FileName, FA_WRITE | FA_OPEN_APPEND);
		f_write(&myFile, &DataM, sizeof(DataM), &testByte);
		f_close(&myFile);

		f_open(&myFile, FileName, FA_WRITE | FA_OPEN_APPEND);
		f_write(&myFile, &Tekst4, sizeof(Tekst4), &testByte);
		f_close(&myFile);

		uint8_t s = sTime.Seconds;
		char DataS[2];
		itoa(s, DataS, 10);
		f_open(&myFile, FileName, FA_WRITE | FA_OPEN_APPEND);
		f_write(&myFile, &DataS, sizeof(DataS), &testByte);
		f_close(&myFile);

		f_open(&myFile, FileName, FA_WRITE | FA_OPEN_APPEND);
		f_write(&myFile, &Tekst3, sizeof(Tekst3), &testByte);
		f_close(&myFile);

		f_open(&myFile, FileName, FA_WRITE | FA_OPEN_APPEND);
		f_write(&myFile, &Tekst3, sizeof(Tekst3), &testByte);
		f_close(&myFile);
		ToggleRGB('R', 0);
	}
}

void SaveRS(char FileName[], uint8_t RS, uint8_t Sensor)
{
	char Tekst2[] = "Tijd:";
	char Tekst3[] = "\r\n";
	char Tekst4[] = ":";
	char Tekst_Temp0[] = "PT12:\r\n";
	char Tekst_Hum0[] = "OMC-160-3:\r\n";

	ToggleRGB('G', 1);

	EmptyBuffer();
	ReadRS(RS, 2, 350);
	if(rxData[0] == 36)
	{
		for(int x = 1; x <= 39; x++)
		{
			if(rxData[x] == 36)
			{
				for(int y = x; y <= 39; y++)
				{
					rxData[y] = 0;
				}
				WriteRS(1, rxData);
				break;
			}
		}
	}
	else
	{
		EmptyBuffer();
	}

	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

	ToggleRGB('G', 0);

	if((f_mount(&myFATFS, SDPath, 1) == FR_OK) && rxData[0] == 36)
	{
		ToggleRGB('R', 1);
		if(Sensor == 1)
		{
			f_open(&myFile, FileName_Measure, FA_WRITE | FA_OPEN_APPEND);
			f_write(&myFile, &Tekst_Temp0, sizeof(Tekst_Temp0), &testByte);
			f_close(&myFile);
		}
		else if (Sensor == 2)
		{
			f_open(&myFile, FileName_Measure, FA_WRITE | FA_OPEN_APPEND);
			f_write(&myFile, &Tekst_Hum0, sizeof(Tekst_Hum0), &testByte);
			f_close(&myFile);
		}
		//Import meusurement in SD card
		f_open(&myFile, FileName, FA_WRITE | FA_OPEN_APPEND);
		f_write(&myFile, &rxData, sizeof(rxData), &testByte);
		f_close(&myFile);

		f_open(&myFile, FileName, FA_WRITE | FA_OPEN_APPEND);
		f_write(&myFile, &Tekst2, sizeof(Tekst2), &testByte);
		f_close(&myFile);

		uint8_t h = sTime.Hours;
		char DataH[2];
		itoa(h, DataH, 10);
		f_open(&myFile, FileName, FA_WRITE | FA_OPEN_APPEND);
		f_write(&myFile, &DataH, sizeof(DataH), &testByte);
		f_close(&myFile);

		f_open(&myFile, FileName, FA_WRITE | FA_OPEN_APPEND);
		f_write(&myFile, &Tekst4, sizeof(Tekst4), &testByte);
		f_close(&myFile);

		uint8_t m = sTime.Minutes;
		char DataM[2];
		itoa(m, DataM, 10);
		f_open(&myFile, FileName, FA_WRITE | FA_OPEN_APPEND);
		f_write(&myFile, &DataM, sizeof(DataM), &testByte);
		f_close(&myFile);

		f_open(&myFile, FileName, FA_WRITE | FA_OPEN_APPEND);
		f_write(&myFile, &Tekst4, sizeof(Tekst4), &testByte);
		f_close(&myFile);

		uint8_t s = sTime.Seconds;
		char DataS[2];
		itoa(s, DataS, 10);
		f_open(&myFile, FileName, FA_WRITE | FA_OPEN_APPEND);
		f_write(&myFile, &DataS, sizeof(DataS), &testByte);
		f_close(&myFile);

		f_open(&myFile, FileName, FA_WRITE | FA_OPEN_APPEND);
		f_write(&myFile, &Tekst3, sizeof(Tekst3), &testByte);
		f_close(&myFile);

		f_open(&myFile, FileName, FA_WRITE | FA_OPEN_APPEND);
		f_write(&myFile, &Tekst3, sizeof(Tekst3), &testByte);
		f_close(&myFile);
		ToggleRGB('R', 0);
	}
}

void SaveSDi(char FileName[], uint8_t Sensor)
{
	char Tekst2[] = "Tijd:";
	char Tekst3[] = "\r\n";
	char Tekst4[] = ":";
	char Tekst_Temp0[] = "PT12:\r\n";
	char Tekst_Hum0[] = "OMC-160-3:\r\n";

	ToggleRGB('G', 1);

	GPIOB -> ODR |= GPIO_PIN_10;

	HAL_LIN_SendBreak(&huart1);
	HAL_Delay(20);
	WriteSDi("0M1!", 200);

	GPIOB -> ODR &= ~GPIO_PIN_10;

	EmptyBuffer();
	ReadSDi(100);

	GPIOB -> ODR |= GPIO_PIN_10;

	HAL_Delay(1700);
	HAL_LIN_SendBreak(&huart1);
	HAL_Delay(20);
	WriteSDi("0D0!", 100);

	GPIOB -> ODR &= ~GPIO_PIN_10;

	EmptyBuffer();
	ReadSDi(300);

	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

	ToggleRGB('G', 0);

	if(f_mount(&myFATFS, SDPath, 1) == FR_OK)
	{
		ToggleRGB('R', 1);
		if(Sensor == 1)
		{
			f_open(&myFile, FileName_Measure, FA_WRITE | FA_OPEN_APPEND);
			f_write(&myFile, &Tekst_Temp0, sizeof(Tekst_Temp0), &testByte);
			f_close(&myFile);
		}
		else if(Sensor == 2)
		{
			f_open(&myFile, FileName_Measure, FA_WRITE | FA_OPEN_APPEND);
			f_write(&myFile, &Tekst_Hum0, sizeof(Tekst_Hum0), &testByte);
			f_close(&myFile);
		}

		f_open(&myFile, FileName, FA_WRITE | FA_OPEN_APPEND);
		f_write(&myFile, &rxData, sizeof(rxData), &testByte);
		f_close(&myFile);

		f_open(&myFile, FileName, FA_WRITE | FA_OPEN_APPEND);
		f_write(&myFile, &Tekst2, sizeof(Tekst2), &testByte);
		f_close(&myFile);

		uint8_t h = sTime.Hours;
		char DataH[2];
		itoa(h, DataH, 10);
		f_open(&myFile, FileName, FA_WRITE | FA_OPEN_APPEND);
		f_write(&myFile, &DataH, sizeof(DataH), &testByte);
		f_close(&myFile);

		f_open(&myFile, FileName, FA_WRITE | FA_OPEN_APPEND);
		f_write(&myFile, &Tekst4, sizeof(Tekst4), &testByte);
		f_close(&myFile);

		uint8_t m = sTime.Minutes;
		char DataM[2];
		itoa(m, DataM, 10);
		f_open(&myFile, FileName, FA_WRITE | FA_OPEN_APPEND);
		f_write(&myFile, &DataM, sizeof(DataM), &testByte);
		f_close(&myFile);

		f_open(&myFile, FileName, FA_WRITE | FA_OPEN_APPEND);
		f_write(&myFile, &Tekst4, sizeof(Tekst4), &testByte);
		f_close(&myFile);

		uint8_t s = sTime.Seconds;
		char DataS[2];
		itoa(s, DataS, 10);
		f_open(&myFile, FileName, FA_WRITE | FA_OPEN_APPEND);
		f_write(&myFile, &DataS, sizeof(DataS), &testByte);
		f_close(&myFile);

		f_open(&myFile, FileName, FA_WRITE | FA_OPEN_APPEND);
		f_write(&myFile, &Tekst3, sizeof(Tekst3), &testByte);
		f_close(&myFile);

		f_open(&myFile, FileName, FA_WRITE | FA_OPEN_APPEND);
		f_write(&myFile, &Tekst3, sizeof(Tekst3), &testByte);
		f_close(&myFile);
		ToggleRGB('R', 0);
	}
}

uint8_t MeasureT(void)
{
	unsigned char buffer[5];
	ToggleRGB('G', 1);
	uint8_t MT = 0xE0;
	HAL_I2C_Master_Transmit(&hi2c1, 0x40<<1, &MT, 1, 100);
	HAL_Delay(20);
	HAL_I2C_Master_Receive(&hi2c1, 0x40<<1, buffer, 2, 100);
	uint16_t rawT = buffer[0]<<8 | buffer[1];
	uint8_t r = (float)((175.72*rawT/65536)-46.85);
	ToggleRGB('G', 0);
	return r;
}

void CreateFile(char FileName[])
{
	if(f_mount(&myFATFS, SDPath, 1) == FR_OK)
	{
		f_open(&myFile, FileName, FA_CREATE_NEW);
		f_close(&myFile);
	}
}

void CreateFileNew(char FileName[])
{
	if(f_mount(&myFATFS, SDPath, 1) == FR_OK)
	{
		f_open(&myFile, FileName, FA_CREATE_ALWAYS);
		f_close(&myFile);
	}
}

void ReadRS(uint8_t RS, uint8_t port, uint16_t TimeOut)
{
	if(port == 2)
	{
		GPIOA -> ODR |= GPIO_PIN_4;
		if(RS == 1)
		{
			GPIOA -> ODR &= ~GPIO_PIN_5;
			GPIOA -> ODR &= ~GPIO_PIN_6;
			GPIOA -> ODR &= ~GPIO_PIN_7;
			HAL_UART_Receive(&huart2, (uint8_t *)rxData, sizeof(rxData), TimeOut);
		}
		else if(RS == 2)
		{
			GPIOA -> ODR |= GPIO_PIN_5;
			GPIOA -> ODR &= ~GPIO_PIN_6;
			GPIOA -> ODR &= ~GPIO_PIN_7;
			HAL_UART_Receive(&huart2, (uint8_t *)rxData, sizeof(rxData), TimeOut);
		}
		else if(RS == 3)
		{
			GPIOA -> ODR |= GPIO_PIN_5;
			GPIOA -> ODR |= GPIO_PIN_6;
			GPIOA -> ODR &= ~GPIO_PIN_7;
			HAL_UART_Receive(&huart2, (uint8_t *)rxData, sizeof(rxData), TimeOut);
		}
		GPIOA -> ODR &= ~GPIO_PIN_4;
	}
	else if(port == 1)
	{
		GPIOB -> ODR |= GPIO_PIN_12;
		if(RS == 1)
		{
			GPIOB -> ODR &= ~GPIO_PIN_13;
			GPIOB -> ODR &= ~GPIO_PIN_14;
			GPIOB -> ODR &= ~GPIO_PIN_15;
			HAL_UART_Receive(&huart6, (uint8_t *)rxData, sizeof(rxData), TimeOut);
		}
		else if(RS == 2)
		{
			GPIOB -> ODR |= GPIO_PIN_13;
			GPIOB -> ODR &= ~GPIO_PIN_14;
			GPIOB -> ODR &= ~GPIO_PIN_15;
			HAL_UART_Receive(&huart6, (uint8_t *)rxData, sizeof(rxData), TimeOut);
		}
		else if(RS == 3)
		{
			GPIOB -> ODR |= GPIO_PIN_13;
			GPIOB -> ODR |= GPIO_PIN_14;
			GPIOB -> ODR &= ~GPIO_PIN_15;
			HAL_UART_Receive(&huart6, (uint8_t *)rxData, sizeof(rxData), TimeOut);
		}
		GPIOB -> ODR &= ~GPIO_PIN_12;
	}
}

void WriteRS(uint8_t port, char Message[])
{
	if(port == 2)
	{
		GPIOA -> ODR |= GPIO_PIN_4;
		HAL_UART_Transmit(&huart2, (uint8_t *)Message, strlen(Message), 30);
		GPIOA -> ODR &= ~GPIO_PIN_4;
	}
	else if(port == 1)
	{
		GPIOB -> ODR |= GPIO_PIN_12;
		HAL_UART_Transmit(&huart6, (uint8_t *)Message, strlen(Message), 30);
		GPIOB -> ODR &= ~GPIO_PIN_12;
	}
}

void ReadSDi(uint16_t TimeOut)
{
	HAL_UART_Receive(&huart1, (uint8_t *)rxData, sizeof(rxData), TimeOut);
}

void WriteSDi(char Message[], uint16_t TimeOut)
{
	HAL_UART_Transmit(&huart1, (uint8_t *)Message, strlen(Message), TimeOut);
}

void EmptyBuffer(void)
{
	/* Empty buffer */
	for(int x=0; x<=39; x++)
	{
		rxData[x] = 0;
	}
}

void SetTime(void)
{
	uint8_t RS232 = 1;
	char TimerHour[2] = {0, 0};
	char TimerMin[2] = {0, 0};
	char TimerSec[2] = {0, 0};

	//Uren instellen
	WriteRS(1, "Uur:\t");

	do
	{
		do
		{
			EmptyBuffer();
			if(sTime.Hours > 0)
			{
				break;
			}
			ReadRS(RS232, 1, 1);
			WriteRS(1, rxData);
			TimerHour[0] = rxData[0];
		}while(rxData[0] < 48 || rxData[0] > 50);

		do
		{
			EmptyBuffer();
			if(sTime.Hours > 0)
			{
				break;
			}
			ReadRS(RS232, 1, 1);
			WriteRS(1, rxData);
			TimerHour[1] = rxData[0];
		}while(rxData[0] < 48 || rxData[0] > 51);

		sTime.Hours = atoi(TimerHour);

		ReadRS(RS232, 1, 1);

	}while(rxData[0] != '\r' || sTime.Hours < 0 || sTime.Hours > 23);

	if(TimerHour[0] > 0 || TimerHour[1] > 0)
	{
		TimerHour[0] = 0;
		TimerHour[1] = 0;
	}

	WriteRS(1, "\r\n\0");

	//Minuten instellen
	WriteRS(1, "Minuten:\t");

	do
	{
		do
		{
			EmptyBuffer();
			if(sTime.Minutes > 0)
			{
				break;
			}
			ReadRS(RS232, 1, 1);
			WriteRS(1, rxData);
			TimerMin[0] = rxData[0];
		}while(rxData[0] < 48 || rxData[0] > 53);

		do
		{
			EmptyBuffer();
			if(sTime.Minutes > 0)
			{
				break;
			}
			ReadRS(RS232, 1, 1);
			WriteRS(1, rxData);
			TimerMin[1] = rxData[0];
		}while(rxData[0] < 48 || rxData[0] > 57);

		sTime.Minutes = atoi(TimerMin);

		ReadRS(RS232, 1, 1);

	}while(rxData[0] != '\r' || sTime.Minutes < 0 || sTime.Minutes > 59);

	if(TimerMin[0] > 0 || TimerMin[1] > 0)
	{
		TimerMin[0] = 0;
		TimerMin[1] = 0;
	}

	WriteRS(1, "\r\n\0");

	//Seconden instellen
	WriteRS(1, "Seconden:\t");
	do
	{
		do
		{
			EmptyBuffer();
			if(sTime.Seconds > 0)
			{
				break;
			}
			ReadRS(RS232, 1, 1);
			WriteRS(1, rxData);
			TimerSec[0] = rxData[0];
		}while(rxData[0] < 48 || rxData[0] > 53);

		do
		{
			EmptyBuffer();
			if(sTime.Seconds > 0)
			{
				break;
			}
			ReadRS(RS232, 1, 1);
			WriteRS(1, rxData);
			TimerSec[1] = rxData[0];
		}while(rxData[0] < 48 || rxData[0] > 57);

		sTime.Seconds = atoi(TimerSec);

		ReadRS(RS232, 1, 1);

	}while(rxData[0] != '\r' || sTime.Seconds < 0 || sTime.Seconds > 59);

	if(TimerSec[0] > 0 || TimerSec[1] > 0)
	{
		TimerSec[0] = 0;
		TimerSec[1] = 0;
	}
}

void RemountSD(void)
{
	if(f_mount(&myFATFS, SDPath, 1) != FR_OK)
	{
		MX_FATFS_DeInit();
		MX_FATFS_Init();
	}
}

void Flash_Write(uint32_t Flash_Address, uint32_t Flash_Data)
{
	HAL_FLASH_Unlock();
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Flash_Address, Flash_Data);
	HAL_FLASH_Lock();
}

uint32_t Flash_Read(uint32_t Flash_Address)
{
	uint32_t Flash_Data;

	Flash_Data = *(uint32_t*) Flash_Address;

	return Flash_Data;
}

void Flash_Erase_SectorSeven(void)
{
	HAL_FLASH_Unlock();
	FLASH_Erase_Sector(FLASH_SECTOR_7, VOLTAGE_RANGE_3);
	HAL_FLASH_Lock();
}

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
