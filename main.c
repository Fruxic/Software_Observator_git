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
uint8_t RS_Choice_Func(void);
uint8_t RS_Poort_Func(void);
uint8_t RS_Sensor_Func(void);
uint8_t SDi_Poort_Func(void);
uint8_t SDi_Sensor_Func(void);
uint8_t Switch_12V_Func(void);
uint8_t Poort_12V_Func(void);
uint8_t Switch_Relay_Func(void);
uint8_t Poort_Relay_Func(void);
uint8_t Create_File_Func(void);
void EmptyBuffer(void);
void SetTime(void);
void RemountSD(void);
void Flash_Write(uint32_t Flash_Address, uint32_t Flash_Data);
uint32_t Flash_Read(uint32_t Flash_Address);
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
	uint8_t Timer2 = 0; //local Timer
	uint8_t RS_Poort; //variable voor keuze RS protocol
	uint8_t RS_Choice; //
	uint8_t SDi; //Varaible voor keuze SDI poort aansluiting
	uint8_t PowerSwitch_12V;
	uint8_t PowerSwitch_Relay;
	uint8_t Switch_12V;
	uint8_t Switch_Relay;
	uint8_t Sensor_RS = 0; //Variable voor sensor keuze op RS poort.
	uint8_t Sensor_SDi = 0; //Variable voor sensor keuze op SDi12 poort.
	uint8_t File = 0;
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

  HAL_TIM_Base_Start_IT(&htim2);
  Timer = 0; //Timer reset.
  RS_Choice = Flash_Read(0x08060000);
  RS_Poort = Flash_Read(0x08060010);
  Sensor_RS = Flash_Read(0x08060020);
  SDi = Flash_Read(0x08060030);
  Sensor_SDi = Flash_Read(0x08060040);
  PowerSwitch_12V = Flash_Read(0x08060050);
  Switch_12V = Flash_Read(0x080600a0);
  PowerSwitch_Relay = Flash_Read(0x08060060);
  Switch_Relay = Flash_Read(0x080600b0);
  HAL_PWR_EnableBkUpAccess();
  HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0);
  HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1);
  HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR2);
  HAL_PWR_DisableBkUpAccess();
  HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
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

	  if(rxData[0] == '\r')
	  {
		  LED = 0;
		  WriteRS(1, "\x1b[1J"); //Clear screen
		  WriteRS(1, "\x1b[f"); //Move cursor to upper left corner

		  //De gebruiker moet alle sensoren eerst aansluiten voordat de gebruiker kan configureren.
		  Start_Up();

		  //De RTC configureren en initialiseren
		  sDate.Date = 7;
		  sDate.Month = RTC_MONTH_JANUARY;
		  sDate.WeekDay = RTC_WEEKDAY_TUESDAY;
		  sDate.Year = 20;
		  HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

		  WriteRS(1, "Stel de tijd in voor je RTC\r\n");
		  SetTime();
		  HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
		  HAL_PWR_EnableBkUpAccess();
		  HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR0, sTime.Seconds);
		  HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, sTime.Minutes);
		  HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR2, sTime.Hours);
		  HAL_PWR_DisableBkUpAccess();
		  WriteRS(1, "\x1b[1J"); //Clear screen
		  WriteRS(1, "\x1b[f"); //Move cursor to upper left corner

		  //Kiezen voor een RS aansluiting
		  RS_Choice = RS_Choice_Func();
	      Flash_Write(0x08060000, RS_Choice);
		  WriteRS(1, "\x1b[1J"); //Clear screen
		  WriteRS(1, "\x1b[f"); //Move cursor to upper left corner
	      //Kiezen tussen de RS-232, RS-422 en RS-485
		  if(RS_Choice == 1)
		  {
			 RS_Poort = RS_Poort_Func();
			 Flash_Write(0x08060010, RS_Poort);
			 WriteRS(1, "\x1b[1J"); //Clear screen
			 WriteRS(1, "\x1b[f"); //Move cursor to upper left corner
			 //Kiezen welke sensor de gebruiker wilt lezen op de gekozen RS poort.
			 Sensor_RS = RS_Sensor_Func();
			 Flash_Write(0x08060020, Sensor_RS);
			 WriteRS(1, "\x1b[1J"); //Clear screen
			 WriteRS(1, "\x1b[f"); //Move cursor to upper left corner
		  }

		  //Kiezen voor een SDi12 aansluiting
		  SDi = SDi_Poort_Func();
		  Flash_Write(0x08060030, SDi);
		  WriteRS(1, "\x1b[1J"); //Clear screen
		  WriteRS(1, "\x1b[f"); //Move cursor to upper left corner
		  //Kiezen welke sensor de gebruiker wilt lezen op de SDi12 poort.
		  if(SDi == 1)
		  {
			  Sensor_SDi = SDi_Sensor_Func();
			  Flash_Write(0x08060040, Sensor_SDi);
			  WriteRS(1, "\x1b[1J"); //Clear screen
			  WriteRS(1, "\x1b[f"); //Move cursor to upper left corner
		  }

		  //Kiezen of de gebruiker een voeding nodig heeft voor zijn sensor
		  PowerSwitch_12V = Switch_12V_Func();
		  Flash_Write(0x08060050, PowerSwitch_12V);
		  WriteRS(1, "\x1b[1J"); //Clear screen
		  WriteRS(1, "\x1b[f"); //Move cursor to upper left corner
		  //Kiezen op welke poort deze sensor zit aangesloten
		  if(PowerSwitch_12V == 1)
		  {
			  Switch_12V = Poort_12V_Func();
			  Flash_Write(0x080600a0, Switch_12V);
			  WriteRS(1, "\x1b[1J"); //Clear screen
			  WriteRS(1, "\x1b[f"); //Move cursor to upper left corner
		  }

		  //Kiezen of de gebruiker een sensor wilt togglen
		  PowerSwitch_Relay = Switch_Relay_Func();
		  Flash_Write(0x08060060, PowerSwitch_Relay);
		  WriteRS(1, "\x1b[1J"); //Clear screen
		  WriteRS(1, "\x1b[f"); //Move cursor to upper left corner
		  //Kiezen op welke poort de sensor zit aangesloten om te togglen
		  Switch_Relay = Flash_Read(0x080600b0);
		  if(PowerSwitch_Relay == 1)
		  {
			  Switch_Relay = Poort_Relay_Func();
			  Flash_Write(0x080600b0, Switch_Relay);
			  WriteRS(1, "\x1b[1J"); //Clear screen
			  WriteRS(1, "\x1b[f"); //Move cursor to upper left corner
		  }
		  else
		  {
			  //Relay reset
			  GPIOC -> ODR |= GPIO_PIN_2;
			  GPIOC -> ODR |= GPIO_PIN_4;
			  HAL_Delay(5);
			  GPIOC -> ODR &= ~GPIO_PIN_2;
			  GPIOC -> ODR &= ~GPIO_PIN_4;
		  }

		  //Bestand aanmaken
		  File = Create_File_Func();
		  Flash_Write(0x080600c0, File);
		  WriteRS(1, "\x1b[1J"); //Clear screen
		  WriteRS(1, "\x1b[f"); //Move cursor to upper left corner

		  WriteRS(1, "Programma start\r\n");

		  Timer = 0; //Timer reset.
		  LED = 1;
	  }
	  EmptyBuffer();
	  RemountSD();

	  if(PowerSwitch_12V == 1)
	  {
		  PowerSwitch_12V = 0;
		  if(Switch_12V == 1)
		  {
			  GPIOA -> ODR |= GPIO_PIN_1;
		  }
		  else if(Switch_12V == 2)
		  {
			  GPIOA -> ODR |= GPIO_PIN_0;
		  }
		  else if(Switch_12V == 3)
		  {
			  GPIOA -> ODR |= GPIO_PIN_1;
			  GPIOA -> ODR |= GPIO_PIN_0;
		  }
	  }

	  if(PowerSwitch_Relay == 1)
	  {
		  PowerSwitch_Relay = 0;
		  if(Switch_Relay == 1)
		  {
			  GPIOC -> ODR |= GPIO_PIN_1;
			  HAL_Delay(5);
			  GPIOC -> ODR &= ~GPIO_PIN_1;
			  //Relay reset
			  GPIOC -> ODR |= GPIO_PIN_2;
			  HAL_Delay(5);
			  GPIOC -> ODR &= ~GPIO_PIN_2;
		  }
		  else if(Switch_Relay == 2)
		  {
			  GPIOC -> ODR |= GPIO_PIN_0;
			  HAL_Delay(5);
			  GPIOC -> ODR &= ~GPIO_PIN_0;
			  //Relay reset
			  GPIOC -> ODR |= GPIO_PIN_4;
			  HAL_Delay(5);
			  GPIOC -> ODR &= ~GPIO_PIN_4;
		  }
		  else if(Switch_Relay == 3)
		  {
			  GPIOC -> ODR |= GPIO_PIN_1;
			  GPIOC -> ODR |= GPIO_PIN_0;
			  HAL_Delay(5);
			  GPIOC -> ODR &= ~GPIO_PIN_1;
			  GPIOC -> ODR &= ~GPIO_PIN_0;
		  }
	  }

	  if(Timer == 1 && Timer2 == 0) //Internal meusurement
	  {
		  MeasureLogInternal();
		  Timer2++;
	  }
	  else if(Timer == 2)//external meusurement
	  {
		  if(RS_Choice == 1)
		  {
			  SaveRS(FileName_Measure, RS_Poort, Sensor_RS);
		  }

		  if(SDi == 1)
		  {
			 SaveSDi(FileName_Measure, Sensor_SDi);
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

void Start_Up(void)
{
	  uint8_t RS232 = 1;
	  WriteRS(1, "Sluit eerst alles aan op de datalogger en druk dan op enter\r\n");
	  do
	  {
		  EmptyBuffer();
		  ReadRS(RS232, 1, 1);
	  }while(rxData[0] != '\r');
	  HAL_FLASH_Unlock();
	  FLASH_Erase_Sector(FLASH_SECTOR_7, VOLTAGE_RANGE_3);
	  HAL_FLASH_Lock();
	  EmptyBuffer();
}

uint8_t RS_Choice_Func(void)
{
	  uint8_t RS_Choice = 0;
	  uint8_t RS232 = 1;
	  WriteRS(1, "Heeft U een sensor die U op een een RS poort wilt aansluiten?\r\n");
	  HAL_Delay(200);
	  WriteRS(1, "type 'j' voor JA\r\n");
	  HAL_Delay(200);
	  WriteRS(1, "type 'n' voor NEE\r\n");
	  do
	  {
		  EmptyBuffer();
		  ReadRS(RS232, 1, 1);
		  WriteRS(1, rxData);
		  switch(rxData[0])
		  {
		  case 'j':
			  RS_Choice = 1;
			  break;
		  case 'n':
			  RS_Choice = 2;
			  break;
		  }
	  }while(rxData[0] != '\r' || RS_Choice >= 3 || RS_Choice == 0);

	  return RS_Choice;
}

uint8_t RS_Poort_Func(void)
{
	  uint8_t RS232 = 1;
	  uint8_t RS_Poort = 0;
	  WriteRS(1, "Welke RS protocol wilt U lezen?\r\n");
	  HAL_Delay(200);
	  WriteRS(1, "type '1' voor RS-232\r\n");
	  HAL_Delay(200);
	  WriteRS(1, "type '2' voor RS-422\r\n");
	  HAL_Delay(200);
	  WriteRS(1, "type '3' voor RS-485\r\n");
	  do
	  {
		  EmptyBuffer();
		  ReadRS(RS232, 1, 1);
		  WriteRS(1, rxData);
		  switch(rxData[0])
		  {
		  	  case '1':
		  		  RS_Poort = 1;
		  		  break;
		  	  case '2':
		  		  RS_Poort = 2;
		  		  break;
		  	  case '3':
		  		  RS_Poort = 3;
		  		  break;
		  }
	  }while(rxData[0] != '\r' || RS_Poort >= 4 || RS_Poort == 0);

	  return RS_Poort;
}

uint8_t RS_Sensor_Func(void)
{
	  uint8_t RS232 = 1;
	  uint8_t Sensor_RS = 0;
	  WriteRS(1, "Welke Sensor wilt u lezen?\r\n");
	  HAL_Delay(200);
	  WriteRS(1, "type '1' voor PT12\r\n");
	  HAL_Delay(200);
	  WriteRS(1, "type '2' voor OMC-160-3\r\n");
	  do
	  {
		  EmptyBuffer();
		  ReadRS(RS232, 1, 1); //Lezen op poort 1 met de RS232 protocol geconfigureerd
		  WriteRS(1, rxData); //Schrijven naar poort 1 met de data die gelezen is van de gebruiker
		  switch(rxData[0])
		  {
		  case '1':
			  Sensor_RS = 1;
			  break;
		  case '2':
			  Sensor_RS = 2;
			  break;
		  }
	  }while(rxData[0] != '\r' || Sensor_RS >= 3 || Sensor_RS == 0);
	  return Sensor_RS;
}

uint8_t SDi_Poort_Func(void)
{
	  uint8_t RS232 = 1;
	  uint8_t SDi = 0;
	  WriteRS(1, "Heeft U ook een SDi12 sensor?\r\n");
	  HAL_Delay(200);
	  WriteRS(1, "type 'j' voor JA\r\n");
	  HAL_Delay(200);
	  WriteRS(1, "type 'n' voor NEE\r\n");
	  do
	  {
		  EmptyBuffer();
		  ReadRS(RS232, 1, 1);
		  WriteRS(1, rxData);
		  switch(rxData[0])
		  {
		  case 'j':
			  SDi = 1;
			  break;
		  case 'n':
			  SDi = 2;
			  break;
		  }
	  }while(rxData[0] != '\r' || SDi >= 3 || SDi == 0);
	  return SDi;
}

uint8_t SDi_Sensor_Func(void)
{
	  uint8_t RS232 = 1;
	  uint8_t Sensor_SDi = 0;
	  WriteRS(1, "Welke Sensor wilt u lezen?\r\n");
	  HAL_Delay(200);
	  WriteRS(1, "type '1' voor PT12\r\n");
	  HAL_Delay(200);
	  WriteRS(1, "type '2' voor OMC-160-3\r\n");
	  do
	  {
		  EmptyBuffer();
		  ReadRS(RS232, 1, 1); //Lezen op poort 1 met de RS232 protocol geconfigureerd
		  WriteRS(1, rxData); //Schrijven naar poort 1 met de data die gelezen is van de gebruiker
		  switch(rxData[0])
		  {
		  case '1':
			  Sensor_SDi = 1;
			  break;
		  case '2':
			  Sensor_SDi = 2;
			  break;
		  }
	  }while(rxData[0] != '\r' || Sensor_SDi >= 3 || Sensor_SDi == 0);
	  return Sensor_SDi;
}

uint8_t Switch_12V_Func(void)
{
	  uint8_t RS232 = 1;
	  uint8_t PowerSwitch = 0;
	  WriteRS(1, "Heeft U een sensor of meerdere die gevoed moeten worden met 12VDC?\r\n");
	  HAL_Delay(200);
	  WriteRS(1, "type 'j' voor JA\r\n");
	  HAL_Delay(200);
	  WriteRS(1, "type 'n' voor NEE\r\n");
	  do
	  {
		  EmptyBuffer();
		  ReadRS(RS232, 1, 1);
		  WriteRS(1, rxData);
		  switch(rxData[0])
		  {
		  case 'j':
			  PowerSwitch = 1;
			  break;
		  case 'n':
			  PowerSwitch = 2;
			  break;
		  }
	  }while(rxData[0] != '\r' || PowerSwitch >= 3 || PowerSwitch == 0);
	  return PowerSwitch;
}

uint8_t Poort_12V_Func(void)
{
	  uint8_t RS232 = 1;
	  uint8_t Switch = 0;
	  WriteRS(1, "Op welke poort heeft u deze aangesloten?\r\n");
	  HAL_Delay(200);
	  WriteRS(1, "type '1' voor poort 1\r\n");
	  HAL_Delay(200);
	  WriteRS(1, "type '2' voor poort 2\r\n");
	  HAL_Delay(200);
	  WriteRS(1, "type '3' voor beide poorten\r\n");
	  do
	  {
		  EmptyBuffer();
		  ReadRS(RS232, 1, 1);
		  WriteRS(1, rxData);
		  switch(rxData[0])
		  {
		  case '1':
			  Switch = 1;
			  break;
		  case '2':
			  Switch = 2;
			  break;
		  case '3':
			  Switch = 3;
			  break;
		  }
	  }while(rxData[0] != '\r' || Switch >= 4 || Switch == 0);
	  return Switch;
}

uint8_t Switch_Relay_Func(void)
{
	  uint8_t RS232 = 1;
	  uint8_t PowerSwitch = 0;
	  WriteRS(1, "Heeft U een sensor of meerdere die u wilt togglen?\r\n");
	  HAL_Delay(200);
	  WriteRS(1, "type 'j' voor JA\r\n");
	  HAL_Delay(200);
	  WriteRS(1, "type 'n' voor NEE\r\n");
	  do
	  {
		  EmptyBuffer();
		  ReadRS(RS232, 1, 1);
		  WriteRS(1, rxData);
		  switch(rxData[0])
		  {
		  case 'j':
			  PowerSwitch = 1;
			  break;
		  case 'n':
			  PowerSwitch = 2;
			  break;
		  }
	  }while(rxData[0] != '\r' || PowerSwitch >= 3 || PowerSwitch == 0);
	  return PowerSwitch;
}

uint8_t Poort_Relay_Func(void)
{
	  uint8_t RS232 = 1; //variable voor Enable RS-232 protocol
	  uint8_t Switch = 0;
	  WriteRS(1, "Op welke poort heeft u deze aangesloten?\r\n");
	  HAL_Delay(200);
	  WriteRS(1, "type '1' voor poort 1\r\n");
	  HAL_Delay(200);
	  WriteRS(1, "type '2' voor poort 2\r\n");
	  HAL_Delay(200);
	  WriteRS(1, "type '3' voor beide poorten\r\n");
	  do
	  {
		  EmptyBuffer();
		  ReadRS(RS232, 1, 1);
		  WriteRS(1, rxData);
		  switch(rxData[0])
		  {
		  case '1':
			  Switch = 1;
			  break;
		  case '2':
			  Switch = 2;
			  break;
		  case '3':
			  Switch = 3;
			  break;
		  }
	  }while(rxData[0] != '\r' || Switch >= 4 || Switch == 0);
	  return Switch;
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

	  /*do
	  {
		  EmptyBuffer();
		  ReadRS(RS, 2, 350);
	  }while(rxData[0] != 36);*/

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

	  /*for(int x = 1; x <= 39; x++)
	  {
		  if(rxData[x] == 36)
		  {
			  for(int y = x; y <= 39; y++)
			  {
				  rxData[y] = 0;
			  }
			  break;
		  }
	  }*/

	  HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	  HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
      HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR0, sTime.Hours);
      HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, sTime.Minutes);
      HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR2, sTime.Seconds);

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
		  WriteRS(1, rxData);

		  GPIOB -> ODR |= GPIO_PIN_10;

		  HAL_Delay(1700);
		  HAL_LIN_SendBreak(&huart1);
		  HAL_Delay(20);
		  WriteSDi("0D0!", 100);

		  GPIOB -> ODR &= ~GPIO_PIN_10;

		  EmptyBuffer();
		  ReadSDi(300);
		  WriteRS(1, rxData);

		  HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
		  HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
	      HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR0, sTime.Hours);
	      HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, sTime.Minutes);
	      HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR2, sTime.Seconds);

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
	  char TimerHour[2];
	  char TimerMin[2];
	  char TimerSec[2];
	  //Uren instellen
	  if(TimerHour[0] > 0 || TimerHour[1] > 0)
	  {
		  TimerHour[0] = 0;
		  TimerHour[1] = 0;
	  }

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
	  if(TimerMin[0] > 0 || TimerMin[1] > 0)
	  {
		  TimerMin[0] = 0;
		  TimerMin[1] = 0;
	  }

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
	  if(TimerSec[0] > 0 || TimerSec[1] > 0)
	  {
		  TimerSec[0] = 0;
		  TimerSec[1] = 0;
	  }

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
