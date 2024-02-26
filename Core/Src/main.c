/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */
uint8_t send_timer[8] = {0xA5, 0x5A, 0x05, 0x82, 0x03, 0x53, 0x00, 0x00};
uint8_t send_minutes1[8] = {0xA5, 0x5A, 0x05, 0x82, 0x03, 0x2D, 0x00, 0x00};
uint8_t send_hour1[8] = {0xA5, 0x5A, 0x05, 0x82, 0x03, 0x29, 0x00, 0x00};
uint8_t send_minutes2[8] = {0xA5, 0x5A, 0x05, 0x82, 0x03, 0x35, 0x00, 0x00};
uint8_t send_hour2[8] = {0xA5, 0x5A, 0x05, 0x82, 0x03, 0x31, 0x00, 0x00};

unsigned int time_minutes;

uint8_t manipul1_time;
uint8_t manipul2_time;

uint8_t wakeup_count_minutes;
uint8_t wakeup_count_hour;

unsigned int duty;
uint8_t rev;
unsigned int led;
uint8_t manipul1 = 0;
uint8_t manipul2 = 0;
uint8_t start = 0;
uint8_t stop = 0;
uint8_t day;
uint8_t month;
uint8_t year;
uint8_t hour;
uint8_t minute;

uint16_t DataBuffer[1] = {0};

uint8_t str[] = "TEST RESPONSE UART\r\n\0";
uint16_t var;

uint8_t USART_RX_STA = 0;
uint8_t USART_RX_END = 0;
uint8_t UART_RX_BUF[9] = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */
RTC_AlarmTypeDef sAlarm;
RTC_TimeTypeDef sTime;
RTC_DateTypeDef sDate;
uint8_t test = 10;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Clean_Buffer(void)
{
	UART_RX_BUF[0] = 0;
	UART_RX_BUF[1] = 0;
	UART_RX_BUF[2] = 0;
	UART_RX_BUF[3] = 0;
	UART_RX_BUF[4] = 0;
	UART_RX_BUF[5] = 0;
	UART_RX_BUF[6] = 0;
	UART_RX_BUF[7] = 0;
	UART_RX_BUF[8] = 0;
}


void Manipule_choose(void)         //Choose the working manipules: 1, 2 or both
{

	switch(UART_RX_BUF[5])
	{
		case 0x82:
			manipul1 = 1;
			manipul2 = 0;
			break;
		case 0x83:
			manipul2 = 1;
			manipul1 = 0;
			break;
		case 0x84:
			manipul1 = 1;
			manipul2 = 1;
			break;
		default: break;			
	}
}



uint8_t speed_send[8] = {0xA5, 0x5A, 0x05, 0x82, 0x03, 0x57, 0x00, 0x00};	
void Manipule_pressset(void)      //Pressets of motor speed; speed_send data send to display to indicate speed info
{	
	switch(UART_RX_BUF[5])
	{
		case 0x7F:			//30%
			speed_send[7] = 0x1E;	
			duty = 15000;
			led = 0xF00;
			HAL_UART_Transmit_DMA(&huart1, speed_send, 8);
		break;
		case 0x80:										//50%
			speed_send[7] = 0x32;
			duty = 25000;
			led = 0xFC00;
			HAL_UART_Transmit_DMA(&huart1, speed_send, 8);
		break;
		case 0x81:										//70%
			speed_send[7] = 0x46;
			duty = 35000;
			led = 0xFF00;
			HAL_UART_Transmit_DMA(&huart1, speed_send, 8);
		break;
		default: break;
	}
}

void PWM_Speed(void)            //PWM Motor Speed change depend on value of display indication
{
	switch(UART_RX_BUF[5])
	{
		case 0x57:
	switch(UART_RX_BUF[8])
	{
		case 0x00:									//0% speed, Duty Cycle 2%, LED indication 1
			duty = 0;
			led = 0x8000;
			speed_send[7] = 0x00;
			break;
		case 0x0A:									//10% speed, Duty Cycle 10%, LED indication 2		
			duty = 5000;
			led = 0xC000;
			speed_send[7] = 0x0A;
			break;
		case 0x14:									//20% speed, Duty Cycle 20%, LED indication 3		
			duty = 10000;
			led = 0xE000;
			speed_send[7] = 0x14;
			break;
		case 0x1E:									//30% speed, Duty Cycle 30%, LED indication 4		
			duty = 15000;
			led = 0xF00;
			speed_send[7] = 0x1E;
			break;
		case 0x28:									//40% speed, Duty Cycle 40%, LED indication 5			
			duty = 20000;
			led = 0xF800;
			speed_send[7] = 0x28;
			break;
		case 0x32:									//50% speed, Duty Cycle 50%, LED indication 6		
			duty = 25000;
			led = 0xFC00;
			speed_send[7] = 0x32;
			break;
		case 0x3C:									//60% speed, Duty Cycle 60%, LED indication 7		
			duty = 30000;
			led = 0xFE00;
			speed_send[7] = 0x3C;
			break;
		case 0x46:									//70% speed, Duty Cycle 70%, LED indication 8		
			duty = 35000;
			led = 0xFF00;
			speed_send[7] = 0x46;
			break;
		case 0x50:									//80% speed, Duty Cycle 80%, LED indication 9
			duty = 40000;
			led = 0xFF80;
			speed_send[7] = 0x50;
			break;
		case 0x5A:									//90% speed, Duty Cycle 90%, LED indication 10
			duty = 45000;
			led = 0xFFC0;
			speed_send[7] = 0x5A;
			break;
		default: break;
	}
	break;
	default: break;
}
}


void Manipule_status(void)			//Manipule start/stop status
{

	switch(UART_RX_BUF[5])
	{
		case 0x86:
			HAL_TIM_Base_Start(&htim1); 
		if (time_minutes != 0){
			if (manipul1 == 1 && manipul2 == 0){
				HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); }
			if (manipul2 == 1 && manipul1 == 0) {
				HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2); }
			if (manipul1 == 1 && manipul2 == 1) {
				HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
				HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2); }
			}
				MX_RTC_Init();
				Clean_Buffer();
		start = 1;
		stop = 0;
			break;
		case 0x87:
			if (time_minutes != 0){
			HAL_TIM_Base_Stop(&htim1);
			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);	
			}
			HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_A);
			HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
			sAlarm.AlarmTime.Minutes = 0x00;
			start = 0;
			stop = 1;
			Clean_Buffer();
			break;
		default: break;
	}
	/*
	if (start == 1 && stop == 0){
					HAL_TIM_Base_Start(&htim1); 
			if (manipul1 == 1 && manipul2 == 0){
				HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); }
			if (manipul2 == 1 && manipul1 == 0) {
				HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2); }
			if (manipul1 == 1 && manipul2 == 1) {
				HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
				HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2); }
			start = 1;
			stop = 0;
			}
	if (start == 0 && stop == 1) {
			HAL_TIM_Base_Stop(&htim1);
			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
			start = 0;
			stop = 1;
	}
	*/
}

void PWM_Duty(void)			//PWM Motor speed setting signal duty cycle
{
	if (start == 1 && stop == 0){
			__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, duty);
			__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, duty);
	}
}

void Reverse_motor (void)		//Reverse function by clicking the button on display
{	
		
		switch(UART_RX_BUF[5])
		{
			case 0x85:
			UART_RX_BUF[5] = 0;
				rev++;
				if (rev > 1){
				rev = 0;
				}
			break;
			case 0x8B:
			UART_RX_BUF[5] = 0;
				rev++;
				if (rev > 1){
				rev = 0;
			}
			default: break;
		}				  
}

void Reverse_Enable (void)
{
	if (rev == 1) {
		HAL_GPIO_WritePin(GPIOA, Motor1_dir_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, Motor2_dir_Pin, GPIO_PIN_SET);   //BLDC motor clockwise direction
	}
	if (rev == 0) {
		HAL_GPIO_WritePin(GPIOA, Motor1_dir_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOA, Motor2_dir_Pin, GPIO_PIN_RESET);		//BLDC motor other direction
	}
}

void SPI_LED_send(uint16_t led)			//LED indication trought SPI interface
{
		DataBuffer[0] = led;
			
		HAL_SPI_Transmit(&hspi1, (uint8_t*)DataBuffer, 1, 5000);
		HAL_Delay(1000);
		HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
			
		HAL_SPI_Transmit(&hspi2, (uint8_t*)DataBuffer, 1, 5000);
		HAL_Delay(1000);
		HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
}


uint8_t RTC_set_send[13] = {0xA5, 0x5A, 0x0A, 0x80, 0x1F, 0x5A, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x01};


void RTC_set(void)
{
	RTC_set_send[6] = year;
	RTC_set_send[7] = month;
	RTC_set_send[8] = day;
	RTC_set_send[10] = hour;
	RTC_set_send[11] = minute;


	switch(UART_RX_BUF[5])
	{
		case 0x05:
			UART_RX_BUF[5] = 0;
			day = UART_RX_BUF[8];
			Clean_Buffer();
			break;
		case 0x19:
			UART_RX_BUF[5] = 0;
			month = UART_RX_BUF[8];
			Clean_Buffer();
			break;
		case 0x1D:
			UART_RX_BUF[5] = 0;
			year = UART_RX_BUF[8];			
			Clean_Buffer();
			break;
		case 0x21:
			UART_RX_BUF[5] = 0;
			hour = UART_RX_BUF[8];
			Clean_Buffer();
			break;
		case 0x25:
			UART_RX_BUF[5] = 0;
			minute = UART_RX_BUF[8];
		  Clean_Buffer();
			break;
		case 0x89:
			HAL_UART_Transmit_DMA(&huart1, RTC_set_send, 13);
			Clean_Buffer();
			break;
		default: break;
	}
	
}	

void Timer_Set(void)
{

	switch(UART_RX_BUF[5])
	{
		case 0x53:
				time_minutes = UART_RX_BUF[8];
				break;
		default: break;
	}
}

void Time_Counter(void)
{
	if (manipul1 == 1 && manipul2 == 0)
	{
		send_minutes1[7] = wakeup_count_minutes;
		send_hour1[7] = wakeup_count_hour;
		HAL_UART_Transmit_DMA(&huart1, send_minutes1, 8);
		HAL_UART_Transmit_DMA(&huart1, send_hour1, 8);
	}
	if (manipul1 == 0 && manipul2 == 1)
	{
		send_minutes2[7] = wakeup_count_minutes;
		send_hour2[7] = wakeup_count_hour;
		HAL_UART_Transmit_DMA(&huart1, send_minutes2, 8);
		HAL_UART_Transmit_DMA(&huart1, send_hour2, 8);
	}
	if (manipul1 == 1 && manipul2 == 1)
	{
		send_minutes1[7] = wakeup_count_minutes;
		send_hour1[7] = wakeup_count_hour;
		send_minutes2[7] = wakeup_count_minutes;
		send_hour2[7] = wakeup_count_hour;
		HAL_UART_Transmit_DMA(&huart1, send_minutes1, 8);
		HAL_UART_Transmit_DMA(&huart1, send_hour1, 8);		
		HAL_UART_Transmit_DMA(&huart1, send_minutes2, 8);
		HAL_UART_Transmit_DMA(&huart1, send_hour2, 8);		
		
	}
		
	 
	  
	 
}


void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc)
{
	wakeup_count_minutes++;
	if (time_minutes > 0){
	time_minutes--;
	}
	if (wakeup_count_minutes == 60){
		wakeup_count_minutes = 0;
		wakeup_count_hour++;
	}
}


uint8_t alarm_test;
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
	
	sAlarm.AlarmTime.Minutes = 0;
	sAlarm.Alarm = RTC_ALARM_A;
	HAL_RTC_SetAlarm_IT(hrtc, &sAlarm, RTC_FORMAT_BIN);
	//time_minutes = 0;
	send_timer[7] = time_minutes;
	HAL_UART_Transmit_DMA(&huart1, send_timer, 8);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);	
	alarm_test = 1;	
}


void HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart)
{
  
  HAL_UART_Receive_DMA(&huart1, UART_RX_BUF, 9);

}

uint8_t speed;
uint8_t duty_button;
uint8_t counter;
uint32_t time_key1_current = 0;
uint32_t time_key1_previous = 0;
uint8_t speed_send_button[8] = {0xA5, 0x5A, 0x05, 0x82, 0x03, 0x57, 0x00, 0x00};	
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
		time_key1_current = HAL_GetTick();
									/* BUTTON 1,2 REVERSE */	
    if (GPIO_Pin == GPIO_PIN_5 && (time_key1_current - time_key1_previous > 50)){		
			rev++;
			if (rev > 1){
				rev = 0;
			}
			time_key1_previous = time_key1_current;
		}
    if (GPIO_Pin == GPIO_PIN_0 && (time_key1_current - time_key1_previous > 50)){		
			rev++;
			if (rev > 1){
				rev = 0;
			}
			time_key1_previous = time_key1_current;
		}
									/* BUTTON 1,2 PLUS */		
    if (GPIO_Pin == GPIO_PIN_3 && (time_key1_current - time_key1_previous > 50)){		
			UART_RX_BUF[5] = 0;
			UART_RX_BUF[8] = 0;
			counter++;
			duty = duty + 5000;
			speed_send[7] = speed_send[7] + 10;
			time_key1_previous = time_key1_current;
		}		
    if (GPIO_Pin == GPIO_PIN_6 && (time_key1_current - time_key1_previous > 50)){		
			UART_RX_BUF[5] = 0;
			UART_RX_BUF[8] = 0;
			counter++;
			duty = duty + 5000;
			speed_send[7] = speed_send[7] + 10;
			time_key1_previous = time_key1_current;
		}
									/* BUTTON 1,2 MINUS */		
    if (GPIO_Pin == GPIO_PIN_4 && (time_key1_current - time_key1_previous > 50)){		
			UART_RX_BUF[5] = 0;
			UART_RX_BUF[8] = 0;
			counter--;
			duty = duty - 5000;
			speed_send[7] = speed_send[7] - 10;
			time_key1_previous = time_key1_current;
		}
    if (GPIO_Pin == GPIO_PIN_7 && (time_key1_current - time_key1_previous > 50)){		
			UART_RX_BUF[5] = 0;
			UART_RX_BUF[8] = 0;
			counter--;
			duty = duty - 5000;
			speed_send[7] = speed_send[7] - 10;
			time_key1_previous = time_key1_current;
		}		
}
		

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

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
	
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
	
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		HAL_UART_Receive_DMA(&huart1, UART_RX_BUF, 9);	
		
		
		Timer_Set();
		Manipule_status();		
		var = HAL_RTCEx_GetWakeUpTimer(&hrtc);
		
		
		Manipule_choose();
		Manipule_pressset();
		PWM_Speed();
		PWM_Duty();
		Reverse_motor();
		Reverse_Enable();
		
		RTC_set();
		
		Timer_Set();
		//Time_Counter();	
	

		send_timer[7] = time_minutes;
		HAL_UART_Transmit_DMA(&huart1, send_timer, 8);
	
		if (speed_send[7] > 90) {speed_send[7] = 90;}
		if (duty > 45000) {duty = 45000;}
		if (speed_send[7] <= 0) {speed_send[7] = 0;}
		if (duty <= 0) {duty = 0;}		
		HAL_UART_Transmit_DMA(&huart1, speed_send, 8);

	}
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
} 

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0;
  sTime.Minutes = 0;
  sTime.Seconds = 0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 1;
  sDate.Year = 0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable the Alarm A
  */
  sAlarm.AlarmTime.Hours = 0;
  sAlarm.AlarmTime.Minutes = time_minutes;
  sAlarm.AlarmTime.Seconds = 0;
  sAlarm.AlarmTime.SubSeconds = 0;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sAlarm.AlarmMask = RTC_ALARMMASK_NONE;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay = 1;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable the WakeUp
  */
  if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 60, RTC_WAKEUPCLOCK_CK_SPRE_16BITS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */
  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 2-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 50000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Motor1_dir_Pin|Motor2_dir_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SPI2_CS_Pin|SPI1_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Motor1_dir_Pin Motor2_dir_Pin */
  GPIO_InitStruct.Pin = Motor1_dir_Pin|Motor2_dir_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Button1_Plus_Pin Button1_Minus_Pin Button1_Rev_Pin Button2_Plus_Pin
                           Button2_Minus_Pin */
  GPIO_InitStruct.Pin = Button1_Plus_Pin|Button1_Minus_Pin|Button1_Rev_Pin|Button2_Plus_Pin
                          |Button2_Minus_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Button2_Rev_Pin */
  GPIO_InitStruct.Pin = Button2_Rev_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Button2_Rev_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI2_CS_Pin SPI1_CS_Pin */
  GPIO_InitStruct.Pin = SPI2_CS_Pin|SPI1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

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
