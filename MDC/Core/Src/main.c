/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>

#include "define.h"
#include "spi.h"
#include "can.h"

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
CAN_HandleTypeDef hcan;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
bool g_main_loop_flag;

typedef struct{
	bool battery;
	bool command;
	bool speed;
	bool spi_timeout;
	bool power_timeout;
	bool locker_timeout;
}Error;

bool isNoError(Error e){
	return (e.battery == false)
			&& (e.command == false)
			&& (e.speed == false)
			&& (e.spi_timeout == false)
			&& (e.power_timeout == false)
			&& (e.locker_timeout == false);
}

bool errorReleaseRequest(bool sw){
	static bool last_sw = false;
	bool request;
	request = (last_sw == true) && (sw == false);
	last_sw = sw;
	return request;
}



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim6)
	{
		if(g_main_loop_flag){
			printf("Control cycle is slow\r\n");
		}
		else{
			g_main_loop_flag = true;
			//printf("send state : %d\r\n",hcan.State);
			sendCanData();
		}
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
  MX_USART2_UART_Init();
  MX_CAN_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(100);
  	Error error = {0};
	PIDController pid;
	pid.kp = 5.0f;
	pid.ki = 0.0f;
	pid.kd = 0.0f;
	pid.kf = 3.0f;
	pid.integral_l = 0.0f;
	pid.integral_r = 0.0f;
	pid.prev_error_l = 0.0f;
	pid.prev_error_r = 0.0f;
	spiInit(&hspi1);
	canInit(&hcan);
	printf("end state : %d\r\n",hcan.State);
	HAL_TIM_Encoder_Start( &htim2, TIM_CHANNEL_ALL );
	HAL_TIM_Encoder_Start( &htim3, TIM_CHANNEL_ALL );
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_Base_Start_IT(&htim6);
	HAL_TIM_Base_Start_IT(&htim7);
	HAL_Delay(100);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	size_t spi_timeout_cnt=0;
	size_t power_timeout_cnt=0;
	size_t locker_timeout_cnt = 0;
	bool control = false;
	bool wait_release_request = false;
  while (1)
  {
  	//wait main_loop_flag
		while(g_main_loop_flag == false){}

		//SPI
		Command command;
		command.can_command.motor_output = false;
		command.can_command.power_off = false;
		command.vel_l = 0.0f;
		command.vel_r = 0.0f;
		Result result = {0};
		if(isSpiUpdated()){
			error.spi_timeout = false;
			spi_timeout_cnt = 0;
		}
		if(spi_timeout_cnt++ >= 5) {
			error.spi_timeout = true;
		}else{
			getSpiData(&command);
		}

		//CAN
		if(isPowerUpdated()) {
			error.power_timeout = false;
			power_timeout_cnt = 0;
		}
		if(power_timeout_cnt++ >= 5) {
			error.power_timeout = true;
		}
		if(error.power_timeout == true) {
			canInit(&hcan);
			printf("power timeout\r\n");
		} else {
			getPowerCanData(&result.power_result);
		}

		if(isLockerUpdated()) {
			error.locker_timeout = false;
			locker_timeout_cnt = 0;
		}
		if(locker_timeout_cnt++ >= 5) {
			error.locker_timeout = true;
		}
		if(error.locker_timeout == true){
			canInit(&hcan);
			printf("locker timeout\r\n");
		} else{
			getLockerCanData(&result.locker_result);
		}

		command.can_command.power_off = false;
		command.can_command.motor_output = false;
		command.can_command.mode = false;
		command.can_command.pull = false;
		command.can_command.release = false;
		setCanData(&command.can_command);

		//encoder
		static int last_cnt_l;
		static int last_cnt_r;
		// todo
		int cnt_l = (int16_t)TIM2->CNT;
		int cnt_r = (int16_t)TIM3->CNT;

		if(abs(last_cnt_l - cnt_l) > 0x8000){
			if (last_cnt_l > 0 && cnt_l < 0 && (last_cnt_l - cnt_l) > 0){
				last_cnt_l -= 0x10000; //オーバーフロー
			}
			else if (last_cnt_l < 0 && cnt_l > 0 && (last_cnt_l - cnt_l) < 0){
				last_cnt_l += 0x10000; //アンダーフロー
			}
		}
		if(abs(last_cnt_r - cnt_r) > 0x8000){
			if (last_cnt_r > 0 && cnt_r < 0 && (last_cnt_r - cnt_r) > 0){
				last_cnt_r -= 0x10000; //オーバーフロー
			}
			else if (last_cnt_r < 0 && cnt_r > 0 && (last_cnt_r - cnt_r) < 0){
				last_cnt_r += 0x10000; //アンダーフロー
			}
		}
		float vel_l = ((float) (cnt_l - last_cnt_l) * ENC_TO_TIRE * RATE) / RESOLUTION;
		float vel_r = -((float) (cnt_r - last_cnt_r) * ENC_TO_TIRE * RATE) / RESOLUTION;
		last_cnt_l = cnt_l;
		last_cnt_r = cnt_r;

		//error check //todo 値決定
		if(abs(command.vel_l) > 5.0f) {
			error.command = true;
			printf("LEFT_COMMAND_ERROR : %f \r\n",command.vel_l);
		} else error.command = false;

		if(abs(command.vel_r) > 5.0f) {
			error.command = true;
			printf("RIGHT_COMMAND_ERROR : %f \r\n",command.vel_r);
		} else error.command = false;

		if(abs(vel_l) > 5.0f){
			error.speed = true;
			printf("LEFT_SPEED_ERROR : %f \r\n",vel_l);
		} else error.speed = false;

		if(abs(vel_r) > 5.0f){
			error.speed = true;
			printf("RIGHT_SPEED_ERROR : %f \r\n",vel_r);
		} else error.speed = false;

		if(result.power_result.i_bat > 80){
			error.battery = true;
			printf("OVER_CURRENT_ERROR : %d \r\n",result.power_result.i_bat);
		} else error.battery = false;

		if(result.power_result.v_bat > 400){
			static size_t n=0;
			if( n++ >10){
				error.battery = true;
				printf("OVER_VOLTAGE_ERROR : %d \r\n",result.power_result.v_bat);
			}else n=0;
		} else error.battery = false;

		if(result.power_result.v_bat < 100){
			static size_t n=0;
			if( n++ >10){
				error.battery = true;
				printf("UNDER_VOLTAGE_ERROR : %d \r\n",result.power_result.v_bat);
			}
			else n=0;
		} else error.battery = false;

		//エラー解除
		if (wait_release_request == true && isNoError(error) == true) {
			if (errorReleaseRequest(result.power_result.emergency)) {
				control = true;
				wait_release_request = false;
			} else {
				control = false;
				HAL_GPIO_TogglePin(Nucleo_LED_GPIO_Port, Nucleo_LED_Pin); //LED
				//printf("wait for Release Request\r\n");
			}
		}

		//コントロール不可
		if(isNoError(error) != true){
			control = false;
			wait_release_request = true;
			HAL_GPIO_WritePin(Nucleo_LED_GPIO_Port, Nucleo_LED_Pin, false); //LED
		}
		if(result.power_result.emergency == true){
			control = false;
			printf("EMERGENCY SWITCH IS PUSHED \r\n");
		}
		else if(result.power_result.motor_output == false){
			control = false;
			printf("MOTOR DRIVER IS NOT ACTIVE \r\n");
		}
		else if(wait_release_request == false){
			control = true;
			HAL_GPIO_WritePin(Nucleo_LED_GPIO_Port, Nucleo_LED_Pin, true); //LED
		}

		//PID control
		float volt_l;
		float volt_r;
		if (control) {
			float e_l = command.vel_l - vel_l;
			float e_r = command.vel_r - vel_r;
			if(fabsf(e_l)>10.0f) e_l = 0;
			if(fabsf(e_r)>10.0f) e_r = 0;
			pid.integral_l += e_l * DT;
			pid.integral_r += e_r * DT;
			pid.integral_l = fmaxf(fminf(pid.integral_l,2.5f),-2.5f);
			pid.integral_r = fmaxf(fminf(pid.integral_r,2.5f),-2.5f);
			volt_l = pid.kf * command.vel_l + pid.kp * e_l + pid.ki * pid.integral_l + pid.kd * (pid.prev_error_l - e_l) * RATE;
			volt_r = pid.kf * command.vel_r + pid.kp * e_r + pid.ki * pid.integral_r + pid.kd * (pid.prev_error_r - e_r) * RATE;
			pid.prev_error_l = e_l;
			pid.prev_error_r = e_r;
			//線形化 （-0.9から+0.9までの電圧ではタイヤが回らないため、特性が線形に近くなるように調整）
			if(fabsf(volt_l) > 0.1f) volt_l = (volt_l > 0.0f) ? volt_l + 3.0f : volt_l - 3.0f;
			if(fabsf(volt_r) > 0.1f) volt_r = (volt_r > 0.0f) ? volt_r + 3.0f : volt_r - 3.0f;
			volt_l = fmaxf(fminf(volt_l, 8.0f), -8.0f);
			volt_r = fmaxf(fminf(volt_r, 8.0f), -8.0f);

		}
		else{
			pid.integral_l = 0.0f;
			pid.integral_r = 0.0f;
			pid.prev_error_l = 0.0f;
			pid.prev_error_r = 0.0f;
			volt_l = 0.0f;
			volt_r = 0.0f;

		}

		//PWM
		uint32_t pwm_l;
		uint32_t pwm_r;
		bool dir_l;
		bool dir_r;
		if (control){
			pwm_l = (uint32_t)(MAX_DUTY_CNT * (fabsf(volt_l)/12.0f));
			pwm_r = (uint32_t)(MAX_DUTY_CNT * (fabsf(volt_r)/12.0f));
			dir_l = (volt_l > 0.0f) ? true : false;
			dir_r = (volt_r < 0.0f) ? true : false;
		}
		else{
			pwm_l = 0;
			pwm_r = 0;
			dir_l = false;
			dir_r = false;
		}
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm_l);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pwm_r);
		HAL_GPIO_WritePin(DIR_L_GPIO_Port, DIR_L_Pin, dir_l);
		HAL_GPIO_WritePin(DIR_R_GPIO_Port, DIR_R_Pin, dir_r);

		//SPI
		result.vel_l = vel_l;
		result.vel_r = vel_r;
		result.cnt_l = TIM2->CNT;
		result.cnt_r = TIM3->CNT;
		setSpiData(&result);

		//other

		g_main_loop_flag = false;
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_TIM1;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 3;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_15TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

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
  hspi1.Init.Mode = SPI_MODE_SLAVE;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_INPUT;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 7999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 19;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 7999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 19;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 7999;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DIR_L_GPIO_Port, DIR_L_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DIR_R_GPIO_Port, DIR_R_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Nucleo_LED_GPIO_Port, Nucleo_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : DIR_L_Pin */
  GPIO_InitStruct.Pin = DIR_L_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DIR_L_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : enc_L_Z_Pin */
  GPIO_InitStruct.Pin = enc_L_Z_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(enc_L_Z_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : enc_R_Z_Pin */
  GPIO_InitStruct.Pin = enc_R_Z_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(enc_R_Z_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DIR_R_Pin */
  GPIO_InitStruct.Pin = DIR_R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DIR_R_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Nucleo_LED_Pin */
  GPIO_InitStruct.Pin = Nucleo_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Nucleo_LED_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int _write(int file, char *ptr, int len)
{
  HAL_UART_Transmit(&huart2,(uint8_t *)ptr,len,10);
  return len;
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
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
