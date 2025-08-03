/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include <stdlib.h>
#include "arm_math.h"
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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
GPIO_TypeDef* IR_LED_PORTS[9] = { GPIOB, GPIOB, GPIOB, GPIOB, GPIOA, GPIOA, GPIOB, GPIOB, GPIOB};
uint32_t IR_LED_PINS[9] = {GPIO_PIN_12, GPIO_PIN_13, GPIO_PIN_14, GPIO_PIN_15, GPIO_PIN_12, GPIO_PIN_15, GPIO_PIN_3, GPIO_PIN_4, GPIO_PIN_5};
uint32_t sensorAdcChannel[9] = {
    ADC_CHANNEL_8, // sensor 0 → PB12
    ADC_CHANNEL_7, // sensor 1 → PB13
    ADC_CHANNEL_6, // sensor 2 → PB14
    ADC_CHANNEL_5, // sensor 3 → PB15
    ADC_CHANNEL_4, // sensor 4 → PA12
    ADC_CHANNEL_3, // sensor 5 → PA15
    ADC_CHANNEL_2, // sensor 6 → PB3
    ADC_CHANNEL_1, // sensor 7 → PA4
    ADC_CHANNEL_0, // sensor 8 → PB5
};

// These variables will store your sensor data. You can change `float` to `float32_t` if you wish.
float32_t ambient_adc[9][1000];
float32_t ir_on_adc[9][1000];
float32_t signal_calib[9][1000];
float32_t signal_runtime[9];
float32_t localMin=0;
float32_t localMax=0;

volatile float32_t SignalAvg[9];
volatile float32_t sumSignal=0;
volatile float32_t calib[9][2];
volatile uint8_t adcDone = 0;
volatile uint32_t adc_buf;
float32_t min_max[9][2];

//PID
float32_t Kp = 2.5f, Ki = 0.0f, Kd = 0.5f;
float32_t weights[9]={-40,-30,-20,-10,0,10,20,30,40};
uint32_t last_update_time = 0;
const uint32_t INTERVAL_MS = 20;
float32_t threshold =120;
float32_t position;
float32_t output;
float32_t error;
float32_t setpoint=0;
float32_t last_known_turn_direction;
uint8_t base_speed = 100;
uint8_t turn_speed = 60;

arm_pid_instance_f32 pid;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void setMotorSpeed(uint8_t motor, int32_t speed);
void delay_us(uint32_t us);
void calibrate(void);
void ReadSensors(void);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
float32_t line_data(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void setMotorSpeed(uint8_t motor, int32_t speed)
{
	// tim1 ch1- left front, ch2- left back, ch3-right front, ch4- right back
    uint16_t pwm = abs(speed);
    if (pwm > 200) pwm = 200;  // Limit max speed

    if (motor == 0) {  // Left motor
        if (speed > 0) {
            TIM1->CCR1 = pwm;
            TIM1->CCR2 = 0;
        } else {
            TIM1->CCR1 = 0;
            TIM1->CCR2 = pwm;
        }
    }
    else if (motor == 1) {  // Right motor
        if (speed > 0) {
            TIM1->CCR3 = pwm;
            TIM1->CCR4 = 0;
        } else {
            TIM1->CCR3 = 0;
            TIM1->CCR4 = pwm;
        }
    }
}
void delay_us(uint32_t us)
{
    uint32_t start = __HAL_TIM_GET_COUNTER(&htim2);
    while ((__HAL_TIM_GET_COUNTER(&htim2) - start) < us);
}
void calibrate(void)
{ 	volatile int sensorIndex = 0;
    volatile int sampleIndex=0;
    localMin= signal_calib[sensorIndex][0];
    localMax =signal_calib[sensorIndex][0];;


    ADC_ChannelConfTypeDef sConfig; // sconfig to change channels

    for(sensorIndex =0; sensorIndex<9 ; sensorIndex++)
    {
    	sConfig.Channel = sensorAdcChannel[sensorIndex]; // to update channels
    	sConfig.Rank = 1; //ADC_REGULAR_RANK_1;
    	HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    	for(sampleIndex=0; sampleIndex <1000; sampleIndex++)
    	{
    		// ir led off
    		HAL_GPIO_WritePin(IR_LED_PORTS[sensorIndex], IR_LED_PINS[sensorIndex], RESET);
    		delay_us(50);
    		HAL_ADC_Start(&hadc1);
    		HAL_ADC_PollForConversion(&hadc1, 0xFFFFFFFFU);
            // 0xFFFFFFFFU is delay so it stays in loop unless adc conversion finished
    		ambient_adc[sensorIndex][sampleIndex]=(float32_t)HAL_ADC_GetValue(&hadc1);

    		if (__HAL_ADC_GET_FLAG(&hadc1, ADC_FLAG_OVR)) {
    		    // Overflow happened
    		    __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_OVR);  // clear it
    		}

            //ir led on
    		HAL_GPIO_WritePin(IR_LED_PORTS[sensorIndex], IR_LED_PINS[sensorIndex], SET);
    		delay_us(50);
    		HAL_ADC_Start(&hadc1);
            HAL_ADC_PollForConversion(&hadc1, 0xFFFFFFFFU);
    	   ir_on_adc[sensorIndex][sampleIndex]=(float32_t)HAL_ADC_GetValue(&hadc1);

    	   if (__HAL_ADC_GET_FLAG(&hadc1, ADC_FLAG_OVR)) {
    	       // Overflow happened
    	       __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_OVR);  // clear it
    	   }


    	   // save difference in array
    	   signal_calib[sensorIndex][sampleIndex] = ambient_adc[sensorIndex][sampleIndex] - ir_on_adc[sensorIndex][sampleIndex];
    	   sumSignal += signal_calib[sensorIndex][sampleIndex];
    	}
    	SignalAvg[sensorIndex] = sumSignal / 1000.0f;
    	sumSignal=0;

    	for (int i = 1; i < 1000; i++)
    	{
    		float32_t val = signal_calib[sensorIndex][i];
    	    if (val < localMin) localMin = val;
    	    if (val > localMax) localMax = val;
    	}
    	min_max[sensorIndex][0]= localMin;
    	min_max[sensorIndex][1] = localMax;


    	HAL_Delay(1);


	}

 // print on oled - calibration done


}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(hadc -> Instance == ADC1){
		 adcDone = 1;
	}

} //callback during runtime
void ReadSensors(void)
{
	   int sensorIndex = 0;


	   ADC_ChannelConfTypeDef sConfig; // sconfig to change channels

	   for(sensorIndex =0; sensorIndex<9 ; sensorIndex++)
	   {
	   	sConfig.Channel = sensorAdcChannel[sensorIndex]; // to update channels
	   	sConfig.Rank = 1; //ADC_REGULAR_RANK_1;
	   	HAL_ADC_ConfigChannel(&hadc1, &sConfig);

			// ir led off
			HAL_GPIO_WritePin(IR_LED_PORTS[sensorIndex], IR_LED_PINS[sensorIndex], RESET);
			delay_us(50);
			  HAL_ADC_Stop_DMA(&hadc1);  // <<< IMPORTANT
			adcDone = 0;

			HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&adc_buf, 1);


			 // wait until conversion is complete
			while (adcDone == 0);

			ambient_adc[sensorIndex][0]=(float32_t)adc_buf;

			if (__HAL_ADC_GET_FLAG(&hadc1, ADC_FLAG_OVR))
			{
			    // Overflow happened
			     __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_OVR);  // clear it
			}



	 		//ir led on
	 		HAL_GPIO_WritePin(IR_LED_PORTS[sensorIndex], IR_LED_PINS[sensorIndex], SET);
	 		delay_us(50);
	 		 HAL_ADC_Stop_DMA(&hadc1);  // <<< IMPORTANT
	 		adcDone = 0;

	 	   HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&adc_buf, 1);

	 	   //callback will be triggered after conversion
	 	   // wait until conversion is complete
	 	   while (adcDone == 0);

	 	  ir_on_adc[sensorIndex][0]=(float32_t)adc_buf;

		   if (__HAL_ADC_GET_FLAG(&hadc1, ADC_FLAG_OVR)) {
		       // Overflow happened
		       __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_OVR);  // clear it
		   }

	 	   // save difference in array
		   signal_runtime[sensorIndex] = ambient_adc[sensorIndex][0] - ir_on_adc[sensorIndex][0];

	}

	//  // print on oled - sensor data took

}
float32_t line_data(void){
	float32_t sum = 0;
	float32_t weighted_sum = 0;
	float32_t onLine = 0;
	for(int i=0;i<9;i++){
		if(signal_runtime[i]<threshold){
			weighted_sum += weights[i];
			sum += 1;
            onLine = 1;
		}
	}
	 if (!onLine) {
		 return 255;  // Line lost condition
	 }

	 return (float32_t)weighted_sum / (float32_t)sum;

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
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim2);

  // Start PWM for motors
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

  // Call calibrate once at the beginning of your program to establish a baseline
  calibrate();
  pid.Kp=Kp;
  pid.Ki=Ki;
  pid.Kd=Kd;
  arm_pid_init_f32(&pid, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  uint32_t current_time = HAL_GetTick();
	  if((current_time-last_update_time)>=INTERVAL_MS){
		  last_update_time=HAL_GetTick();
		  ReadSensors();
		  position=line_data();
		  if (position == 255)
		  {
			  arm_pid_reset_f32(&pid);
			  if (last_known_turn_direction == 1) { // We were heading into a right turn
				  setMotorSpeed(0, turn_speed);
				  setMotorSpeed(1, -turn_speed);
			  } else if (last_known_turn_direction == -1) { // We were heading into a left turn
				  setMotorSpeed(0, -turn_speed);
				  setMotorSpeed(1, turn_speed);
			  }

		  } else {
			  if (position > 0) {
				  last_known_turn_direction = 1; // Line is to the right
			  } else if (position < 0) {
				  last_known_turn_direction = -1; // Line is to the left
			  }
			  error = ((float32_t)position - (float32_t)setpoint);
			  output = arm_pid_f32(&pid, error);
			  setMotorSpeed(0, base_speed + (int32_t)output);
			  setMotorSpeed(1, base_speed - (int32_t)output);
		  }
	  }


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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 192;
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
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};
  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_112CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_0;
  sConfigInjected.InjectedRank = 1;
  sConfigInjected.InjectedNbrOfConversion = 1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_3CYCLES;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_NONE;
  sConfigInjected.ExternalTrigInjecConv = ADC_INJECTED_SOFTWARE_START;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.InjectedOffset = 0;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  htim1.Init.Prescaler = 960-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 200-1;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 96-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
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
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10|GPIO_PIN_12
                          |GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB2 PB10 PB12
                           PB13 PB14 PB15 PB3
                           PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10|GPIO_PIN_12
                          |GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA12 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
