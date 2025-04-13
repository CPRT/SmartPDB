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
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define ADC_FILTER_BUFFER_SIZE 20

typedef struct {
  float history[ADC_FILTER_BUFFER_SIZE];
  float sum;
  uint8_t index;
  uint8_t count;
  float filtered_value;
} MovingAverageFilter_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define TEMP_CHANNEL_IDX 0
#define CURRENT_CHANNEL_IDX 1
#define NUM_ADC_CHANNELS 2

// sensor and adc params
#define MCP9701_TC (0.010f) // temp coeffiecient (10.0 mV/C)
#define MCP9701_V0C (0.500f)
#define V_REF (3.3f)
#define ADC_RESOLUTION_BITS 12
#define ADC_MAX_VALUE ((1 << ADC_RESOLUTION_BITS) - 1)

// current sensor params
#define INA5181_GAIN (100.0f)
#define SHUNT_RESISTOR_OHMS (0.01f)

// fan control params
#define TEMP_MIN_THRESHOLD (25.0f)
#define TEMP_MAX_THRESHOLD (75.0f)
#define FAN_PWM_TIMER_HANDLE htim2
#define FAN_PWM_CHANNEL TIM_CHANNEL_1
#define FAN_PWM_MAX_COUNT (htim2.Init.Period)
#define FAN_PWM_MIN_DUTY_PERCENT (20)
#define FAN_PWM_MAX_DUTY_PERCENT (100)

#define FAN_PWM_MIN_DUTY                                                       \
  ((uint32_t)(FAN_PWM_MAX_COUNT * FAN_PWM_MIN_DUTY_PERCENT) / 100.0f)
#define FAN_PWM_MAX_DUTY                                                       \
  ((uint32_t)(FAN_PWM_MAX_COUNT * FAN_PWM_MAX_DUTY_PERCENT) / 100.0f)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define SAFE_SET_PWM_COMPARE(htim, channel, value)                             \
  do {                                                                         \
    uint32_t _new_compare = (value);                                           \
    uint32_t _max_compare = __HAL_TIM_GET_AUTORELOAD(htim);                    \
    if (_new_compare > _max_compare) {                                         \
      _new_compare = _max_compare;                                             \
    }                                                                          \
    __HAL_TIM_SET_COMPARE(htim, channel, _new_compare);                        \
  } while (0)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
volatile uint16_t adc_raw_values[NUM_ADC_CHANNELS];
volatile bool g_adc_conversion_complete = false; // DMA shit

MovingAverageFilter_t temp_filter;
MovingAverageFilter_t current_filter;

float g_filtered_temperature_c = 0.0f;
float g_filtered_current_amps = 0.0f;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Initialize_Moving_Average_Filter(MovingAverageFilter_t *filter) {
  memset(filter->history, 0, sizeof(filter->history));
  filter->sum = 0.0f;
  filter->index = 0;
  filter->count = 0;
  filter->filtered_value = 0.0f;
}

float Update_Moving_Average_Filter(MovingAverageFilter_t *filter,
                                   float new_value) {
  if (filter->count >= ADC_FILTER_BUFFER_SIZE) {
    filter->sum -= filter->history[filter->index];
  } else {
    filter->count++;
  }

  filter->sum += new_value;
  filter->history[filter->index] = new_value;

  filter->index = (filter->index + 1) % ADC_FILTER_BUFFER_SIZE;

  filter->filtered_value = filter->sum / filter->count;
  return filter->filtered_value;
}

void Process_ADC_Data(void) {
  // read from DMA buff
  uint16_t raw_temp = adc_raw_values[TEMP_CHANNEL_IDX];

  // check if numbers aren't fucked
  if (raw_temp < ADC_MAX_VALUE) {
    float voltage_temp = ((float)raw_temp / ADC_MAX_VALUE) * V_REF;
    float temp_celsius_unfiltered = (voltage_temp - MCP9701_V0C) / MCP9701_TC;
    g_filtered_temperature_c =
        Update_Moving_Average_Filter(&temp_filter, temp_celsius_unfiltered);
  }

  // read from DMA buff
  uint16_t raw_current = adc_raw_values[CURRENT_CHANNEL_IDX];

  // check if numbers aren't fucked
  if (raw_current < ADC_MAX_VALUE) {
    float voltage_current = ((float)raw_current / ADC_MAX_VALUE) * V_REF;
    float denom = INA5181_GAIN * SHUNT_RESISTOR_OHMS;
    if (denom > 1e-6) { // cant be dividing by zero lmao
      float current_amps_unfiltered = voltage_current / denom;
      g_filtered_current_amps = Update_Moving_Average_Filter(
          &current_filter, current_amps_unfiltered);
    }
  }
}

void Update_Fan_Speed(float temperature) {
  uint32_t pwm_pulse; // pulsate that thang, ayo what🍆

  if (temperature <= TEMP_MIN_THRESHOLD) {
    pwm_pulse = FAN_PWM_MIN_DUTY;
  } else if (temperature >= TEMP_MAX_THRESHOLD) {
    pwm_pulse = FAN_PWM_MAX_DUTY;
  } else {
    float temp_range = TEMP_MAX_THRESHOLD - TEMP_MIN_THRESHOLD;
    float duty_range = (float)(FAN_PWM_MAX_DUTY - FAN_PWM_MIN_DUTY);

    if (temp_range > 1e-6) { // sneaky div by 0 wont pass
      pwm_pulse = FAN_PWM_MIN_DUTY +
                  (uint32_t)(duty_range * (temperature - TEMP_MIN_THRESHOLD));

    } else {
      pwm_pulse = FAN_PWM_MIN_DUTY;
    }

    if (pwm_pulse < FAN_PWM_MIN_DUTY)
      pwm_pulse = FAN_PWM_MIN_DUTY;
    if (pwm_pulse > FAN_PWM_MAX_DUTY)
      pwm_pulse = FAN_PWM_MAX_DUTY;
  }

  // ripped the bong before writting this macro, could be downhill, proceed with
  // caution
  SAFE_SET_PWM_COMPARE(&FAN_PWM_TIMER_HANDLE, FAN_PWM_CHANNEL, pwm_pulse);
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

  /* USER CODE BEGIN 1 */
  float local_temperature_c;

  // TODO: Unused for now, until we decide how to display
  // float local_current_amps;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
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
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  Initialize_Moving_Average_Filter(&temp_filter);
  Initialize_Moving_Average_Filter(&current_filter);

  // ADC calibration, tbh never used this but some stack overflow dude told me
  // it helps
  if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK) {
    Error_Handler();
  }

  // rev up the DMA engine brrrrrrrr
  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_raw_values, NUM_ADC_CHANNELS) !=
      HAL_OK) {
    Error_Handler();
  }

  // rev up adc triggger timer brrrrrr
  if (HAL_TIM_Base_Start(&htim3) != HAL_OK) {
    Error_Handler();
  }

  // rev up the pwm timer brrrrrr
  if (HAL_TIM_PWM_Start(&FAN_PWM_TIMER_HANDLE, FAN_PWM_CHANNEL) != HAL_OK) {
    Error_Handler();
  }

  // rev up the fans brrr brr brrrrrrrrrrrrrrr
  SAFE_SET_PWM_COMPARE(&FAN_PWM_TIMER_HANDLE, FAN_PWM_CHANNEL,
                       FAN_PWM_MIN_DUTY);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    while (!g_adc_conversion_complete) {
      // low power mode
      __WFI();
    }

    g_adc_conversion_complete = false;
    Process_ADC_Data();

    local_temperature_c = g_filtered_temperature_c;

    // TODO: not yet determined how this will be displayed
    // local_current_amps = g_filtered_current_amps;

    Update_Fan_Speed(local_temperature_c);
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
   */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  if (HAL_ADC_Init(&hadc1) != HAL_OK) {
    Error_Handler();
  }

  /** Configure Regular Channel
   */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    Error_Handler();
  }

  /** Configure Regular Channel
   */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 639;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);
}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 63;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
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
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
