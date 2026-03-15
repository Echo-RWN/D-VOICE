/* USER CODE BEGIN Header */
#include <math.h>
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "adc.h"
#include "tim.h"
#include "gpio.h"

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

/* USER CODE BEGIN PV */
// 三声道ADC采样值（0~4095）
uint16_t adc_sample_value[3] = {0, 0, 0};
// 三声道PWM比较值（0~424）
uint32_t pwm_compare_value[3] = {0, 0, 0};

// 低音低通滤波 500Hz
#define BASS_CH 2
#define Fs 50000.0f//采样率
#define Fc 500.0f//截止频率
#define LP_ALPHA (2*M_PI*Fc)/(2*M_PI*Fc + Fs)
#define TIM1_ARR 424              // TIM1 ARR值424（400kHz载波）
#define ADC_MAX_VALUE 4095        // 12位ADC最大值
#define ADC_MID_VALUE 2048        // ADC中点（1.65V偏置）
#define PWM_MID_VALUE 212         // PWM中点212（50%占空比）
#define GAIN_FACTOR 1.0f          // 增益系数（可调整，1.0为无增益，最大1.5避免削波）
static uint16_t bass_prev = ADC_MID_VALUE;//定义低音滤波初始值为ADC中点
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  // 仅处理TIM6中断，防止其他定时器干扰
  if (htim == NULL || htim->Instance != TIM6) return;

  // 1. 启动ADC采样（单次模式，连续采样3个通道）
  HAL_ADC_Start(&hadc1);

  // 2. 等待采样完成（超时1ms，保证中断快速执行）
  if (HAL_ADC_PollForConversion(&hadc1, 1) == HAL_OK) // 轮询等待转换完成，超时时间1ms，保证中断快速执行
  {
    // 3. 读取3个声道的ADC采样值（按Sequence顺序）
    adc_sample_value[0] = HAL_ADC_GetValue(&hadc1); // 声道1（原有通道）
    HAL_ADC_PollForConversion(&hadc1, 1);    // 等待下一个通道采样完成
    adc_sample_value[1] = HAL_ADC_GetValue(&hadc1); // 声道2（PA1/IN1）
    HAL_ADC_PollForConversion(&hadc1, 1);    // 等待下一个通道采样完成
    adc_sample_value[2] = HAL_ADC_GetValue(&hadc1); // 声道3（PA2/IN2）

    // 第一阶低音500Hz低通滤波
    adc_sample_value[BASS_CH] = LP_ALPHA*adc_sample_value[BASS_CH] + (1-LP_ALPHA)*bass_prev;
    bass_prev = adc_sample_value[BASS_CH];

    // 4. 遍历处理每个声道的ADC→PWM映射（复用逻辑，减少冗余）
    for (uint8_t ch = 0; ch < 3; ch++)
    {
      // 中点对称映射（避免偏置失真）
      int32_t adc_offset = (int32_t)adc_sample_value[ch] - ADC_MID_VALUE;//减去ADC中点，得到交流音频信号（正负对称，消除直流偏置）
      adc_offset = (int32_t)((float)adc_offset * GAIN_FACTOR);           //应用增益系数，放大/缩小音频信号
      int32_t pwm_offset = (adc_offset * PWM_MID_VALUE) / ADC_MID_VALUE; //按比例映射到PWM偏移范围
      pwm_compare_value[ch] = PWM_MID_VALUE + pwm_offset;                //加上PWM中点，得到最终PWM比较值

      // 边界保护（防止削波）
      pwm_compare_value[ch] = (pwm_compare_value[ch] > TIM1_ARR) ? TIM1_ARR : pwm_compare_value[ch];
      pwm_compare_value[ch] = (pwm_compare_value[ch] < 0) ? 0 : pwm_compare_value[ch];
    }

    // 5. 更新所有PWM通道（避免中断抖动，保证同步）
    __disable_irq();//暂时屏蔽所有中断,保证同步
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_compare_value[0]); // 声道1→CH1
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm_compare_value[1]); // 声道2→CH2（PA9）
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pwm_compare_value[2]); // 声道3→CH3（PA10）
    __enable_irq();
  }

  // 6. 停止ADC，降低功耗+减少噪声
  HAL_ADC_Stop(&hadc1);
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
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  // 1. ADC自校准（必须！降低采样偏移，提升音质）
  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK)
  {
    Error_Handler();
  }
  // 2. 启动TIM6定时中断（50kHz采样触发）
  if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK)
  {
    Error_Handler();
  }

  // 3. 启动TIM1主PWM+互补PWM（400kHz载波）
  //声道1：PA8和PB13
  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  // 启动互补PWM（PB13），全桥核心
  if (HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  // 声道2：CH2（PA9）
  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  // 声道3：CH3（PA10）
  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
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
