/* USER CODE BEGIN Header */
/**
  * @file           : main.c
  * @brief          : Main program body
  */
/* USER CODE END Header */

#include "main.h"
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "chirp_gen.h"
#include "sensors.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* USER CODE BEGIN PTD */
/* 0 = LFM Chirp, 1 = Geometric Sweep, 2 = Phase-Coded */
volatile uint8_t modulation_mode = 0;
/* USER CODE END PTD */

/* USER CODE BEGIN PD */
/* USER CODE END PD */
/* USER CODE BEGIN PFP */
void SystemClock_Config(void);
static void debug_print_params(ChirpParams p);
/* USER CODE END PFP */
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* USER CODE BEGIN PFP */
static void debug_print_params(ChirpParams p);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
static void debug_print_params(ChirpParams p)
{
    char buf[160];
    int f0_khz  = (int)(p.f0 / 1000.0f);
    int f1_khz  = (int)(p.f1 / 1000.0f);
    int dur_us  = (int)(p.duration * 1000000.0f);
    int amp_pct = (int)(p.amplitude * 100.0f);

    int len = snprintf(buf, sizeof(buf),
        "raw1=%4lu raw2=%4lu raw3=%4lu  |  f0=%dkHz f1=%dkHz dur=%dus amp=%d%% mode=%d\r\n",
        (unsigned long)p.raw1, (unsigned long)p.raw2, (unsigned long)p.raw3,
        f0_khz, f1_khz, dur_us, amp_pct, modulation_mode);

    HAL_UART_Transmit(&huart1, (uint8_t *)buf, len, 100);
}
/* USER CODE END 0 */

int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_DAC_Init();
  MX_TIM6_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
  sensors_init(&hadc1);
  chirp_debug_gpio_init();

  chirp_gen_compute(chirp_buffer, CHIRP_BUFFER_LEN, CHIRP_F0_HZ, CHIRP_F1_HZ, FS_ACTUAL_HZ);

  {
      const char *boot_msg = "\r\n--- Sonar Chirp firmware booted, UART OK ---\r\n";
      HAL_UART_Transmit(&huart1, (uint8_t *)boot_msg, strlen(boot_msg), 100);
  }
  /* USER CODE END 2 */

  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* --- K1 Button on PE3: Mode Selector --- */
    if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3) == GPIO_PIN_RESET)
    {
        HAL_Delay(50); /* Debounce */
        if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3) == GPIO_PIN_RESET)
        {
            modulation_mode++;
            if (modulation_mode > 2) {
                modulation_mode = 0;
            }
            /* Wait for button release */
            while (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3) == GPIO_PIN_RESET) {}
        }
    }

    /* --- K0 Button on PE4: Fire Waveform --- */
    if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_4) == GPIO_PIN_RESET)
    {
        while (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_4) == GPIO_PIN_RESET) {}

        ChirpParams p = sensors_read();
        debug_print_params(p);

        /* Let the shared ADC/DAC analog rail settle */
        HAL_Delay(2);

        uint32_t len = 0;

        switch(modulation_mode) {
            case 0:
                len = chirp_gen_compute_var(p.f0, p.f1, p.duration, p.amplitude);
                break;
            case 1:
                len = geometric_sweep_compute(p.f0, p.f1, p.duration, p.amplitude);
                break;
            case 2:
                len = phase_coded_compute(p.f0, p.duration, p.amplitude);
                break;
        }

        HAL_StatusTypeDef ret = chirp_fire_var(&hdac, &htim6, len);

        if (ret != HAL_OK && ret != HAL_BUSY)
        {
            Error_Handler();
        }
    }
  }
  /* USER CODE END 3 */
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac)
{
    chirp_dma_complete_cb(hdac, &htim6);
}
/* USER CODE END 4 */

void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
