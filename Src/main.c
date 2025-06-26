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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "W5500.h"
#include "W5500_REG.h"
#include "usart.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
static spi_t spi_handle;
static w5500_t w5500;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// ===== W5500 TEST FUNCTIONS =====
bool W5500_BasicTest(void)
{
    printf("=== TEST W5500 START ===\r\n");

    // Konfiguracja SPI
    static const spi_config_t spi_config = {
        .spi = SPI1,
        .dma = DMA2,
        .dmastreamtx = 3,
        .dmastreamrx = 2,
        .dmachanneltx = 3,
        .dmachannelrx = 3,
        .BRpre = 4,              // Wolniejsza prędkość dla testu
        .CSPort = GPIOD,
        .CSPin = 14,
        .MISOPort = GPIOA,
        .MISOPin = 6,
        .MOSIPort = GPIOA,
        .MOSIPin = 7,
        .SCKPort = GPIOA,
        .SCKPin = 5
    };

    // 1. Inicjalizacja SPI
    printf("1. Inicjalizacja SPI...\r\n");
    spi_init_config(&spi_handle, &spi_config);
    spi_init_peripheral(&spi_handle);
    printf("   SPI OK\r\n");

    // 2. Inicjalizacja W5500
    printf("2. Inicjalizacja W5500...\r\n");
    W5500_InitDefault(&w5500, &spi_handle);
    W5500_InitPeripheral(&w5500);

    if (!w5500.is_initialized) {
        printf("   BŁĄD: W5500 nie zainicjowane!\r\n");
        return false;
    }
    printf("   W5500 zainicjowane\r\n");

    // 3. Test wersji układu
    printf("3. Test wersji układu...\r\n");
    uint8_t version = W5500_GetVersion(&w5500);
    printf("   Wersja: 0x%02X ", version);

    if (version == 0x04) {
        printf("(OK)\r\n");
    } else {
        printf("(BŁĄD - oczekiwano 0x04)\r\n");
        return false;
    }

    // 4. Self-test
    printf("4. Self-test...\r\n");
    if (W5500_SelfTest(&w5500)) {
        printf("   Self-test PASSED\r\n");
    } else {
        printf("   Self-test FAILED\r\n");
        return false;
    }

    // 5. Test konfiguracji sieciowej
    printf("5. Test konfiguracji sieciowej...\r\n");
    w5500_network_config_t net_config;
    W5500_GetNetworkConfig(&w5500, &net_config);

    printf("   MAC: %02X:%02X:%02X:%02X:%02X:%02X\r\n",
           net_config.mac[0], net_config.mac[1], net_config.mac[2],
           net_config.mac[3], net_config.mac[4], net_config.mac[5]);

    printf("   IP:  %d.%d.%d.%d\r\n",
           net_config.ip[0], net_config.ip[1],
           net_config.ip[2], net_config.ip[3]);

    // 6. Test PHY
    printf("6. Test PHY...\r\n");
    bool link_up = W5500_IsLinkUp(&w5500);
    printf("   Link: %s\r\n", link_up ? "UP" : "DOWN");
    uint8_t phy_config = W5500_GetPHYConfig(&w5500);
    printf("   PHY Config: 0x%02X\r\n", phy_config);

    printf("=== TEST W5500 ZAKOŃCZONY POMYŚLNIE ===\r\n");
    return true;
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
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* System interrupt init*/
  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

    W5500_BasicTest();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      bool link_up = W5500_IsLinkUp(&w5500);
      printf("   Link: %s\r\n", link_up ? "UP" : "DOWN");
      uint8_t phy_config = W5500_GetPHYConfig(&w5500);
      printf("   PHY Config: 0x%02X\r\n", phy_config);
      for (volatile size_t i = 0; i < 180000000; i++) {}

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
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_5);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_5)
  {
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  LL_PWR_EnableOverDriveMode();
  LL_RCC_HSE_EnableBypass();
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {

  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_4, 180, LL_RCC_PLLP_DIV_2);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  while (LL_PWR_IsActiveFlag_VOS() == 0)
  {
  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_4);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_2);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_Init1msTick(180000000);
  LL_SetSystemCoreClock(180000000);
  LL_RCC_SetTIMPrescaler(LL_RCC_TIM_PRESCALER_TWICE);
}

/* USER CODE BEGIN 4 */
int __io_putchar(int ch) {
    return USART_PutChar(ch);
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
