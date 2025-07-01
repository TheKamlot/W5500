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
bool rx_read_ready = false;
w5500_packet_info_t packet_s0;
bool processed_last_packet = true;
uint8_t rx_buffer[1500];
uint8_t tx_buffer[1500];
uint16_t tx_length = 0;

spi_t spi_handle;
spi_config_t spi_config;
w5500_t w5500;
socket_t s0;
bool buf_rx_0_done = false;
bool buf_tx_0_done = false;
bool packet_ready = false;
enum reciver_state s0_state = no_data;
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
    spi_config.spi = SPI1;
    spi_config.dma = DMA2;
    spi_config.dmastreamtx = 3;
    spi_config.dmastreamrx = 0;
    spi_config.dmachanneltx = 3;
    spi_config.dmachannelrx = 3;
    spi_config.BRpre = 4;           // Wolniejsza prędkość dla testu
    spi_config.CSPort = GPIOD;
    spi_config.CSPin = 14;
    spi_config.MISOPort = GPIOA;
    spi_config.MISOPin = 6;
    spi_config.MOSIPort = GPIOA;
    spi_config.MOSIPin = 7;
    spi_config.SCKPort = GPIOA;
    spi_config.SCKPin = 5;

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
    s0.socket_num = 0;
    s0.protocol = W5500_Sn_MR_PROTOCOL_MACRAW;
    W5500_SocketOpen(&w5500, &s0);
    uint8_t int_conf = 0x01;
    W5500_WriteReg(&w5500, W5500_SIMR, W5500_BSB_COMMON_REG, &int_conf, 1);
    int_conf = 0b00000100;
    W5500_WriteReg(&w5500, W5500_Sn_IR, W5500_BSB_SOCKET0_REG, &int_conf, 1);
    int_conf = 0b11110100;
    W5500_WriteReg(&w5500, W5500_Sn_MR, W5500_BSB_SOCKET0_REG, &int_conf, 1);
    bool link_up = false;
    uint8_t phy_config = 0x00;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      if (!link_up) {
          link_up = W5500_IsLinkUp(&w5500);
          phy_config = W5500_GetPHYConfig(&w5500);
          printf("   Link: %s\r\n", link_up ? "UP" : "DOWN");
          printf("   PHY Config: 0x%02X\r\n", phy_config);
          for (volatile size_t i = 0; i < 18000000; i++) {}
      }


    /* USER CODE END WHILE */


  /* USER CODE BEGIN 3 */

      //Obsługa przerwania - przypisanie dlaczego przerwanie
      if (s0_state ==no_data && !spi_is_dma_tx_busy(&spi_handle) && !spi_is_dma_rx_busy(&spi_handle) && w5500.isInterrupts) {
          uint8_t ir = 0;
          W5500_ReadReg(&w5500, W5500_IR,W5500_BSB_COMMON_REG, &ir, 1);
          if (ir != 0) {
              if (ir & W5500_IR_CONFLICT) {
                  printf("IP CONFLICT\r\n");
              }
              if (ir & W5500_IR_UNREACH) {
                  uint8_t ip[4] = {0,0,0,0};
                  W5500_ReadReg(&w5500, W5500_UIPR0, W5500_BSB_COMMON_REG, &ip[0], 1);
                  W5500_ReadReg(&w5500, W5500_UIPR1, W5500_BSB_COMMON_REG, &ip[1], 1);
                  W5500_ReadReg(&w5500, W5500_UIPR2, W5500_BSB_COMMON_REG, &ip[2], 1);
                  W5500_ReadReg(&w5500, W5500_UIPR3, W5500_BSB_COMMON_REG, &ip[3], 1);
                  printf("IP %d.%d.%d.%d UNREACHABLE\r\n", ip[0], ip[1], ip[2], ip[3]);
              }
          }
          W5500_WriteReg(&w5500, W5500_IR,W5500_BSB_COMMON_REG, &ir, 1);
          uint8_t sir = 0;
          W5500_ReadReg(&w5500, W5500_SIR,W5500_BSB_COMMON_REG, &sir, 1);
          if (sir != 0) {
              if (sir & (1 << 0)) {
                  uint8_t SnIR = 0;
                  W5500_ReadReg(&w5500, W5500_Sn_IR,W5500_BSB_SOCKET0_REG, &SnIR, 1);
                  if (SnIR & W5500_Sn_IR_CON) {
                      s0.flags.is_open = true;
                  }
                  if (SnIR & W5500_Sn_IR_DISCON){
                      s0.flags.is_open = false;
                  }
                  if (SnIR & W5500_Sn_IR_RECV) {
                      rx_read_ready = true;
                  }
                  if (SnIR & W5500_Sn_IR_TIMEOUT) {
                      printf("TIMEOUT on socket 0\r\n");
                  }
                  if (SnIR & W5500_Sn_IR_SENDOK) {
                      printf("SendOK on 0\r\n");
                  }

                  // if (SnIR == 0) {
                  //     w5500.isInterrupts = false;
                  // }
              }
          }


      }
      //Ramka przygotowana do wysłania do bufora W5500 - wysłać do bufora
      if (packet_ready && s0_state == dma_busy) {
          //
          packet_ready = false;
      }
      if (rx_read_ready && (s0_state == no_data || s0_state == data_read)) {
          int16_t status = W5500_SocketReceiveMACRAW(&w5500, &s0, rx_buffer, 1512, &packet_s0);
          if (status == W5500_ERROR) {
              printf("Error while receiving packet\r\n");
          } else if (status == W5500_INCOMPLETE_PACKET) {
              rx_read_ready = false;
          } else if (status == W5500_DMA_IN_PROGRESS) {
              printf("Oczekiwanie na dane\r\n");
          }else if (status == W5500_NO_DATA){
              rx_read_ready = false;
          } else if (status > 0) {
              printf("Packet received\r\n");
              uint8_t SnIR = 0;
              W5500_ReadReg(&w5500, W5500_Sn_IR,W5500_BSB_SOCKET0_REG, &SnIR, 1);
              W5500_WriteReg(&w5500, W5500_Sn_IR,W5500_BSB_SOCKET0_REG, &SnIR, 1);
              rx_read_ready = false;
          }
      }
      if (s0_state == socket_info_read) {
          char* data = (char*)(rx_buffer+42);
          printf("Odczytano dane: %s \n \r", data);
          s0_state = no_data;
          uint8_t SnIR = 0;
          // W5500_ReadReg(&w5500, W5500_Sn_IR,W5500_BSB_SOCKET0_REG, &SnIR, 1);
          // W5500_WriteReg(&w5500, W5500_Sn_IR,W5500_BSB_SOCKET0_REG, &SnIR, 1);
          //rx_read_ready = false;
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
