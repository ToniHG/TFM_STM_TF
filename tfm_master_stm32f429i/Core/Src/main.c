/* USER CODE BEGIN Header */
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
#include <string.h>
#include <stdio.h>
#include "cmsis_os.h"
#include "can_protocol.h"
#include "fault_tolerance.h"
#include "stm32f429i_discovery_lcd.h"
#include "stm32f429i_discovery_sdram.h"
#include "stm32f429i_discovery_ts.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

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
CAN_HandleTypeDef hcan2;                      // Handle for CAN2 (used as Master in this example)
CRC_HandleTypeDef hcrc;                       // Handle for CRC (used for CRC16 calculation in fault tolerance)
DMA2D_HandleTypeDef hdma2d;                   // Handle for DMA2D (used for efficient LCD updates)
I2C_HandleTypeDef hi2c3;                      // Handle for I2C3 (used for touch screen, if needed)
LTDC_HandleTypeDef hltdc;                     // Handle for LTDC (used for driving the LCD)
SPI_HandleTypeDef hspi5;                      // Handle for SPI5 (used for the SD card, if needed)
TIM_HandleTypeDef htim1;                      // Handle for TIM1 (used for timing, if needed)
UART_HandleTypeDef huart1;                    // Handle for UART (used for debugging, e.g. with printf)
SDRAM_HandleTypeDef hsdram1;                  // Handle for SDRAM (used by the LCD)
volatile ft_status_t last_msg_status = FT_OK; // Status of the last processed message (for display purposes)
QueueHandle_t can_rx_queue;                   // FreeRTOS queue to receive CAN messages from the ISR
TaskHandle_t TaskProcess_Handle;              // Handle for the task that processes CAN messages
TaskHandle_t TaskDisplay_Handle;              // Handle for the task that updates the display
/* Structure for RTOS CAN messages */
typedef struct {
    uint32_t sender_id;
    can_frame_payload_t frame;
} rtos_can_msg_t;
/* Enum for GUI states */
typedef enum {
    DISPLAY_MAIN = 0,
    DISPLAY_NODE_1,
    DISPLAY_NODE_2,
    DISPLAY_NODE_3,
    DISPLAY_CONTROL
} estado_gui_t;
estado_gui_t actual_display = DISPLAY_MAIN;  // Variable to track the current screen being displayed
uint8_t update_display = 1;                 // Flag to force repainting the screen when changing states


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN2_Init(void);
static void MX_CRC_Init(void);
static void MX_DMA2D_Init(void);
static void MX_FMC_Init(void);
static void MX_I2C3_Init(void);
static void MX_LTDC_Init(void);
static void MX_SPI5_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_LCD_Display_Init(void);
void CAN_Safe_Transmit(CAN_HandleTypeDef *hcan, uint32_t target_can_id, can_frame_payload_t *frame_to_send);
void Task_ProcessData(void *argument);
void Task_UpdateDisplay(void *argument);

/* Private user code ---------------------------------------------------------*/
/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
  /* MCU Configuration--------------------------------------------------------*/
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  /* Configure the system clock */
  SystemClock_Config();
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN2_Init();
  MX_CRC_Init();
  MX_I2C3_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_LCD_Display_Init();
  /* Initialize memory for Fault Tolerance can context */
  ft_init_context();
  /* Create the queue for CAN messages */
  can_rx_queue = xQueueCreate(10, sizeof(can_frame_payload_t));
  /* Create the task for processing CAN messages high priority */
  xTaskCreate(Task_ProcessData, "Proccesador", 256, NULL, 2, &TaskProcess_Handle);
  /* Create the task for updating the display low priority */
  xTaskCreate(Task_UpdateDisplay, "Display", 256, NULL, 1, &TaskDisplay_Handle);
  /* Start scheduler */
  vTaskStartScheduler();

  /* Infinite loop */
  while (1)
  {
    NULL;
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void) {
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
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
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

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void) {
  /* Configure CAN2 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 9;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_7TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  /* Configure  CAN filter to listen to all messages */
  CAN_FilterTypeDef canfilterconfig;
  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
  canfilterconfig.FilterBank = 14;
  canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  canfilterconfig.FilterIdHigh = 0x0000;
  canfilterconfig.FilterIdLow = 0x0000;
  canfilterconfig.FilterMaskIdHigh = 0x0000;
  canfilterconfig.FilterMaskIdLow = 0x0000;
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
  canfilterconfig.SlaveStartFilterBank = 14;

  /* Initialize CAN */
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* Initialize CAN filter and interrupts for reception */
  if (HAL_CAN_ConfigFilter(&hcan2, &canfilterconfig) != HAL_OK) {
      Error_Handler();
  }
  if (HAL_CAN_Start(&hcan2) != HAL_OK) {
      Error_Handler();
  }
  if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
      Error_Handler();
  }
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void) {
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DMA2D Initialization Function
  * @param None
  * @retval None
  */
static void MX_DMA2D_Init(void) {
  hdma2d.Instance = DMA2D;
  hdma2d.Init.Mode = DMA2D_M2M;
  hdma2d.Init.ColorMode = DMA2D_OUTPUT_ARGB8888;
  hdma2d.Init.OutputOffset = 0;
  hdma2d.LayerCfg[1].InputOffset = 0;
  hdma2d.LayerCfg[1].InputColorMode = DMA2D_INPUT_ARGB8888;
  hdma2d.LayerCfg[1].AlphaMode = DMA2D_NO_MODIF_ALPHA;
  hdma2d.LayerCfg[1].InputAlpha = 0;
  if (HAL_DMA2D_Init(&hdma2d) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DMA2D_ConfigLayer(&hdma2d, 1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void) {
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief LTDC Initialization Function
  * @param None
  * @retval None
  */
static void MX_LTDC_Init(void) {

  /* USER CODE BEGIN LTDC_Init 0 */

  /* USER CODE END LTDC_Init 0 */

  LTDC_LayerCfgTypeDef pLayerCfg = {0};
  LTDC_LayerCfgTypeDef pLayerCfg1 = {0};

  /* USER CODE BEGIN LTDC_Init 1 */

  /* USER CODE END LTDC_Init 1 */
  hltdc.Instance = LTDC;
  hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
  hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
  hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
  hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
  hltdc.Init.HorizontalSync = 7;
  hltdc.Init.VerticalSync = 3;
  hltdc.Init.AccumulatedHBP = 14;
  hltdc.Init.AccumulatedVBP = 5;
  hltdc.Init.AccumulatedActiveW = 654;
  hltdc.Init.AccumulatedActiveH = 485;
  hltdc.Init.TotalWidth = 660;
  hltdc.Init.TotalHeigh = 487;
  hltdc.Init.Backcolor.Blue = 0;
  hltdc.Init.Backcolor.Green = 0;
  hltdc.Init.Backcolor.Red = 0;
  if (HAL_LTDC_Init(&hltdc) != HAL_OK)
  {
    Error_Handler();
  }
  pLayerCfg.WindowX0 = 0;
  pLayerCfg.WindowX1 = 0;
  pLayerCfg.WindowY0 = 0;
  pLayerCfg.WindowY1 = 0;
  pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_ARGB8888;
  pLayerCfg.Alpha = 0;
  pLayerCfg.Alpha0 = 0;
  pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
  pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
  pLayerCfg.FBStartAdress = 0;
  pLayerCfg.ImageWidth = 0;
  pLayerCfg.ImageHeight = 0;
  pLayerCfg.Backcolor.Blue = 0;
  pLayerCfg.Backcolor.Green = 0;
  pLayerCfg.Backcolor.Red = 0;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg, 0) != HAL_OK)
  {
    Error_Handler();
  }
  pLayerCfg1.WindowX0 = 0;
  pLayerCfg1.WindowX1 = 0;
  pLayerCfg1.WindowY0 = 0;
  pLayerCfg1.WindowY1 = 0;
  pLayerCfg1.PixelFormat = LTDC_PIXEL_FORMAT_ARGB8888;
  pLayerCfg1.Alpha = 0;
  pLayerCfg1.Alpha0 = 0;
  pLayerCfg1.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
  pLayerCfg1.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
  pLayerCfg1.FBStartAdress = 0;
  pLayerCfg1.ImageWidth = 0;
  pLayerCfg1.ImageHeight = 0;
  pLayerCfg1.Backcolor.Blue = 0;
  pLayerCfg1.Backcolor.Green = 0;
  pLayerCfg1.Backcolor.Red = 0;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg1, 1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LTDC_Init 2 */

  /* USER CODE END LTDC_Init 2 */

}

/**
  * @brief SPI5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI5_Init(void) {

  /* USER CODE BEGIN SPI5_Init 0 */

  /* USER CODE END SPI5_Init 0 */

  /* USER CODE BEGIN SPI5_Init 1 */

  /* USER CODE END SPI5_Init 1 */
  /* SPI5 parameter configuration*/
  hspi5.Instance = SPI5;
  hspi5.Init.Mode = SPI_MODE_MASTER;
  hspi5.Init.Direction = SPI_DIRECTION_2LINES;
  hspi5.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi5.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi5.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi5.Init.NSS = SPI_NSS_SOFT;
  hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi5.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi5.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI5_Init 2 */

  /* USER CODE END SPI5_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void) {

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void) {
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
}

/* FMC initialization function */
static void MX_FMC_Init(void) {
  FMC_SDRAM_TimingTypeDef SdramTiming = {0};

  /** Perform the SDRAM1 memory initialization sequence
  */
  hsdram1.Instance = FMC_SDRAM_DEVICE;
  /* hsdram1.Init */
  hsdram1.Init.SDBank = FMC_SDRAM_BANK2;
  hsdram1.Init.ColumnBitsNumber = FMC_SDRAM_COLUMN_BITS_NUM_8;
  hsdram1.Init.RowBitsNumber = FMC_SDRAM_ROW_BITS_NUM_12;
  hsdram1.Init.MemoryDataWidth = FMC_SDRAM_MEM_BUS_WIDTH_16;
  hsdram1.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
  hsdram1.Init.CASLatency = FMC_SDRAM_CAS_LATENCY_3;
  hsdram1.Init.WriteProtection = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
  hsdram1.Init.SDClockPeriod = FMC_SDRAM_CLOCK_PERIOD_2;
  hsdram1.Init.ReadBurst = FMC_SDRAM_RBURST_DISABLE;
  hsdram1.Init.ReadPipeDelay = FMC_SDRAM_RPIPE_DELAY_1;
  /* SdramTiming */
  SdramTiming.LoadToActiveDelay = 2;
  SdramTiming.ExitSelfRefreshDelay = 7;
  SdramTiming.SelfRefreshTime = 4;
  SdramTiming.RowCycleDelay = 7;
  SdramTiming.WriteRecoveryTime = 3;
  SdramTiming.RPDelay = 2;
  SdramTiming.RCDDelay = 2;

  if (HAL_SDRAM_Init(&hsdram1, &SdramTiming) != HAL_OK)
  {
    Error_Handler( );
  }

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, NCS_MEMS_SPI_Pin|CSX_Pin|OTG_FS_PSO_Pin, GPIO_PIN_RESET);
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ACP_RST_GPIO_Port, ACP_RST_Pin, GPIO_PIN_RESET);
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, RDX_Pin|WRX_DCX_Pin, GPIO_PIN_RESET);
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, LD3_Pin|LD4_Pin, GPIO_PIN_RESET);
  /*Configure GPIO pins : NCS_MEMS_SPI_Pin CSX_Pin OTG_FS_PSO_Pin */
  GPIO_InitStruct.Pin = NCS_MEMS_SPI_Pin|CSX_Pin|OTG_FS_PSO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  /*Configure GPIO pins : B1_Pin MEMS_INT1_Pin MEMS_INT2_Pin TP_INT1_Pin */
  GPIO_InitStruct.Pin = B1_Pin|MEMS_INT1_Pin|MEMS_INT2_Pin|TP_INT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  /*Configure GPIO pin : ACP_RST_Pin */
  GPIO_InitStruct.Pin = ACP_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ACP_RST_GPIO_Port, &GPIO_InitStruct);
  /*Configure GPIO pin : OTG_FS_OC_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OC_GPIO_Port, &GPIO_InitStruct);
  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);
  /*Configure GPIO pins : OTG_HS_DM_Pin OTG_HS_DP_Pin */
  GPIO_InitStruct.Pin = OTG_HS_DM_Pin|OTG_HS_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF12_OTG_HS_FS;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  /*Configure GPIO pin : TE_Pin */
  GPIO_InitStruct.Pin = TE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TE_GPIO_Port, &GPIO_InitStruct);
  /*Configure GPIO pins : RDX_Pin WRX_DCX_Pin */
  GPIO_InitStruct.Pin = RDX_Pin|WRX_DCX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
  /*Configure GPIO pins : LD3_Pin LD4_Pin */
  GPIO_InitStruct.Pin = LD3_Pin|LD4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
}

/**
  * @brief LCD_Display Initialization Function
  * @param None
  * @retval None
  */
static void MX_LCD_Display_Init(void) {
  /* Initialize SDRAM */
  BSP_SDRAM_Init();
  /* Initialize LCD panel display*/
  BSP_LCD_Init();
  /* Configure LCD layers to external SDRAM */
  BSP_LCD_LayerDefaultInit(1, 0xD0000000);
  /* Select the configured layer */
  BSP_LCD_SelectLayer(1);
  /* Draw the initial screen */
  BSP_LCD_Clear(LCD_COLOR_BLACK);
  /* Initialize touch screen */
  BSP_TS_Init(240, 320);
}

/**
 * @brief  Función universal para enviar tramas seguras al bus CAN
 * @param  hcan: Puntero al periférico CAN (&hcan1 en Esclavo, &hcan2 en Master)
 * @param  target_can_id: El ID del mensaje (ej. CAN_ID_MASTER_CMD o CAN_ID_SLAVE_TELEMETRY)
 * @param  frame_to_send: Puntero a la estructura ya rellenada con tus datos
 */
void CAN_Safe_Transmit(CAN_HandleTypeDef *hcan, uint32_t target_can_id, can_frame_payload_t *frame_to_send) {
  CAN_TxHeaderTypeDef tx_header;
  uint32_t tx_mailbox;
  uint8_t tx_data[8];

  // 1. Configure CAN Header
  tx_header.StdId = target_can_id; 
  tx_header.ExtId = 0;
  tx_header.IDE = CAN_ID_STD;
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.DLC = 8; // Siempre 8 bytes
  tx_header.TransmitGlobalTime = DISABLE;
  // 2. Treat our frame with the Fault Tolerance library to add CRC and sequence number
  ft_prepare_tx_frame(frame_to_send, SLAVE1_ID);
  // 3. Add data to the CAN payload
  memcpy(tx_data, frame_to_send, sizeof(can_frame_payload_t));
  // 4. Transmit the message and check for errors
  if (HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data, &tx_mailbox) != HAL_OK) {
      // Podrías encender un LED de error de hardware aquí
      Error_Handler();
  }
}

/**
 * @brief  Callback universal de Recepción (Salta automáticamente al llegar un mensaje)
 * @param  hcan: Puntero al periférico CAN que ha recibido el mensaje (ej. &hcan1 o &hcan2)
 * @retval None
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  CAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8];
  rtos_can_msg_t incoming_msg;
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  /* Get the received message from the CAN peripheral */
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK) {
    /* Safe slave id*/
    incoming_msg.sender_id = rx_header.StdId;
    /* Send the received message to the FreeRTOS queue */
    xQueueSendFromISR(can_rx_queue, &incoming_msg, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}

/**
 * @brief  Callback de interrupción: Confirma que el envío terminó con éxito
 */
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan) {
    // Aquí puedes poner un toggle de un LED verde para saber que el cableado está bien
}

/**
 * @brief  Task de FreeRTOS para procesar los mensajes recibidos del bus CAN
 * @param  argument: No se usa, pero es un puntero que podrías usar para pasar datos a la tarea si quisieras
 * @retval None
 */
void Task_ProcessData(void *argument) {
  /* Data to process from Queue */
  rtos_can_msg_t msg_to_process;

  while(1) {
      /* Wait for a message to be received on the CAN queue */
      if (xQueueReceive(can_rx_queue, &msg_to_process, portMAX_DELAY) == pdPASS) {
        /* Process the message with the Fault Tolerance library to check CRC and sequence */
        last_msg_status = ft_process_message(&msg_to_process.frame, msg_to_process.sender_id);
          
        /* Treatment of fault conditions */
        if (last_msg_status == FT_SYNC_REQUIRED) {
            //TODO: SYNC PROCEDURE (e.g. request the latest valid data from the other node, or reset sequence number, etc.)
        }
        else if (last_msg_status == FT_ERR_CRC_FAILED) {
            //TODO: CRC ERROR TREATMENT (e.g. discard message, request retransmission, etc.)
        }
      }
  }
}

/**
 * @brief  Functions to print text on the LCD at specific coordinates (for simplicity, we use a basic function that prints character by character)
 * @param  x: X coordinate on the LCD
 * @param  y: Y coordinate on the LCD
 * @param  text: Text to print
 * @retval None
 */
void print_at_anyfont(uint16_t x, uint16_t y, char* text) {
    int i = 0;
    uint16_t font_width = BSP_LCD_GetFont()->Width;
    
    while(text[i] != '\0') {
        BSP_LCD_DisplayChar(x + (i * font_width), y, text[i]);
        i++;
    }
}

/**
 * @brief  Task de FreeRTOS para actualizar la pantalla
 * @param  argument: No se usa, pero es un puntero que podrías usar para pasar datos a la tarea si quisieras
 * @retval None
 */
void Task_UpdateDisplay(void *argument) {
  char txt[40];
  TS_StateTypeDef TS_State;

  while(1) {
    /* Check touch state */
    BSP_TS_GetState(&TS_State);
    
    if (TS_State.TouchDetected) {
        uint16_t x = TS_State.X;
        uint16_t y = TS_State.Y;

        if (actual_display == DISPLAY_MAIN) {
          /* Take action based on touch coordinates */
          if (x < 120 && y > 160) { actual_display = DISPLAY_NODE_1; update_display = 1; }
          else if (x > 120 && y > 160) { actual_display = DISPLAY_NODE_2; update_display = 1; }
          else if (x < 120 && y < 160) { actual_display = DISPLAY_NODE_3; update_display = 1; }
          else if (x > 120 && y < 160) { actual_display = DISPLAY_CONTROL; update_display = 1; }
        } 
        else {
          /* If submenus check for return button */
          if (50 > y) { 
              actual_display = DISPLAY_MAIN; 
              update_display = 1; 
          }
          /* Check for control panel interactions */
          else if (actual_display == DISPLAY_CONTROL) {
              if (y > 60 && y < 110) { 
                  slave_contexts[0].is_muted = !slave_contexts[0].is_muted; 
                  slave_contexts[0].consecutive_crc_errors = 0;
              }
              else if (y > 120 && y < 170) { 
                  slave_contexts[1].is_muted = !slave_contexts[1].is_muted; 
                  slave_contexts[1].consecutive_crc_errors = 0;
              }
              else if (y > 180 && y < 230) { 
                  slave_contexts[2].is_muted = !slave_contexts[2].is_muted; 
                  slave_contexts[2].consecutive_crc_errors = 0;
              }
          }
        }
        /* To avoid excessive CPU usage */
        vTaskDelay(pdMS_TO_TICKS(250)); 
    }
    /* Render of menus */
    if (update_display) {
        BSP_LCD_Clear(LCD_COLOR_WHITE);      
        BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
        if (actual_display == DISPLAY_MAIN) {
            /* Render main menu */
            BSP_LCD_SetTextColor(LCD_COLOR_DARKGRAY);
            BSP_LCD_DrawLine(120, 0, 120, 320);
            BSP_LCD_DrawLine(0, 160, 240, 160);

            BSP_LCD_SetFont(&Font16);
            BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
            print_at_anyfont(15, 60, "SLAVE 1");
            print_at_anyfont(135, 60, "SLAVE 2");
            print_at_anyfont(15, 220, "SLAVE 3");
            BSP_LCD_SetTextColor(LCD_COLOR_RED);
            print_at_anyfont(135, 220, "PANNEL");
            print_at_anyfont(135, 240, "CONTROL");
        }
        else {
            /* Render back button */
            BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
            BSP_LCD_FillRect(0, 280, 240, 40); 
            BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
            BSP_LCD_SetBackColor(LCD_COLOR_BLUE);
            BSP_LCD_SetFont(&Font16);
            print_at_anyfont(60, 292, "[ < BACK ]");
            BSP_LCD_SetBackColor(LCD_COLOR_WHITE); 
            BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
        }
        update_display = 0; 
    }

    /* Render dynamic data */
    BSP_LCD_SetFont(&Font16);
    
    /* Display node information */
    for (int i = 0; i < 3; i++) {
      if ((actual_display == DISPLAY_NODE_1 && i == 0) ||
          (actual_display == DISPLAY_NODE_2 && i == 1) ||
          (actual_display == DISPLAY_NODE_3 && i == 2)) 
      {
        BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
        sprintf(txt, "STATISTICS NODE %d", i+1);
        print_at_anyfont(10, 20, txt);
        
        BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
        BSP_LCD_SetFont(&Font12);
        sprintf(txt, "Data: %04lu   ", slave_contexts[i].last_valid_data);
        print_at_anyfont(10, 60, txt);
        sprintf(txt, "Seq number: %04lu   ", slave_contexts[i].expected_seq_num);
        print_at_anyfont(10, 80, txt);
        sprintf(txt, "Faults CRC: %lu    ", slave_contexts[i].stats_crc_errors);
        print_at_anyfont(10, 110, txt);
        sprintf(txt, "Lost Frames: %lu  ", slave_contexts[i].stats_frames_lost);
        print_at_anyfont(10, 130, txt);

        /* Actual state */
        if (slave_contexts[i].is_muted) {
            BSP_LCD_SetTextColor(LCD_COLOR_RED);
            print_at_anyfont(10, 160, "ESTADO: MUTEADO (RED)");
        } else if (slave_contexts[i].consecutive_crc_errors > 0) {
            BSP_LCD_SetTextColor(LCD_COLOR_ORANGE);
            print_at_anyfont(10, 160, "ESTADO: ALERTA CRC!  ");
        } else {
            BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
            print_at_anyfont(10, 160, "ESTADO: ACTIVO (OK)  ");
        }
      }
    }

    /* Render control panel */
    if (actual_display == DISPLAY_CONTROL) {
      BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
      print_at_anyfont(10, 20, "GESTION DE NODOS");
      
      BSP_LCD_SetFont(&Font12);
      for(int i = 0; i < 3; i++) {
        uint16_t y_pos = 80 + (i * 60);
        
        BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
        sprintf(txt, "NODO %d:", i+1);
        print_at_anyfont(10, y_pos, txt);
        
        /* Status indicator */
        if (slave_contexts[i].is_muted) {
          BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
          BSP_LCD_FillCircle(180, y_pos + 5, 15);
          print_at_anyfont(80, y_pos, "[ OFF ] ");
        } else {
          BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
          BSP_LCD_FillCircle(180, y_pos + 5, 15);
          BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
          print_at_anyfont(80, y_pos, "[ ACTIVE ] ");
        }
      }
    }
    /* Refresh control (5 Hz)*/
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void){
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
