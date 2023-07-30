/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
//#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdio.h>
#include "string.h"
#include "stdlib.h"
#include "FreeRTOS.h"
#include "timers.h"
#include "bno055.h"
#include "bno055_conf.h"
// #include "bno055/bno055.h"
// #include "bno055/bno_config.h"
#include "task.h"
#include "event_groups.h"
#include "semphr.h"
#include "queue.h"
#include "w25qxx.h"
#include "ssd1306_tests.h"
#include "ssd1306_fonts.h"

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
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart1;


/* USER CODE BEGIN PV */
#define temp_addr 0x1
#define acc_addr 0x2
#define lia_addr 0x3
#define gyr_addr 0x4

/*Initialize global variable*/
bno055_t bno;
error_bno err;
char str[100];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
s32 temperature = 0;
/* Definition task of FreeRTOS */
TaskHandle_t read_bno055_Handler;
TaskHandle_t write_to_flash_Handler;
TaskHandle_t display_ssd1306_Handler;

void read_bno055(void *argument);
void write_to_flash(void *argument);
void display_ssd1306(void *argument);

/* Create queues and Semaphore*/

xQueueHandle flash_queue;
xQueueHandle ssd_queue;

SemaphoreHandle_t semaphoreA;
SemaphoreHandle_t semaphoreB;
SemaphoreHandle_t counting_semaphore;

/* Create data structure to save data */
typedef struct
{
	float temp;
	bno055_vec3_t acc;
	bno055_vec3_t gyr;
	bno055_vec3_t lia;
} data;

static inline void delay_ms(u32 ms);

void w25q128_block_write_4bytes(uint32_t addr, int32_t *ptxINT32data, uint16_t index)
{
    uint8_t temp = 0;
    uint32_t offset = 4 * index; // Calculate the offset within the block for each byte.

    // Loop through the 4 bytes of the 32-bit integer.
    for (uint8_t i = 0; i < 4; i++)
    {
        // Extract the i-th byte from the 32-bit integer.
        temp = (uint8_t)(*ptxINT32data >> (24 - 8 * i));

        // Write the extracted byte to the Flash memory at the appropriate address.
        W25qxx_WriteBlock(&temp, addr, offset + i, 1);
    }
}

void w25q128_block_read_4bytes(uint32_t addr, int32_t *pRxINT32data, uint16_t index)
{
    uint8_t temp = 0;
    uint32_t offset = 4 * index; // Calculate the offset within the block for each byte.
    *pRxINT32data = 0;
    // Loop through the 4 bytes of the 32-bit integer.
    for (uint8_t i = 0; i < 4; i++)
    {
        // Write the extracted byte to the Flash memory at the appropriate address.
        W25qxx_ReadBlock(&temp, addr, offset + i, 1);
        *pRxINT32data |= (int32_t)(temp << (24-8*i));
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
	int8_t temp_read = 0;
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
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  W25qxx_Init();
  bno = (bno055_t){
         .i2c = &hi2c1, .addr = BNO_ADDR, .mode = BNO_MODE_IMU, ._temp_unit = 0,
     };

  //	  status_led_toggle();
   //   error_led_toggle();
    //  delay_ms(1000);
   //   status_led_toggle();
    //  error_led_toggle();

  if ((err = bno055_init(&bno)) == BNO_OK) {
	  printf("[+] BNO055 init success\r\n");
  } else {
	  printf("[!] BNO055 init failed\r\n");
	  printf("%s\n", bno055_err_str(err));
	  Error_Handler();
  }
  delay_ms(100);
  err = bno055_set_unit(&bno, BNO_TEMP_UNIT_C, BNO_GYR_UNIT_DPS,
						BNO_ACC_UNITSEL_M_S2, BNO_EUL_UNIT_DEG);
  if (err != BNO_OK) {
	  printf("[BNO] Failed to set units. Err: %d\r\n", err);
  } else {
	  printf("[BNO] Unit selection success\r\n");
  }


  flash_queue = xQueueCreate(5, sizeof(data));
  ssd_queue = xQueueCreate(5, sizeof(data));


  if (flash_queue == 0 || ssd_queue == 0)
  {
	  char *notif = "unable to create queue";
	  HAL_UART_Transmit(&huart1, (uint8_t*)notif, strlen(notif), 100);
  }
  else
  {
	  char *notif = "Create queue succesfully";
	  HAL_UART_Transmit(&huart1, (uint8_t*)notif, strlen(notif), 100);
  }
  semaphoreA = xSemaphoreCreateBinary();
  semaphoreB = xSemaphoreCreateBinary();
  counting_semaphore = xSemaphoreCreateCounting(3,0);


  xTaskCreate(read_bno055, "read_bno055", 256, NULL, 1, &read_bno055_Handler);
  xTaskCreate(write_to_flash, "write_to_flash", 256, NULL, 1, &write_to_flash_Handler);
  xTaskCreate(display_ssd1306, "display_ssd1306", 256, NULL, 1, &display_ssd1306_Handler);

  vTaskStartScheduler();
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */


  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	     // bno055_temperature(&bno, &temperature);

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
}
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
  huart1.Init.BaudRate = 9600;
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, FLASH_CS_Pin|LED_CS_Pin|LED_DC_Pin|LED_RES_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : FLASH_CS_Pin LED_CS_Pin LED_DC_Pin LED_RES_Pin */
  GPIO_InitStruct.Pin = FLASH_CS_Pin|LED_CS_Pin|LED_DC_Pin|LED_RES_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void read_bno055(void *argument)

{
	  data measured_data;
	  s32 temperature = 0;
	  bno055_vec3_t acc = {0, 0, 0};
	  bno055_vec3_t lia = {0, 0, 0};
	  bno055_vec3_t gyr = {0, 0, 0};
	  bno055_vec3_t mag = {0, 0, 0};
	  bno055_vec3_t grv = {0, 0, 0};
	  bno055_euler_t eul = {0, 0, 0};
	  bno055_vec4_t qua = {0, 0, 0};
	while(1)
	{
		//strcpy(str, "TASK1 take semaphore A and start measuring BNO055 \n");
		//HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 100);
		bno.temperature(&bno, &temperature);
		bno.acc(&bno, &acc);
		bno.linear_acc(&bno, &lia);
		bno.gyro(&bno, &gyr);
		bno.mag(&bno, &mag);
		bno.gravity(&bno, &grv);
		bno.euler(&bno, &eul);
		bno.quaternion(&bno, &qua);
		/*
		sprintf("[+] Temperature: %2d°C\r\n", temperature);
		sprintf("[+] ACC - x: %+2.2f | y: %+2.2f | z: %+2.2f\r\n", acc.x, acc.y,
			   acc.z);
		sprintf("[+] LIA - x: %+2.2f | y: %+2.2f | z: %+2.2f\r\n", lia.x, lia.y,
			   lia.z);
		sprintf("[+] GYR - x: %+2.2f | y: %+2.2f | z: %+2.2f\r\n", gyr.x, gyr.y,
			   gyr.z);
		sprintf("[+] MAG - x: %+2.2f | y: %+2.2f | z: %+2.2f\r\n", mag.x, mag.y,
			   mag.z);
		sprintf("[+] GRV - x: %+2.2f | y: %+2.2f | z: %+2.2f\r\n", grv.x, grv.y,
			   grv.z);
		sprintf("[+] Roll: %+2.2f | Pitch: %+2.2f | Yaw: %+2.2f\r\n", eul.roll,
			   eul.pitch, eul.yaw);
		sprintf("[+] QUA - w: %+2.2f | x: %+2.2f | y: %+2.2f | z: %+2.2f\r\n",
			   qua.w, qua.x, qua.y, qua.z);
			   */
		measured_data.temp = temperature;
		measured_data.acc = acc;
		measured_data.lia = lia;
		measured_data.gyr = gyr;

		if (xQueueSend(flash_queue, &measured_data, portMAX_DELAY) == pdPASS)
		{
			strcpy(str, "Measure data succesfully and send data to flash memory");
			HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 500);
		}
		else
		{
			strcpy(str, "flash queue full \n\n");
			HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 500);
		}
		xSemaphoreGive(counting_semaphore);
		vTaskDelay(pdMS_TO_TICKS(500));
		}
}
int8_t index_write = 0;
void write_to_flash(void *argument)
{
	char str[100];
	data received_data;
	W25qxx_EraseBlock(temp_addr);
	W25qxx_EraseBlock(acc_addr);
	W25qxx_EraseBlock(lia_addr);
	W25qxx_EraseBlock(gyr_addr);

	while(1)
	{
		sprintf(str, "IN TASK2 AND START TO WRITE FLASH");
		HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 500);
		xSemaphoreTake(counting_semaphore, portMAX_DELAY);
		xSemaphoreTake(counting_semaphore, portMAX_DELAY);
		if (xQueueReceive(flash_queue, &received_data, portMAX_DELAY) == pdTRUE)
		{

			w25q128_block_write_4bytes(temp_addr, &received_data.temp, index_write);
			w25q128_block_write_4bytes(acc_addr, &received_data.acc, index_write);
			w25q128_block_write_4bytes(lia_addr, &received_data.lia, index_write);
			w25q128_block_write_4bytes(gyr_addr, &received_data.temp, index_write);

			sprintf(str, "Write data to memory");
			HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 500);

			if (xQueueSend(ssd_queue, &received_data,portMAX_DELAY) == pdTRUE)
			{
				sprintf(str, "Step to LCD Display");
				HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 500);
			}
			else
			{
				sprintf(str, "OLED queue full");
				HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 500);
			}
		}
		else
			{
				sprintf(str, "flash queue empty");
				HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 500);
			}
		index_write++ ;
		xSemaphoreGive(semaphoreA);
		xSemaphoreGive(semaphoreB);
		sprintf(str, "TASK 2 release semaphore A and B and sleep");
		HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 500);
		vTaskDelay(pdMS_TO_TICKS(1));
		}

	}

void display_ssd1306(void *argument)
{
	ssd1306_Init();
	char str[100];
	data read_data_from_flash;
	int8_t index_read = index_write;
	while(1)
	{
		w25q128_block_read_4bytes(temp_addr, &read_data_from_flash.temp, index_read);
		w25q128_block_read_4bytes(acc_addr , &read_data_from_flash.acc, index_read);
		w25q128_block_read_4bytes(lia_addr , &read_data_from_flash.lia, index_read);
		w25q128_block_read_4bytes(gyr_addr , &read_data_from_flash.gyr, index_read);
		char temp_str[20], acc_str[30], lia_str[30], gyr_str[30];
		sprintf(temp_str, "Temperature : %.2f", read_data_from_flash.temp);
		ssd1306_SetCursor(2,2);
		ssd1306_WriteString(temp_str, Font_6x8, 0x01);

		sprintf(acc_str, "Acc : %.2f %.2f %.2f", read_data_from_flash.acc.x,read_data_from_flash.acc.y,read_data_from_flash.acc.z);
		ssd1306_SetCursor(2,12);
		ssd1306_WriteString(temp_str, Font_6x8, 0x01);

		sprintf(lia_str, "Lia : %.2f %.2f %.2f", read_data_from_flash.lia.x,read_data_from_flash.lia.y,read_data_from_flash.lia.z);
		ssd1306_SetCursor(2,12);
		ssd1306_WriteString(lia_str, Font_6x8, 0x01);

		sprintf(gyr_str, "Gyr : %.2f %.2f %.2f", read_data_from_flash.gyr.x,read_data_from_flash.gyr.y,read_data_from_flash.gyr.z);
		ssd1306_SetCursor(2,12);
		ssd1306_WriteString(temp_str, Font_6x8, 0x01);

		sprintf(str, "Display data to LCD");
		HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 500);
		index_read++;

	}
	ssd1306_UpdateScreen();
	sprintf(str, "TASK3 release semaphore B");
	vTaskDelay(pdMS_TO_TICKS(1000));
}

static inline void delay_ms(u32 ms) { HAL_Delay(ms); }
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */


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
