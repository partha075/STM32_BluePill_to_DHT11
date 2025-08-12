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
#include <stdio.h>  // for snprintf()
#include <stdlib.h>
#include <string.h>
#include "stm32f1xx_hal.h"



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
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
#define DHT11_PORT GPIOA
#define DHT11_PIN GPIO_PIN_2
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
/*void init_dht11(dht11_t *dht, TIM_HandleTypeDef *htim, GPIO_TypeDef* port, uint16_t pin){
	dht->htim = htim3;
	dht->port = port;
	dht->pin = pin;
}
*/

typedef struct DHT11_Data
{
  uint8_t timestamp;
  uint8_t temperature;
  uint8_t humidity;
  struct DHT11_Data* next;
} DHT11_Data;

DHT11_Data* head = NULL;
DHT11_Data* tail = NULL;
int node_count = 0;
#define MAX_NODES 50

void print_first_n(int n)
{
    DHT11_Data* current = head;
    int count = 0;
    char msg[64];

    while (current != NULL && count < n)
    {
        snprintf(msg, sizeof(msg), "[%lu] Temp: %d°C, Hum: %d%%\r\n",
                 (unsigned long)current->timestamp, current->temperature, current->humidity);
        uart_print(msg);
        current = current->next;
        count++;
    }
}

void print_last_n(int n)
{
    if (n > node_count)
        n = node_count;

    int skip = node_count - n;
    DHT11_Data* current = head;
    while (skip-- > 0 && current)
        current = current->next;

    char msg[64];
    while (current)
    {
        snprintf(msg, sizeof(msg), "[%lu] Temp: %d°C, Hum: %d%%\r\n",
                 (unsigned long)current->timestamp, current->temperature, current->humidity);
        uart_print(msg);
        current = current->next;
    }
}
void store_values(uint8_t temperature, uint8_t humidity)
{
	DHT11_Data* new_node  = malloc(sizeof(DHT11_Data));
	new_node->temperature = temperature;
	new_node->humidity = humidity;
	new_node->timestamp = HAL_GetTick();
	new_node->next = NULL;

	if (head == NULL)
	{
	    head = tail = new_node;
	}
	else
	{
	    tail->next = new_node;
	    tail = new_node;
	 }

	    node_count++;

	    if(node_count > MAX_NODES)
	    {
	    	DHT11_Data* temp_node = head;
	    	head = head->next;
	    	free(temp_node);
	    	node_count--;
	    }
}
void uart_print(const char *msg)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
}

void handle_uart_command(char *cmd)
{
    int n = atoi(cmd);

    if (n == 0 && cmd[0] != '0')
    {
        printf("Invalid command\r\n");
        return;
    }

    if (n > 0)
    {
        printf("Last %d readings:\r\n", n);
        print_last_n(n);
    }
    else
    {
        printf("First %d readings:\r\n", -n);
        print_first_n(-n);
    }
}

void delay_us(uint16_t us) {
  __HAL_TIM_SET_COUNTER(&htim3, 0);
  while (__HAL_TIM_GET_COUNTER(&htim3) < us);
}

void DHT11_Set_Pin_Output(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = DHT11_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStruct);
}

void DHT11_Set_Pin_Input(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = DHT11_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStruct);
}

uint8_t DHT11_Read(DHT11_Data *data)
{
	uint16_t mTime1 = 0, mTime2 = 0, mBit = 0;
	uint8_t humVal = 0, tempVal = 0, parityVal = 0, genParity = 0;
	uint8_t mData[40]={0};


HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, GPIO_PIN_SET);
HAL_Delay(180);
  // Start signal
  DHT11_Set_Pin_Output();
  HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, GPIO_PIN_RESET);
  HAL_Delay(18);
  __disable_irq();
  HAL_TIM_Base_Start(&htim3);
  HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, GPIO_PIN_SET);
  //delay_us(40);
  DHT11_Set_Pin_Input();

  // Wait for DHT response
  __HAL_TIM_SET_COUNTER(&htim3,0);
  while (HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN) == GPIO_PIN_SET)
  {
    if ((uint16_t)__HAL_TIM_GET_COUNTER(&htim3) > 500)
	{
    	uart_print("📟Failure At First Stage\r\n");
		__enable_irq();
		return 0; // timeout
    }
  }

  __HAL_TIM_SET_COUNTER(&htim3,0);
  while (HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN) == GPIO_PIN_RESET)
  {
    if ((uint16_t)__HAL_TIM_GET_COUNTER(&htim3) > 500)
    {
    	uart_print("📟Failure At Second Stage\r\n");
    	__enable_irq();
    	return 0;
    }
  }
  mTime1 = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
  __HAL_TIM_SET_COUNTER(&htim3,0);
  while (HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN) == GPIO_PIN_SET) {
    if ((uint16_t)__HAL_TIM_GET_COUNTER(&htim3) > 500)
    {
    	uart_print("📟Failure At tHIRD Stage\r\n");
    	__enable_irq();
    	return 0;
    }
  mTime2 = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
  }
  //if answer is wrong return
  	if(mTime1 < 75 && mTime1 > 85 && mTime2 < 75 && mTime2 > 85)
  	{
  		uart_print("📟Failure At 4TH Stage\r\n");
  		__enable_irq();
  		return 0;
  	}


  // Read 40 bits (5 bytes)
  	for(int j = 0; j < 40; j++)
  	{
  	  __HAL_TIM_SET_COUNTER(&htim3, 0);
      while (HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN) == GPIO_PIN_RESET) // wait for high
      {
    	  //delay_us(40); // wait 40us
    	  if((uint16_t)__HAL_TIM_GET_COUNTER(&htim3) > 500)
    	  {
    		  uart_print("📟Failure At D1 Stage\r\n");
      				__enable_irq();
      				return 0;
    	  }
      }
      __HAL_TIM_SET_COUNTER(&htim3, 0);
      while (HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN) == GPIO_PIN_SET) // wait for high
            {
          	  //delay_us(40); // wait 40us
          	  if((uint16_t)__HAL_TIM_GET_COUNTER(&htim3) > 500)
          	  {
          		uart_print("📟Failure At D2 Stage\r\n");
            				__enable_irq();
            				return 0;
          	  }
          }
      mTime1 = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
      if(mTime1 > 20 && mTime1 < 30)
      		{
      			mBit = 0;
      		}
      else if(mTime1 > 60 && mTime1 < 80) //if pass time 70 uS set as HIGH
      		{
      			 mBit = 1;
      		}

      //set i th data in data buffer
      mData[j] = mBit;


      }
  		HAL_TIM_Base_Stop(&htim3); //stop timer
  		__enable_irq(); //enable all interrupts

  		humVal = 0;
  		tempVal = 0;
  		//get hum value from data buffer
  		for(int i = 0; i < 8; i++)
  		{
  			humVal += mData[i];
  			humVal <<= 1;

  		}
  		humVal >>= 1;

  		//get temp value from data buffer
  		for(int i = 16; i < 24; i++)
  		{
  			tempVal += mData[i];
  			tempVal <<= 1;
  		}
  		tempVal >>= 1;

  		parityVal = 0;
  		for(int i = 32; i < 40; i++)
		{
			parityVal += mData[i];
			parityVal <<= 1;

		}
  		parityVal >>= 1;



  		genParity = humVal + tempVal;
  		//if(parityVal != genParity)
  		//	humVal = -humVal;

  		data->temperature = tempVal;
  		data->humidity = humVal;


  		return 1;

  }







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
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  HAL_TIM_Base_Start(&htim3);
  /* USER CODE BEGIN 2 */

  //DHT11_Data sensor;
  char msg[64];
  uint8_t index = 0;
  uart_print("📟 STM32 DHT11 UART Debugger Begin\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      HAL_Delay(2000);  // Wait between readings

      DHT11_Data sensor_data;

      if (DHT11_Read(&sensor_data) == 1)
      {
          store_values(sensor_data.temperature, sensor_data.humidity);

          snprintf(msg, sizeof(msg), "✅ Temp: %d°C, Hum: %d%%\r\n",
                   sensor_data.temperature, sensor_data.humidity);
          uart_print(msg);
      }
      else
      {
          uart_print("❌ DHT11 read failed\r\n");
      }

      // ✅ Add this UART check right here
      char rx_buffer[10] = {0};
      if (HAL_UART_Receive(&huart1, (uint8_t *)rx_buffer, sizeof(rx_buffer)-1, 100) == HAL_OK)
      {
          handle_uart_command(rx_buffer);
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DHT11_PORT, DHT11_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : DHT11_Pin */
  GPIO_InitStruct.Pin = DHT11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStruct);

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
