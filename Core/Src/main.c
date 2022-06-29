/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
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
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
// Bejövő adatok buffere
char buffer_in[68];
// Változó a bejövő parancsnak
char command_in[68];
// Változó (és egyben buffer) a kimenő parancshoz
char command_out[68];
// A frame-eket tároló tömb
bool frames[250][64] = { false };
// Flag, ami jelzi, hogy lejátszásra kész-e az animáció
bool animation_ready = false;
// Flag, ami jelzi, hogy éppen feltöltés alatt van-e egy animáció
bool incoming_animation = false;
// Flag, ami jelzi, hogy éppen parancs érkezett, amit le kell kezelni
bool command_in_to_handle = false;
// A tárolt animáció frame-jeinek száma
int frame_number = 0;
// Éppen elmentett frame-ek száma (mentés során fontos)
int frames_saved = 0;
// Jelenleg hanyadik frame-nél tartunk
int current_frame = 0;
// Változó a lejátszás sebességéhez
int time = 1000;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void frames_to_command(char* c)
{
	// Függvény, ami az aktuális frame-et paranccsá alakítja

	// Az első karaktere egy '/' jel, ami azt jelzi, hogy parancs érkezett
	c[0] = '/';
	// Majd végigmegyünk a frame 64 db pixelén
	for (int i = 0; i < 64; i++)
	{
		// És ha a frame adott pixele igaz tárolt értékű
		if (frames[current_frame][i])
			// Akkor a parancsba 1-est állítunk be
			c[i + 1] = '1';
		// Ellenkező esetben pedig 0-t
		else
			c[i + 1] = '0';
	}
	// Nem feledkezünk el a sorvége és a lezáró karakterekről sem
	c[65] = '\r';
	c[66] = '\n';
	c[67] = '\0';
}

void handle_command()
{
	// Függvény, ami jelekezeli a beérkezett parancsot

	// Hogyha frame-ekre vonatkozó parancs jött be és azok közül is az első
	// (ez tartalmazza, hogy hány tovább valódi frame fog beérkezni)
	if (command_in[0] == '/' && !incoming_animation)
	{
		// Akkor az animációnk inenntől nem kész a lejátszásra
		animation_ready = false;
		// Ugyanis éppen töltünk le egy új animációt
		incoming_animation = true;
		// Bekérjük a beérkező frame-ek számát
		sscanf(command_in, "/%d\r\n", &frame_number);
		// És beállítjuk, hogy jelenleg 0 frame van elmentve ezekből
		frames_saved = 0;
	}
	// Hogyha frame-ekre vonatkozó parancs érkezik, de már nem az első
	else if (command_in[0] == '/' && incoming_animation)
	{
		// Akkor végigmegyünk a parancson (frame pixeljein)
		for (int i = 1; i < 65; i++)
		{
			// És ha a parancs adott bitje 0
			if (command_in[i] == '0')
				// Akkor hamis értéket mentünk el a pixelhez
				frames[frames_saved][i - 1] = false;
			// Hogyha pedig az adott bit 1-es
			else if (command_in[i] == '1')
				// Akkor igaz értéket mentünk el a pixelhez
				frames[frames_saved][i - 1] = true;
		}
		// Egy újabb frame elémentésre került
		frames_saved++;

		// Hogyha már megvagyunk minden frame-mel
		if (frames_saved == frame_number)
		{
			// Akkor az animációnk észen áll a lejátszásra
			animation_ready = true;
			// És már nincsen letöltés
			incoming_animation = false;
			// És kezdjük a legjátszást a legelső frame-től
			current_frame = 0;
		}
	}
	// Hogyha az első gombot nyomták meg
	else if (!strcmp(command_in, "b0\r\n"))
		// Akkor 1 FPS-nek megfelelő időt állítunk be
		time = 1000;
	// Hogyha a második gombot nyomták meg
	else if (!strcmp(command_in, "b1\r\n"))
		// Akkor 10 FPS-nek megfelelő időt állítunk be
		time = 100;
	// Hogyha a harmadik gombot nyomták meg
	else if (!strcmp(command_in, "b2\r\n"))
		// Akkor 25 FPS-nek megfelelő időt állítunk be
		time = 40;
	// Hogyha a negyedik gombot nyomták meg
	else if (!strcmp(command_in, "b3\r\n"))
		// Akkor 30.3 FPS-nek megfelelő időt állítunk be
		time = 33;
}

void send_frame()
{
	// Függvény, mi kiküldi a jelenlegi frame parancsát

	// Először is elkészítjük a parancsot
	frames_to_command(command_out);
	// Majd kiküldjük az UART-on
	HAL_UART_Transmit_DMA(&huart2, (uint8_t*)command_out, 68);

	// Hogyha az utolsó frame-nél voltunk
	if (current_frame == frame_number - 1)
		// Akkor ugrunk az első frame-re
		current_frame = 0;
	else
		// Egyébként csak a következőre
		current_frame++;
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
  /* USER CODE BEGIN 2 */
  // Elindítjuk a fogadást UART-on keresztül DMA segítségével
  HAL_UART_Receive_DMA(&huart2, (uint8_t*)buffer_in, 68);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // Hogyha parancsunk van, amit le kell kezelni
	  if (command_in_to_handle)
	  {
		  // Akkor meghívjuk az erre hivatott függvényt
		  handle_command();
		  // És a flag-et reset-eljük
		  command_in_to_handle = false;
	  }
	  // Hogyha viszont az animáció kész a lejátszásra
	  else if (animation_ready)
	  {
		  // Akkor kiküldjük a jelenlegi frame-et
		  send_frame();
		  // És várunk annyi időt, hogy az a beállított FPS értéknek megfeleljen
		  HAL_Delay(time);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	// Callback függvény, ami akkor hívódik meg, ha sikeresen érkezett adat az UART-on

	// A bemeneti buffert gyorsan elmentjük
	strcpy(command_in, buffer_in);
	// Beállítjuk a flag-et, ami jelzi, hogy érkezett parancs
	command_in_to_handle = true;

	// Majd engedélyezzük a következő üzenet beérkezését is
	HAL_UART_Receive_DMA(&huart2, (uint8_t*)buffer_in, 68);
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
