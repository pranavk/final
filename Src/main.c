/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

#include "stdlib.h"

#include "string.h"
#include "motor.h"
#include "rfid.h"
#include "loadcell.h"
#include "lcd.h"

I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi2;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI2_Init(void);
static void MX_UART3_Init(void);

void transmit(char c) 
{
	while (!(USART3->ISR & USART_ISR_TXE)) {
		// busy wait for trasnmit to become empty
	}
	// write data into transmit data register
	USART3->TDR = c;
}

void transmitval(int val, char negative) {
	int newval = val / 10;
	if (newval == 0) {
		if (negative) {
			transmit('-');
		}
		transmit('0' + val % 10);
		return;
	}

	transmitval(val/10, negative);
	transmit('0' + val % 10);
}

void transmitstr(char *str) {
	int len = strlen(str);
	for (int i = 0; i < len; i++) {
		transmit(str[i]);
	}
	transmit('\r');
	transmit('\n');
}

uint32_t debouncer_cals = 0;

void calorieButton(int foodEaten)
{
	debouncer_cals = (debouncer_cals << 1); // Always shift every loop iteration
	
	if (GPIOA->IDR & 0x00000001) // If input signal is set/high
	{ 
		debouncer_cals |= 0x01; // Set lowest bit of bit-vector
			// calories calcuation
		// 100 calories per gram
		int cals = foodEaten * 100;
		char *str = (char*)malloc(20);
		sprintf(str, "Cals: %d", cals);
		lcdPrintPrimaryStr(str);
	}
	if (debouncer_cals == 0xFFFFFFFF) { // If high, do this
	
	}
	
	if (debouncer_cals == 0x00000000) { // If low, do this
	}
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
 
  HAL_Init();


  /* Configure the system clock */
  SystemClock_Config();
	
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C2_Init();
  MX_SPI2_Init();
	MX_UART3_Init();

	TM_MFRC522_Init();
	HX711_Init(CHB_32);

	motorPinSetup();	
	testButtonSetup();
	
 if (!lcdInit(&hi2c2, (uint8_t)0x27)) {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
	 }
//	
	int max = 0;
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);

		uint32_t data = HXGetAvgValue();
		int value = (int)data;			
	//	char negative = value < 0;
	//	transmitval(negative ? -value : value, negative);
	//	transmit('\r');
	//	transmit('\n');	
		
		int foodEaten = max - value;
	//	transmitval(foodEaten, 0);
		calorieButton(foodEaten);
		//transmit('\r');
		//transmit('\n');	
		
		uint8_t cardid;
		if (TM_MFRC522_Check(&cardid) == MI_OK) {
			transmitstr("RFID Detected");
			
		
			while (value < 1000) {
				transmitstr("Not enough food.");
				
				motorForward();
				HAL_Delay(2000);
				
				uint32_t data = HXGetAvgValue();
				value = (int)data;			
				char negative = value < 0;
				transmitval(negative ? -value : value, negative);
				transmit('\r');
				transmit('\n');
			}
			
			transmitstr("Enough food! Stopping the motor...");
			motorStop();
			max = value;
		} 
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
		HAL_Delay(200);
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x2000090E;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

static void MX_UART3_Init(void) {
	__HAL_RCC_USART3_CLK_ENABLE();
	// Set PC4,5 to AF mode
	GPIOC->MODER |= GPIO_MODER_MODER4_1;
	GPIOC->MODER &= ~GPIO_MODER_MODER4_0;
	GPIOC->MODER |= GPIO_MODER_MODER5_1;
	GPIOC->MODER &= ~GPIO_MODER_MODER5_0;

	// Set AF1
	// 4 is USART3_TX; 5 is USART3_RX
	GPIOC->AFR[0] &= ~(GPIO_AFRL_AFSEL4);
	GPIOC->AFR[0] |= (0x1 << GPIO_AFRL_AFSEL4_Pos);
	GPIOC->AFR[0] &= ~(GPIO_AFRL_AFSEL5);
	GPIOC->AFR[0] |= (0x1 << GPIO_AFRL_AFSEL5_Pos);
	
	// 8 MHz / 115200 bits = 69.4444
	USART3->BRR = 0x45; // equivalent to 69
	
	// enable transmittter/ receiver and main peripheral enable bit
	USART3->CR1 |= USART_CR1_TE;
	USART3->CR1 |= USART_CR1_RE;
	USART3->CR1 |= USART_CR1_RXNEIE;
	
	USART3->CR1 |= USART_CR1_UE;
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

 /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

	// NSS config
	GPIO_InitTypeDef initStr1;
	initStr1.Pin = GPIO_PIN_12;
	initStr1.Mode = GPIO_MODE_OUTPUT_PP;
	initStr1.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOB, &initStr1);
	// Set state to HIGH initially
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
	
	// Load cell configuration
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	
	// Other LEDs; we are using 6 and 7 for load cell
	GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
