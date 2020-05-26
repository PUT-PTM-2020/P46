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
  * COPYRIGHT(c) 2020 STMicroelectronics
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
#include <string.h>
#include "st7735.h"
#include "fonts.h"
#include "lis3dsh.h"
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
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile uint16_t pulse_count1; // Licznik impulsow
volatile uint16_t positionsX; // Licznik przekreconych pozycji
volatile uint16_t pulse_count2; // Licznik impulsow
volatile uint16_t positionsY; // Licznik przekreconych pozycji
volatile uint16_t color=0;
volatile uint16_t size=1;
volatile uint16_t COLOR=ST7735_BLACK;
volatile uint16_t mode=0;
volatile uint16_t stage=0;
volatile uint16_t fail1=0;
volatile uint16_t fail2=0;
volatile uint16_t fail3=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */
void menu()
{
ST7735_FillScreen(ST7735_CYAN);
ST7735_FillRectangle(64, 15, 30, 30, ST7735_BLACK);
ST7735_FillRectangle(64, 65, 30, 30, ST7735_BLACK);
ST7735_FillRectangle(64, 115, 30, 30, ST7735_BLACK);

ST7735_FillRectangle(79, 16, 4, 28, ST7735_RED);

ST7735_FillRectangle(70, 70, 18, 18, ST7735_RED);
ST7735_FillRectangle(70, 74, 15, 10, ST7735_BLACK);

ST7735_FillRectangle(66, 126, 26, 8, ST7735_RED);
ST7735_FillRectangle(75, 117, 8, 26, ST7735_RED);
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void init() {
    ST7735_Init();

    const char ready[] = "Ready!\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t*)ready, sizeof(ready)-1, HAL_MAX_DELAY);
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
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  LISInit();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  init();
  ST7735_FillScreen(ST7735_WHITE);
  positionsX=0;
  positionsY=0;
  while (1)
  {
	  //zmiana trybu
	  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13) == GPIO_PIN_RESET && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_RESET)
	  	  {
		  for (int i=0; i <3000000; i++){}
		  if(mode==0){
			  ST7735_FillScreen(ST7735_CYAN);
			  menu();
			  mode=1;}
		  else if(mode==1){
			  stage=0;
			  ST7735_FillScreen(ST7735_WHITE);
			  mode=0;}
	  	  }

	  if(mode==0)
	  {
	  //zmiana koloru
	  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == GPIO_PIN_SET)
	  {
	  	for (int i=0; i <3000000; i++){}
	  	COLOR = ST7735_BLACK;
	  }
	  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_14) == GPIO_PIN_SET)
	  {
	  	for (int i=0; i <3000000; i++){}
	  	COLOR = ST7735_BLUE;
	  }
	  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_13) == GPIO_PIN_SET)
	  {
	  	for (int i=0; i <3000000; i++){}
	  	COLOR = ST7735_RED;
	  }
	  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) == GPIO_PIN_SET)
	  {
	  	for (int i=0; i <3000000; i++){}
	  	COLOR = ST7735_GREEN;
	  }
	  	  //zmiana rozmiaru
	  	  if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6) == GPIO_PIN_SET)
	  	  {
	  	  	for (int i=0; i <3000000; i++){}
	  	  	size=0;
	  	  }
	  	  if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9) == GPIO_PIN_SET)
	  	  {
	  		size=1;
	  	  	for (int i=0; i <3000000; i++){}
	  	  }
	  	  if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7) == GPIO_PIN_SET)
	  	  {
	  		size=2;
	  	  	for (int i=0; i <3000000; i++){}
	  	  }
	  	  if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6) == GPIO_PIN_SET)
	  	  {
	  		size=3;
	  	  	for (int i=0; i <3000000; i++){}
	  	  }

	  if (size==1)
	  {
	  if(TIM1->CNT<20)TIM1->CNT=20;
	  if(TIM1->CNT>652)TIM1->CNT=652;
	  pulse_count1 = TIM1->CNT; // X
	  positionsY = pulse_count1/4;// X

	  if(TIM2->CNT<20)TIM2->CNT=20;
	  if(TIM2->CNT>524)TIM2->CNT=524;
	  pulse_count2 = TIM2->CNT; // Y
	  positionsX = pulse_count2/4; // Y

	  ST7735_FillRectangle(positionsX-5, positionsY-5, 2, 2, COLOR);

	  }
	  if (size==2)
	  	  {
	  	  if(TIM1->CNT<20)TIM1->CNT=20;
	  	  if(TIM1->CNT>648)TIM1->CNT=648;
	  	  pulse_count1 = TIM1->CNT; // X
	  	  positionsY = pulse_count1/4;// X

	  	  if(TIM2->CNT<20)TIM2->CNT=20;
	  	  if(TIM2->CNT>520)TIM2->CNT=520;
	  	  pulse_count2 = TIM2->CNT; // Y
	  	  positionsX = pulse_count2/4; // Y

	  	  ST7735_FillRectangle(positionsX-5, positionsY-5, 3, 3, COLOR);

	  	  }
	  if (size==3)
	  	  {
	  	  if(TIM1->CNT<20)TIM1->CNT=20;
	  	  if(TIM1->CNT>644)TIM1->CNT=644;
	  	  pulse_count1 = TIM1->CNT; // X
	  	  positionsY = pulse_count1/4;// X

	  	  if(TIM2->CNT<20)TIM2->CNT=20;
	  	  if(TIM2->CNT>516)TIM2->CNT=516;
	  	  pulse_count2 = TIM2->CNT; // Y
	  	  positionsX = pulse_count2/4; // Y

	  	  ST7735_FillRectangle(positionsX-5, positionsY-5, 4, 4, COLOR);
	  	  }
	  }

	  if(mode==1)
	  {
		  if(stage==0)
		  {

		  }
		  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == GPIO_PIN_SET || fail1==1)
		  	  {
		  	  	for (int i=0; i <3000000; i++){}
		  	  	ST7735_FillScreen(ST7735_WHITE);
		  	    ST7735_FillRectangle(0, 0, 64, 160, ST7735_BLACK);
		  	    ST7735_FillRectangle(80, 0, 48, 160, ST7735_BLACK);
		  	  	stage=1;
		  	  	TIM1->CNT=0;
		  	  	TIM2->CNT=16;
		  	  	fail1=0;
		  	  }
		  	  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_14) == GPIO_PIN_SET || fail2==1)
		  	  {

		  	  	for (int i=0; i <3000000; i++){}
		  	  ST7735_FillScreen(ST7735_WHITE);
		  	  ST7735_FillRectangle(0, 0, 128, 16, ST7735_BLACK);
		  	  ST7735_FillRectangle(0, 144, 128, 16, ST7735_BLACK);
		  	  ST7735_FillRectangle(0, 32, 112, 96, ST7735_BLACK);
		  	  	stage=2;
		  	  TIM1->CNT=4;
		  	  TIM2->CNT=0;
		  	  fail2=0;


		  	  }
		  	  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_13) == GPIO_PIN_SET || fail3==1)
		  	  {
		  	  	for (int i=0; i <3000000; i++){}
		  	  ST7735_FillScreen(ST7735_WHITE);
		  	  //                   y  x
		  	  ST7735_FillRectangle(0, 64, 96, 32, ST7735_BLACK);
		  	  ST7735_FillRectangle(32, 32, 32, 96, ST7735_BLACK);
		  	  ST7735_FillRectangle(112, 0, 16, 160, ST7735_BLACK);
		  	  ST7735_FillRectangle(0, 0, 112, 16, ST7735_BLACK);
		  	  ST7735_FillRectangle(0, 144, 112, 16, ST7735_BLACK);
		  	  ST7735_FillRectangle(0, 16, 16, 32, ST7735_BLACK);
		  	  ST7735_FillRectangle(0, 112, 16, 32, ST7735_BLACK);
		  	  ST7735_FillRectangle(80, 16, 32, 32, ST7735_BLACK);
		  	  ST7735_FillRectangle(80, 112, 32, 32, ST7735_BLACK);
		  	  stage=3;
		  	  TIM1->CNT=12;
		  	  TIM2->CNT=0;
		  	  fail3=0;
		  	  }
		  	  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) == GPIO_PIN_SET)
		  	  {
		  	  	for (int i=0; i <3000000; i++){}
		  	  	COLOR = ST7735_GREEN;
		  	  }
	  if(stage==1)
	  {
		  if(TIM2->CNT>16 || TIM2->CNT<16)
		  fail1=1;
		  if(TIM1->CNT<0) TIM1->CNT=0;
		  pulse_count1 = TIM1->CNT/4; // X
		  positionsY = pulse_count1*16;// X

		  pulse_count2 = TIM2->CNT/4; // Y
		  positionsX = pulse_count2*16; // Y

		  ST7735_FillRectangle(positionsX, positionsY, 16, 16, ST7735_RED);
		  		  	  if(TIM1->CNT>34 && TIM2->CNT==16)
		  		  	  {
		  		  		for (int i=0; i <50000; i++){}
		  		  		  stage=0;
		  		  		  menu();
		  		  	  }
  	  }
	  if(stage==2)
	  {
		 if(TIM1->CNT<4 || TIM1->CNT>36 ||(TIM1->CNT>8 &&TIM1->CNT<32 && TIM2->CNT<28))
		  fail2=1;
		  	  pulse_count1 = TIM1->CNT/4; // X
		  	  positionsY = pulse_count1*16;// X

		  	  if(TIM2->CNT>32)TIM2->CNT=32;
		  	  if(TIM2->CNT<0)TIM2->CNT=0;
		  	  pulse_count2 = TIM2->CNT/4; // Y
		  	  positionsX = pulse_count2*16; // Y

		  	  ST7735_FillRectangle(positionsX, positionsY, 16, 16, ST7735_RED);
		  	  if(TIM1->CNT>30 && TIM2->CNT<6)
		  	  {
		  		for (int i=0; i <50000; i++){}
		  		  stage=0;
		  		  menu();
		  	  }
	  }
	  if(stage==3)
	  {
		  if((TIM1->CNT<12 && TIM2->CNT<4) ||(TIM1->CNT>28 && TIM2->CNT<4) ||(TIM1->CNT<12 && TIM2->CNT>20)||(TIM1->CNT>28 && TIM2->CNT>20)||(TIM1->CNT<24 &&TIM1->CNT>16 && TIM2->CNT<24)||(TIM1->CNT<32 && TIM1->CNT>8 && TIM2->CNT>8 && TIM2->CNT<16)||TIM1->CNT<4 || TIM2->CNT>28 || TIM1->CNT>32)
		  		  fail3=1;
		  pulse_count1 = TIM1->CNT/4; // X
  		  positionsY = pulse_count1*16;// X

  		  pulse_count2 = TIM2->CNT/4; // Y
  		  positionsX = pulse_count2*16; // Y

  		  ST7735_FillRectangle(positionsX, positionsY, 16, 16, ST7735_RED);

  		  if(TIM1->CNT<26 && TIM1->CNT>22 && TIM2->CNT<6 && TIM2->CNT>0)
  		  {
  			  for (int i=0; i <50000; i++){}
  				  		  stage=0;
  				  		  menu();
  		  }
	  }

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

  /**Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
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
  hspi1.Init.Direction = SPI_DIRECTION_1LINE;
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
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 680;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 15;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 15;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 552;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 15;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 15;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  huart2.Init.BaudRate = 9600;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LCD_DC_Pin|LCD_RST_Pin|GPIO_PIN_10|GPIO_PIN_13 
                          |GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PE3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PC6 PC7 PC8 PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_DC_Pin LCD_RST_Pin PA10 PA13 
                           PA14 PA15 */
  GPIO_InitStruct.Pin = LCD_DC_Pin|LCD_RST_Pin|GPIO_PIN_10|GPIO_PIN_13 
                          |GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PD0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_CS_Pin */
  GPIO_InitStruct.Pin = LCD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_CS_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
