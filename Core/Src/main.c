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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "sd.h"

#include "images.h"
#include "sounds.h"
#include "sprite.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum InputMode {
	JOYSTICK_INPUT,
	MOUSE_INPUT,
	TOUCH_INPUT
} input_mode_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define PADDLE_INIT_LEFT	144
#define PADDLE_INIT_TOP 	190

#define BALL_INIT_LEFT		157
#define BALL_INIT_TOP 		186

#define BRICK_COLUMN_SIZE 	13
#define BRICK_ROW_SIZE 		5

#define TOP_OFFSET 			40

#define MAX_NUM_LIVES		3
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s3;
DMA_HandleTypeDef hdma_spi3_tx;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

FATFS fs;
FIL file;
UINT bytes_transacted;
uint8_t in_text_buffer[5];
uint8_t out_text_buffer[5] = "XXXXX";

uint8_t i2c_data[10];
int16_t mouse_vx = 0;

extern volatile int screen_refreshed;

int gameplay_enabled = 0;
input_mode_t input_mode = JOYSTICK_INPUT;

unsigned int player_score = 0;
unsigned int player_high_score = 0;
unsigned int num_powerups = 1;

screen_t screen = {
		(unsigned char*)0x20020000,	// screen start address
		320,					// screen width
		200 					// screen height
};

int paddle_direction = 0; // left: -1, right: 1, stationary: 0

sprite_t paddle_sprite;
sprite_t ball_sprite;
sprite_t brick_sprites[BRICK_COLUMN_SIZE * BRICK_ROW_SIZE];
sprite_t heart_sprites[MAX_NUM_LIVES];
sprite_t ext_powerup_sprite;

int ball_angle = 30;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2S3_Init(void);
/* USER CODE BEGIN PFP */

void init_title_layout();

void draw_title_layout();

void init_play_layout();

void draw_play_layout();

void reset_play_layout();

void init_end_layout();

void draw_end_layout();

void check_gameplay_enabled_change();

void check_input_mode_change();

void handle_paddle_mechanics();

void check_gameplay_enabled_change();

void handle_ball_mechanics();

void update_score();

int game_won();

int game_lost();

void load_high_scores();

void save_high_scores();

void display_high_scores();

void draw_current_high_score();

void handle_powerup_mechanics();

void play_sound(const uint16_t* sound, uint16_t length);

void create_highscore_file();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_I2S3_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */

  // Configure I2C device

  i2c_data[0] = 0x20;
  i2c_data[1] = 0x47;
  HAL_I2C_Master_Transmit(&hi2c1, 0x32, i2c_data, 2, 10);
  i2c_data[0] = 0x23;
  i2c_data[1] = 0x00;
  HAL_I2C_Master_Transmit(&hi2c1, 0x32, i2c_data, 2, 10);

  f_mount(&fs, "", 1);

  create_highscore_file();

  // Initialize layouts
  init_title_layout();
  init_play_layout();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  i2c_data[0] = 0x28 | 0x80;

	  HAL_I2C_Master_Transmit(&hi2c1, 0x32, i2c_data, 1, 10);
	  HAL_I2C_Master_Receive(&hi2c1, 0x32, i2c_data, 6, 10);

	  uint16_t tmp = i2c_data[0] + (i2c_data[1] << 8);
	  mouse_vx = *((int16_t*)&tmp);

	  //tmp = i2c_data[2] + (i2c_data[3] << 8);
	  //int16_t vy = *((int16_t*)&tmp); // Get the y-velocity (not used).

	  if (screen_refreshed) {
		  check_input_mode_change();

		  check_gameplay_enabled_change();

		  if (gameplay_enabled) {
			  handle_paddle_mechanics();
			  handle_ball_mechanics();
			  handle_powerup_mechanics();
		  }

		  update_score();
		  display_high_scores();

		  // Check if the round is lost.
		  if (ball_sprite.top + ball_sprite.height >= paddle_sprite.top + paddle_sprite.height) {
			  // remove a life and a heart... deep
			  paddle_sprite.hp--;
			  clear_sprite(heart_sprites + paddle_sprite.hp, &screen);

			  reset_play_layout();
		  }

		  if (game_lost() || game_won()) {
			  save_high_scores();
			  init_end_layout();
		  } else {
			  draw_play_layout();
		  }

		  screen_refreshed = 0;
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Macro to configure the PLL multiplication factor 
  */
  __HAL_RCC_PLL_PLLM_CONFIG(16);
  /** Macro to configure the PLL clock source 
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSI);
  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 192;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_8K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

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
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PD9 PD10 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

}

/* USER CODE BEGIN 4 */

void create_highscore_file() {
	// Write data to file.
	f_open(&file, "score.txt", FA_WRITE | FA_CREATE_ALWAYS);

	out_text_buffer[0] = 48;
	out_text_buffer[1] = 48;
	out_text_buffer[2] = 48;
	out_text_buffer[3] = 48;
	out_text_buffer[4] = 48;

	f_write(&file, out_text_buffer, 5, &bytes_transacted);

	// Close file.
	f_close(&file);
}


void load_high_scores() {
	player_high_score = 0;

	// Read data from file.
	FRESULT temp = f_open(&file, "score.txt", FA_READ);
	f_read(&file, in_text_buffer, 5, &bytes_transacted);
	if (bytes_transacted != 0) {
		player_high_score += 10000 * (in_text_buffer[0] - 48);
		player_high_score += 1000 * (in_text_buffer[1] - 48);
		player_high_score += 100 * (in_text_buffer[2] - 48);
		player_high_score += 10 * (in_text_buffer[3] - 48);
		player_high_score += (in_text_buffer[4] - 48);
	}

	f_close(&file);
}


void save_high_scores() {
	if (player_high_score < player_score)
		player_high_score = player_score;

	 // Write data to file.
	  f_open(&file, "score.txt", FA_WRITE);

	  out_text_buffer[0] = (player_high_score / 10000) % 10 + 48;
	  out_text_buffer[1] = (player_high_score / 1000) % 10 + 48;
	  out_text_buffer[2] = (player_high_score / 100) % 10 + 48;
	  out_text_buffer[3] = (player_high_score / 10) % 10 + 48;
	  out_text_buffer[4] = player_high_score % 10 + 48;

	  f_write(&file, out_text_buffer, 5, &bytes_transacted);

	  // Close file.
	  f_close(&file);
}


void display_high_scores() {
	draw_current_high_score();
}

void display_number(unsigned int number, int left, int top) {
	uint32_t* start_address = (uint32_t*)(screen.start + screen.width * top + left);
	uint8_t digit = 0;
	uint32_t* digitptr;
	uint32_t* scrcopyptr;

	for (int i = 0; i < 5; i++) {
		digit = number % 10;
		number /= 10;

		scrcopyptr = start_address;
		digitptr = (uint32_t*)(digits + (digit << 3));
		for (int i = 0; i < 9; i++)
		{
			for (int j = 0; j < 2; j++)
			{
				*scrcopyptr++ = *digitptr++;
			}
			digitptr += 18;
			scrcopyptr += 78;

		}
		start_address -= 2;
	}
}

void init_title_layout() {
	int num_flashes = 10;
	draw_image(&screen, title_label_image, 72, 10, TITLE_LABEL_WIDTH, TITLE_LABEL_HEIGHT);
	draw_image(&screen, instr_label_image, 108, 100, INSTR_LABEL_WIDTH, INSTR_LABEL_HEIGHT);

	while (0 < num_flashes) { // Doesn't want to work :(
		clear_box(&screen, LOADING_LABEL_WIDTH, LOADING_LABEL_HEIGHT, 106, 180);
		draw_image(&screen, loading_label_image, 106, 180, LOADING_LABEL_WIDTH, LOADING_LABEL_HEIGHT);
		HAL_Delay(20);
		num_flashes--;
	}

	clear_screen(&screen);
}


void init_play_layout() {
	load_high_scores();
	player_score = 0;

	// Setup paddle
	init_sprite(&paddle_sprite, paddle_image, PADDLE_WIDTH, PADDLE_HEIGHT, 144, 190, MAX_NUM_LIVES);

	// Setup ball
	ball_angle = 30;
	init_sprite(&ball_sprite, ball_image, BALL_WIDTH, BALL_HEIGHT, 157, 186, 1);

	uint8_t* brick_images[] = {
			grey_brick_image,
			red_brick_image,
			orange_brick_image,
			yelllow_brick_image,
			green_brick_image
	};

	const unsigned int BRICK_H_PADDING = 2;
	const unsigned int BRICK_V_PADDING = 4;
	const unsigned int BRICK_LEFT_OFFSET = 5;

	for (int i = 0; i < BRICK_ROW_SIZE; i++) {
		for (int j = 0; j < BRICK_COLUMN_SIZE; j++) {
			int brick_index = i * BRICK_COLUMN_SIZE + j;
			int left_position = BRICK_LEFT_OFFSET + j * (BRICK_H_PADDING + BRICK_H_PADDING * (j < BRICK_COLUMN_SIZE) + BRICK_WIDTH);
			int top_position = TOP_OFFSET + i * (BRICK_V_PADDING + BRICK_HEIGHT);
			int num_lives = (i == 0) ? 2 : 1; // First row should have 2 lives and all other rows should have 1 life.

			init_sprite(brick_sprites + brick_index, brick_images[i], BRICK_WIDTH, BRICK_HEIGHT, left_position, top_position, num_lives);
		}
	}

	// Setup heart sprites
	for (int i = 0; i < 3; i++) {
		init_sprite(heart_sprites + i, heart_image, HEART_WIDTH, HEART_HEIGHT, 5 + i * (HEART_WIDTH + 10), 5, 1);
		draw_sprite(heart_sprites + i, &screen);
	}

	// Print labels
	draw_image(&screen, score_label_image, 220, 14, SCORE_LABEL_WIDTH, SCORE_LABEL_HEIGHT);
	draw_image(&screen, high_score_label_image, 100, 14, HIGH_SCORE_LABEL_WIDTH, HIGH_SCORE_LABEL_HEIGHT);

	// Draw highscore
	draw_current_high_score();

	// Setup powerup
	init_sprite(&ext_powerup_sprite, ext_powerup_image, EXT_POWERUP_WIDTH, EXT_POWERUP_HEIGHT, 144, 190, 0);

}


void draw_current_high_score() {
	display_number(player_high_score, 200, 13);
}


void reset_play_layout() {
	// Disable gameplay
	gameplay_enabled = 0;

	// Reset Paddle
	clear_sprite(&paddle_sprite, &screen);
	paddle_sprite.left = PADDLE_INIT_LEFT;
	paddle_sprite.top = PADDLE_INIT_TOP;

	// Reset Ball
	clear_sprite(&ball_sprite, &screen);
	ball_sprite.left = BALL_INIT_LEFT;
	ball_sprite.top = BALL_INIT_TOP;
	ball_angle = 30;
}


void draw_play_layout() {
	// Draw ball
	draw_sprite(&ball_sprite, &screen);

	// Draw alive bricks
	for (int i = 0; i < BRICK_COLUMN_SIZE * BRICK_ROW_SIZE; i++) {
		if (sprite_is_alive(brick_sprites + i))
			draw_sprite(brick_sprites + i, &screen);
	}

	// Draw paddle
	draw_sprite(&paddle_sprite, &screen);

	if (ext_powerup_sprite.hp != 0)
		draw_sprite(&ext_powerup_sprite, &screen);
}


void init_end_layout() {
	clear_screen(&screen);

	int num_flashes = 10;

	// Play sound
	play_sound(game_lost_sound, GAME_LOST_SOUND_LEN);

	if (game_lost())
		draw_image(&screen, (unsigned char*) lost_label_image, 38, 60, LOST_LABEL_WIDTH, LOST_LABEL_HEIGHT);
	else
		draw_image(&screen, (unsigned char*) won_label_image, 38, 60, WON_LABEL_WIDTH, WON_LABEL_HEIGHT);

	while (0 < num_flashes) { // Doesn't want to work :(
		clear_box(&screen, LOADING_LABEL_WIDTH, LOADING_LABEL_HEIGHT, 106, 180);
		draw_image(&screen, loading_label_image, 106, 180, LOADING_LABEL_WIDTH, LOADING_LABEL_HEIGHT);
		HAL_Delay(20);
		num_flashes--;
	}

	clear_screen(&screen);
	init_play_layout();
}


void check_gameplay_enabled_change() {
	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)) {
		gameplay_enabled = 1;

		// Play sound
		play_sound(ball_launch_sound, BALL_LAUNCH_SOUND_LEN);
	}
}


void check_input_mode_change() {
	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1)) input_mode = JOYSTICK_INPUT;
	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2)) input_mode = MOUSE_INPUT;
	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3)) input_mode = TOUCH_INPUT;
}


void handle_paddle_mechanics() {
	// Get user inputs
	switch (input_mode) {
	case JOYSTICK_INPUT:
		paddle_direction = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_9) - HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_10);
		break;
	case MOUSE_INPUT:
		paddle_direction = (0 < mouse_vx) - (mouse_vx < 0);
		break;
	case TOUCH_INPUT:
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 10);
		uint16_t analog_value = HAL_ADC_GetValue(&hadc1);
		const uint16_t MIDDLE_POS = 2042;
		paddle_direction = (MIDDLE_POS < analog_value) - (analog_value < MIDDLE_POS);
		break;
	}

	const int PADDLE_STEP = 8;

	int new_left = paddle_sprite.left + paddle_direction * PADDLE_STEP;

	paddle_direction *= (paddle_direction < 0 && 0 <= new_left) || (0 < paddle_direction && new_left + paddle_sprite.width <= screen.width);

	if (paddle_direction != 0) { // i.e. the paddle moved
		// Clear paddle
		clear_sprite(&paddle_sprite, &screen);

		// Move paddle
		increment_sprite_position(&paddle_sprite, paddle_direction * PADDLE_STEP, 0);
	}
}


void handle_ball_mechanics() {

	// 1. Save the ball's current position
	int old_left = ball_sprite.left;
	int old_top = ball_sprite.top;

	// Clear old ball from screen
	clear_sprite(&ball_sprite, &screen);

	// 2. Update the ball's position
	const unsigned int BALL_STEP = 10;
	step_sprite_in_direction(&ball_sprite, BALL_STEP, ball_angle);

	// 3. Handle ball's collisions
	check_deflect_off_bounary(&ball_sprite, &ball_angle, 0, screen.width, TOP_OFFSET, screen.height);

	if (check_deflect_off_sprite(&ball_sprite, &paddle_sprite, &ball_angle, old_left, old_top)) {
		// Adjust balls angle
		// NOTE: On exit the ball's angle should be between 0 and 180
		if (paddle_direction < 0) // Paddle going left
			ball_angle = (ball_angle < 90) ? ball_angle / 2 : 180 - (180 - ball_angle) * 2;
		if (0 < paddle_direction) // Paddle going right
			ball_angle = (ball_angle < 90) ? ball_angle * 2 : 180 - (180 - ball_angle) / 2;
	}

	for (int i = 0; i < BRICK_COLUMN_SIZE * BRICK_ROW_SIZE; i++) {
		sprite_t* brick_sprite = brick_sprites + i;

		if (check_deflect_off_sprite(&ball_sprite, brick_sprite, &ball_angle, old_left, old_top)) {
			// Decrease brick lives
			brick_sprite->hp--;

			if (!sprite_is_alive(brick_sprite)) {
				// Clear the brick
				clear_sprite(brick_sprite, &screen);

				// Play sound
				play_sound(brick_break_sound, BRICK_BREAK_SOUND_LEN);

				// Update player score if the brick broke
				switch (i / BRICK_COLUMN_SIZE) {
				case 0: // Grey row
					player_score += 10;
					break;
				case 1: // Red row
					player_score += 7;
					break;
				case 2: // Orange row
					player_score += 5;
					break;
				case 3: // Yellow row
					player_score += 3;
					break;
				case 4: // Green row
					player_score += 1;
					break;
				}

				// Activate powerup
				if (num_powerups > 0) {
					ext_powerup_sprite.hp = 1;
					set_sprite_position(&ext_powerup_sprite, brick_sprite->left, brick_sprite->top);

					num_powerups--;
				}
			}
		}
	}

	// 4. Pray that no glitches occur
}


void handle_powerup_mechanics() {
	// Extender powerup
	if (ext_powerup_sprite.hp != 0) {
		clear_sprite(&ext_powerup_sprite, &screen);
		if (ext_powerup_sprite.top + ext_powerup_sprite.height <= screen.height)
			increment_sprite_position(&ext_powerup_sprite, 0, 2);
		else
			ext_powerup_sprite.hp = 0;

		// check that powerup was captured by paddle
		int inbetween_paddle = paddle_sprite.left < ext_powerup_sprite.left + ext_powerup_sprite.width && ext_powerup_sprite.left < paddle_sprite.left + paddle_sprite.width;
		if (inbetween_paddle && paddle_sprite.top <= ext_powerup_sprite.top + ext_powerup_sprite.height) {

			paddle_sprite.image = ext_paddle_image;
			paddle_sprite.width = EXT_PADDLE_WIDTH;
			paddle_sprite.height = EXT_PADDLE_HEIGHT;
			increment_sprite_position(&paddle_sprite, -10, 0);

			ext_powerup_sprite.hp = 0;
		}
	}
}


void update_score() {
	display_number(player_score, 300, 13);
}


int game_won() {
	for (int i = 0; i < BRICK_COLUMN_SIZE * BRICK_ROW_SIZE; i++) {
		if (brick_sprites[i].hp != 0)
			return 0;
	}
	return 1;
}


int game_lost() {
	return paddle_sprite.hp <= 0;
}


void play_sound(const uint16_t* sound, uint16_t length) {
	HAL_I2S_Transmit_DMA(&hi2s3, (uint16_t*) sound, length);
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
