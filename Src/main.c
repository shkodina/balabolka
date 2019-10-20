/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "fatfs.h"


/* USER CODE BEGIN Includes */
#include <string.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac;
DMA_HandleTypeDef hdma_dac_ch2;

SD_HandleTypeDef hsd;
HAL_SD_CardInfoTypedef SDCardInfo;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
//============================================================================
//	DEFINES			DEFINES			DEFINES			DEFINES			DEFINES			DEFINES					
//============================================================================	
#include "pip_define.h"
//----------------------------------------------------------------------------
enum BOOL {FALSE = 0, TRUE = 1};
//----------------------------------------------------------------------------
uint8_t thread_player_is_runing = FALSE;
//----------------------------------------------------------------------------
FATFS fatfs;  /* File system object for SD card logical drive */
FIL file;     /* File object */
char SDPath[4]; /* SD card logical drive path */
FRESULT res;
//----------------------------------------------------------------------------
#define BUFFLEN 1024
int bufflen = 	BUFFLEN;	
volatile char cur_buff = 0;
volatile static int16_t buff[2][BUFFLEN];	
volatile uint16_t buff_play_len[2];
//----------------------------------------------------------------------------
volatile char need_stop = FALSE;
volatile char need_update_buffer = FALSE;
//volatile int16_t addon = 2047 << 4;
#define addon (2047 << 4)
//----------------------------------------------------------------------------
#define ADC_CHANNELS_COUNT 2
enum ADCCH {adc_v_light = 0, adc_v_resist};
uint32_t adc_v[ADC_CHANNELS_COUNT] = {0};
//----------------------------------------------------------------------------
#define GROUPS_COUNT 10
#define MAX_FILE_COUNT_PER_GROUP 100
uint16_t files[GROUPS_COUNT][MAX_FILE_COUNT_PER_GROUP];
uint8_t files_count[GROUPS_COUNT] = {0};
//----------------------------------------------------------------------------
enum Random {NOTRANDOM, RANDOM};


// PAUSE PERIODS !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
enum DelayPool {DELAY_POOL_0 = 0, DELAY_POOL_1 = 1, DELAY_POOL_2 = 2, DELAY_POOL_3 = 3};
uint16_t delay_pool_bounds[4][2] = {{10, 15}, {20, 30}, {40, 50}, {60, 70}}; 
//uint16_t delay_pool_bounds[4][2] = {{10, 60}, {60, 240}, {240, 600}, {600, 1800}}; 


#define DELAY_TIMER_NO_SET 0
enum TimePlayMode {DAY = 0, NIGHT, DAY_AND_NIGHT};
volatile char 		is_need_play = FALSE;
volatile static uint16_t next_track = 0;
volatile uint16_t need_paly_track_number = 0;
//----------------------------------------------------------------------------
struct Settings {
	char is_random;
	char delay_pool;
	char time_play_mode;
	char selected_groups [GROUPS_COUNT];
	char selected_groups_count;
	uint16_t total_track_count;
	char is_updated;
	char is_power_on;
} machine_settings;
//----------------------------------------------------------------------------
enum BUTTONS {BUTTON_POWER = 0,
							BUTTON_RANDOM,
							BUTTON_PAUSE,
							BUTTON_MODE,
							BUTTON_GROUP_0,
							BUTTON_GROUP_1,
							BUTTON_GROUP_2,
							BUTTON_GROUP_3,
							BUTTON_GROUP_4,
							BUTTON_GROUP_5,
							BUTTON_GROUP_6,
							BUTTON_GROUP_7,
							BUTTON_GROUP_8,
							BUTTON_GROUP_9
};
#define BUTTONS_COUNT 14
volatile GPIO_PinState last_button_state[BUTTONS_COUNT] = {0};

#define BUTTON_PRESSED GPIO_PIN_RESET
#define BUTTON_NOT_PRESSED GPIO_PIN_SET
//============================================================================
//	DEFINES			DEFINES			DEFINES			DEFINES			DEFINES			DEFINES						
//============================================================================																		
//============================================================================
//	LOG			LOG			LOG			LOG			LOG			LOG			LOG			LOG			LOG			
//============================================================================																		
#ifdef LOGDEBUG
char str[128];
void Log(char * text){
	sprintf(str, "%s", text);
	HAL_UART_Transmit(&huart3, (uint8_t *)str, strlen(str), 1000);
}
#else
inline void Log(char * text){};
#endif
//============================================================================
//	LOG			LOG			LOG			LOG			LOG			LOG			LOG			LOG			LOG
//============================================================================
//============================================================================
//	funck		funck		funck		funck		funck		funck		funck		funck		funck					
//============================================================================
FRESULT m_0_scan_files ( char* path );

void pl_00_play_wav(char *);
void pl_01_play_stop(void);
void pl_02_next_buf(void);
void pl_04_prepare_buffer(int len);
void pl_05_amp_on(void);
void pl_06_amp_off(void);

uint16_t xx_00_random_between(uint16_t min, uint16_t max);

void mx_00_machine_init(void);
void mx_01_read_settings_from_eeprom(void);
void mx_02_check_buttons_and_update_settings(void);
void mx_03_write_settings_to_eeprom(void);
void mx_04_light_leds_by_settings(void);
char mx_05_check_light(void);
void mx_06_light_leds_off(void);
void mx_08_light_leds_for_shutdown_off(void);
void mx_09_blink_power_led(void);
uint16_t mx_07_select_track(uint16_t);

void xx_01_poll_adc(void);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc); //overload for tests

void fl_01_flash_unlock(void);
void fl_02_erase_page_by_address(uint32_t address_from_page);
void fl_03_write_data(uint32_t address, uint16_t value);
void fl_04_flash_lock(void);
uint16_t fl_05_read_data(uint32_t);

//============================================================================
//	funck		funck		funck		funck		funck		funck		funck		funck		funck
//============================================================================
//============================================================================
//	STATE MACHINE		STATE MACHINE		STATE MACHINE		STATE MACHINE		
//============================================================================
enum States {INIT, START, CHECK_BUTTONS, LIGHT_LEDS, CHECK_LIGHTS, CHECK_PLAY, SELECT_DELAY, MAKE_DELAY, SELECT_WAV, PLAY, STOP, SHUTING_DOWN, POWER_OFF};
//============================================================================
char mx____INIT(){
	Log("-- INIT --\n\r");
	mx_00_machine_init();
	//mx_04_light_leds_by_settings();	
	pl_05_amp_on();  	
	return START;
}
//============================================================================
char mx____START(uint32_t * timer){
	Log("-- START --\n\r");
	//return LIGHT_LEDS;
	//return SELECT_WAV;

  *timer = 10;
  return MAKE_DELAY;
}
//============================================================================
void mx____machine_step(){ // poll every 10ms
	//HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);	
  //static uint32_t timer = DELAY_TIMER_NO_SET;
	static uint32_t timer = 10;
	static char state = INIT;
	
	
	switch (state){
		case INIT:
			state = mx____INIT();
			break;
		
		case START:
		  state = mx____START(&timer);
			break;
		
		case CHECK_BUTTONS: 
			#ifdef FULLOGDEFINE
				Log("-- CHECK_BUTTONS --\n\r");
		  #endif
			mx_02_check_buttons_and_update_settings();
			{
				static char is_power_on_lstate = FALSE;
				
				if (machine_settings.is_power_on == FALSE){
					if (is_power_on_lstate == TRUE) {
						state = SHUTING_DOWN;
					}else{
						state = POWER_OFF;
					}
					is_power_on_lstate = machine_settings.is_power_on;
					break;
				}else{
					if(is_power_on_lstate == FALSE){
						Log("---- WAIKE UP --\n\r");
						is_power_on_lstate = machine_settings.is_power_on;
						pl_05_amp_on(); 
						state = LIGHT_LEDS;	
						
						if (machine_settings.is_random == RANDOM){
							Log("------ IN RANDOM --\n\r");
							state = SELECT_WAV;
						}else{
							Log("------ IN 0 TRACK --\n\r");
							next_track = -1;
							state = SELECT_WAV;
							//next_track = 0;
							//mx_07_select_track(next_track);
							//state = PLAY;
						}
						//state = SELECT_WAV;
						//state = MAKE_DELAY;
						break;
					}
					is_power_on_lstate = machine_settings.is_power_on;
				}	
			}
		
			if (machine_settings.is_updated == TRUE){
				mx_03_write_settings_to_eeprom();
				machine_settings.is_updated = FALSE;
				state = LIGHT_LEDS;
				break;
			}
			state = CHECK_LIGHTS;
			break;
			
		case LIGHT_LEDS:
			Log("-- LIGHT_LEDS --\n\r");
			mx_04_light_leds_by_settings();
			state = CHECK_LIGHTS;
			break;
		
		case CHECK_LIGHTS:
			#ifdef FULLOGDEFINE
			  Log("-- CHECK_LIGHTS --\n\r");
		  #endif
		{
			char light = mx_05_check_light();
			static char is_need_go_start = TRUE;
			
			if(light == DAY && machine_settings.time_play_mode == NIGHT){
				state = 	CHECK_BUTTONS;
				is_need_play = FALSE;
				is_need_go_start = TRUE;
				break;
			}		
			
			if(light == NIGHT && machine_settings.time_play_mode == DAY){
				state = 	CHECK_BUTTONS;
				is_need_play = FALSE;
				is_need_go_start = TRUE;
				break;
			}
			
			if (machine_settings.time_play_mode == DAY_AND_NIGHT){
				if (light == DAY){
					HAL_GPIO_WritePin(PORT_LED_MODE_DAY, PIN_LED_MODE_DAY, GPIO_PIN_SET);	
					HAL_GPIO_WritePin(PORT_LED_MODE_NIGHT, PIN_LED_MODE_NIGHT, GPIO_PIN_RESET);	
				}
				if (light == NIGHT){
					HAL_GPIO_WritePin(PORT_LED_MODE_DAY, PIN_LED_MODE_DAY, GPIO_PIN_RESET);	
					HAL_GPIO_WritePin(PORT_LED_MODE_NIGHT, PIN_LED_MODE_NIGHT, GPIO_PIN_SET);	
				}
			}
			
			if (is_need_go_start == TRUE){
				is_need_go_start = FALSE;
				state = START;
				break;
			}
		}
			
			state = CHECK_PLAY;
			break;
				
		case CHECK_PLAY:
			#ifdef FULLOGDEFINE
			  Log("-- CHECK_PLAY --\n\r");
		  #endif
		
			if (is_need_play == TRUE){
				state = CHECK_BUTTONS;
				//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);	
			}else{
				state = SELECT_DELAY;
				//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
			}
		break;
		
		case SELECT_DELAY:
			#ifdef FULLOGDEFINE
			  Log("-- SELECT_DELAY --\n\r");
		  #endif
			if (timer == DELAY_TIMER_NO_SET){
				Log("-- SELECT new read delay in SELECT DELAY --\n\r");
				timer = xx_00_random_between(	delay_pool_bounds[machine_settings.delay_pool][0], 
																			delay_pool_bounds[machine_settings.delay_pool][1]);
				timer *= 10;
			}
			state = MAKE_DELAY;
			break;
			
		case MAKE_DELAY:
			#ifdef FULLOGDEFINE
			  Log("-- MAKE_DELAY --\n\r");
		  #endif
			if(--timer == DELAY_TIMER_NO_SET){ // time to play next
				state = SELECT_WAV;
			}else{
				state = CHECK_BUTTONS;
			}
			break;
			
		case SELECT_WAV:
			Log("-- SELECT_WAV --\n\r");
		
		{
			if(machine_settings.selected_groups_count == 0){
				state = CHECK_BUTTONS;
				break;
			}
			
			//static uint16_t next_track = -1;
			if (machine_settings.is_random == RANDOM){
				next_track = xx_00_random_between(0, machine_settings.total_track_count);
			}else{
				if (next_track++ >= machine_settings.total_track_count){
					next_track = 0;
				}
			}
			
			mx_07_select_track(next_track);
		}
			state = PLAY;
			break;
	
		case PLAY:
			Log("-- PLAY --\n\r");
			is_need_play = TRUE;
			state = CHECK_BUTTONS;
			break;
		case STOP:
			break;
		case SHUTING_DOWN:
			Log("-- SHUTING_DOWN --\n\r");
		  is_need_play = FALSE;
			mx_08_light_leds_for_shutdown_off();	
		  pl_06_amp_off();
			if (thread_player_is_runing == TRUE){
				mx_09_blink_power_led();
				state = SHUTING_DOWN;
			}else{
				mx_06_light_leds_off();	
				state = POWER_OFF;
			}
		  break;
		case POWER_OFF:
			Log("-- POWER_OFF --\n\r");
		  // sleep? wait?
			state = CHECK_BUTTONS;
			break;
		default:
			break;
	}
}
//============================================================================
//	STATE MACHINE		STATE MACHINE		STATE MACHINE		STATE MACHINE		
//============================================================================

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_SDIO_SD_Init();
  MX_FATFS_Init();
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  MX_DAC_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();

  /* USER CODE BEGIN 2 */
	
	if(f_mount(&fatfs, SDPath, 0) == FR_OK)
  {
		Log("mount ok\n\r");
		char buff[64];
		strcpy(buff, SDPath);

		m_0_scan_files(buff);	
		
		Log("collect files ok\n\r");
	}else{
		Log("mount error\n\r");
		Error_Handler();		
	}

	HAL_ADC_Start_DMA(&hadc1,adc_v,ADC_CHANNELS_COUNT);
	HAL_TIM_Base_Start(&htim7);
	HAL_TIM_Base_Start_IT(&htim7);
	HAL_TIM_Base_Start(&htim6);
	HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
	HAL_DAC_Start(&hdac, DAC_CHANNEL_2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	//pl_00_play_fake_wav();
  while (1)
  {
			if (is_need_play == TRUE){
				thread_player_is_runing = TRUE;
				char fname[64] = {0};
				sprintf(fname, "%04d.WAV", need_paly_track_number);
				
				Log("++++ try to play ");
				Log(fname);
				Log("\n\r\n\r");
				//HAL_TIM_Base_Start(&htim6);
				pl_00_play_wav(fname);
				pl_01_play_stop();
				
				is_need_play = FALSE;
				HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2,DAC_ALIGN_12B_R, 2048);
				HAL_DAC_Start(&hdac, DAC_CHANNEL_2);
				thread_player_is_runing = FALSE;
			}
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* DAC init function */
static void MX_DAC_Init(void)
{

  DAC_ChannelConfTypeDef sConfig;

    /**DAC Initialization 
    */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**DAC channel OUT1 config 
    */
  sConfig.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Noise wave generation on DAC OUT1 
    */
  if (HAL_DACEx_NoiseWaveGenerate(&hdac, DAC_CHANNEL_1, DAC_LFSRUNMASK_BITS9_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**DAC channel OUT2 config 
    */
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SDIO init function */
static void MX_SDIO_SD_Init(void)
{

  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 8;

}

/* TIM6 init function */
static void MX_TIM6_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 500;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 1;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM7 init function */
static void MX_TIM7_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 31999;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 20;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA2_Channel4_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel4_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel4_5_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_0|GPIO_PIN_3 
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_10 
                          |GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14 
                          |GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_7|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC0 PC3 
                           PC5 PC6 PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_0|GPIO_PIN_3 
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC15 PC1 PC2 PC4 */
  GPIO_InitStruct.Pin = GPIO_PIN_15|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA2 PA6 PA9 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_6|GPIO_PIN_9|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA3 PA7 PA8 PA10 
                           PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_10 
                          |GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB2 PB15 PB4 
                           PB6 PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_15|GPIO_PIN_4 
                          |GPIO_PIN_6|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB12 PB13 PB14 
                           PB3 PB5 PB7 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14 
                          |GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_7|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
//==========================================================================================
//	MY FUNCK		MY FUNCK		MY FUNCK		MY FUNCK		MY FUNCK		MY FUNCK		MY FUNCK
//==========================================================================================
void pl_00_play_wav_on_exit(){
		need_stop = 1;
		HAL_TIM_Base_Start(&htim7);
		Log("Stop Play\n\r");
}
void pl_00_play_wav(char * filename){
	
	if(f_open(&file, filename, FA_READ) != FR_OK){
		Log("open read error\n\r");
		pl_00_play_wav_on_exit();
	}	
	Log("open read ok\n\r");

	if(f_lseek(&file,0x2c) != FR_OK){
		Log("lseek error\n\r");
		f_close(&file);		
		pl_00_play_wav_on_exit();
	}
	Log("lseek ok\n\r");
				
	uint32_t bytesread;				
	
	cur_buff = 0;
	// START PLAY
	// HAL_TIM_Base_Start(&htim6);
	
	if(f_read(&file, buff[cur_buff], bufflen*2, &bytesread) == FR_OK){
		HAL_DAC_Start(&hdac, DAC_CHANNEL_2);
		pl_04_prepare_buffer(bufflen);
		need_stop = 0;
		
		
		pl_02_next_buf();	
		
	}else{
			// ERROR MAY BE ??
		Log("! ! ! ! f_read error\n\r");
	}
		
	// START PREPARE BYFFER
	while ((f_read(&file, buff[cur_buff], bufflen*2, &bytesread) == FR_OK)
					&&
					(bytesread == bufflen*2)
					&&
					need_stop != 1)
	{
		
			HAL_TIM_Base_Start(&htim7);
			pl_04_prepare_buffer(bufflen);
			need_update_buffer  = 0;

			
			while (need_update_buffer == 0
							&&
						need_stop != 1 ){
				
				if (is_need_play == FALSE){
					Log("==== USER EXIT set is_need_play = FALSE while play\n\r");
					need_stop = 1;
				}
				
				//if (need_stop) break;
			}
						
			HAL_TIM_Base_Stop(&htim7);
	}
	
	
	
	if(bytesread != bufflen*2){
		Log("bytesread != bufflen*2\n\r");
	}else{
		Log("f_read != FR_OK\n\r");
	}
	
	f_close(&file);
	pl_00_play_wav_on_exit();
		
}
//==========================================================================================
void pl_01_play_stop(){
	Log("++++ PLAY STOP in pl_01_play_stop\n\r");
	need_stop = 1;
	HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_2);
	HAL_DAC_Stop(&hdac, DAC_CHANNEL_2);
	//HAL_TIM_Base_Stop(&htim6);
	cur_buff=0;
	
	memset(buff[0], 0, BUFFLEN*2);
	memset(buff[1], 0, BUFFLEN*2);
}
//==========================================================================================
void pl_02_next_buf(){
	if (need_stop){ 
		HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_2);
		HAL_DAC_Stop(&hdac, DAC_CHANNEL_2);
		return;
	}

	/*
	HAL_UART_Transmit(&huart3, (uint8_t *)buff[cur_buff], buff_play_len[cur_buff]*2, 1000);
	Log("                ");
	*/
	/*
	if (buff[cur_buff][buff_play_len[cur_buff] - 3] == 0)
	{
		need_stop = 1;
		char st[64];
		sprintf(st, "EXIT buff[cur_buff][buff_play_len[cur_buff] - 3] = %d \n\r", 
		buff[cur_buff][buff_play_len[cur_buff] - 3]);
		Log(st);
		return;
	}
	*/
	/*
	uint64_t bfavr = 0;
	for (int i = 0; i < buff_play_len[cur_buff]; i++){
			bfavr += buff[cur_buff][i];
	}
	
	if (bfavr / buff_play_len[cur_buff] < (1024 << 4)) {
		HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_2);
		HAL_DAC_Stop(&hdac, DAC_CHANNEL_2);
		need_stop = TRUE;
		return;
	}
	*/
	
	/*
	HAL_UART_Transmit(&huart3, (uint8_t *)buff[cur_buff], buff_play_len[cur_buff]*2, 1000);
	Log("                ");
	*/
	HAL_TIM_Base_Stop(&htim6);
	HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_2, (uint32_t*)buff[cur_buff], buff_play_len[cur_buff]*2, DAC_ALIGN_12B_L);
	HAL_TIM_Base_Start(&htim6);
	cur_buff++;
	cur_buff &= 1;
	need_update_buffer = TRUE;
}
//==========================================================================================
FRESULT m_0_scan_files (
    char* path        /* Start node to be scanned (***also used as work area***) */
)
{
	Log("Start scan files \n\r");
    FRESULT res;
    DIR dir;
    static FILINFO fno;
	
	
		//prepare arrays
		for (int i = 0; i < GROUPS_COUNT; i++){
			files_count[i] = 0;
			for (int j = 0; j < MAX_FILE_COUNT_PER_GROUP; j++){
				files[i][j] = 0;
			}
		}

    res = f_opendir(&dir, path);                       /* Open the directory */
    if (res == FR_OK) {
        for (;;) {
            res = f_readdir(&dir, &fno);                   /* Read a directory item */
            if (res != FR_OK || fno.fname[0] == 0) break;  /* Break on error or end of dir */
            if (fno.fattrib & AM_DIR) {                    /* It is a directory */
							;
            } else {                                       /* It is a file. */
							Log("Find file\n\r");
								//sprintf(str, "%s\r\n",fno.fname);
								//HAL_UART_Transmit(&huart2, str, strlen(str), 1000);
							Log(fno.fname);
							//Log("before search 0 \n\r");
                //sprintf("%s/%s\n", path, fno.fname);
							//Log("before search 1 \n\r");
								if (strstr(fno.fname, ".WAV")){
									Log("found WAV \n\r");
									
									char snum[4];
									int num = 0;
									memcpy(snum, fno.fname, 4);
									sscanf(snum, "%d", &num);
														
									char cur_group = num / 100;
									files_count[cur_group]++;
									files[cur_group][files_count[cur_group] - 1] = num;
									Log("---------\n\r");
								}
								//Log("after search \n\r");
            }
        }
        f_closedir(&dir);
    }else{
			Log("Open dir ERROR\n\r");
		}

		for (int i = 0; i < GROUPS_COUNT; i++){
			char grstr[64];
			sprintf(grstr, "in group %d files %d \n\r", i, files_count[i]);
			Log(grstr);
		}

		
    return res;
}
//==========================================================================================
void pl_04_prepare_buffer(int len){
	/*
	HAL_UART_Transmit(&huart3, (uint8_t *)buff[cur_buff], len*2, 1000);
	Log("                ");
  */
	
	char n = cur_buff;
	for (int i = 0; i < len; i++){
		buff[n][i] = buff[n][i] + addon;
	};
	buff_play_len[n] = len;	
	
	/*
  HAL_UART_Transmit(&huart3, (uint8_t *)buff[n], buff_play_len[n], 1000);
	Log("************");
	*/
}
//==========================================================================================
uint16_t xx_00_random_between(uint16_t min, uint16_t max){
	
	int d = max - min + 1;
	int v = 0;
	do {
		for(int x = 0; x < adc_v[adc_v_light]*100; x++);
		v = HAL_DAC_GetValue(&hdac, DAC_CHANNEL_1);
//		char st[64];
//		sprintf(st, "v=%d \n\r", v);
//		Log(st);
		
	}while (v < d);
	
	int rnd = v % d;
//	int rnd = v / d;
//	int r = min + (v - rnd*d);
	int r = min + rnd;
	// 	TODO
		char st[64];
		sprintf(st, "my random min=%d max=%d random=%d result=%d rnd=%d delta=%d\n\r", min, max, v, r, rnd, d);
		Log(st);
	return r;
}
//==========================================================================================
void mx_0_log_machine_settings(){
	char st[128];
	sprintf(st, "is_random=%d\n\rdelay_pool=%d\n\rtime_play_mode=%d\n\rselected_groups_count=%d\n\ris_power_on=%d\n\r", 
	machine_settings.is_random,
	machine_settings.delay_pool,
	machine_settings.time_play_mode,
	machine_settings.selected_groups_count,
	machine_settings.is_power_on);
	Log(st);	
	
	Log("Selected groups: [");
	for ( int i = 0; i < machine_settings.selected_groups_count; i++){
		sprintf(st, " %d", machine_settings.selected_groups[i]);
		Log(st);
	}
	Log(" ]\n\r");
}
//=================================
void mx_00_machine_init(){
/*
	// DEBUG
	machine_settings.is_random = RANDOM; // RANDOM NOTRANDOM
	machine_settings.delay_pool = DELAY_POOL_1;
	machine_settings.time_play_mode = DAY_AND_NIGHT;
	
	machine_settings.selected_groups[0] = 0;
	machine_settings.selected_groups[1] = 1;
	machine_settings.selected_groups[2] = 2;
	machine_settings.selected_groups[3] = 3;
	machine_settings.selected_groups[4] = 4;
	machine_settings.selected_groups[5] = 5;
	machine_settings.selected_groups[6] = 6;
	machine_settings.selected_groups[7] = 7;
	machine_settings.selected_groups[8] = 8;
	machine_settings.selected_groups[9] = 9;
	machine_settings.selected_groups_count = 1;

	machine_settings.is_power_on = TRUE;
	machine_settings.is_updated = FALSE;
*/
	mx_01_read_settings_from_eeprom();

	Log("\n\r read from eeprom\n\r");
  mx_0_log_machine_settings();
	
	// count files to play
	for(char i = 0; i < machine_settings.selected_groups_count; i++){
		machine_settings.total_track_count += files_count[ machine_settings.selected_groups[i] ];
	}
	
	for (int i = 0; i < BUTTONS_COUNT; i++){
		last_button_state[i] = BUTTON_NOT_PRESSED;
	}
}
//==========================================================================================
void mx_01_read_settings_from_eeprom()
{
/*	{
		char st[64];
		sprintf(
			st,
			"FROM EEPROM rnd=%d pool=%d tplm=%d sgc=%d \n\r",
			fl_05_read_data(FlashStartAddress + FLASH_ADDR_SHIFT_is_random),
			fl_05_read_data(FlashStartAddress + FLASH_ADDR_SHIFT_delay_pool),
			fl_05_read_data(FlashStartAddress + FLASH_ADDR_SHIFT_time_play_mode),
			fl_05_read_data(FlashStartAddress + FLASH_ADDR_SHIFT_selected_groups_count)	
		);
		Log(st);
		
		machine_settings.is_power_on = TRUE;
		machine_settings.is_updated = TRUE;
		
		return;
	}
*/	
	
	machine_settings.is_random = 
		fl_05_read_data(FlashStartAddress + FLASH_ADDR_SHIFT_is_random);
	if (machine_settings.is_random > 1) machine_settings.is_random = 1;
	
	machine_settings.delay_pool = 
			fl_05_read_data(FlashStartAddress + FLASH_ADDR_SHIFT_delay_pool);
	if (machine_settings.delay_pool > 3) machine_settings.delay_pool = 1;
	
	machine_settings.time_play_mode = 
			*(__IO uint16_t*)(FlashStartAddress + FLASH_ADDR_SHIFT_time_play_mode);
	if (machine_settings.time_play_mode > 2) machine_settings.time_play_mode = 1;

	machine_settings.is_power_on = 
			*(__IO uint16_t*)(FlashStartAddress + FLASH_ADDR_SHIFT_is_power_on);
	if (machine_settings.is_power_on > 2) machine_settings.is_power_on = FALSE;

	machine_settings.selected_groups_count = 
			fl_05_read_data(FlashStartAddress + FLASH_ADDR_SHIFT_selected_groups_count);
	if (machine_settings.selected_groups_count > 10) machine_settings.selected_groups_count = 0;
		
	if (machine_settings.selected_groups_count > 0){	
		for ( int i = 0; i < machine_settings.selected_groups_count; i++){
			machine_settings.selected_groups[i] = 
				fl_05_read_data(FlashStartAddress + FLASH_ADDR_SHIFT_selected_groups + i*2);
			if (machine_settings.selected_groups[i] > 9){
				machine_settings.selected_groups[i] = 0;
			}
		}
	}

	machine_settings.is_updated = TRUE;
}
//==========================================================================================
char validate_button_pressed(int i, GPIO_PinState cur_pin_state){
  if (last_button_state[i] == cur_pin_state){
		return FALSE;
  }else{
		if (last_button_state[i] == BUTTON_PRESSED && cur_pin_state == BUTTON_NOT_PRESSED){
			last_button_state[i] = cur_pin_state;
			return TRUE;
		}else{
			last_button_state[i] = cur_pin_state;
			return FALSE;
		}
  }
}
//-----------------------------------------------
void sort_inc_group(){
	for ( int i = 0 ; i < machine_settings.selected_groups_count; i++){
		for (int j = 0; j < machine_settings.selected_groups_count-1; j++){
			if (machine_settings.selected_groups[j] > 
				  machine_settings.selected_groups[j + 1]){
				uint16_t b = machine_settings.selected_groups[j];
				machine_settings.selected_groups[j] = machine_settings.selected_groups[j+1];
				machine_settings.selected_groups[j+1] = b;		
			}
		}
	}
}
//-------------------------------------------------------------------------
void toggle_group (char group_number){
  if (files_count[group_number] == 0){
    return; // no files in this group
  }
	
  for (int i = 0; i < machine_settings.selected_groups_count; i++){
    if (machine_settings.selected_groups[i] == group_number){ // delete, recalc total and exit
			machine_settings.selected_groups_count--; // delete
			for (int j = i; j < machine_settings.selected_groups_count ; j++){ // shift all groups left to one position
				machine_settings.selected_groups[j] = machine_settings.selected_groups[j+1];
			}
			machine_settings.total_track_count -= files_count[group_number]; // recalc total
			sort_inc_group();
      return; // exit
		}
  }  
  
	// if we are here, we should add new group
	machine_settings.selected_groups[machine_settings.selected_groups_count] = group_number;
	machine_settings.selected_groups_count++;
	machine_settings.total_track_count += files_count[group_number];
	sort_inc_group();
	/*
	Log("Selected groups: ");
	char str[64];
	for (int i =0; i < machine_settings.selected_groups_count; i++){
		sprintf(str, " %d", machine_settings.selected_groups[i]);
		Log(str);
	}
	Log("\n\r");
	*/
	return;
/*	
  for (int i = 0; i < machine_settings.selected_groups_count; i++){
		if (group_number < machine_settings.selected_groups[i]){ // insert, shift right, recalc total and exit
			for (int j = machine_settings.selected_groups_count; j >= i; j--){ // shift right
				machine_settings.selected_groups[j] = machine_settings.selected_groups[j-1];
			}
			machine_settings.selected_groups[i] = group_number; // insert
			machine_settings.total_track_count += files_count[group_number]; // recalc total
			return; // exit
		}
  } 
	*/
}
//-------------------------------------------------------------------------
void mx_02_check_buttons_and_update_settings(){
for (int i = 0; i < BUTTONS_COUNT; i++){
  switch(i){
    case BUTTON_POWER:
      if ( validate_button_pressed (i, HAL_GPIO_ReadPin(PORT_BUTTON_POWER, PIN_BUTTON_POWER)) == TRUE){
        if (machine_settings.is_power_on == TRUE){
					machine_settings.is_power_on = FALSE;
					machine_settings.is_updated = TRUE;
				}else{
					machine_settings.is_power_on = TRUE;
				}
				mx_03_write_settings_to_eeprom();
				//Log("BUTTON_POWER pressed\n\r");
      }
    break;
	
    case BUTTON_RANDOM:
      if ( validate_button_pressed (i, HAL_GPIO_ReadPin(PORT_BUTTON_RANDOM, PIN_BUTTON_RANDOM)) == TRUE){
        if (machine_settings.is_random == RANDOM){
          machine_settings.is_random = NOTRANDOM;
					next_track = -1;
					//mx_07_select_track(next_track);
        }else{
          machine_settings.is_random = RANDOM;
        }
        machine_settings.is_updated = TRUE;
      }
    break;
	
    case BUTTON_PAUSE:
      if ( validate_button_pressed (i, HAL_GPIO_ReadPin(PORT_BUTTON_PAUSE, PIN_BUTTON_PAUSE)) == TRUE){
        machine_settings.delay_pool++;
				if (machine_settings.delay_pool > DELAY_POOL_3){
					machine_settings.delay_pool = DELAY_POOL_0;
				}
        machine_settings.is_updated = TRUE;
      }
    break;
	
    case BUTTON_MODE:
      if ( validate_button_pressed (i, HAL_GPIO_ReadPin(PORT_BUTTON_MODE, PIN_BUTTON_MODE)) == TRUE){
        machine_settings.time_play_mode++;
		if (machine_settings.time_play_mode > DAY_AND_NIGHT){
		  machine_settings.time_play_mode = DAY;
		}
        machine_settings.is_updated = TRUE;
      }
    break;
	
    case BUTTON_GROUP_0:
      if ( validate_button_pressed (i, HAL_GPIO_ReadPin(PORT_BUTTON_GROUP_0, PIN_BUTTON_GROUP_0)) == TRUE){
        toggle_group(0);
        machine_settings.is_updated = TRUE;
      }
    break;
    case BUTTON_GROUP_1:
     if ( validate_button_pressed (i, HAL_GPIO_ReadPin(PORT_BUTTON_GROUP_1, PIN_BUTTON_GROUP_1)) == TRUE){
        toggle_group(1);
        machine_settings.is_updated = TRUE;
      }
    break;
    case BUTTON_GROUP_2:
      if ( validate_button_pressed (i, HAL_GPIO_ReadPin(PORT_BUTTON_GROUP_2, PIN_BUTTON_GROUP_2)) == TRUE){
        toggle_group(2);
        machine_settings.is_updated = TRUE;
      }
    break;
    case BUTTON_GROUP_3:
      if ( validate_button_pressed (i, HAL_GPIO_ReadPin(PORT_BUTTON_GROUP_3, PIN_BUTTON_GROUP_3)) == TRUE){
        toggle_group(3);
        machine_settings.is_updated = TRUE;
      }
    break;
    case BUTTON_GROUP_4:
      if ( validate_button_pressed (i, HAL_GPIO_ReadPin(PORT_BUTTON_GROUP_4, PIN_BUTTON_GROUP_4)) == TRUE){
        toggle_group(4);
        machine_settings.is_updated = TRUE;
      }
    break;
    case BUTTON_GROUP_5:
      if ( validate_button_pressed (i, HAL_GPIO_ReadPin(PORT_BUTTON_GROUP_5, PIN_BUTTON_GROUP_5)) == TRUE){
        toggle_group(5);
        machine_settings.is_updated = TRUE;
      }
    break;
    case BUTTON_GROUP_6:
      if ( validate_button_pressed (i, HAL_GPIO_ReadPin(PORT_BUTTON_GROUP_6, PIN_BUTTON_GROUP_6)) == TRUE){
        toggle_group(6);
        machine_settings.is_updated = TRUE;
      }
    break;
    case BUTTON_GROUP_7:
      if ( validate_button_pressed (i, HAL_GPIO_ReadPin(PORT_BUTTON_GROUP_7, PIN_BUTTON_GROUP_7)) == TRUE){
        toggle_group(7);
        machine_settings.is_updated = TRUE;
      }
    break;
    case BUTTON_GROUP_8:
       if ( validate_button_pressed (i, HAL_GPIO_ReadPin(PORT_BUTTON_GROUP_8, PIN_BUTTON_GROUP_8)) == TRUE){
        toggle_group(8);
        machine_settings.is_updated = TRUE;
      }
   break;
    case BUTTON_GROUP_9:
       if ( validate_button_pressed (i, HAL_GPIO_ReadPin(PORT_BUTTON_GROUP_9, PIN_BUTTON_GROUP_9)) == TRUE){
        toggle_group(9);
        machine_settings.is_updated = TRUE;
      }
   break;
  }
}	
};
//==========================================================================================
void mx_03_write_settings_to_eeprom(){ // STUB TODO
	
	Log("Write to eeprom\n\r");
  mx_0_log_machine_settings();

	fl_01_flash_unlock();
	
	fl_02_erase_page_by_address(FlashStartAddress + 1);
	
	fl_03_write_data(	FlashStartAddress + FLASH_ADDR_SHIFT_is_random,
										machine_settings.is_random);	
	fl_03_write_data(	FlashStartAddress + FLASH_ADDR_SHIFT_delay_pool,
										machine_settings.delay_pool);
	fl_03_write_data(	FlashStartAddress + FLASH_ADDR_SHIFT_time_play_mode,
										machine_settings.time_play_mode);
	fl_03_write_data(	FlashStartAddress + FLASH_ADDR_SHIFT_selected_groups_count,
										machine_settings.selected_groups_count);
	fl_03_write_data(	FlashStartAddress + FLASH_ADDR_SHIFT_is_power_on,
										machine_settings.is_power_on);
										
	for ( int i = 0; i < GROUPS_COUNT; i++){
		fl_03_write_data(	FlashStartAddress + FLASH_ADDR_SHIFT_selected_groups + i*2,
											machine_settings.selected_groups[i]);
	}

	fl_04_flash_lock();	
}
//==========================================================================================
void mx_04_light_leds_by_settings(){
	if (machine_settings.is_power_on == TRUE){
		HAL_GPIO_WritePin(PORT_LED_POWER, PIN_LED_POWER, GPIO_PIN_SET);	
	}else{
		HAL_GPIO_WritePin(PORT_LED_POWER, PIN_LED_POWER, GPIO_PIN_RESET);
	}

	if (machine_settings.is_random == RANDOM){
		HAL_GPIO_WritePin(PORT_LED_RANDOM, PIN_LED_RANDOM, GPIO_PIN_SET);	
	}else{
		HAL_GPIO_WritePin(PORT_LED_RANDOM, PIN_LED_RANDOM, GPIO_PIN_RESET);
	}

	HAL_GPIO_WritePin(PORT_LED_PAUSE_0, PIN_LED_PAUSE_0, GPIO_PIN_RESET);	
	HAL_GPIO_WritePin(PORT_LED_PAUSE_1, PIN_LED_PAUSE_1, GPIO_PIN_RESET);	
	HAL_GPIO_WritePin(PORT_LED_PAUSE_2, PIN_LED_PAUSE_2, GPIO_PIN_RESET);	
	HAL_GPIO_WritePin(PORT_LED_PAUSE_3, PIN_LED_PAUSE_3, GPIO_PIN_RESET);	

	switch (machine_settings.delay_pool){
		case DELAY_POOL_0:
			HAL_GPIO_WritePin(PORT_LED_PAUSE_0, PIN_LED_PAUSE_0, GPIO_PIN_SET);	
			break;
		case DELAY_POOL_1:
			HAL_GPIO_WritePin(PORT_LED_PAUSE_0, PIN_LED_PAUSE_1, GPIO_PIN_SET);	
			break;
		case DELAY_POOL_2:
			HAL_GPIO_WritePin(PORT_LED_PAUSE_0, PIN_LED_PAUSE_2, GPIO_PIN_SET);	
			break;
		case DELAY_POOL_3:
			HAL_GPIO_WritePin(PORT_LED_PAUSE_0, PIN_LED_PAUSE_3, GPIO_PIN_SET);	
			break;
		default:
			break;
	}
	
	HAL_GPIO_WritePin(PORT_LED_MODE_DAY, PIN_LED_MODE_DAY, GPIO_PIN_RESET);	
	HAL_GPIO_WritePin(PORT_LED_MODE_NIGHT, PIN_LED_MODE_NIGHT, GPIO_PIN_RESET);	
	HAL_GPIO_WritePin(PORT_LED_MODE_DAYnNIGHT, PIN_LED_MODE_DAYnNIGHT, GPIO_PIN_RESET);	

	switch (machine_settings.time_play_mode){
		case DAY:
			HAL_GPIO_WritePin(PORT_LED_MODE_DAY, PIN_LED_MODE_DAY, GPIO_PIN_SET);	
			break;
		case NIGHT:
			HAL_GPIO_WritePin(PORT_LED_MODE_NIGHT, PIN_LED_MODE_NIGHT, GPIO_PIN_SET);	
			break;
		case DAY_AND_NIGHT:
			HAL_GPIO_WritePin(PORT_LED_MODE_DAYnNIGHT, PIN_LED_MODE_DAYnNIGHT, GPIO_PIN_SET);	
			break;
		default:
			break;
	}
		
	HAL_GPIO_WritePin(PORT_LED_GROUP_0, PIN_LED_GROUP_0, GPIO_PIN_RESET);	
	HAL_GPIO_WritePin(PORT_LED_GROUP_1, PIN_LED_GROUP_1, GPIO_PIN_RESET);	
	HAL_GPIO_WritePin(PORT_LED_GROUP_2, PIN_LED_GROUP_2, GPIO_PIN_RESET);	
	HAL_GPIO_WritePin(PORT_LED_GROUP_3, PIN_LED_GROUP_3, GPIO_PIN_RESET);	
	HAL_GPIO_WritePin(PORT_LED_GROUP_4, PIN_LED_GROUP_4, GPIO_PIN_RESET);	
	HAL_GPIO_WritePin(PORT_LED_GROUP_5, PIN_LED_GROUP_5, GPIO_PIN_RESET);	
	HAL_GPIO_WritePin(PORT_LED_GROUP_6, PIN_LED_GROUP_6, GPIO_PIN_RESET);	
	HAL_GPIO_WritePin(PORT_LED_GROUP_7, PIN_LED_GROUP_7, GPIO_PIN_RESET);	
	HAL_GPIO_WritePin(PORT_LED_GROUP_8, PIN_LED_GROUP_8, GPIO_PIN_RESET);	
	HAL_GPIO_WritePin(PORT_LED_GROUP_9, PIN_LED_GROUP_9, GPIO_PIN_RESET);	
	
	for (int i = 0; i < machine_settings.selected_groups_count; i++){
		switch (machine_settings.selected_groups[i]){
			case 0:
				HAL_GPIO_WritePin(PORT_LED_GROUP_0, PIN_LED_GROUP_0, GPIO_PIN_SET);	
				break;
			case 1:
				HAL_GPIO_WritePin(PORT_LED_GROUP_1, PIN_LED_GROUP_1, GPIO_PIN_SET);	
				break;
			case 2:
				HAL_GPIO_WritePin(PORT_LED_GROUP_2, PIN_LED_GROUP_2, GPIO_PIN_SET);	
				break;
			case 3:
				HAL_GPIO_WritePin(PORT_LED_GROUP_3, PIN_LED_GROUP_3, GPIO_PIN_SET);	
				break;
			case 4:
				HAL_GPIO_WritePin(PORT_LED_GROUP_4, PIN_LED_GROUP_4, GPIO_PIN_SET);	
				break;
			case 5:
				HAL_GPIO_WritePin(PORT_LED_GROUP_5, PIN_LED_GROUP_5, GPIO_PIN_SET);	
				break;
			case 6:
				HAL_GPIO_WritePin(PORT_LED_GROUP_6, PIN_LED_GROUP_6, GPIO_PIN_SET);	
				break;
			case 7:
				HAL_GPIO_WritePin(PORT_LED_GROUP_7, PIN_LED_GROUP_7, GPIO_PIN_SET);	
				break;
			case 8:
				HAL_GPIO_WritePin(PORT_LED_GROUP_8, PIN_LED_GROUP_8, GPIO_PIN_SET);	
				break;
			case 9:
				HAL_GPIO_WritePin(PORT_LED_GROUP_9, PIN_LED_GROUP_9, GPIO_PIN_SET);	
				break;
			default:
				break;
		}
	}
}
//==========================================================================================
 
char mx_05_check_light(){
	static char last_state = DAY;
	
	if ( 	last_state == DAY ){ // was day
		if (adc_v[adc_v_light] > adc_v[adc_v_resist]){ // STILL DAY
			return last_state;
		}else{ // become night
			last_state = NIGHT;
			return last_state;
		}
	}else{ // was night
		if (adc_v[adc_v_light] < adc_v[adc_v_resist] + DAY_NIGHT_THRESHOLD_DELTA){ // still night
			return last_state;
		}else{ // become day
			last_state = DAY;
			return last_state;
		}
	}
}
//==========================================================================================
void mx_06_light_leds_off(){
	HAL_GPIO_WritePin(PORT_LED_POWER, PIN_LED_POWER, GPIO_PIN_RESET);	
	HAL_GPIO_WritePin(PORT_LED_RANDOM, PIN_LED_RANDOM, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(PORT_LED_PAUSE_0, PIN_LED_PAUSE_0, GPIO_PIN_RESET);	
	HAL_GPIO_WritePin(PORT_LED_PAUSE_1, PIN_LED_PAUSE_1, GPIO_PIN_RESET);	
	HAL_GPIO_WritePin(PORT_LED_PAUSE_2, PIN_LED_PAUSE_2, GPIO_PIN_RESET);	
	HAL_GPIO_WritePin(PORT_LED_PAUSE_3, PIN_LED_PAUSE_3, GPIO_PIN_RESET);	
	HAL_GPIO_WritePin(PORT_LED_MODE_DAY, PIN_LED_MODE_DAY, GPIO_PIN_RESET);	
	HAL_GPIO_WritePin(PORT_LED_MODE_NIGHT, PIN_LED_MODE_NIGHT, GPIO_PIN_RESET);	
	HAL_GPIO_WritePin(PORT_LED_MODE_DAYnNIGHT, PIN_LED_MODE_DAYnNIGHT, GPIO_PIN_RESET);	
	HAL_GPIO_WritePin(PORT_LED_GROUP_0, PIN_LED_GROUP_0, GPIO_PIN_RESET);	
	HAL_GPIO_WritePin(PORT_LED_GROUP_1, PIN_LED_GROUP_1, GPIO_PIN_RESET);	
	HAL_GPIO_WritePin(PORT_LED_GROUP_2, PIN_LED_GROUP_2, GPIO_PIN_RESET);	
	HAL_GPIO_WritePin(PORT_LED_GROUP_3, PIN_LED_GROUP_3, GPIO_PIN_RESET);	
	HAL_GPIO_WritePin(PORT_LED_GROUP_4, PIN_LED_GROUP_4, GPIO_PIN_RESET);	
	HAL_GPIO_WritePin(PORT_LED_GROUP_5, PIN_LED_GROUP_5, GPIO_PIN_RESET);	
	HAL_GPIO_WritePin(PORT_LED_GROUP_6, PIN_LED_GROUP_6, GPIO_PIN_RESET);	
	HAL_GPIO_WritePin(PORT_LED_GROUP_7, PIN_LED_GROUP_7, GPIO_PIN_RESET);	
	HAL_GPIO_WritePin(PORT_LED_GROUP_8, PIN_LED_GROUP_8, GPIO_PIN_RESET);	
	HAL_GPIO_WritePin(PORT_LED_GROUP_9, PIN_LED_GROUP_9, GPIO_PIN_RESET);	

}
//==========================================================================================
void mx_08_light_leds_for_shutdown_off(){
  HAL_GPIO_WritePin(PORT_LED_POWER, PIN_LED_POWER, GPIO_PIN_SET);	// POWER_LED_IS_ON
	HAL_GPIO_WritePin(PORT_LED_RANDOM, PIN_LED_RANDOM, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(PORT_LED_PAUSE_0, PIN_LED_PAUSE_0, GPIO_PIN_RESET);	
	HAL_GPIO_WritePin(PORT_LED_PAUSE_1, PIN_LED_PAUSE_1, GPIO_PIN_RESET);	
	HAL_GPIO_WritePin(PORT_LED_PAUSE_2, PIN_LED_PAUSE_2, GPIO_PIN_RESET);	
	HAL_GPIO_WritePin(PORT_LED_PAUSE_3, PIN_LED_PAUSE_3, GPIO_PIN_RESET);	
	HAL_GPIO_WritePin(PORT_LED_MODE_DAY, PIN_LED_MODE_DAY, GPIO_PIN_RESET);	
	HAL_GPIO_WritePin(PORT_LED_MODE_NIGHT, PIN_LED_MODE_NIGHT, GPIO_PIN_RESET);	
	HAL_GPIO_WritePin(PORT_LED_MODE_DAYnNIGHT, PIN_LED_MODE_DAYnNIGHT, GPIO_PIN_RESET);	
	HAL_GPIO_WritePin(PORT_LED_GROUP_0, PIN_LED_GROUP_0, GPIO_PIN_RESET);	
	HAL_GPIO_WritePin(PORT_LED_GROUP_1, PIN_LED_GROUP_1, GPIO_PIN_RESET);	
	HAL_GPIO_WritePin(PORT_LED_GROUP_2, PIN_LED_GROUP_2, GPIO_PIN_RESET);	
	HAL_GPIO_WritePin(PORT_LED_GROUP_3, PIN_LED_GROUP_3, GPIO_PIN_RESET);	
	HAL_GPIO_WritePin(PORT_LED_GROUP_4, PIN_LED_GROUP_4, GPIO_PIN_RESET);	
	HAL_GPIO_WritePin(PORT_LED_GROUP_5, PIN_LED_GROUP_5, GPIO_PIN_RESET);	
	HAL_GPIO_WritePin(PORT_LED_GROUP_6, PIN_LED_GROUP_6, GPIO_PIN_RESET);	
	HAL_GPIO_WritePin(PORT_LED_GROUP_7, PIN_LED_GROUP_7, GPIO_PIN_RESET);	
	HAL_GPIO_WritePin(PORT_LED_GROUP_8, PIN_LED_GROUP_8, GPIO_PIN_RESET);	
	HAL_GPIO_WritePin(PORT_LED_GROUP_9, PIN_LED_GROUP_9, GPIO_PIN_RESET);	
}
//==========================================================================================
void mx_09_blink_power_led(void){
	static unsigned char timer = 0;
	timer++;
	if (timer < 30) {
		HAL_GPIO_WritePin(PORT_LED_POWER, PIN_LED_POWER, GPIO_PIN_RESET);
		return;
	}
	if (timer < 60) {
		HAL_GPIO_WritePin(PORT_LED_POWER, PIN_LED_POWER, GPIO_PIN_SET);
		return;
	}
	timer = 0;
}
//==========================================================================================
uint16_t mx_07_select_track(uint16_t next_track){
	uint16_t x = 0;
	for(char i = 0; i < machine_settings.selected_groups_count; i++){
		x += files_count[ machine_settings.selected_groups[i] ];
		if (x > next_track){ //next track in 'i' group	
			char st[64];
			sprintf(st, "x > next_track ==>> x=%d nt=%d\n\r", x, next_track);
			Log(st);				
			uint16_t pos = next_track - (x - files_count[ machine_settings.selected_groups[i] ]);
			need_paly_track_number = files[machine_settings.selected_groups[i]][pos]; // may be [pos - 1]
			
			// forexit
			i = machine_settings.selected_groups_count + 1;
		}
	}

	return need_paly_track_number;
}
//==========================================================================================
void xx_01_poll_adc(){
	static int polltic = POLLADCTICKDEF;
	if (polltic-- <= 0){
		polltic = POLLADCTICKDEF;
		HAL_ADC_Start_IT(&hadc1);
	}
}
//==========================================================================================
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
/*	if (hadc->Instance == ADC1){
		char st[64];
		sprintf(st, "adc v0=%d v1=%d \n\r", adc_v[0], adc_v[1]);
		Log(st);
	}*/
}
//==========================================================================================
void pl_05_amp_on(){
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2,DAC_ALIGN_12B_R, 2048);
	HAL_DAC_Start(&hdac, DAC_CHANNEL_2);
	HAL_GPIO_WritePin(PORT_STAND_BY, PIN_STAND_BY, GPIO_PIN_RESET);
};
//==========================================================================================
void pl_06_amp_off(){
  HAL_GPIO_WritePin(PORT_STAND_BY, PIN_STAND_BY, GPIO_PIN_SET);	
};
//==========================================================================================
void fl_01_flash_unlock(void){
  FLASH->KEYR = FLASH_KEY1;
  FLASH->KEYR = FLASH_KEY2;	
};
//==========================================================================================
void fl_02_erase_page_by_address(uint32_t address_from_page){
	while((FLASH->SR&FLASH_SR_BSY)){};
	if (FLASH->SR & FLASH_SR_EOP) {
		FLASH->SR = FLASH_SR_EOP;
	}
  FLASH->CR |= FLASH_CR_PER; //Page Erase Set
  FLASH->AR = FlashStartAddress; //Page Address
  FLASH->CR |= FLASH_CR_STRT; //Start Page Erase
	while((FLASH->SR&FLASH_SR_BSY)){};
	if (FLASH->SR & FLASH_SR_EOP) {
		FLASH->SR = FLASH_SR_EOP;
	}
  FLASH->CR &= ~FLASH_CR_PER; //Page Erase Clear
};
//==========================================================================================
void fl_03_write_data(uint32_t address, uint16_t value){
	while((FLASH->SR&FLASH_SR_BSY)){};
	if (FLASH->SR & FLASH_SR_EOP) {
		FLASH->SR = FLASH_SR_EOP;
	}
	FLASH->CR |= FLASH_CR_PG;
	*(__IO uint16_t*)(address) = value;
};
//==========================================================================================
void fl_04_flash_lock(void){
  FLASH->CR &= ~FLASH_CR_PG; 
  FLASH->CR |= FLASH_CR_LOCK;		
};
//==========================================================================================
uint16_t fl_05_read_data(uint32_t address){
	return (*(__IO uint16_t*)(address));
};
//==========================================================================================
//	MY FUNCK		MY FUNCK		MY FUNCK		MY FUNCK		MY FUNCK		MY FUNCK		MY FUNCK
//==========================================================================================
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
