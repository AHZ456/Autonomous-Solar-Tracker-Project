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
#include <string.h>
#include <stdio.h>
#include "/home/zinou/holder.h"
#include "/home/zinou/holder.c"
#include "/home/zinou/i2c-lcd.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UPR_LMT 25//500//25
#define LWR_LMT 125//600//125
#define STP 75//90  //75
#define RGT 67//62 //67
#define LFT 85//600 //85
#define sJANH 17
#define sJANM 42
#define sAPRH 19
#define sAPRM 10
#define sOCTH 18
#define sOCTM 31
#define MinBat 2597
//to do:
//OR GATE BUTTONS
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
const uint16_t threshold=300,off=5000,WIRES=1500,JITTER=25000,stime=2000;
uint32_t buffer[11];
uint16_t UL=0,DL=0,UR=0,DR=0,PVD=0,diff=0;
uint16_t timer=0,ML=0,previous=0,jit=0,stimer=0,suml=0,sumr=0,sumd=0,sumu=0,pinil=0;
uint8_t cd=1,cf=0,c=0,b=0;

char msg[26];

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	UL=(uint16_t)buffer[0];
	DL=(uint16_t)buffer[1];
	UR=(uint16_t)buffer[2];
	DR=(uint16_t)buffer[3];
	PVD=(uint16_t)buffer[4];
	diff=(uint16_t)buffer[5];
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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_ADC_Start_DMA(&hadc1, buffer, 6);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
   HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
   TIM1->CCR4=LWR_LMT;
	RTC_SETUP_LSE(0, 0, 10, 0);

	if(BKP->DR6==0){
		while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7)!=1);
		//SET TIME AND DATE
		SetReg(1, 11, 3, 9, 1, 0);
		//SetReg(1,11,3,23,59,50);
		//SetReg(year, month, day, hours, minutes, seconds);
		lcd_init();
		lcd_clear();
			        lcd_put_cur(0, 0);
			       // HAL_UART_Transmit(&huart3, "ON---\n", 7, HAL_MAX_DELAY);
			        lcd_send_string("ON-------------");
			        HAL_Delay(1000);
	}
	//SetReg(uint8_t year,uint8_t month ,uint8_t day,uint8_t hours,uint8_t minutes,uint8_t seconds)
	else if(BKP->DR6==1){
	if(BKP->DR7 ==1){
	ReadFromBackUp();
	WriteToDR7DR8(0,BKP->DR8);
	}
	else if(BKP->DR8==1){
	READBKP_AFTERSLEEP();
	WriteToDR7DR8(BKP->DR7,0);
	}
	}

	Calendar();

			Zcalendar time = { 0 };
			Update(&time);
if(PVD<=MinBat){
	lcd_clear();
		        lcd_put_cur(0, 0);
		       // HAL_UART_Transmit(&huart3, "\n\n PVD, please wait until they've been recharged or use the manual charging wires.\n\n", 105,HAL_MAX_DELAY);
		                snprintf(msg,26,",C:%.1lfmA",(double)(((diff*3.3/4095)/5/2)*1000));
		                //output current?
		               // HAL_UART_Transmit(&huart3, msg, strlen(msg), HAL_MAX_DELAY);
		                for(int i=0;i<3;i++){
		                	lcd_clear();
		                	HAL_Delay(600);
		                	lcd_put_cur(0, 0);
		                		        lcd_send_string("CD01:R");
		                		                lcd_put_cur(1, 0);
		                		                snprintf(msg,26,",C:%.1lfmA",(double)(((diff*3.3/4095)/5/2)*1000));
		                	lcd_send_string(msg);
		                	HAL_Delay(1000);

		                }
		        //something to turn off the regulator(for good)
		        SHUTDOWN();
}
			if ((time.Months >= 1) && (time.Months < 4)) {
				if (time.Hours > sJANH) {
					//HAL_UART_Transmit(&huart3, "\n\nWake up time 7:49\n\n", 22,HAL_MAX_DELAY);
					lcd_clear();
					lcd_put_cur(0, 0);
			        lcd_send_string("W:7:49");
					lcd_put_cur(0, 6);
					 snprintf(msg,26,",C:%.1lfmA",(double)(((diff*3.3/4095)/5/2)*1000));
					//HAL_UART_Transmit(&huart3, msg, strlen(msg), HAL_MAX_DELAY);
					lcd_send_string(msg);
			        lcd_put_cur(1, 0);
			        snprintf(msg, 17, "%d:%d:%d:%d:%d", time.Months, time.Days,
			        				time.Hours, time.Minutes, time.Seconds);
			        lcd_send_string(msg);
			       // HAL_UART_Transmit(&huart3, msg, strlen(msg), HAL_MAX_DELAY);
									HAL_Delay(100);
									ALR_SET(7,49,0);
									VBAT();


				}
				else if ((time.Hours == sJANH) && (time.Minutes >= sJANM)) {
					//HAL_UART_Transmit(&huart3, "\n\nWake up time 7:49\n\n", 22,HAL_MAX_DELAY);
			                            lcd_clear();
			                            lcd_put_cur(0, 0);
								        lcd_send_string("W:7:49");
								        lcd_put_cur(0, 6);
								        					 snprintf(msg,26,",C:%.1lfmA",(double)(((diff*3.3/4095)/5/2)*1000));


								        				//	HAL_UART_Transmit(&huart3, msg, strlen(msg), HAL_MAX_DELAY);
								        					lcd_send_string(msg);
								        			        lcd_put_cur(1, 0);
								        			        snprintf(msg, 17, "%d:%d:%d:%d:%d", time.Months, time.Days,
								        			        				time.Hours, time.Minutes, time.Seconds);
								        			       // HAL_UART_Transmit(&huart3, msg, strlen(msg), HAL_MAX_DELAY);
								        			        lcd_send_string(msg);
								        									HAL_Delay(100);
								        									ALR_SET(7,49,0);
								        									VBAT();



				}
			}

			else if ((time.Months >= 4) && (time.Months < 10)) {
				if (time.Hours > sAPRH) {
					//HAL_UART_Transmit(&huart3, "\n\nWake up time 6:00\n\n", 22,HAL_MAX_DELAY);
					lcd_clear();
					lcd_put_cur(0, 0);
								        lcd_send_string("W:6:00");
								        lcd_put_cur(0, 6);
								        snprintf(msg,26,",C:%.1lfmA",(double)(((diff*3.3/4095)/5/2)*1000));
								        				//	HAL_UART_Transmit(&huart3, msg, strlen(msg), HAL_MAX_DELAY);
								        					lcd_send_string(msg);
								        lcd_put_cur(1, 0);
								        snprintf(msg, 17, "%d:%d:%d:%d:%d", time.Months, time.Days,
								        				time.Hours, time.Minutes, time.Seconds);
								       // HAL_UART_Transmit(&huart3, msg, strlen(msg), HAL_MAX_DELAY);
								        lcd_send_string(msg);
												ALR_SET(6,0,0);
												VBAT();

				} else if (time.Hours == sAPRH && time.Minutes >= sAPRM) {
					//HAL_UART_Transmit(&huart3, "\n\nWake up time 6:00\n\n", 22,HAL_MAX_DELAY);
					lcd_clear();
										lcd_put_cur(0, 0);
													        lcd_send_string("W:6:00");
													        lcd_put_cur(0, 6);
													        snprintf(msg,26,",C:%.1lfmA",(double)(((diff*3.3/4095)/5/2)*1000));
													        					//HAL_UART_Transmit(&huart3, msg, strlen(msg), HAL_MAX_DELAY);
													        					lcd_send_string(msg);
													        lcd_put_cur(1, 0);
													        snprintf(msg, 17, "%d:%d:%d:%d:%d", time.Months, time.Days,
													        				time.Hours, time.Minutes, time.Seconds);
													        //HAL_UART_Transmit(&huart3, msg, strlen(msg), HAL_MAX_DELAY);
													        lcd_send_string(msg);
																	ALR_SET(6,0,0);
																	VBAT();


				}
			}

			else if (time.Months >= 10) {
				if (time.Hours > sOCTH) {
			//HAL_UART_Transmit(&huart3, "\n\nWake up time 7:30\n\n", 22,HAL_MAX_DELAY);

			lcd_clear();
			lcd_put_cur(0, 0);
			lcd_send_string("W:7:30");
			lcd_put_cur(0, 6);
			snprintf(msg, 26, ",C:%.1lf mA",
					(double) (((diff * 3.3 / 4095) / 5 / 2) * 1000));
		//	HAL_UART_Transmit(&huart3, msg, strlen(msg), HAL_MAX_DELAY);
			lcd_send_string(msg);
			lcd_put_cur(1, 0);
			snprintf(msg, 17, "%d:%d:%d:%d:%d", time.Months, time.Days,
					time.Hours, time.Minutes, time.Seconds);
			lcd_send_string(msg);
			//ALR_SET(7,30,0);
			ALR_SET(0,1,0);
			VBAT();
				}
				else if (time.Hours == sOCTH && time.Minutes >= sOCTM) {
				//	HAL_UART_Transmit(&huart3, "\n\nWake up time 7:30\n\n", 22,HAL_MAX_DELAY);
					lcd_clear();
					lcd_put_cur(0, 0);
													        lcd_send_string("W:7:30");
													        lcd_put_cur(0, 6);
													        			snprintf(msg, 26, ",C:%.1lf mA",
													        					(double) (((diff * 3.3 / 4095) / 5 / 2.2) * 1000));
													        		//	HAL_UART_Transmit(&huart3, msg, strlen(msg), HAL_MAX_DELAY);
													        			lcd_send_string(msg);
													        lcd_put_cur(1, 0);
													        snprintf(msg, 17, "%d:%d:%d:%d:%d", time.Months, time.Days,
													        				time.Hours, time.Minutes, time.Seconds);
													       // HAL_UART_Transmit(&huart3, msg, strlen(msg), HAL_MAX_DELAY);
													        lcd_send_string(msg);
												//ALR_SET(7,30,0);
													        ALR_SET(0,1,0);
												VBAT();
				}
			}
ML=HAL_GetTick();
previous=HAL_GetTick();
jit=HAL_GetTick();
timer=HAL_GetTick();
stimer=HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  	  sumr=UR+DR;
	  	  suml=UL+DL;
	  	  sumu=UL+UR;
	  	  sumd=DL+DR;








	  	  if(sumr > suml+threshold){
	  	    TIM1->CCR3=RGT;
	  	    previous=HAL_GetTick();

	  	  }
	  	  else if(sumr+threshold < suml){
	  		  TIM1->CCR3=LFT;
	  	  previous=HAL_GetTick();
	  	  }
	  	  else if(sumr <= suml+threshold && sumr+threshold >= suml){
	  		  TIM1->CCR3=STP;
	  	  }
	  	  if(sumu > sumd+threshold){
	  	  if(TIM1->CCR4 >UPR_LMT){
	  		  TIM1->CCR4--;
	  	    HAL_Delay(5);
	  	    previous=HAL_GetTick();
	  	    stimer=HAL_GetTick();


	  	  }
	  	  }
	  	  else if(sumu+threshold < sumd){
	  	  if(TIM1->CCR4 < LWR_LMT){
	  		  TIM1->CCR4++;
	  	    HAL_Delay(5);
	  	    previous=HAL_GetTick();
	  	    stimer=HAL_GetTick();


	  	  }
	  	  }
	  	 // snprintf(msg,10,"%d\n",PVD);
	  		//HAL_UART_Transmit(&huart3, msg, strlen(msg), HAL_MAX_DELAY);

	  	  ML=HAL_GetTick();
	  	  if(ML-timer>WIRES){
	  	    b=1;
	  	  }


	  	  pinil=HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3);
	  	  //pinil=1;// FOR DEMO
	  	  if(!pinil && !c){
	  		  stimer=HAL_GetTick();
	  		  c=1;
	  	  }
	  	  else if(pinil){
	  		  c=0;
	  	  }
	  	  Update(&time);
	  	 ML=HAL_GetTick();
	  	  if( (ML-previous >= off)  || (  (!pinil) && (b)&& (ML-stimer>=stime)  ) || (ML-jit>=JITTER)  ){


	  	    ML=HAL_GetTick();
	  	    if(ML-previous >= off){
	  	     // HAL_UART_Transmit(&huart3, "\nIDEAL///////////////////////////\n",36, HAL_MAX_DELAY);
					lcd_clear();
					lcd_put_cur(0, 0);
					lcd_send_string("CND:ID");
					lcd_put_cur(0, 6);
					 snprintf(msg,26,",C:%.1lfmA",(double)(((diff*3.3/4095)/5/2)*1000));
					 lcd_send_string(msg);
					lcd_put_cur(1, 0);
					snprintf(msg, 17, "%d:%d:%d:%d:%d", time.Months, time.Days,
							time.Hours, time.Minutes, time.Seconds);
					lcd_send_string(msg);
				//	HAL_UART_Transmit(&huart3, "PREVIOUS\n", 10, HAL_MAX_DELAY);

	  	    }
	  	    //buttons
	  	    else if( (  (!pinil) && (b) && (ML-stimer>=stime))  ){
					lcd_clear();
					lcd_put_cur(0, 0);
					lcd_send_string("CND:WI");
					lcd_put_cur(0, 6);
					 snprintf(msg,26,",C:%.1lfmA",(double)(((diff*3.3/4095)/5/2)*1000));
					 lcd_send_string(msg);
					lcd_put_cur(1, 0);
					snprintf(msg, 17, "%d:%d:%d:%d:%d", time.Months, time.Days,
							time.Hours, time.Minutes, time.Seconds);
					lcd_send_string(msg);
					//HAL_UART_Transmit(&huart3, "WIRES\n", 7, HAL_MAX_DELAY);

	  	    }
	  	    else if(ML-jit >= JITTER){
					lcd_clear();
					lcd_put_cur(0, 0);
					lcd_send_string("CND:JI");
					lcd_put_cur(0, 6);
					 snprintf(msg,26,",C:%.1lfmA",(double)(((diff*3.3/4095)/5/2)*1000));
					 lcd_send_string(msg);
					lcd_put_cur(1, 0);
					snprintf(msg, 17, "%d:%d:%d:%d:%d", time.Months, time.Days,
							time.Hours, time.Minutes, time.Seconds);
					lcd_send_string(msg);
					//HAL_UART_Transmit(&huart3, "JITTER\n", 8, HAL_MAX_DELAY);

	  	    }
	  	  ML=HAL_GetTick();
	  	  previous=HAL_GetTick();
	  	  jit=HAL_GetTick();
	  	  timer=HAL_GetTick();
	  	  stimer=HAL_GetTick();
	  	  ALR_SET_PR(0, 0, 10);
	  	  VBAT();




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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV16;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 6;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_6;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 20-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000-1;
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
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Transistor_pin_GPIO_Port, Transistor_pin_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PD0 PD1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PA6 PA8 PA9 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10
                           PB11 PB13 PB14 PB15
                           PB4 PB5 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Transistor_pin_Pin */
  GPIO_InitStruct.Pin = Transistor_pin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Transistor_pin_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure peripheral I/O remapping */
  __HAL_AFIO_REMAP_PD01_ENABLE();

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
