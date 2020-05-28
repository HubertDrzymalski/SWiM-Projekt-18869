/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
void ustawPiny(int tab[3][8]){
  //zielone
  	 HAL_GPIO_WritePin(GPIOA, led1g_Pin, tab[0][0]);
  	 HAL_GPIO_WritePin(GPIOA, led2g_Pin, tab[0][1]);
  	 HAL_GPIO_WritePin(GPIOA, led3g_Pin, tab[0][2]);
  	 HAL_GPIO_WritePin(GPIOC, led4g_Pin, tab[0][3]);
  	 HAL_GPIO_WritePin(GPIOA, led5g_Pin, tab[0][4]);
  	 HAL_GPIO_WritePin(GPIOB, led6g_Pin, tab[0][5]);
  	 HAL_GPIO_WritePin(GPIOB, led7g_Pin, tab[0][6]);
  	 HAL_GPIO_WritePin(GPIOB, led8g_Pin, tab[0][7]);
  //czerowne
  	 HAL_GPIO_WritePin(GPIOC,led1r_Pin, tab[1][0]);
  	 HAL_GPIO_WritePin(GPIOB,led2r_Pin, tab[1][1]);
  	 HAL_GPIO_WritePin(GPIOB,led3r_Pin, tab[1][2]);
  	 HAL_GPIO_WritePin(GPIOC,led4r_Pin, tab[1][3]);
  	 HAL_GPIO_WritePin(GPIOC,led5r_Pin, tab[1][4]);
  	 HAL_GPIO_WritePin(GPIOC,led6r_Pin, tab[1][5]);
  	 HAL_GPIO_WritePin(GPIOA,led7r_Pin, tab[1][6]);
 	 HAL_GPIO_WritePin(GPIOA,led8r_Pin, tab[1][7]);
  //zolte
 	 HAL_GPIO_WritePin(GPIOC, led1y_Pin, tab[2][0]);
 	 HAL_GPIO_WritePin(GPIOB, led3y_Pin, tab[2][1]);
 	 HAL_GPIO_WritePin(GPIOC, led2y_Pin, tab[2][2]);
 	 HAL_GPIO_WritePin(GPIOA, led4y_Pin, tab[2][3]);
 	 HAL_GPIO_WritePin(GPIOA, led5y_Pin, tab[2][4]);
 	 HAL_GPIO_WritePin(GPIOC, led6y_Pin, tab[2][5]);
 	 HAL_GPIO_WritePin(GPIOC, led7y_Pin, tab[2][6]);
 	 HAL_GPIO_WritePin(led8y_GPIO_Port, led8y_Pin, tab[2][7]);
};
void OdZielnonychDoCzerownych(void){
	int rytm[3][8]={{0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0}};
		  for(int i=0;i<3;i++){
			  for(int j=0;j<8;j++){
				  rytm[i][j]=1;
				  ustawPiny(rytm);
				  HAL_Delay(50);
			  }
		  }
}
void odCzerwonychDoZielonych(void){
	int rytm[3][8]={{0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0}};
		  for(int i=2;i>=0;i--){
			  for(int j=0;j<8;j++){
				  rytm[i][j]=1;
				  ustawPiny(rytm);
				  HAL_Delay(50);
			  }
		  }
}
void pionowoOdLewej(void){
	int rytm[3][8]={{0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0}};
		  for(int i=0;i<8;i++){
			  for(int j=0;j<3;j++){
				  rytm[j][i]=1;
				  ustawPiny(rytm);
				  HAL_Delay(50);
			  }
		  }
}
void pionowoOdPrawej(void){
	int rytm[3][8]={{0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0}};
		  for(int i=7;i>=0;i--){
			  for(int j=2;j>=0;j--){
				  rytm[j][i]=1;
				  ustawPiny(rytm);
				  HAL_Delay(50);
			  }
		  }
}
void snake(void){
	int rytm[3][8]={{0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0}};
	int kierunek=0;
	for(int i=0;i<8;i++){
		if(kierunek==1){
		for(int j=0;j<3;j++){
			  rytm[j][i]=1;
			  ustawPiny(rytm);
			  HAL_Delay(100);
		}
		}
		if(kierunek==0){
		for(int j=2;j>=0;j--){
			  rytm[j][i]=1;
			  ustawPiny(rytm);
			  HAL_Delay(100);
		}
		}
		kierunek=!kierunek;
}
}
void zeruj(){
	int rytm[3][8]={{0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0}};
	ustawPiny(rytm);
}
void indeks(){
	int rytm[3][8]={{0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0}};
	//1
	for(int i=0;i<8;i++){
		rytm[0][i]=1;
	}
	ustawPiny(rytm);
	HAL_GPIO_WritePin(GPIOB,led2r_Pin,SET);
	HAL_GPIO_WritePin(GPIOC, led2y_Pin,SET);
	HAL_Delay(1500);
	zeruj();
	//////////////////
	for(int i=0;i<8;i++){
		for(int j=0;j<3;j++){
			rytm[j][i]=0;
		}
	}
	//////////////////
	//8
	for(int i=0;i<8;i++){
		rytm[0][i]=1;
		rytm[2][i]=1;
	}
	ustawPiny(rytm);
	HAL_GPIO_WritePin(GPIOC,led1r_Pin,SET);
	HAL_GPIO_WritePin(GPIOA,led8r_Pin,SET);
 	HAL_GPIO_WritePin(GPIOC,led4r_Pin,SET);
 	HAL_GPIO_WritePin(GPIOC,led5r_Pin,SET);
	HAL_Delay(1500);
	zeruj();
	HAL_Delay(500);
	//////////////////
	for(int i=0;i<8;i++){
		for(int j=0;j<3;j++){
			rytm[j][i]=0;
		}
	}
	//////////////////
	//8
	for(int i=0;i<8;i++){
		rytm[0][i]=1;
		rytm[2][i]=1;
	}
	ustawPiny(rytm);
	HAL_GPIO_WritePin(GPIOC,led1r_Pin,SET);
	HAL_GPIO_WritePin(GPIOA,led8r_Pin,SET);
 	HAL_GPIO_WritePin(GPIOC,led4r_Pin,SET);
 	HAL_GPIO_WritePin(GPIOC,led5r_Pin,SET);
	HAL_Delay(1500);
	zeruj();
	//////////////////
	for(int i=0;i<8;i++){
		for(int j=0;j<3;j++){
			rytm[j][i]=0;
		}
	}
	//////////////////
	//6
	for(int i=0;i<8;i++){
		rytm[2][i]=1;
	}
	ustawPiny(rytm);
	HAL_GPIO_WritePin(GPIOC,led4r_Pin, SET);
	HAL_GPIO_WritePin(GPIOA,led8r_Pin, SET);
	HAL_GPIO_WritePin(GPIOC,led1r_Pin, SET);
 	HAL_GPIO_WritePin(GPIOA, led1g_Pin, SET);
 	HAL_GPIO_WritePin(GPIOC, led4g_Pin, SET);
 	HAL_GPIO_WritePin(GPIOA, led5g_Pin, SET);
 	HAL_GPIO_WritePin(GPIOB, led6g_Pin, SET);
    HAL_GPIO_WritePin(GPIOB, led7g_Pin, SET);
    HAL_GPIO_WritePin(GPIOB, led8g_Pin, SET);
	HAL_Delay(1500);
	zeruj();
	//////////////////
	for(int i=0;i<8;i++){
		for(int j=0;j<3;j++){
			rytm[j][i]=0;
		}
	}
	//////////////////
	//9
	for(int i=0;i<8;i++){
		rytm[0][i]=1;
	}
	ustawPiny(rytm);
	HAL_GPIO_WritePin(GPIOC,led4r_Pin, SET);
	HAL_GPIO_WritePin(GPIOA,led8r_Pin, SET);
	HAL_GPIO_WritePin(GPIOC,led1r_Pin, SET);
	 HAL_GPIO_WritePin(GPIOC, led1y_Pin, SET);
	 HAL_GPIO_WritePin(GPIOB, led3y_Pin, SET);
	 HAL_GPIO_WritePin(GPIOC, led2y_Pin, SET);
	 HAL_GPIO_WritePin(GPIOA, led4y_Pin, SET);
	 HAL_GPIO_WritePin(led8y_GPIO_Port, led8y_Pin,SET);
	HAL_Delay(1500);
	zeruj();

}
void usmiech(){
	int rytm[3][8]={{0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0}};
	for(int i=1;i<7;i++){
		rytm[0][i]=1;
		ustawPiny(rytm);
	}
	HAL_GPIO_WritePin(GPIOC,led1r_Pin,SET);
	HAL_GPIO_WritePin(GPIOA,led8r_Pin,SET);
	HAL_GPIO_WritePin(GPIOC, led6y_Pin,SET);
	HAL_GPIO_WritePin(GPIOC, led2y_Pin,SET);
}
void nieparzyste(){
	int rytm[3][8]={{0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0}};
	for(int i=0;i<8;i++){
		if(i%2==0){
		rytm[1][i]=1;
		rytm[0][i]=1;
		rytm[2][i]=1;
		ustawPiny(rytm);
		}
	}
}
void parzyste(){
	int rytm[3][8]={{0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0}};
	for(int i=0;i<8;i++){
		if(i%2==1){
		rytm[1][i]=1;
		rytm[0][i]=1;
		rytm[2][i]=1;
		ustawPiny(rytm);
		}
	}
}
void mysz(){
	int rytm[3][8]={{0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0}};
	int kierunek=1;
	for(int i=0;i<3;i++){
		if(kierunek==1){for(int j=0;j<8;j++){
			rytm[i][j]=1;
			ustawPiny(rytm);
			HAL_Delay(75);
			rytm[i][j]=0;
		}}
		if(kierunek==0){for(int j=7;j>=0;j--){
			rytm[i][j]=1;
			ustawPiny(rytm);
			HAL_Delay(75);
			rytm[i][j]=0;
		}}
		kierunek=!kierunek;
	}
}
void myszPowrot(){
	int rytm[3][8]={{0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0}};
	int kierunek=0;
	for(int i=2;i>=0;i--){
		if(kierunek==1){for(int j=0;j<8;j++){
			rytm[i][j]=1;
			ustawPiny(rytm);
			HAL_Delay(75);
			rytm[i][j]=0;
		}}
		if(kierunek==0){for(int j=7;j>=0;j--){
			rytm[i][j]=1;
			ustawPiny(rytm);
			HAL_Delay(75);
			rytm[i][j]=0;
		}}
		kierunek=!kierunek;
	}
}
void yellow(){
	int rytm[3][8]={{0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0},{1,1,1,1,1,1,1,1}};
		ustawPiny(rytm);
}
void green(){
	int rytm[3][8]={{1,1,1,1,1,1,1,1},{0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0}};
		ustawPiny(rytm);
}
void red(){
	int rytm[3][8]={{0,0,0,0,0,0,0,0},{1,1,1,1,1,1,1,1},{0,0,0,0,0,0,0,0}};
		ustawPiny(rytm);
}
uint16_t v[1];
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
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  HAL_ADC_Start_DMA(&hadc1,v,1);
	    /* Infinite loop */
	    /* USER CODE BEGIN WHILE */
	  	  uint8_t buffer[20];
	  	    while (1)
	  	    {
	  	    	buffer[0]=0;
	  	    	HAL_UART_Receive(&huart2,buffer,20,100);
	  	    	while(buffer[0]=='m'){
	  	    		v[0] = HAL_ADC_GetValue(&hadc1);
	  	    		int rytm[3][8]={{0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0}};
	  	    			  if(v[0]>475){
	  	    					for(int i=0;i<3;i++){
	  	    						for(int j=0;j<8;j++){
	  	    							rytm[i][j]=1;
	  	    							ustawPiny(rytm);
	  	    						}
	  	    			  }
	  	    			  }
	  	    			  if(v[0]<475&&v[0]>350){
	  	    					for(int i=0;i<3;i++){
	  	    						for(int j=0;j<7;j++){
	  	    							rytm[i][j]=1;
	  	    							ustawPiny(rytm);
	  	    						}
	  	    			  }
	  	    			  }
	  	    			  if(v[0]<350&&v[0]>300){
	  	    					for(int i=0;i<3;i++){
	  	    						for(int j=0;j<6;j++){
	  	    							rytm[i][j]=1;
	  	    							ustawPiny(rytm);
	  	    						}
	  	    			  }
	  	    			  }
	  	    			  if(v[0]<300&&v[0]>200){
	  	    					for(int i=0;i<3;i++){
	  	    						for(int j=0;j<5;j++){
	  	    							rytm[i][j]=1;
	  	    							ustawPiny(rytm);
	  	    						}
	  	    			  }
	  	    			  }
	  	    			  if(v[0]<200&&v[0]>100){
	  	    					for(int i=0;i<3;i++){
	  	    						for(int j=0;j<4;j++){
	  	    							rytm[i][j]=1;
	  	    							ustawPiny(rytm);
	  	    						}
	  	    			  }
	  	    			  }
	  	    			  if(v[0]<100&&v[0]>75){
	  	    					for(int i=0;i<3;i++){
	  	    						for(int j=0;j<3;j++){
	  	    							rytm[i][j]=1;
	  	    							ustawPiny(rytm);
	  	    						}
	  	    			  }
	  	    			  }
	  	    			  if(v[0]<75&&v[0]>50){
	  	    					for(int i=0;i<3;i++){
	  	    						for(int j=0;j<2;j++){
	  	    							rytm[i][j]=1;
	  	    							ustawPiny(rytm);
	  	    						}
	  	    			  }
	  	    			  }
	  	    			  if(v[0]<50){
	  	    					for(int i=0;i<3;i++){
	  	    						for(int j=0;j<1;j++){
	  	    							rytm[i][j]=1;
	  	    							ustawPiny(rytm);
	  	    						}
	  	    			  }
	  	    			  }
	  	    	HAL_UART_Receive(&huart2,buffer,20,100);
	  	    	zeruj();
	  	    	}
	  	  	  while(buffer[0]=='1'){
	  	  		  OdZielnonychDoCzerownych();
	  	  		  HAL_UART_Receive(&huart2,buffer,20,100);
	  	  	  }
	  	  	  while(buffer[0]=='2'){
	  	  		  odCzerwonychDoZielonych();
	  	  		  HAL_UART_Receive(&huart2,buffer,20,100);
	  	  	  }
	  	  	  while(buffer[0]=='3'){
	  	  		  pionowoOdLewej();
	  	  		  HAL_UART_Receive(&huart2,buffer,20,100);
	  	  	  }
	  	  	  while(buffer[0]=='4'){
	  	  		  pionowoOdPrawej();
	  	  		  HAL_UART_Receive(&huart2,buffer,20,100);
	  	  	  }
	  	  	  while(buffer[0]=='5'){
	  	  		  snake();
	  	  		  HAL_UART_Receive(&huart2,buffer,20,100);
	  	  	  }
	  	  	  while(buffer[0]=='6'){
	  	  		  indeks();
	  	  		  HAL_UART_Receive(&huart2,buffer,20,100);
	  	  	  }
	  	  	  while(buffer[0]=='7'){
	  	  		  usmiech();
	  	  		  HAL_UART_Receive(&huart2,buffer,20,100);
	  	  	  }
	  	  	  while(buffer[0]=='8'){
	  	  		  parzyste();
	  	  		  HAL_Delay(150);
	  	  		  nieparzyste();
	  	  		  HAL_UART_Receive(&huart2,buffer,20,100);
	  	  	  }
	  	  	  while(buffer[0]=='9'){
	  	  		  mysz();
	  	  		  myszPowrot();
	  	  		  HAL_UART_Receive(&huart2,buffer,20,100);
	  	  	  }
	  	  	  while(buffer[0]=='y'){
	  	  		  yellow();
	  	  		  HAL_UART_Receive(&huart2,buffer,20,100);
	  	  	  }
	  	  	  while(buffer[0]=='g'){
	  	  		  green();
	  	  		  HAL_UART_Receive(&huart2,buffer,20,100);
	  	  	  }
	  	  	  while(buffer[0]=='r'){
	  	  		  red();
	  	  		  HAL_UART_Receive(&huart2,buffer,20,100);
	  	  	  }
	  	  	  if(buffer[0]=='0'){
	  	  		  zeruj();
	  	  	  }
	  	    }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
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
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  HAL_GPIO_WritePin(led8y_GPIO_Port, led8y_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, led1y_Pin|led2y_Pin|led7y_Pin|led6y_Pin 
                          |led6r_Pin|led5r_Pin|led4g_Pin|led4r_Pin 
                          |led1r_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, led5y_Pin|led4y_Pin|led1g_Pin|led2g_Pin 
                          |led3g_Pin|led5g_Pin|led8r_Pin|led7r_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, led3y_Pin|led6g_Pin|GPIO_PIN_12|led7g_Pin 
                          |led8g_Pin|led2r_Pin|led3r_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : guzik_Pin */
  GPIO_InitStruct.Pin = guzik_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(guzik_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : led8y_Pin */
  GPIO_InitStruct.Pin = led8y_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(led8y_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : led1y_Pin led2y_Pin led7y_Pin led6y_Pin 
                           led6r_Pin led5r_Pin led4g_Pin led4r_Pin 
                           led1r_Pin */
  GPIO_InitStruct.Pin = led1y_Pin|led2y_Pin|led7y_Pin|led6y_Pin 
                          |led6r_Pin|led5r_Pin|led4g_Pin|led4r_Pin 
                          |led1r_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : led5y_Pin led4y_Pin led1g_Pin led2g_Pin 
                           led3g_Pin led5g_Pin led8r_Pin led7r_Pin */
  GPIO_InitStruct.Pin = led5y_Pin|led4y_Pin|led1g_Pin|led2g_Pin 
                          |led3g_Pin|led5g_Pin|led8r_Pin|led7r_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : led3y_Pin led6g_Pin PB12 led7g_Pin 
                           led8g_Pin led2r_Pin led3r_Pin */
  GPIO_InitStruct.Pin = led3y_Pin|led6g_Pin|GPIO_PIN_12|led7g_Pin 
                          |led8g_Pin|led2r_Pin|led3r_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
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
