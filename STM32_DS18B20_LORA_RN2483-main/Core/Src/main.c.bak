#include "main.h"
#include "stm32l4xx_hal.h"
#include "math.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
uint8_t Data[50];
uint8_t LoRa_Rx[1];
uint8_t Data_2[50];
char	Data_3[50];
char	*remaining;
long	Data_4;
uint8_t Data_5[50];
uint8_t Data_6[50];
uint8_t Data_7[50];


#include "stdio.h"
#include "string.h"
uint8_t Data_2[50];

uint8_t Presence = 0;
uint8_t Temp_byte1;
uint8_t Temp_byte2;
uint16_t TEMP;
float Temperature=5;

uint8_t Dat[]="Hello_Pantelis\r\n";
uint8_t Moisture;
uint8_t BLE[5];
uint8_t buff_reset[]="sys reset\r\n";
uint8_t buff_vdd[]="sys get vdd\r\n";
uint8_t buff_ver[]="sys get ver\r\n";
uint8_t buff_hweui[]="sys get hweui\r\n";
uint8_t buff_sleep[]="sys sleep 5000\r\n";
uint8_t buff_appeui[]="mac get appeui\r\n";
uint8_t buff_deveui[]="mac get deveui\r\n";
uint8_t buffvdd[13]="sys get vdd\r\n";
uint8_t buff_radiotx[]="radio tx 48656c6C6F\r\n";
uint8_t buff_radiorx[]="radio rx 0\r\n";
uint8_t buff_macpause[]="mac pause\r\n";
uint8_t buff_radio_set_wdt[]="radio set wdt 2000\r\n";



volatile float temp;
volatile uint8_t humi;
volatile uint8_t read_data[2];
volatile uint16_t reg=0;



uint8_t Received_Data;
uint8_t Count_Flag=0;
extern uint8_t Lora_Rx[1];
uint8_t receive[5];



ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

////////Protocol Function Implementation For Receiver Side////Place Inside RxCpltCallback/////////////
void Receive_Data_With_Protocol(){
  	if((LoRa_Rx[0]==0x72)&&(Count_Flag==0)){
  		Count_Flag=1;
  		Data_2[0]=LoRa_Rx[0];
  	}
  	else if((LoRa_Rx[0]==0x61)&&(Count_Flag==1)){
	  	Count_Flag++;
	  	Data_2[1]=LoRa_Rx[0];
  	}
	else if((LoRa_Rx[0]==0x64)&&(Count_Flag==2)){
		Count_Flag++;
		Data_2[2]=LoRa_Rx[0];
	}
	else if((LoRa_Rx[0]==0x69)&&(Count_Flag==3)){
		Count_Flag++;
		Data_2[3]=LoRa_Rx[0];
	}
	else if((LoRa_Rx[0]==0x6f)&&(Count_Flag==4)){
		Count_Flag++;
		Data_2[4]=LoRa_Rx[0];
	}
	else if((LoRa_Rx[0]==0x5f)&&(Count_Flag==5)){
		Count_Flag++;
		Data_2[5]=LoRa_Rx[0];
	}
	else if((LoRa_Rx[0]==0x72)&&(Count_Flag==6)){
		Count_Flag++;
		Data_2[6]=LoRa_Rx[0];
	}
	else if((LoRa_Rx[0]==0x78)&&(Count_Flag==7)){
		Count_Flag++;
		Data_2[7]=LoRa_Rx[0];
	}
	else if((LoRa_Rx[0]==0x20)&&(Count_Flag==8)){
		Count_Flag++;
		Data_2[8]=LoRa_Rx[0];
	}
	else if(Count_Flag==9){
		Count_Flag++;
		Data_2[9]=LoRa_Rx[0];
	}
	else if(Count_Flag==10){
		Count_Flag++;
		Data_2[10]=LoRa_Rx[0];
	}
	else if(Count_Flag==11){
		Count_Flag++;
		Data_2[11]=LoRa_Rx[0];
		//HAL_UART_Transmit(&huart2,Data_2,12,1000);
		Count_Flag=0;
		Data_3[0]=Data_2[9];
		Data_3[1]=Data_2[10];
		Data_3[2]=Data_2[11];
		//HAL_UART_Transmit(&huart2,Data_3,3,1000);
		Data_4=strtol(Data_3,&remaining,16);
		sprintf(Data_5,"Temp is equal to %d",Data_4);
		HAL_UART_Transmit(&huart2,Data_5,20,1000);
	}
}
//////////////////////////////////////////////////////////////////////////////////////////////

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	Receive_Data_With_Protocol();
	//HAL_UART_Transmit(&huart2,LoRa_Rx,1,1000);
}

////////For Reception Device////////////
void Lora_Temp_Reception(){
	Temperature=(float)Data_4/16;
	sprintf(Data_5,"Temperature_Recep is %f\r\n",Temperature);
	HAL_UART_Transmit(&huart2,Data_5,strlen((char*)Data_5),1000);
	HAL_Delay(50);
}
////////For Transmition Device////////////
void Lora_Temp_Data_Transmision(){
	sprintf((char*)buff_radiotx,"radio tx %d\r\n",TEMP);
	HAL_UART_Transmit(&huart5,buff_radiotx,strlen((char*)buff_radiotx),1000);
	HAL_Delay(50);
}

///////////////////HX711 Functions/////////////////////

void Set_Pin_Output (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void Set_Pin_Input (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void delay (uint32_t us)
{
    __HAL_TIM_SET_COUNTER(&htim1,0);
    while ((__HAL_TIM_GET_COUNTER(&htim1))<us);
}

int32_t HX711_value(void)
{
  uint32_t data = 0;
  float grammaria = 0;
  uint32_t  startTime = HAL_GetTick();
  while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13) == GPIO_PIN_SET)
  {
    if(HAL_GetTick() - startTime > 150)
      return 0;
  }
  for(int8_t i=0; i<24 ; i++)
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
    delay(1);
    data = data << 1;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
    delay(1);
    if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13) == GPIO_PIN_SET)
      data ++;
  }
  data = data ^ 0x800000;
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
  delay(1);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
  delay(1);
  sprintf((char*)Data_6,"Data is equal to %lu\r\n",data);
  HAL_UART_Transmit(&huart2,Data_6,strlen((char*)Data_6),1000);
  grammaria=(0.327*data);
  grammaria=grammaria/22734451.2;
  grammaria=grammaria/0.1203;
  sprintf((char*)Data_7,"Weight is equal to %.4f Grams\r\n",grammaria);
  HAL_UART_Transmit(&huart2,Data_7,strlen((char*)Data_7),1000);
  return data;
}

uint8_t DS18B20_Start (void)
{
	uint8_t Response = 0;
	Set_Pin_Output(GPIOC, GPIO_PIN_13);   // set the pin as output
	HAL_GPIO_WritePin (GPIOC, GPIO_PIN_13, 0);  // pull the pin low
	delay (480);   // delay according to datasheet

	Set_Pin_Input(GPIOC, GPIO_PIN_13);    // set the pin as input
	delay (80);    // delay according to datasheet

	if (!(HAL_GPIO_ReadPin (GPIOC, GPIO_PIN_13))) Response = 1;    // if the pin is low i.e the presence pulse is detected
	else Response = -1;

	delay (400); // 480 us delay totally.

	return Response;
}

void DS18B20_Write (uint8_t data)
{
	Set_Pin_Output(GPIOC, GPIO_PIN_13);  // set as output

	for (int i=0; i<8; i++)
	{

		if ((data & (1<<i))!=0)  // if the bit is high
		{
			// write 1

			Set_Pin_Output(GPIOC, GPIO_PIN_13);  // set as output
			HAL_GPIO_WritePin (GPIOC, GPIO_PIN_13, 0);  // pull the pin LOW
			delay (1);  // wait for 1 us

			Set_Pin_Input(GPIOC, GPIO_PIN_13);  // set as input
			delay (50);  // wait for 60 us
		}

		else  // if the bit is low
		{
			// write 0

			Set_Pin_Output(GPIOC, GPIO_PIN_13);
			HAL_GPIO_WritePin (GPIOC, GPIO_PIN_13, 0);  // pull the pin LOW
			delay (50);  // wait for 60 us

			Set_Pin_Input(GPIOC, GPIO_PIN_13);
		}
	}
}

uint8_t DS18B20_Read (void)
{
	uint8_t value=0;

	Set_Pin_Input(GPIOC, GPIO_PIN_13);

	for (int i=0;i<8;i++)
	{
		Set_Pin_Output(GPIOC, GPIO_PIN_13);   // set as output

		HAL_GPIO_WritePin (GPIOC, GPIO_PIN_13, 0);  // pull the data pin LOW
		delay (1);  // wait for > 1us

		Set_Pin_Input(GPIOC, GPIO_PIN_13);  // set as input
		if (HAL_GPIO_ReadPin (GPIOC, GPIO_PIN_13))  // if the pin is HIGH
		{
			value |= 1<<i;  // read = 1
		}
		delay (50);  // wait for 60 us
	}
	return value;
}
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_UART5_Init(void);
static void MX_TIM1_Init(void);
static void MX_I2C1_Init(void);

int main(void)
{

  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_UART5_Init();
  MX_TIM1_Init();
  MX_I2C1_Init();

  HAL_TIM_Base_Start(&htim1);


  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
  HAL_Delay(2000);

  //HAL_UART_Transmit(&huart1,Data,strlen((char*)Data),1000);
  //HAL_UART_Receive_IT(&huart2,LoRa_Rx,1);

  //HAL_UART_Receive_IT(&huart1,BLE,1);


  HAL_UART_Receive_IT(&huart1,BLE,1);

  HAL_UART_Receive_IT(&huart5,LoRa_Rx,1);

  HAL_UART_Transmit(&huart5,buff_reset,strlen((char*)buff_reset),1000);
  HAL_Delay (100);
  HAL_UART_Transmit(&huart5,buff_vdd,strlen((char*)buff_vdd),1000);
  HAL_Delay (100);
  HAL_UART_Transmit(&huart5,buff_ver,strlen((char*)buff_ver),1000);
  HAL_Delay (100);
  HAL_UART_Transmit(&huart5,buff_hweui,strlen((char*)buff_hweui),1000);
  HAL_Delay (100);
  HAL_UART_Transmit(&huart5,buff_appeui,strlen((char*)buff_appeui),1000);
  HAL_Delay (100);
  HAL_UART_Transmit(&huart5,buff_deveui,strlen((char*)buff_deveui),1000);
  HAL_Delay (100);
  sprintf((char*)Data_6,"Welcome Kostis\r\n");
  HAL_UART_Transmit(&huart2,Data_6,strlen((char*)Data_6),1000);
  while (1){
  	  ///////////Temperature Readings And Conversion///////////

	  	  /*Presence = DS18B20_Start ();
	  	  HAL_Delay (1);
	  	  DS18B20_Write (0xCC);
	  	  DS18B20_Write (0x44);
	  	  HAL_Delay (250);

	  	  Presence = DS18B20_Start ();
	      HAL_Delay(1);
	      DS18B20_Write (0xCC);
	      DS18B20_Write (0xBE);

	      Temp_byte1 = DS18B20_Read();
	  	  Temp_byte2 = DS18B20_Read();
	  	  TEMP = (Temp_byte2<<8)|Temp_byte1;
	  	  Temperature = (float)TEMP/16;*/
	  	  //sprintf(Data_2,"Temperature is %d\r\n",TEMP);
	  	  //HAL_UART_Transmit(&huart2,Data_2,strlen((char*)Data_2),1000);
	  	  //sprintf(Data,"Temperature is %x\r\n",TEMP);
	  	  //HAL_UART_Transmit(&huart2,Data,strlen((char*)Data),1000);
	  	  //sprintf(Data,"Temperature is %f\r\n",Temperature);
	  	  //HAL_UART_Transmit(&huart2,Data,strlen((char*)Data),1000);
	  	  HX711_value();
	  	  /*HAL_UART_Transmit(&huart5,buff_macpause,strlen((char*)buff_macpause),1000);
	  	  HAL_Delay (1000);
	  	  HAL_UART_Transmit(&huart5,buff_radio_set_wdt,strlen((char*)buff_radio_set_wdt),1000);
	  	  HAL_Delay (1000);
	  	  HAL_UART_Transmit(&huart5,buff_radiorx,strlen((char*)buff_radiorx),1000);*/
	  	  //Lora_Temp_Reception();
	  	  //Lora_Temp_Data_Transmision();
	  	  //HAL_Delay (1000);
	  	  //Lora_Temp_Reception();
	  	  //Lora_Temp_Data_Transmision();
		  //HAL_UART_Transmit(&huart2,Data_2,1,1000);
		  //memset(BLE,0,1*sizeof(BLE[0]));
		  //HAL_UART_Transmit(&huart2,LoRa_Rx,strlen((char*)LoRa_Rx),1000);
		  //Lora_Temp_Reception();
		  //HAL_ADC_Start(&hadc1);
		  //HAL_Delay(100);
		  //HAL_ADC_GetValue(&hadc1);
		  //Moisture = HAL_ADC_GetValue(&hadc1);
		  /////HeartBeat Operation For Live Indication///////
		  //HAL_UART_Transmit(&huart1,Data,strlen((char*)Data),1000);
		  //HAL_UART_Transmit(&huart1,buff_reset,strlen((char*)buff_reset),1000);*/

		  /*HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7|GPIO_PIN_8, GPIO_PIN_SET);
	  	  HAL_Delay(100);
	  	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7|GPIO_PIN_8, GPIO_PIN_RESET);
	  	  //HAL_UART_Transmit(&huart1,buffvdd,strlen((char*)buffvdd),1000);
	  	  HAL_Delay(1000);
	  	  //HAL_UART_Transmit(&huart1,Data,strlen((char*)Data),1000);
	  	  sprintf(Data_2,"Temperature is %f\r\n",Temperature);
	  	  HAL_UART_Transmit(&huart2,Data_2,strlen((char*)Data_2),1000);*/

	  //HAL_ADC_Start(&hadc1);
	  //HAL_Delay(100);
	  //HAL_ADC_GetValue(&hadc1);
	  //Moisture = HAL_ADC_GetValue(&hadc1);

	  /////HeartBeat Operation For Live Indication///////
	  //HAL_UART_Transmit(&huart1,Data,strlen((char*)Data),1000);
	  /*HAL_UART_Transmit(&huart1,buff_reset,strlen((char*)buff_reset),1000);*/

	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7|GPIO_PIN_8, GPIO_PIN_SET);
	  HAL_Delay(100);
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7|GPIO_PIN_8, GPIO_PIN_RESET);
	  //HAL_UART_Transmit(&huart1,buffvdd,strlen((char*)buffvdd),1000);
	  HAL_Delay(1000);
	  //HAL_UART_Transmit(&huart1,Data,strlen((char*)Data),1000);
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}



//void USART1_IRQHandler(void)
//{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  //HAL_UART_IRQHandler(&huart1);
  //HAL_UART_Receive_IT(&huart1,BLE,5);
  //HAL_UART_Transmit(&huart2,BLE,strlen((char*)BLE),1000);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
//}




void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  HAL_UART_Receive_IT(&huart1,BLE,1);
  HAL_UART_Transmit(&huart2,BLE,strlen((char*)BLE),1000);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  HAL_UART_Receive_IT(&huart2,LoRa_Rx,1);
  //HAL_UART_Transmit(&huart2,BLE,strlen((char*)BLE),1000);

  //HAL_UART_Receive_IT(&huart2,BLE,1);
	//if ((BLE[0]==0x72){
	//memset(LoRa_Rx,0,15*sizeof(LoRa_Rx[0]));
  HAL_UART_Receive_IT(&huart2,Data_2,1);
  HAL_UART_Transmit(&huart2,Data_2,strlen((char*)Data_2),1000);

  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

void UART5_IRQHandler(void)
{
  /* USER CODE BEGIN UART5_IRQn 0 */

  /* USER CODE END UART5_IRQn 0 */
  HAL_UART_IRQHandler(&huart5);
  HAL_UART_Receive_IT(&huart5,LoRa_Rx,1);
  HAL_UART_Transmit(&huart2,LoRa_Rx,1,1000);
  HAL_UART_Transmit(&huart2,LoRa_Rx,strlen((char*)LoRa_Rx),1000);
  /* USER CODE BEGIN UART5_IRQn 1 */

  /* USER CODE END UART5_IRQn 1 */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_11;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_UART5|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_ADC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Uart5ClockSelection = RCC_UART5CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 4;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 16;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV6;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_10B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_16;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
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
  hi2c1.Init.Timing = 0x20303E5D;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
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

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 48-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0xffff;
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
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 57600;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PC7 PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
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
