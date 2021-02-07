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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/*
 * 0x080e0000 u32 sysconfig
 * 0x080c0000 u32 edidconf
 * 0x080c0100 u32[64] edid
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "i2cdump_gen.h"
#include "utils.h"
#include "stmflash.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define __TC358840XBG_I2C_ADDR 0x0f
#define TC358840XBG_I2C_ADDR ( __TC358840XBG_I2C_ADDR << 1 )
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

uint8_t isInited=0;
volatile uint8_t isConnected=0;
volatile uint8_t isIniting=0;
volatile uint8_t pwmVal=31;
volatile uint8_t sysInitMode=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
static void replay_i2c(const i2c_op *ops, const uint16_t size);
static void i2c_conf_init();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin==GPIO_PIN_1){
		if(isIniting==1){
			char uartbuf[64];
			sprintf(uartbuf,"WARN: HDMI changed during init\n");
			CDC_Transmit_FS((uint8_t*)uartbuf,strlen(uartbuf));
		}
		/*char uartbuf[64];
		uint8_t i2cbuf[4]={0x00,0x00,0x00,0x00};
		//sprintf(uartbuf,"INT detected\n");
		i2cbuf[0]=0x85;
		i2cbuf[1]=0x0b;
		i2cbuf[2]=0xff;
		HAL_I2C_Master_Transmit(&hi2c2,TC358840XBG_I2C_ADDR,i2cbuf,3,10);
		i2cbuf[0]=0x00;
		i2cbuf[1]=0x14;
		i2cbuf[2]=0xbf;
		i2cbuf[3]=0x0f;
		HAL_I2C_Master_Transmit(&hi2c2,TC358840XBG_I2C_ADDR,i2cbuf,4,10);
		//CDC_Transmit_FS((uint8_t*)uartbuf,strlen(uartbuf));
		if(isInited){
			i2cbuf[0]=checkSysReg();
			sprintf(uartbuf,"SYS STATUS: %.2x\n",i2cbuf[0]);
			CDC_Transmit_FS((uint8_t*)uartbuf,strlen(uartbuf));
			if(!((i2cbuf[0] & 0x0f) ==0x0f)){	// disconnected, init
				setPWMPulse(63);
				isConnected=0;
				//replay_i2c(i2cops_init,sizeof(i2cops_init)/sizeof(i2c_op));
				sprintf(uartbuf,"HDMI Disconnected\n");
				CDC_Transmit_FS((uint8_t*)uartbuf,strlen(uartbuf));
			}else{
				isConnected=1;
				setPWMPulse(pwmVal);
				sprintf(uartbuf,"HDMI Connected\n");
				CDC_Transmit_FS((uint8_t*)uartbuf,strlen(uartbuf));
			}
		}*/
	}
}

void replay_i2c(const i2c_op *ops, const uint16_t size){

	HAL_Delay(10);
	for(uint16_t i=0;i<size;++i){
		HAL_Delay(ops[i].timestamp);
		uint8_t i2cbuf[6];
		if(ops[i].operate==0){	// Write
			i2cbuf[0]=ops[i].address / 256;
			i2cbuf[1]=ops[i].address % 256;
			if(ops[i].datasize==2){
				i2cbuf[2]=ops[i].data / 256;
				i2cbuf[3]=ops[i].data % 256;
			}else if(ops[i].datasize==1){
				i2cbuf[2]=ops[i].data % 256;
			}else if(ops[i].datasize==4){
				i2cbuf[2]= ops[i].data/(256*256*256);
				i2cbuf[3]= (ops[i].data % (256*256*256))/(256*256);
				i2cbuf[4]= (ops[i].data % (256*256))/(256);
				i2cbuf[5]= (ops[i].data % (256));
			}
			HAL_I2C_Master_Transmit(&hi2c2,TC358840XBG_I2C_ADDR,i2cbuf,ops[i].datasize+2,10);
		}else{	// Read
			i2cbuf[0]=ops[i].address / 256;
			i2cbuf[1]=ops[i].address % 256;
			HAL_I2C_Master_Transmit(&hi2c2,TC358840XBG_I2C_ADDR,i2cbuf,2,10);
			HAL_I2C_Master_Receive(&hi2c2,TC358840XBG_I2C_ADDR,i2cbuf,ops[i].datasize,10);
		}
	}
}
void i2c_conf_init(){
	HAL_NVIC_DisableIRQ(EXTI1_IRQn);
	isIniting=1;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
	replay_i2c(i2cops_init_be,sizeof(i2cops_init_be)/sizeof(i2c_op));
	if(STMFLASH_ReadWord(0x080e0000)==0xabcdefab){	// user defined EDID

	}else{	// preset EDID
		uint8_t i2cbuf[3]={0x8c,0x00,0x00};
		for(uint16_t i=0;i<256;++i){
			i2cbuf[1]=i;
			i2cbuf[2]=i2cops_init_edid[i];
			HAL_I2C_Master_Transmit(&hi2c2,TC358840XBG_I2C_ADDR,i2cbuf,3,10);
		}
	}
	replay_i2c(i2cops_init_ae,sizeof(i2cops_init_ae)/sizeof(i2c_op));
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
	isIniting=0;
	HAL_NVIC_EnableIRQ(EXTI1_IRQn);
}
void setPWMPulse(uint16_t pulseWidth){
	/*TIM_OC_InitTypeDef sConfigOC = {0};
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = pulseWidth;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);*/
	TIM3->CCR1=pulseWidth;
}
void updateBcaklight(uint16_t val){
	pwmVal=val;
	u32tou8 flashdata;
	flashdata.u8[0]=0xab;
	flashdata.u8[1]=pwmVal;
	flashdata.u8[2]=sysInitMode;
	STMFLASH_Write(0x080e0000,&flashdata.u32,1);
}
void setInitMode(uint8_t mode){
	sysInitMode=mode;
	u32tou8 flashdata;
	flashdata.u8[0]=0xab;
	flashdata.u8[1]=pwmVal;
	flashdata.u8[2]=sysInitMode;
	STMFLASH_Write(0x080e0000,&flashdata.u32,1);
}
uint8_t checkSysReg(){
	uint8_t i2cbuf[2]={0x00,0x00};
	i2cbuf[0]=0x85;
	i2cbuf[1]=0x20;
	HAL_I2C_Master_Transmit(&hi2c2,TC358840XBG_I2C_ADDR,i2cbuf,2,10);
	HAL_I2C_Master_Receive(&hi2c2,TC358840XBG_I2C_ADDR,i2cbuf,1,10);
	return i2cbuf[0];
}
uint8_t checkSysRegA(uint16_t addr){
	uint8_t i2cbuf[2]={0x00,0x00};
	i2cbuf[0]=addr/256;
	i2cbuf[1]=addr%256;
	HAL_I2C_Master_Transmit(&hi2c2,TC358840XBG_I2C_ADDR,i2cbuf,2,10);
	HAL_I2C_Master_Receive(&hi2c2,TC358840XBG_I2C_ADDR,i2cbuf,1,10);
	return i2cbuf[0];
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
  MX_USB_DEVICE_Init();
  MX_I2C2_Init();
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();

  u32tou8 flashdata;
  flashdata.u32= STMFLASH_ReadWord(0x080e0000);
  if(flashdata.u8[0]==0xab){
	  pwmVal=flashdata.u8[1];
	  sysInitMode=flashdata.u8[2];
  }else{
	  flashdata.u8[0]=0xab;
	  flashdata.u8[1]=31;
	  flashdata.u8[2]=0x00;
	  STMFLASH_Write(0x080e0000,&flashdata.u32,1);
  }

  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);	// Backlight PWM
  TIM3->CCR1=63;

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);	// RESETN=0 for 10ms, TOSHIBA
  HAL_Delay(10);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);	// RESETN=1

  HAL_Delay(500);

  /* dump TOSHIBA Chip and Revision ID */
  uint8_t i2cbuf[4]={0x00,0x00,0x00,0x00};
  char uartbuf[64];
  if(HAL_I2C_Master_Transmit(&hi2c2,TC358840XBG_I2C_ADDR,i2cbuf,2,10)!=HAL_OK){
	  sprintf(uartbuf,"I2C write to TC358840XBG failed\n");
  }else{
	  if(HAL_I2C_Master_Receive(&hi2c2,TC358840XBG_I2C_ADDR,i2cbuf,2,10)!=HAL_OK){
		  sprintf(uartbuf,"I2C read from TC358840XBG failed\n");
	  }else{
		  sprintf(uartbuf,"TC358840XBG ChipID=0x%.2x, RevisionID=0x%.2x\n",i2cbuf[1],i2cbuf[0]);
	  }
  }
  CDC_Transmit_FS((uint8_t*)uartbuf,strlen(uartbuf));

  // configure TOSHIBA
  i2c_conf_init();
  isInited=1;
  uint8_t sysreg=checkSysReg();
  sprintf(uartbuf,"SYS STATUS: %.2x\n",sysreg);
  CDC_Transmit_FS((uint8_t*)uartbuf,strlen(uartbuf));
  if((sysreg& 0x0f)==0x0f){	// Detected on boot?
	  isConnected=1;
  }else{
	  isConnected=0;
  }
  HAL_Delay(50);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	sysreg=checkSysReg();
	/*sprintf(uartbuf,"SYS STATUS L: %.2x\n",sysreg);
	CDC_Transmit_FS((uint8_t*)uartbuf,strlen(uartbuf));*/
	if((sysreg& 0x0f)==0x0f){	// detected
		if(!isConnected){	// first detected, hdmi ready
			if(sysInitMode==1){
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);	// RESETN=0 for 10ms, TOSHIBA
				HAL_Delay(10);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);	// RESETN=1
				HAL_Delay(500);
				i2c_conf_init();
			}
			setPWMPulse(pwmVal);	// recover
			HAL_Delay(3000);
			isConnected=1;
		}
	}else{	// lost
		if(isConnected){	// first lost
			setPWMPulse(63);	// backlight off
			if(sysInitMode==0){
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);	// RESETN=0 for 10ms, TOSHIBA
				HAL_Delay(10);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);	// RESETN=1
				HAL_Delay(500);
				i2c_conf_init();
			}
			isConnected=0;
		}
	}
	HAL_Delay(200);
	/*sprintf(uartbuf,"Heartbeat\n");
	CDC_Transmit_FS((uint8_t*)uartbuf,strlen(uartbuf));
	HAL_Delay(1000);*/
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
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
  hspi2.Init.NSS = SPI_NSS_HARD_OUTPUT;
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 2624;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 63;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 63;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA4 PA5 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

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
