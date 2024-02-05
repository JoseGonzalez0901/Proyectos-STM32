/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
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
#include<Nema17.h>
#include"stdio.h"
#include"string.h"
#include"stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define PRIMERA_FILA_Y 400
#define SEGUNDA_FILA_Y 1040

#define PRIMERA_COLUMNA_X 700
#define SEGUNDA_COLUMNA_X 1140

#define STOP 0
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

TypedefMotor Motores[Max_motors];
char buff[100]="\0";
uint8_t Rx='\0';
uint32_t Velocidad=700;
int var=0;
int Orificio=0;
int Alimentar=0;
int Modo=0;
int Next=0;
int x[4]={PRIMERA_COLUMNA_X,SEGUNDA_COLUMNA_X,STOP,SEGUNDA_COLUMNA_X};
int y[4]={PRIMERA_FILA_Y,STOP,SEGUNDA_FILA_Y,STOP};
int xm[4]={PRIMERA_COLUMNA_X,PRIMERA_COLUMNA_X+SEGUNDA_COLUMNA_X,PRIMERA_COLUMNA_X+SEGUNDA_COLUMNA_X,PRIMERA_COLUMNA_X};
int ym[4]={PRIMERA_FILA_Y,PRIMERA_FILA_Y,PRIMERA_FILA_Y+SEGUNDA_FILA_Y,PRIMERA_FILA_Y+SEGUNDA_FILA_Y};
int Tiempo=30;
int count=0;
enum{Orificio1,Orificio2,Orificio3,Orificio4,FREE};
enum{Automatico,Manual,Inicio,NA};
int flash=0;
uint8_t final[3]={0xFF,0xFF,0xFF};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Recibir(char *string)
{
	if(strstr(string,"automatico"))
	{
		Modo=Automatico;

	}
	else if(strstr(string,"Alimentar"))
	{
		Alimentar=1;
		count=Tiempo;
	}
	else if(strstr(string,"Detener"))
	{
		Alimentar=0;
	}
	else if(strstr(string,"manual"))
	{
		Orificio=FREE;
		Modo=Manual;

		Config_motor_caracteristicas(&Motores[1],0,1);
		Config_motor_caracteristicas(&Motores[0],0,0);
	}
	else if(strstr(string,"1"))
	{
		Orificio=Orificio1;
		Config_motor_caracteristicas(&Motores[1],0,1);
		Config_motor_caracteristicas(&Motores[0],0,0);

	}
	else if(strstr(string,"2"))
	{
		Orificio=Orificio2;
		Config_motor_caracteristicas(&Motores[1],0,1);
		Config_motor_caracteristicas(&Motores[0],0,0);
	}
	else if(strstr(string,"3"))
	{
		Orificio=Orificio3;
		Config_motor_caracteristicas(&Motores[1],0,1);
		Config_motor_caracteristicas(&Motores[0],0,0);
	}
	else if(strstr(string,"4"))
	{
		Orificio=Orificio4;
		Config_motor_caracteristicas(&Motores[1],0,1);
		Config_motor_caracteristicas(&Motores[0],0,0);
	}
	else if(strstr(string,"S"))
	{
		Tiempo+=30;
	}
	else if(strstr(string,"R"))
	{
		Tiempo-=30;
	}
	else if(strstr(string,"principal"))
	{
		Modo=Inicio;
		Alimentar=0;
		Next=0;
		var=0;
	}
}
void Move(int var)
{
		if(!Motores[0].Move)
		  {
			if(Modo==Automatico)
			{
				  Config_motor_caracteristicas(&Motores[0],y[var],Motores[0].Dir);

			}
			else if(Modo==Manual)
			{
				  Config_motor_caracteristicas(&Motores[0],ym[var],Motores[0].Dir);

			}
		 }
		  if(!Motores[1].Move)
		  {
			  if(Modo==Automatico)
			  {
				  Config_motor_caracteristicas(&Motores[1],x[var],Motores[1].Dir);
			  }
			  else if(Modo==Manual)
			  {
				  Config_motor_caracteristicas(&Motores[1],xm[var],Motores[1].Dir);
			  }

		  }
}
void Transmit(UART_HandleTypeDef huart,char *string)
{
	HAL_UART_Transmit(&huart,(uint8_t*)string,strlen(string),HAL_MAX_DELAY);
}
void Enviar(int dato,char *string)
{
	char mensaje[50]="\0";
	if(strstr(string,"Reloj"))
	{
		sprintf(mensaje,"n1.val=%i",dato);
	}
	else if(strstr(string,"Orificio"))
	{
		sprintf(mensaje,"n0.val=%i",dato);
	}
	else if(strstr(string,"FinishAuto"))
	{
		if(dato==0)
		{
			sprintf(mensaje,"r0.val=%i",dato);
		}
		else
		{
			sprintf(mensaje,"r1.val=%i",dato);
		}

	}
	Transmit(huart1,mensaje);
	Transmit(huart1,(char*)final);

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
  MX_USART2_UART_Init();
  MX_TIM6_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  Modo=Inicio;
  Orificio=FREE;
  HAL_UART_Receive_IT(&huart1,&Rx,1);
  HAL_TIM_Base_Start(&htim6);
  Config_motor_Pin(&Motores[0],X_STEP_GPIO_Port,X_DIR_GPIO_Port,X_STEP_Pin,X_DIR_Pin);
  Config_motor_Pin(&Motores[1],Y_STEP_GPIO_Port,Y_DIR_GPIO_Port,Y_STEP_Pin,Y_DIR_Pin);
  uint32_t start = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  if(Alimentar)
	  {
		Movimiento_simultaneo(Motores,Velocidad);
		if(Modo==Automatico)
		{
		Enviar(var,"Orificio");
		Enviar(count,"Reloj");
		}
		else if(Modo==Manual)
		{
		Enviar(count, "Reloj");
		}
	  }
	switch (Modo) {
		case Automatico:

			if(Alimentar&&!Next)
			{
				if(var==Orificio1)
				{
					Config_motor_caracteristicas(&Motores[0],0,0);//Y
					Config_motor_caracteristicas(&Motores[1],0,1);//X

				}
				else if(var==Orificio3)
				{
					Config_motor_caracteristicas(&Motores[0],0,0);//Y
				}
				else if(var==Orificio4)
				{
					Config_motor_caracteristicas(&Motores[1],0,0);//X

				}
				if(var>Orificio4)
				{
					Enviar(0,"FinishAuto");
					Enviar(1,"FinishAuto");
					Modo=Inicio;
					Alimentar=0;
					var=0;
				}
				else
				{
					Move(var++);
					Next=1;
				}

			}
			else if(Next&&!Motores[0].Move&&!Motores[1].Move)
			{

				if(!((HAL_GetTick() - start) < 1000))
				{
					if(Alimentar)
					{
					  start = HAL_GetTick();


					  count--;
					  flash=!flash;

					}
					//enviar valor de count
				}
				else if(count<=0)
				{
					Next=0;

					count=Tiempo;

				}
			}

			break;
		case Manual:
			if(Alimentar&&!Next)
			{
				if(Orificio!=FREE)
				{
				Move(Orificio);
				Next=1;
				}

			}
			else if(Next&&!Motores[0].Move&&!Motores[1].Move)
			{
				if(!((HAL_GetTick() - start) < 1000))
				{
					if(Alimentar)
					{
						start = HAL_GetTick();
						count--;
					}
					//enviar valor de count
				}
				else if(count<=0)
				{
					Next=0;
					Alimentar=0;
					Orificio=FREE;
					while(1)
					{
						Movimiento_simultaneo(Motores,Velocidad);
						if(!Motores[0].Move)//Y
						{
							if(!HAL_GPIO_ReadPin(Limit_y_GPIO_Port,Limit_y_Pin))
							{
								Config_motor_caracteristicas(&Motores[0],1,1);
							}
						}
						if(!Motores[1].Move)//X
						{
							if(!HAL_GPIO_ReadPin(Limit_x_GPIO_Port,Limit_x_Pin))
							{
								Config_motor_caracteristicas(&Motores[1],1,0);
							}
						}
						if(HAL_GPIO_ReadPin(Limit_y_GPIO_Port,Limit_y_Pin)&&HAL_GPIO_ReadPin(Limit_x_GPIO_Port,Limit_x_Pin))
						{
							break;
						}

					}
					count=Tiempo;
				}
			}
			break;
		case Inicio:
			Movimiento_simultaneo(Motores,Velocidad);
			if(!Motores[0].Move)//Y
			{
				if(!HAL_GPIO_ReadPin(Limit_y_GPIO_Port,Limit_y_Pin))
				{
					Config_motor_caracteristicas(&Motores[0],1,1);
				}
			}
			if(!Motores[1].Move)//X
			{
				if(!HAL_GPIO_ReadPin(Limit_x_GPIO_Port,Limit_x_Pin))
				{
					Config_motor_caracteristicas(&Motores[1],1,0);
				}
			}
			break;
		default:
			break;
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 25;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 50-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 0xffff-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
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
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_GREEN_Pin|Enable_Pin|X_STEP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Y_DIR_Pin|Y_STEP_Pin|X_DIR_Pin|Z_STEP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Limit_x_Pin Limit_y_Pin */
  GPIO_InitStruct.Pin = Limit_x_Pin|Limit_y_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_GREEN_Pin */
  GPIO_InitStruct.Pin = LED_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED_GREEN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Y_DIR_Pin Y_STEP_Pin X_DIR_Pin Z_STEP_Pin */
  GPIO_InitStruct.Pin = Y_DIR_Pin|Y_STEP_Pin|X_DIR_Pin|Z_STEP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Enable_Pin X_STEP_Pin */
  GPIO_InitStruct.Pin = Enable_Pin|X_STEP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	 static int i=0;
	 if(Rx=='.')
	 {
		 buff[i]='\0';
		 Recibir(buff);
		 i=0;
	 }
	 else if(i<100)
	 {
		 buff[i]=Rx;
		 i++;
	 }
	 else
	 {
		 i=0;
	 }

	HAL_UART_Receive_IT(&huart1,&Rx,1);
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
