/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "i2c-lcd.h"
#include "stm32f1_rc522.h"
#include "string.h"
#include "stdio.h"
#include "ds1307_for_stm32_hal.h"
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
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

//Variables utilizadas
int bandera=0,i=0,enter=0,bandera,j=0,n=0;
char palabra[100],actual[5],buff[20];
int segundos=0,minutos=0,horas=0,year=0,dia=0,mes=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t estado;
uint8_t str[MAX_LEN];
uint8_t data[1]=" ";
char identificacion[5];

struct base//Estructura que almacena el ID de la tarjeta y el nombre del usuario
{
    char c[5];
    char nombre[100];
}tarjetas[10];

//Esta funcion retornara un 1 en caso de a ver encontrado una tarjeta y un 0 en el caso contrario
int reconocer(){

	 j=0;
	int count=0;
	do {
		for (int var = 0; var < 5; ++var) {//Busqueda de coincidencia de los ID de las tarjetas
											//En la estructura
			if (tarjetas[j].c[var]==identificacion[var])
				    {
				    	count++;
				    }
		}
		if (count==5) {
			return 1;

		} else {
		count=0;
		}

		 j++;
	} while (j!=10);


	return 0;
}

//Funcion para agregar tarjetas
//Con la ayuda de la funcion de reconocer podemos saber si ya la tarjeta
//Leida se puede o no agregar en caso de ya estar agregada nos retornara un 1
//indicando que ya esta agregada en el caso contrario un 0
int agregar(){
	int pos=0;
if(reconocer()){
	return 1;
}
for (int var = 0; var < 10; ++var) {//Posicionamiento en un espacio vacio de la estructura
									//Para poder agregar la tarjeta
	if(tarjetas[var].c[0]=='\0'){
		pos=var;
		break;
	}
}
memcpy(tarjetas[pos].c,identificacion,5);//Copiando tarjeta leida a la posicion pertinente
return 0;
}
void eliminar(int pos){//eliminacion de tarjetas
	for (int var = 0; var < 5; ++var) {
		tarjetas[pos].c[var]='\0';
	}
}
void clean(){//Limpiador del array que almacena los comandos de eliminar y agregar
	for (int var = 0; var < 100; ++var) {
		palabra[var]='\0';
	}
}
void call(){//Llamado de el lector de tarjeta
		estado=MFRC522_Request(PICC_REQIDL,str);//Si estado=0 significa que a pasado una tarjeta por el lector
												//En caso contrario permanecera en 1 o 2 dependiendo si hay o no un error
		 estado=MFRC522_Anticoll(str);
		  memcpy(identificacion,str,5);//Copiando la tarjeta leida al array que indica la tarjeta actual
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
  MX_SPI1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  //Inicializacion de los perifericos y los protocolos de comunicacion
  lcd_init();
MFRC522_Init();
HAL_UART_Receive_IT(&huart2, data, 1);
DS1307_Init(&hi2c1);

//Seteo de la Hora
// DS1307_SetTimeZone(-4,00);
//DS1307_SetHour(11);
//DS1307_SetMinute(12);
//DS1307_SetSecond(0);

//DS1307_SetDate(30);
//DS1307_SetMonth(4);
//DS1307_SetYear(2022);

//Inicializacion de la tarjeta principal
memcpy(tarjetas[0].nombre,"Miguel",6);//Nombre del usuario principal

//ID de la tarjeta principal en Hexadecimal
tarjetas[0].c[0]=0xAC;
tarjetas[0].c[1]=0x6;
tarjetas[0].c[2]=0xA;
tarjetas[0].c[3]=0x4A;
tarjetas[0].c[4]=0xEA;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (!bandera) {//Mensaje el cual se presentara cuando no se haya hecho una accion en el lector de tarjeta
		  lcd_put_cur(0,0);
		  lcd_send_string("Proyecto Final");
		  lcd_put_cur(1,0);
		  lcd_send_string("   de micro");
	}
	 call();//llamado de la funcion de la tarjeta
	 if (reconocer()&&j==0) {//Bandera la cual indica que se ejercio alguna accion en el lector de tarjetas
		bandera=1;
	}


	 //eliminar y agregar tarjetas
	  if(enter==2&&bandera){//confirmacion que se desea borrar o agregar alguna tarjeta

		  if(strcmp(palabra,"agregar")==0){//Confirmacion de que se pidio agregar una tarjeta

			  lcd_clear();//Limpiador de la pantalla lcd
			  lcd_put_cur(0,0);//posicionamiento de la lcd
			  //mensaje dirigido al usuario
			  lcd_send_string("Introduzca");
			  lcd_put_cur(1,0);
			  lcd_send_string("la tarjeta");
			  reconocer();//llamado a la funcion de reconocer

			  if(estado==0&&j>0){//en el caso que la tarjeta introducida
			  if(agregar()==0){//no sea la principal y no este agregada se agregara

				  //Mensaje indicando que se agrego correctamente
				  lcd_clear();
				  lcd_put_cur(0,0);
				  lcd_send_string("Agregado");
				  lcd_put_cur(1,0);
				  lcd_send_string("correctamente");
				  HAL_Delay(3000);//Tiempo de visualizacion
				  lcd_clear();

			  	}
			  	else{//Mensaje para el usuario en caso de que la tarjeta se encuentre registrada

			  		 lcd_clear();
					  lcd_put_cur(0,0);
					  lcd_send_string("Este usuario");
					  lcd_put_cur(1,0);
					  lcd_send_string("ya esta registrado");
			  		HAL_Delay(3000);//Tiempo de visualizacion
			  		lcd_clear();
			  	}
			  //Reinicio del programa para la entrada de una nueva tarjeta
			  	 clean();
			  	 i=0;
			  	 enter=0;
			  	 bandera=0;
			  }
		 	  }
		  else if(strcmp(palabra,"eliminar")==0){//Confirmacion de que se pidio eliminar una tarjeta

			  //Mensaje al usuario
			  lcd_clear();
			  lcd_put_cur(0,0);
			  lcd_send_string("Introduzca");
			  lcd_put_cur(1,0);
			  lcd_send_string("   la tarjeta");

			  	  reconocer();//llamado a la funcion de reconocer
			  			  if(estado==0&&j>0){//En el caso de reconocer una tarjeta que no sea la principal
			  				  if(reconocer()){//Se procede a borrarla en la ubicacion correspondiente dada por la variable J

			  					  //Eliminacion del ID de la tarjeta y el nombre de usuario
			  					  for (int var = 0; var < 100; ++var)
			  					{		if (var<5) {
			  						tarjetas[j].c[var]='\0';
											}
										tarjetas[j].nombre[var]='\0';
									}

			  					  //Mensaje para el usuario en el caso de que se efectue la eliminacion
			  					  lcd_clear();
								  lcd_put_cur(0,0);
								  lcd_send_string("Elimiacion");
								  lcd_put_cur(1,0);
								  lcd_send_string("exitosa");
								  HAL_Delay(3000);//Tiempo de visualizacion
								  lcd_clear();
			  							  			  }
			  				  else {//En el caso de que la tarjeta introducida no este agregada se presenta el siguiente mensaje
							  lcd_clear();
							  lcd_put_cur(0,0);
							  lcd_send_string("Este usuario no");
							  lcd_put_cur(1,0);
							  lcd_send_string("esta registrado");
			  					  HAL_Delay(3000);
							  	lcd_clear();
							}
			  				//Reinicio del programa para la entrada de una nueva tarjeta
			  				 clean();
							 i=0;
							 enter=0;
							 bandera=0;
			  				  }
		  }
	  }
	  else if(estado==0){//Abrir y cerrar puerta
		  if (reconocer()) {//Si la tarjeta introducida es reconocida se cambia el estado de la puerta

		  	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);//Estado de la puerta

		  	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3)) {//Si la puerta esta abierta

		  		//Se presenta el nombre del usuario correspondiente y la hora y fecha al momento
		  		 lcd_clear();
		  		 lcd_put_cur(0,0);
				 lcd_send_string("Saludo");
				 lcd_put_cur(0,8);
				 lcd_send_string(tarjetas[j].nombre);//impresion del nombre luego del saludo

				 //Cargar fecha del momento
				 horas = DS1307_GetHour();
				 minutos = DS1307_GetMinute();
				 segundos = DS1307_GetSecond();
				 dia=DS1307_GetDate();
				 mes=DS1307_GetMonth();
				 year=DS1307_GetYear();

				sprintf(buff,"%2d:%2d %2d/%2d/%2d",horas,minutos,dia,mes,year);//Array que almacena la fecha y su estilo
				lcd_put_cur(1,0);
				lcd_send_string(buff);//Impresion de la fecha en la fila de abajo
				HAL_Delay(5000);//Tiempo de visualizacion
				lcd_clear();

		  				}
		  			  	 else
		  			  	 {//En el caso de que la puerta pase de abierto a cerrado
		  			  		 //Se enviara el siguiente mensaje
		  			  	lcd_clear();
		  			  	lcd_put_cur(0,0);
						 lcd_send_string("Adios");
						 HAL_Delay(3000);//Tiempo de visualizacion
						 lcd_clear();
		  			  	 }
		  }
		  else
		  {//En el caso de que la tarjeta introducida no es reconocida
			  //Se envia el siguiente mensaje
			  lcd_clear();
			  lcd_put_cur(0,0);
			  lcd_send_string("Este usuario no");
			  lcd_put_cur(1,0);
			  lcd_send_string("esta registrado");
			  HAL_Delay(3000);//Tiempo de visualizacion
			  lcd_clear();
		  }
		  bandera=0;//reinicio para la entrada o la salida del siguiente usuario

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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
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
  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c1.Init.Timing = 0x00303D5B;
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)//Callback del uart
{
	 HAL_UART_Transmit(&huart2,data, 1,10);//Transmisión por UART

	 //Almacenamiento por teclado y activacion de la bandera para eliminar o borrar
	 if(strcmp(palabra,"eliminar")==0){
		 enter=2;
		 i=0;
	 }
	 else if(strcmp(palabra,"agregar")==0&&enter){
		 if(data[0]!='\r')
				  {
				  tarjetas[n].nombre[i]=data[0];//Almacenamiento del nombre de usuario
				  i++;
				  }
		 	 else {
		 		 enter=2;
		 		 i=0;
		 	}

	 }

	 else if(data[0]!='\r')
		 	 		  {
		 	 		  palabra[i]=data[0];//Almacenamiento de los comandos ya sea eliminar o borrar
		 	 		  i++;
		 	 		  }
	 else {
		 for (int var = 0; var < 10; ++var) {//Ubicacion del nombre de usuario
		 	if(tarjetas[var].nombre[0]=='\0'){
		 		n=var;
		 		break;
		 	}
		 }
		 enter=1;
		 i=0;

	}


					if (data[0]=='\b') {//Borrar en el caso de equivocacion
						clean();
						enter=0;
						i=0;
					}



	HAL_UART_Receive_IT(&huart2, data, 1);//Recivir por UART
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
