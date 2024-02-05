
/*
 * Uart.c
 *
 *  Created on: Jun 8, 2023
 *      Author: Jorge Luis Martinez Suarez.
 */
#include "UART.h"
char buff[BUFF_SIZE];
uint8_t Rx;
extern UART_HandleTypeDef huart3;
void Uart_init()
{
	HAL_UART_Receive_IT(&huart3, &Rx,1);
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	static int i=0;
		 if(Rx=='\n')
		 {

			 buff[i]='\0';
			 Decoder(buff);
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

		HAL_UART_Receive_IT(&huart3,&Rx,1);

}
void Transmit(char *pData)
{
	HAL_UART_Transmit(&huart3,(uint8_t*)pData,strlen(pData),HAL_MAX_DELAY);
}


