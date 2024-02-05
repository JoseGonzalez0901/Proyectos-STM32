/*
 * uart.h
 *
 *  Created on: Jun 8, 2023
 *      Author: mecat
 */

#ifndef INC_UART_H_
#define INC_UART_H_

#include "stm32f3xx_hal.h"
#include "string.h"
#include "MQTT.h"
#define BUFF_SIZE 150
void Transmit(char *pData);
void Uart_init();

#endif /* INC_UART_H_ */


