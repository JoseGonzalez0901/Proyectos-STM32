/*
 * Nema17.h
 *
 *  Created on: Mar 26, 2023
 *      Author: joser
 */
#include "main.h"
#define Max_motors 2
typedef struct
{
	GPIO_TypeDef *Port_Step;
	uint16_t 	  Pin_Step;
	GPIO_TypeDef *Port_Dir;
	uint16_t 	  Pin_Dir;
	unsigned int Pasos;
	unsigned int Dir:1;
	unsigned int Move:1;

}TypedefMotor;

//Tiempo entre pasos
void delay (TIM_HandleTypeDef timer ,uint32_t delay);

//Configuracion de motor
void Config_motor_Pin(TypedefMotor *Motor,GPIO_TypeDef *GPIOx_Step,GPIO_TypeDef *GPIOX_dir, uint16_t GPIO_Pin_Step,uint16_t GPIO_Pin_dir);

//Funcion para configurar los pasos de los motores y direcciones
void Config_motor_caracteristicas(TypedefMotor *Motor,int Pasos,int Dir);

//Movimiento de los motores del sistema de manera simultanea
void Movimiento_simultaneo(TypedefMotor Motor[],uint32_t velocidad );

//Movimiento de los motores del sistema un motor
void Movimiento_de_motor(TypedefMotor *Motor,uint32_t velocidad );
