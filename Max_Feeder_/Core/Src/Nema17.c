/*
 * Nema17.c
 *
 *  Created on: Mar 26, 2023
 *      Author: joser
 */
#include "Nema17.h"

extern TIM_HandleTypeDef htim6;
void delay (TIM_HandleTypeDef timer ,uint32_t delay)
{
	  __HAL_TIM_SET_COUNTER(&timer, 0);
	  while (__HAL_TIM_GET_COUNTER(&timer) < delay);
}
void Config_motor_Pin(TypedefMotor *Motor,GPIO_TypeDef *GPIOx_Step,GPIO_TypeDef *GPIOX_dir, uint16_t GPIO_Pin_Step,uint16_t GPIO_Pin_dir)
{

	Motor->Port_Step=GPIOx_Step;
	Motor->Pin_Step=GPIO_Pin_Step;
	Motor->Port_Dir=GPIOX_dir;
	Motor->Pin_Dir=GPIO_Pin_dir;
}
void Config_motor_caracteristicas(TypedefMotor *Motor,int Pasos,int Dir)
{
	Motor->Dir=Dir;
	Motor->Pasos=Pasos;
	HAL_GPIO_WritePin(Motor->Port_Dir,Motor->Pin_Dir,Motor->Dir);
}
void Movimiento_simultaneo(TypedefMotor Motor[],uint32_t velocidad )
{
	for (int var = 0; var < Max_motors; ++var)
	{

		if(Motor[var].Pasos>0)
		{
		Motor[var].Move=1;
		HAL_GPIO_WritePin(Motor[var].Port_Step,Motor[var].Pin_Step,1);
		}
	}
	delay(htim6,velocidad);
	for (int var = 0; var < Max_motors; ++var) {
		if(Motor[var].Pasos>0)
		{
	    HAL_GPIO_WritePin(Motor[var].Port_Step,Motor[var].Pin_Step,0);
		Motor[var].Pasos=Motor[var].Pasos-1;
		}
		else
		{
			Motor[var].Move=0;
		}
	}
	delay(htim6,velocidad);
}
void Movimiento_de_motor(TypedefMotor *Motor,uint32_t velocidad )
{
	HAL_GPIO_WritePin(Motor->Port_Dir,Motor->Pin_Dir,Motor->Dir);

	if(Motor->Pasos>0)
	{
	Motor->Move=1;
	HAL_GPIO_WritePin(Motor->Port_Step,Motor->Pin_Step,1);
	delay(htim6,velocidad);
    HAL_GPIO_WritePin(Motor->Port_Step,Motor->Pin_Step,0);
    delay(htim6,velocidad);
	Motor->Pasos=Motor->Pasos-1;
	}
	else
	{
		Motor->Move=0;
	}

}

