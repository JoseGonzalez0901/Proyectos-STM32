/*
 * Interrupciones.c
 *
 *  Created on: Jul 2, 2023
 *      Author: Informatica
 */

#include "Interrupciones.h"
#include "MQTT.h"

extern TIM_HandleTypeDef htim17;
extern bandera Flag;
static unsigned int Count_Gps=0,Count_Medidas=0,Count_Medidas2=0,Count_Medidas3=0,Count_Medidas4=0,GPS_timeout=0,MQTT_timeout=0;
void Timers_Init()
{

	  HAL_TIM_Base_Start_IT(&htim17);

}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
if(Flag.mqtt_command)
{
	MQTT_timeout++;
	if(MQTT_timeout>10)
	{
		Flag.mqtt_command=0;
	}
}
else
{
	MQTT_timeout=0;
}
///////////////////////////////////////////
if(Flag.gps_command)
{
	GPS_timeout++;
	if(GPS_timeout>10)
	{
		Flag.gps_command=0;
	}
}
else
{
	GPS_timeout=0;
}
///////////////////////////////////
if(Count_Medidas<5)
{
	Count_Medidas++;
}
else
{
	Count_Medidas=0;
	Flag.Send_Medidas=1;
}

////////////////////////////////////////
if(Count_Medidas2<5)
{
	Count_Medidas2++;
}
else
{
	Count_Medidas2=0;
	Flag.Send_Medidas2=1;
}
///////////////////////////////////////
if(Count_Medidas3<5)
{
	Count_Medidas3++;
}
else
{
	Count_Medidas3=0;
	Flag.Send_Medidas3=1;
}
////////////////////////////////////////
if(Count_Medidas4<5)
{
	Count_Medidas4++;
}
else
{
	Count_Medidas4=0;
	Flag.Send_Medidas4=1;
}
/////////////////////////////////////////
if(Count_Gps<2)
{
	Count_Gps++;
}
else
{
	Count_Gps=0;
	if(!Flag.gps_command&&Flag.gps)
	{
		Flag.Send_GPS=1;
	}
}
}


