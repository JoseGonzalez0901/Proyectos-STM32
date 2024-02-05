/*
 * ESP_01.c
 *
 *  Created on: Jan 9, 2023
 *      Author: joser
 */
#include "ESP_01.h"
extern char buff[100];
extern UART_HandleTypeDef huart2;
extern int flag;
void FUNC_ESP_INIT(char*SSID,char *PASSWORD,char *Server,char *Cliente)
{
	for(;;)
	{

			  FUNC_WIFI_CONECTAR(SSID,PASSWORD);
			  FUNC_MQTT_CONECTAR_SERVIDOR(Server,Cliente);
		  if(Flag.Server_conect)
		  {
			  break;
		  }



	}
}

void FUNC_WIFI_CONECTAR(char *SSID,char *PASSWORD)
{
	char *mensaje;
	int   tam=0;
	if(!Flag.Wifi_conect)
	{
	tam=strlen(SSID)+strlen(PASSWORD)+strlen(Conectar_Wifi)+4;
	mensaje=calloc(tam,sizeof(char));
	sprintf(mensaje,"%s\"%s\",\"%s\"",Conectar_Wifi,SSID,PASSWORD);
	Transmit(mensaje,huart2);
	Transmit((char*)String_final,huart2);
	free(mensaje);
	}
}
void FUNC_WIFI_CONFIGURAR_MODO(char Conf_wifi)
{
//	char *mensaje;
//	int   tam=0;
//	//tam=strlen(SSID)+strlen(PASSWORD)+4;
//	mensaje=calloc(tam,sizeof(char));
//	sprintf(mensaje,"%s\"%s\",\"%s\"",Conectar_Wifi,SSID,PASSWORD);
//	Transmit(mensaje,huart1);
//	Transmit((char*)String_final,huart1);
//	free(mensaje);
//	do
//	{
//		HAL_UART_Receive_IT(&huart1,(uint8_t*)buff,strlen(buff)+1);
//	} while (!strstr(buff,"OK"));
}
/////////////////////////////////////////////////
void FUNC_MQTT_CONECTAR_SERVIDOR(char *BROKER, char *ID)
{
	char *mensaje;
	int   tam=0;
	if(Flag.Wifi_conect&&!Flag.Server_conect)
	{
		//AT+MQTTUSERCFG=0,1,"ESP32","espressif","1234567890",0,0,""
	tam=strlen(Configurar_ID)+strlen(Scheme)+strlen(ID)+strlen(comillas)+strlen(comillas)+10;
	mensaje=calloc(tam,sizeof(char));
	sprintf(mensaje,"%s%s,\"%s\",%s,%s,0,0,%s",Configurar_ID,Scheme,ID,comillas,comillas,comillas);
	Transmit(mensaje,huart2);
	Transmit((char*)String_final,huart2);
	HAL_Delay(100);
	free(mensaje);
	HAL_Delay(100);
	//AT+MQTTCONN=0,"192.168.31.113",1883,0
	tam=strlen(Conectar_broker)+strlen(BROKER)+strlen(Puerto)+4;
	mensaje=calloc(tam,sizeof(char));
	sprintf(mensaje,"%s\"%s\",%s,%s",Conectar_broker,BROKER,Puerto,Reconnect_server);
	Transmit(mensaje, huart2);
	Transmit((char*)String_final,huart2);
	free(mensaje);
	}
	if(Flag.Server_conect)
	{
		Flag.INIT=0;
	}
}
void FUNC_MQTT_PUBLICAR(char *Mensaje,char *Topico)
{
	char *mensaje;
	int   tam=0;
	if(Flag.Server_conect)
	{
		//AT+MQTTPUB=0,"topic","test",1,0
	tam=strlen(Publicar)+strlen(Topico)+strlen(Mensaje)+strlen(QOS)+strlen(Retain)+10;
	mensaje=calloc(tam,sizeof(char));
	sprintf(mensaje,"%s\"%s\",\"%s\",%s,%s",Publicar,Topico,Mensaje,QOS,Retain);
	Transmit(mensaje, huart2);
	Transmit((char*)String_final,huart2);
	free(mensaje);
	}
}
void FUNC_MQTT_SUBSCRIBIR(char *Topico)
{
	char *mensaje;
	int   tam=0;
	if(Flag.Server_conect)
	{
		//AT+MQTTSUB=0,"topic",1
	tam=strlen(Subscribirse)+strlen(Topico)+strlen(QOS)+10;
	mensaje=calloc(tam,sizeof(char));
	sprintf(mensaje,"%s\"%s\",%s",Subscribirse,Topico,QOS);
	Transmit(mensaje, huart2);
	Transmit((char*)String_final,huart2);
	free(mensaje);
	}
}
///////////////////////////////////////////////////
//void FUNC_ESP_INIT(char *SSID,char *PASSWORD,char *BROKER,char *ID,int Conf_wifi);
void FUNC_ESP_RESET(char *Reset_comand)
{
	Transmit(Reset_comand, huart2);
	Transmit((char*)String_final,huart2);
}
//void FUNC_ESP_ENVIO_LIBRE(char *Mensaje);
////////////////////////////////////////////////
void Transmit(char *Mensaje,UART_HandleTypeDef huart)
{
	//HAL_UART_Transmit_IT(&huart,(uint8_t*)Mensaje,strlen(Mensaje));
	HAL_UART_Transmit(&huart, (uint8_t*)Mensaje,strlen(Mensaje),HAL_MAX_DELAY);
}
void Enviar_Esp01(int mensaje)
{
  if(mensaje==B0ON)
  {
    FUNC_MQTT_PUBLICAR("B0ON","B0");
    HAL_GPIO_WritePin(Rele_1_GPIO_Port,Rele_1_Pin, ON);
    HAL_GPIO_WritePin(Toma1_GPIO_Port,Toma1_Pin,OFF);

  }
  else if(mensaje==B0OFF)
  {
    FUNC_MQTT_PUBLICAR("B0OFF","B0");
    HAL_GPIO_WritePin(Rele_1_GPIO_Port,Rele_1_Pin, OFF);
    HAL_GPIO_WritePin(Toma1_GPIO_Port,Toma1_Pin,ON);

  }
   else if(mensaje==B1ON)
  {
    FUNC_MQTT_PUBLICAR("B1ON","B1");
    HAL_GPIO_WritePin(Rele_2_GPIO_Port,Rele_2_Pin, ON);
    HAL_GPIO_WritePin(Toma2_GPIO_Port,Toma2_Pin,OFF);

  }
  else if(mensaje==B1OFF)
  {
    FUNC_MQTT_PUBLICAR("B1OFF","B1");
    HAL_GPIO_WritePin(Rele_2_GPIO_Port,Rele_2_Pin, OFF);
    HAL_GPIO_WritePin(Toma2_GPIO_Port,Toma2_Pin,ON);

  }
   else if(mensaje==B2ON)
  {
    FUNC_MQTT_PUBLICAR("B2ON","B2");
    HAL_GPIO_WritePin(Rele_3_GPIO_Port,Rele_3_Pin, ON);
  }
  else if(mensaje==B2OFF)
  {
    FUNC_MQTT_PUBLICAR("B2OFF","B2");
    HAL_GPIO_WritePin(Rele_3_GPIO_Port,Rele_3_Pin, OFF);
  }
  else  if(mensaje==B3ON)
  {
    FUNC_MQTT_PUBLICAR("B3ON","B3");
    HAL_GPIO_WritePin(Rele_4_GPIO_Port,Rele_4_Pin, ON);
  }
  else if(mensaje==B3OFF)
  {
    FUNC_MQTT_PUBLICAR("B3OFF","B3");
    HAL_GPIO_WritePin(Rele_4_GPIO_Port,Rele_4_Pin, ON);
  }
  Flag.Enviar=0;
}

void Recibir_Esp01(char *Mensaje)
{
	if(strstr(Mensaje,"GOT IP"))
	{
		Flag.Wifi_conect=1;
	}
	else if(strstr(Mensaje,"ready"))
	{
		Flag.INIT=1;
	}
	else if(strstr(Mensaje,"+MQTTCONNECTED:"))
	{
		Flag.Server_conect=1;
	    HAL_GPIO_WritePin(LED_Micro_GPIO_Port,LED_Micro_Pin,ON);

	}
	else if(strstr(Mensaje,"B0ON"))
		    {
			// HAL_GPIO_WritePin()
			 Flag.Enviar=B0ON;
		    }
		    else if(strstr(Mensaje,"B0OFF"))
		    {
			// HAL_GPIO_WritePin()
			 Flag.Enviar=B0OFF;
		    }
			else if(strstr(Mensaje,"B1ON"))
		    {
			// HAL_GPIO_WritePin()
			 Flag.Enviar=B1ON;
		    }
		    else if(strstr(Mensaje,"B1OFF"))
		    {
			 //HAL_GPIO_WritePin()
			 Flag.Enviar=B1OFF;
		    }
			else if(strstr(Mensaje,"B2ON"))
		    {
			 //HAL_GPIO_WritePin()
			 Flag.Enviar=B2ON;
		    }
		    else if(strstr(Mensaje,"B2OFF"))
		    {
			 //HAL_GPIO_WritePin()
			 Flag.Enviar=B2OFF;
		    }
			else if(strstr(Mensaje,"B3ON"))
		    {
			// HAL_GPIO_WritePin()
			 Flag.Enviar=B3ON;
		    }
		    else if(strstr(Mensaje,"B3OFF"))
		    {
			 //HAL_GPIO_WritePin()
			 Flag.Enviar=B3OFF;
		    }





}


