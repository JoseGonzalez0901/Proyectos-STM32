/*
 * ESP_01.c
 *
 *  Created on: Jan 9, 2023
 *      Author: joser
 */
#include "ESP_01.h"
extern char buff[100];
extern UART_HandleTypeDef huart1;
TypedefFlag Flag;

void FUNC_WIFI_CONECTAR(char *SSID,char *PASSWORD)
{
	char *mensaje;
	int   tam=0;
	if(!Flag.Wifi_conect)
	{
	tam=strlen(SSID)+strlen(PASSWORD)+strlen(Conectar_Wifi)+4;
	mensaje=calloc(tam,sizeof(char));
	sprintf(mensaje,"%s\"%s\",\"%s\"",Conectar_Wifi,SSID,PASSWORD);
	Transmit(mensaje,huart1);
	Transmit((char*)String_final,huart1);
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
	if(Flag.Wifi_conect)
	{
		//AT+MQTTUSERCFG=0,1,"ESP32","espressif","1234567890",0,0,""
	tam=strlen(Configurar_ID)+strlen(Scheme)+strlen(ID)+strlen(comillas)+strlen(comillas)+10;
	mensaje=calloc(tam,sizeof(char));
	sprintf(mensaje,"%s%s,\"%s\",%s,%s,0,0,%s",Configurar_ID,Scheme,ID,comillas,comillas,comillas);
	Transmit(mensaje,huart1);
	Transmit((char*)String_final,huart1);
	HAL_Delay(100);
	free(mensaje);
	HAL_Delay(100);
	//AT+MQTTCONN=0,"192.168.31.113",1883,0
	tam=strlen(Conectar_broker)+strlen(BROKER)+strlen(Puerto)+4;
	mensaje=calloc(tam,sizeof(char));
	sprintf(mensaje,"%s\"%s\",%s,%s",Conectar_broker,BROKER,Puerto,Reconnect_server);
	Transmit(mensaje, huart1);
	Transmit((char*)String_final,huart1);
	free(mensaje);
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
	Transmit(mensaje, huart1);
	Transmit((char*)String_final,huart1);
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
	Transmit(mensaje, huart1);
	Transmit((char*)String_final,huart1);
	free(mensaje);
	}
}
///////////////////////////////////////////////////
//void FUNC_ESP_INIT(char *SSID,char *PASSWORD,char *BROKER,char *ID,int Conf_wifi);
void FUNC_ESP_RESET(char *Reset_comand)
{
	Transmit(Reset_comand, huart1);
	Transmit((char*)String_final,huart1);
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
  }
  else if(mensaje==B0OFF)
  {
    FUNC_MQTT_PUBLICAR("B0OFF","B0");
  }
   else if(mensaje==B1ON)
  {
    FUNC_MQTT_PUBLICAR("B1ON","B1");
  }
  else if(mensaje==B1OFF)
  {
    FUNC_MQTT_PUBLICAR("B1OFF","B1");
  }
   else if(mensaje==B2ON)
  {
    FUNC_MQTT_PUBLICAR("B2ON","B2");
  }
  else if(mensaje==B2OFF)
  {
    FUNC_MQTT_PUBLICAR("B2OFF","B2");
  }
  else  if(mensaje==B3ON)
  {
    FUNC_MQTT_PUBLICAR("B3ON","B3");
  }
  else if(mensaje==B3OFF)
  {
    FUNC_MQTT_PUBLICAR("B3ON","B3");
  }
}
void Recibir(char *Mensaje)
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
	}
	else if(strstr(Mensaje,"+MQTTSUBRECV:"))
	{
	Flag.OK=!Flag.OK;
	HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port,LED_GREEN_Pin);
	}






}


