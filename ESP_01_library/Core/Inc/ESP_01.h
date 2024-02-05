/*
 * ESP_01.h
 *
 *  Created on: Jan 9, 2023
 *      Author: joser
 */

#ifndef INC_ESP_01_H_
#define INC_ESP_01_H_

#include"stdio.h"
#include"string.h"
#include"stdlib.h"
#include"main.h"

#define String_final "\r\n"
#define Reset 		 "AT+RST"
///////////////////////////////////////////
#define Conectar_Wifi "AT+CWJAP="
#define Configuracion_Wifi "AT+CWMODE="
#define	station_mode '1'
#define	soft_mode '2'
#define soft_staion_y_station_mode '3'
/////////////////////////////////////////
//AT+MQTTUSERCFG=0,1,"ESP32","espressif","1234567890",0,0,""
//AT+MQTTCONN=0,"192.168.31.113",1883,0
//AT+MQTTSUB=0,"topic",1
//AT+MQTTPUB=0,"topic","test",1,0
#define Configurar_ID "AT+MQTTUSERCFG=0,"
#define Conectar_broker "AT+MQTTCONN=0,"
#define Subscribirse "AT+MQTTSUB=0,"
#define Publicar "AT+MQTTPUB=0,"
#define Retain "0"
#define Puerto "1883"
#define QOS "2"
#define Scheme "1"
#define comillas "\"\""
#define Reconnect_server "1"
#define User_name "AT+MQTTLONGUSERNAME=0,"
#define B0ON 0
#define B0OFF 1
#define B1ON 2
#define B1OFF 3
#define B2ON 4
#define B2OFF 5
#define B3ON  6
#define B3OFF 7
////////////////////////////////////////////


typedef struct flags
{
	unsigned int Wifi_conect:1;
	unsigned int Server_conect:1;
	unsigned int OK:1;
	unsigned int ERROR:1;
	unsigned int Busy:1;
	unsigned int Subs_succes:1;
	unsigned int INIT:1;
	unsigned int Enviar;
}TypedefFlag;


void FUNC_WIFI_CONECTAR(char *SSID,char *PASSWORD);
void FUNC_WIFI_CONFIGURAR_MODO(char Conf_wifi);
///////////////////////////////////////////////
void FUNC_MQTT_CONECTAR_SERVIDOR(char *BROKER, char *ID);
void FUNC_MQTT_PUBLICAR(char *Mensaje,char *Topico);
void FUNC_MQTT_SUBSCRIBIR(char *Topico);
/////////////////////////////////////////////////
void FUNC_ESP_INIT(char *SSID,char *PASSWORD,char *BROKER,char *ID,int Conf_wifi);
void FUNC_ESP_RESET(char *Reset_comand);
void FUNC_ESP_ENVIO_LIBRE(char *Mensaje);
////////////////////////////////////////////////
void Transmit(char *Mensaje,UART_HandleTypeDef huart);
void Recibir(char *Mensaje);










#endif /* INC_ESP_01_H_ */
