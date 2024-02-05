#ifndef INC_MQTT_H_
#define INC_MQTT_H_

#include "main.h"
#include "stdlib.h"
#include "stdio.h"
#include "string.h"
#include "Comandos_AT.h"
#include "UART.h"
///////////////MQTT/////////////////
#define Iniciar_MQTT "AT+CMQTTSTART\r\n"
#define Parar_MQTT "AT+CMQTTSTOP\r\n"
#define Nombre_del_cliente "AT+CMQTTACCQ=0,"
#define Conectar_al_Broker "AT+CMQTTCONNECT=0,"
#define AT_Topico "AT+CMQTTTOPIC=0,"
#define Cargar_mensaje "AT+CMQTTPAYLOAD=0,"
#define Publicar "AT+CMQTTPUB=0,2,60\r\n"
#define AT_Subscribirse "AT+CMQTTSUB=0,"
#define Subscribirse "AT+CMQTTSUB=0\r\n"
#define Quitar_Suscripcion "AT+CMQTTUNSUB=0,"
#define Desconexion_del_Broker "AT+CMQTTDISC=0,120\r\n"
#define Liberacion_del_Cliente "AT+CMQTTREL=0\r\n"


typedef struct LAGS
{
	unsigned int  Subscripcion:1;
	unsigned int  servidor:1;
	unsigned int  MQTT_Start:1;
	unsigned int  gps:1;
	unsigned int  Radio_on:1;
	unsigned int  Subscripcion_sucess:1;
	unsigned int  Sbscripcion_terminada:1;
	unsigned int  gps_command:1;
	unsigned int  mqtt_command:1;
	unsigned int  Send_GPS:1;
	unsigned int  Send_Medidas:1;
	unsigned int  Send_Medidas2:1;
	unsigned int  Send_Medidas3:1;
	unsigned int  Send_Medidas4:1;
	unsigned int  Reset_Tarjeta:1;
}bandera;

unsigned int FUNC_MQTT_CONNECT_SERVER(char *cliente);
unsigned int FUNC_SEND_MQTT(char *mensaje,char *topico);
unsigned int FUNC_Subscribirse_MQTT(char *topico);
int Decoder(char *string);
unsigned int FUNC_SEND_GPS();
unsigned int FUNC_RESET();

#endif
