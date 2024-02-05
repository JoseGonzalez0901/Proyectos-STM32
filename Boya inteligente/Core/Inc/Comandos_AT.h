#ifndef COMANDOS_AT_H_
#define COMANDOS_AT_H_
#include "MQTT.h"
//#include "Funciones_necesarias.h"
/////////////Tarjeta///////////////////
#define CPIN "AT+CPIN?\n\r"//Confirmacion de estado de la tarjeta
#define CGDCONT_Question "AT+CGDCONT?\r\n"//APN
#define CGDCONT_Value "AT+CGDCONT=1,\"IP\","//Colocar APN
#define Revision_de_Red "AT+CREG?\r\n"//Comprobacion de conexion a red
#define Revision_de_Datos "AT+CGREG?\r\n"//Comprobacion de conexion a la red de datos
#define Calidad_de_Red "at+csq\r\n"//Calidad de red
#define Reset "at+creset\r\n"//Resetear la tarjeta
#define AT_GPS_COLD "AT+CGPSCOLD\r\n"
#define AT_GPS_HOT "AT+CGPSHOT\r\n"
#define AT_GPS_INFO "AT+CGNSSINFO\r\n"
#define AT_GPS_ON "AT+CGNSSPWR=1,1\r\n"
#define AT_ID "AT+CGSN"
#define OK "OK"
#define Error "ERROR"
#define Sin_conexion "ERROR 500"
#define True 1
#define False 0
#define String_final "\r\n"


#endif
