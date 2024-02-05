/*
 * Comandos_AT.c
 *
 *  Created on: 11 oct. 2022
 *      Author: Informatica
 */
#include <Comandos_AT.h>
///MQTT configuration//////
char Conf_broker_add[]="tcp://mqtt-dashboard.com";//"tcp://broker.hivemq.com";
char Conf_port[]="1883";
char keepalive_time[]=",60";
char clean_session[]=",1";
char username[] = "";
char password[] = "";
char topic[] = "pot/adc/1";
char *client_name;
unsigned char Conf_SSL='0';
/////APN configuration//////
char APN[]="\"internet.ideasclaro.com.do\"";
/////SMS/////
char number[12]="\0";
char *message;
char control_Z=26;


