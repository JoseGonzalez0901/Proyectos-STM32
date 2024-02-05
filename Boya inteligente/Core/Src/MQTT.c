
#include "MQTT.h"
extern char Conf_broker_add[];
extern char Conf_port[];
extern char keepalive_time[];
extern char clean_session[];
extern char *client_name;
extern unsigned char Conf_SSL;
extern bandera Flag;
char Gps_Info[150];
#define time 10


unsigned int FUNC_RESET()
{
	Flag.MQTT_Start=0;
	Flag.Radio_on=0;
	Flag.Sbscripcion_terminada=0;
	Flag.Send_GPS=0;
	Flag.Send_Medidas=0;
	Flag.Send_Medidas2=0;
	Flag.Send_Medidas3=0;
	Flag.Send_Medidas4=0;
	Flag.Subscripcion=0;
	Flag.Subscripcion_sucess=0;
	Flag.gps=0;
	Flag.gps_command=0;
	Flag.mqtt_command=0;
	Flag.servidor=0;
	Transmit(Reset);
	return 0;
}
int Decoder(char *string)
{
	if(strstr(string,"PB DONE"))//Tarjeta iniciada y chip correcto
	{
		Flag.Radio_on=1;//Indicacion que el radio esta encendido correctamente
		return 0;
	}

	if(strstr(string,"+CMQTTCONNECT: 0,0"))//conectado al servidor
	{
		Flag.servidor=1;//Indicador de que el radio esta conectado al servidor
		Flag.mqtt_command=0;//Liberando el uso del MQTT
		return 0;
	}

	if(strstr(string,"+CMQTTSTART: 0"))//Mensaje que indica si el MQTT esta encendido
	{
		Flag.MQTT_Start=1;//Indicacion de que el MQTT esta encendido
		Flag.mqtt_command=0;//Liberando el uso del MQTT
		return 0;
	}

	if(strstr(string,"+CMQTTSUB: 0,0"))//subscripcion exitosa
	{
	Flag.Subscripcion_sucess=1;//Indicacion de que se suscribio de manera exitosa
	Flag.mqtt_command=0;//Liberando el uso del MQTT
	return 0;
	}
   if (strstr (string, "+CMQTTPUB: 0,0")) //Publicacion exitosa
	{

	  Flag.mqtt_command = 0;
	  return 0;
	}
   if (strstr (string, "ERROR")) //Publicacion exitosa
	{

	  Flag.Reset_Tarjeta=1;
	  return 0;
	}
   if (strstr (string, "+CMQTTPUB: 0,11")) //Publicacion exitosa
	{

	  Flag.Reset_Tarjeta=1;
	  return 0;
	}
   if (strstr (string, "+CMQTTPUB=0,1,60")) //Publicacion exitosa
	{

	   Flag.mqtt_command = 0;
	  return 0;
	}

	if(strstr(string,"+CGNSSPWR: READY!"))//Mensaje que indica el encendido del GPS
	{
		Flag.gps=1;//Indicacion de que el GPS esta listo para usarse
		return 0;
	}
	if(strstr(string,"+CGNSSINFO:"))//respuesta del gps
	{
	strcpy(Gps_Info,string);

	Flag.gps_command=0;
	return 0;
	}
	return 0;
}

unsigned int FUNC_SEND_GPS()
{
	FUNC_SEND_MQTT(Gps_Info,"mapps34");
	return 0;
}

unsigned int FUNC_MQTT_CONNECT_SERVER(char *cliente)
{
	int tam=0;
	char *x="\0";



			tam=strlen(cliente)+strlen(Nombre_del_cliente)+strlen(String_final);
			x=calloc(tam,sizeof(char));
			sprintf(x,"%s\"%s\",%c%s",Nombre_del_cliente,cliente,Conf_SSL,String_final);
			Transmit(x);
			free(x);
			HAL_Delay(time);
			tam=strlen(Conectar_al_Broker)+strlen(Conf_broker_add)+strlen(Conf_port)+strlen(keepalive_time)+strlen(clean_session)+strlen(String_final);
			x=calloc(tam,sizeof(char));
			sprintf(x,"%s\"%s:%s\"%s%s%s",Conectar_al_Broker,Conf_broker_add,Conf_port,keepalive_time,clean_session,String_final);
			Transmit(x);
			free(x);

	return 0;
}
unsigned int FUNC_SEND_MQTT(char *mensaje,char *topico)
{
		int tam=0;
		char *x="\0";
		//Enviando tama;o del topico
		tam=strlen(AT_Topico)+strlen(String_final)+1+strlen(topico);
		x=calloc(tam,sizeof(char));
		sprintf(x,"%s%d%s",AT_Topico,strlen(topico),String_final);
		Transmit(x);
		free(x);
		HAL_Delay(700);

		//Enviando Nombre del topico
		tam=strlen(topico)+strlen(String_final);
		x=calloc(tam,sizeof(char));
		sprintf(x,"%s%s",topico,String_final);
		Transmit(x);
		free(x);
		HAL_Delay(700);

		//cargando mensaje
		tam=strlen(Cargar_mensaje)+strlen(String_final)+1+strlen(mensaje);
		x=calloc(tam,sizeof(char));
		sprintf(x,"%s%d%s",Cargar_mensaje,strlen(mensaje),String_final);
		Transmit(x);
		free(x);
		HAL_Delay(700);

		//Enviando mensaje
		tam=strlen(mensaje)+strlen(String_final);
		x=calloc(tam,sizeof(char));
		sprintf(x,"%s%s",mensaje,String_final);
		Transmit(x);
		free(x);
		HAL_Delay(700);
		//Publicar
		Transmit(Publicar);
		HAL_Delay(700);


return 0;
}
unsigned int FUNC_Subscribirse_MQTT(char *topico)
{

	int tam=0;
	char *x="\0";
	size_t tmp = 0;
	//Enviando comando At para la subscripcion
	tam= strlen(AT_Subscribirse)+strlen(String_final)+strlen(topico)+2;
	x=calloc(tam,sizeof(char));
	tmp = strlen(topico);
	sprintf(x,"%s%d,1%s",AT_Subscribirse,(unsigned int)tmp,String_final);
	Transmit(x);
	free(x);
	HAL_Delay(250);
	//Enviando topico
	tam=strlen(topico)+strlen(String_final);
	x=calloc(tam,sizeof(char));
	sprintf(x,"%s%s",topico,String_final);
	Transmit(x);
	free(x);
	HAL_Delay(250);
	//Comando para Subscribirse
	Transmit(Subscribirse);
	HAL_Delay(250);



return 0;
}


