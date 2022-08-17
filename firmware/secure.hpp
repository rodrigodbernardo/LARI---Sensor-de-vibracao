/*
------ CHAVES UTILIZADAS NOS FIRMWARES ------

IMPORTANTE: ESTE ARQUIVO NÃO DEVE SER ENVIADO AO GITHUB E DEVE PERMANECER 
ATUALIZADO NO GOOGLE DRIVE DO PROJETO OU OUTRO MEIO NÃO PUBLICO


UTILIZAÇÃO:

1. FAÇA O DOWNLOAD DESTE ARQUIVO PARA SUA MÁQUINA E MOVA PARA A PASTA A SER UTILIZADA
2. COMENTE TODAS AS VARIAVEIS E CONSTANTES QUE NAO SERAO UTILIZADAS EM SEU PROGRAMA

ESTES PASSOS DEVEM SER REPETIDOS PARA CADA ARQUIVO DE CHAVE BAIXADO, SEM EXCLUIR OS ANTERIORES

CERTIFIQUE-SE DE QUE O ARQUIVO PRINCIPAL ESTÁ ATUALIZADO NO GOOGLE DRIVE. É PREFERIVEL MANTER APENAS
UM ARQUIVO DE CHAVES NO GOOGLE DRIVE, PARA EVITAR CONFUSÃO

*/

//DATA:20/10/2021 10:30 


//
// Chaves relativas à conexão Wi-Fi
//

//char * ssid[]      = {"Mi 9"};
//char * password[]  = {"12345678ab"};

int network_number = 2;
char* WLAN_SSID[] = {"Mi 9", "FLAVIO 02"};
char* WLAN_PASS[] = {"12345678ab", "8861854611"};

//
// Chaves relativas à conexão MQTT
//

#define IO_SERVER      "192.168.207.168"
#define IO_SERVERPORT  1883
#define IO_USERNAME    "cliente"
#define IO_KEY         "cliente"

const char* fastCaptureDataTopic  = "/feeds/captura_rapida/dados";
const char* accResultTopic        = "/feeds/classificacao/res_acc";
const char* gyrResultTopic        = "/feeds/classificacao/res_gyr";
const char* classResultTopic      = "/feeds/classificacao/res_class";

String mqttuser = "cliente";
String mqttpass = "cliente";
