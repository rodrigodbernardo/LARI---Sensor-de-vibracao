#include <Wire.h>
#include <PubSubClient.h>
#include <ESP8266WiFiMulti.h>
#include <ArduinoJson.h>
//#include "WiFiTask.h"

#include "secure.hpp"

//  DEFINIÇÃO DO FIRMWARE

#define Version "1.0.0.0"
#define MakeFirmwareInfo(k, v) "&_FirmwareInfo&k=" k "&v=" v "&FirmwareInfo_&"

//  DEFINIÇÃO DOS PINOS DE COMUNICAÇÃO COM O SENSOR

#define MPU_SDA D6
#define MPU_SCL D5
#define MPU_POW D1

//  DEFINIÇÃO DOS ENDEREÇOS DE MEMÓRIA DO SENSOR UTILIZADOS

const uint8_t MPU_ADDR      = 0x68;
const uint8_t WHO_AM_I      = 0x75;
const uint8_t PWR_MGMT_1    = 0x6B;
const uint8_t GYRO_CONFIG   = 0x1B;           // Registrador que configura a escala do giroscópio.
const uint8_t ACCEL_CONFIG  = 0x1C;          // Registrador que configura a escala do acelerômetro.
const uint8_t ACCEL_XOUT    = 0x3B;            //
const uint8_t GYRO_SCALE    = 8;               // Escala do giroscópio
const uint8_t ACCEL_SCALE   = 8;              // Escala do acelerômetro

//  VARIÁVEIS DE TEMPORIZAÇÃO

unsigned long long Tp = 0; //prevCheckTime
unsigned long long Tc; //CurrTime
unsigned long long T = 0; //Interval
unsigned long long Ts = 5000;

//  VARIÁVEIS DE ESPECIFICAÇÃO

const int nc = 100;     //n_capt
int       np = 1;       //n_pack

//  VARIÁVEIS DO EXTRATOR DE CARACTERÍSTICAS

const int w_size = 11;
const int w_border = w_size / 2;

double rms_acc;
double rms_gyr;

//  VARIÁVEIS DO CLASSIFICADOR

const int n_classes = 3;

double threshold[n_classes] = {7255, 7666, 9672};
int result_class = 0;

//  VARIÁVEIS DE ARMAZENAMENTO

int16_t MPU_data[nc][6];
int16_t MPU_temp[nc];
double MSD[nc - (2 * w_border)][6];

//  VARIÁVEIS DIVERSAS

String names[7] = {"AcX:", ",AcY:", ",AcZ:", ",GyX:", ",GyY:", ",GyZ:", ",Tmp:"};
int cmd;
String status;
bool offlineMode = 0;

//////////////////  CALLBACK MQTT   //////////////////


void callback(char *topic, byte *payload, unsigned int length)
{
  StaticJsonDocument<256> mqtt_input;
  deserializeJson(mqtt_input, payload);

  cmd   = mqtt_input["cmd"];
  np    = mqtt_input["npk"];
  Ts    = mqtt_input["spe"];

  status  = "COMANDO RECEBIDO:\n---\nComando: "   ; status += cmd;
  status += "\nPacotes: "                         ; status += np;
  status += "\nCapturas: "                        ; status += nc;
  status += "\nPeríodo amostral: "                ; status += Ts; status += " us\n";

  Serial.println(status);

  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

//  Objetos do sistema

ESP8266WiFiMulti wifiMulti;

WiFiClient espClient;
PubSubClient mqtt(IO_SERVER, IO_SERVERPORT, callback, espClient);

void setup()
{
  pinMode(D1, OUTPUT);
  pinMode(D0, OUTPUT);
  pinMode(D4, OUTPUT);

  Serial.begin(500000);


  Serial.println("Sistema de monitoramento de condição - LARI IFCE");
  Serial.println("Desenvolvido por: Rodrigo D. B. de Araújo");
  Serial.print("Versão do firmware: ");
  Serial.println(Version);
  Serial.println("\n");

  setMPU();
  //setWifi();
  //setMqtt();
}

void loop()
{
  if (statusMPU()) {
    Serial.println("Conexão com o sensor não identificada.");
    setMPU();
  }

  if (!offlineMode) {
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("\nConexão com a internet não identificada.");
      setWifi();
    } else {
      mqtt.loop();

      if (mqtt.connected())
      {
        mqtt.loop();
      }else
      {
        Serial.println("\nConexão com o Broker MQTT não estabelecida.");
        setMqtt();
      }
    }
  }
  /*
    switch (cmd) {
      case 1:
        Serial.println("\n\n======================================");
        Serial.println("\n1 - ANÁLISE EM TEMPO REAL\n");
        Serial.println("======================================\n");
        break;

      case 2:
        Serial.println("\n\n======================================");
        Serial.println("\n2 - CLASSIFICAÇÃO LOCAL CONTÍNUA\n");
        Serial.println("======================================\n");
        break;
    }
  */
  cmd = 2;
  switch (cmd) {
    case 1:
    {
      readRawMPU(1);
      printMPU();
      break;
    }
    case 2:
    {
      readRawMPU(nc);
      featureExtraction();
      dataClassification();

      if (!offlineMode)
        sendClass();
      
      delay(1000);
      break;
    }
    case 3:
      break;
  }
}

///////////////// FUNCOES \\\\\\\\\\\\\\\\\

void setWifi()
{
  WiFi.mode(WIFI_STA);

  for (int i = 0; i < network_number; i++)
    wifiMulti.addAP(WLAN_SSID[i], WLAN_PASS[i]);

  Serial.print("Conectando à rede Wi-Fi...");

  wifiMulti.run();
  for (uint8_t count = 5; WiFi.status() != WL_CONNECTED && count > 0; count--) {
    Serial.print(".");

    digitalWrite(D4, LOW); delay(100);
    digitalWrite(D4, HIGH); delay(900);

  }
  if ( WiFi.status() != WL_CONNECTED) {
    Serial.println("Conexão não estabelecida. Iniciando modo de funcionamento offline.");
    offlineMode = 1;
  }
  else {
    Serial.println("\n\n======================================");
    Serial.println("Rede conectada!");
    Serial.print("SSID: "); Serial.println(WiFi.SSID());
    Serial.print("Endereço IP: "); Serial.println(WiFi.localIP());
    Serial.println("======================================\n");
  }
}

void setMPU()
{
  Serial.print("Conectando ao sensor...");

  do {
    Serial.print(".");

    Wire.endTransmission();
    Wire.begin(MPU_SDA, MPU_SCL);
    delay(500);

    writeMPU(PWR_MGMT_1, 0);             // ACORDA O SENSOR. ENVIAR 64 PARA DORMIR O SENSOR.
    writeMPU(GYRO_CONFIG, GYRO_SCALE);   // CONFIGURA A ESCALA DO GIROSCÓPIO - +-250 °/s -->
    writeMPU(ACCEL_CONFIG, ACCEL_SCALE); // CONFIGURA A ESCALA DO ACELERÔMETRO - +-4G

    digitalWrite(D0, LOW); delay(100);
    digitalWrite(D0, HIGH); delay(500);

  } while (statusMPU());

  Serial.println("\n\n======================================");
  Serial.println("Sensor conectado!");
  Serial.println("Modelo: MPU 6050");
  Serial.println("======================================\n");

}


void writeMPU(int reg, int val)
{
  //
  // COMUNICACAO I2C COM O SENSOR - NAO MODIFICAR
  //
  Wire.beginTransmission(MPU_ADDR); // inicia comunicação com endereço do MPU6050
  Wire.write(reg);                  // envia o registro com o qual se deseja trabalhar
  Wire.write(val);                  // escreve o valor no registro
  Wire.endTransmission();           // termina a transmissão
}

bool statusMPU()
{
  if (readRegMPU(WHO_AM_I) == 104)// 104 = respondeu OK
    return 0;

  return 1;
}

uint8_t readRegMPU(uint8_t reg)        // aceita um registro como parâmetro
{
  uint8_t data;
  Wire.beginTransmission(MPU_ADDR);     // inicia comunicação com endereço do MPU6050
  Wire.write(reg);                      // envia o registro com o qual se deseja trabalhar
  Wire.endTransmission(false);          // termina transmissão mas continua com I2C aberto (envia STOP e START)
  Wire.requestFrom(MPU_ADDR, 1);        // configura para receber 1 byte do registro escolhido acima
  data = Wire.read();                   // lê o byte e guarda em 'data'
  return data;                          //retorna 'data'
}

void readRawMPU(int iterations)
{

  Tp = micros();
  int capt_counter = 0;

  while (capt_counter < iterations)
  {
    Tc = micros();
    T = Tc - Tp;

    if (T >= Ts)
    {
      //Serial.println(capt_counter);
      Serial.println(T);
      //delay(Ts);
      Tp = Tc;
      yield();

      //Serial.printf("Captura %i: ", capt_counter);

      Wire.beginTransmission(MPU_ADDR);
      Wire.write(ACCEL_XOUT);
      Wire.endTransmission(false);
      Wire.requestFrom(MPU_ADDR, (uint8_t)(14));

      for (int i = 0; i < 3; i++) // LÊ OS DADOS DE ACC
        MPU_data[capt_counter][i] = Wire.read() << 8 | Wire.read();

      MPU_temp[capt_counter] = Wire.read() << 8 | Wire.read(); //LÊ OS DADOS DE TEMP

      for (int i = 3; i < 6; i++) // LÊ OS DADOS DE GYR
        MPU_data[capt_counter][i] = Wire.read() << 8 | Wire.read();

      capt_counter++;
    }
  }
}

void printMPU()
{
  for (int i = 0; i < 6; i++) // LÊ OS DADOS DE ACC
  {
    Serial.print(names[i]);
    Serial.print(MPU_data[0][i]);
  }
  //Serial.print(names[6]);
  //Serial.print(MPU_temp[0]);
  Serial.println();
}

//////////////////////////////////////
//////////////////////////////////////  FUNÇÕES RELACIONADAS À COMUNICAÇÃO MQTT
//////////////////////////////////////

void setMqtt()
{
  Serial.print("Conectando ao broker MQTT...");
  String clientId = "ESP8266Client-";
  clientId += String(random(0xffff), HEX);

  for (uint8_t count = 5; !mqtt.connected() && count > 0; count--) {
    Serial.print(".");
    mqtt.connect(clientId.c_str(), "cliente", "cliente");

    digitalWrite(D4, LOW); delay(100);
    digitalWrite(D4, HIGH);
  }
  if (!mqtt.connected()) {
    Serial.println("Conexão não estabelecida. Iniciando modo de funcionamento offline.");
    offlineMode = 1;
  } else {
    Serial.println("\n\n======================================");
    Serial.println("Broker MQTT conectado!");
    Serial.println("Endereço do Broker: ??");
    Serial.println("======================================\n");

    //mqtt.subscribe(topicSCADA_ESP);
  }
}

void sendData()
{
  String out_msg = "";
  mqtt.publish(fastCaptureDataTopic, "AcX,AcY,AcZ,GyX,GyY,GyZ,Tmp");

  for (int i = 0; i < nc; i++) {
    out_msg = "";

    for (int j = 0; j < 6; j++)
    {
      //out_msg += names[j];
      out_msg += MPU_data[i][j];
      out_msg += ",";
    }
    out_msg += MPU_temp[i];
    mqtt.publish(fastCaptureDataTopic, out_msg.c_str());
    Serial.println(out_msg);
  }
  mqtt.publish(fastCaptureDataTopic, "fim");
}

void sendClass()
{
  String acc_res = "RMS ACC: ", gyr_res = "RMS GYR: ", class_res = "CLASS: ";

  acc_res += rms_acc;
  gyr_res += rms_gyr;
  class_res += result_class;

  mqtt.publish(accResultTopic, acc_res.c_str());
  mqtt.publish(gyrResultTopic, gyr_res.c_str());
  mqtt.publish(classResultTopic, class_res.c_str());
}

//////////////////////////////////////
//////////////////////////////////////  CLASSIFICAÇÃO DE DADOS
//////////////////////////////////////

void featureExtraction()
{
  for (int axis = 0; axis < 6; axis++)
  {

    // Calcula o MSD

    for (int centralElement = w_border; centralElement < nc - w_border; centralElement++)
    {
      // 1. Media
      double msd_mean = 0;
      for (int i = (centralElement - w_border); i <= centralElement + w_border; i++)
      {
        msd_mean += (MPU_data[i][axis]);
      }
      msd_mean /= w_size;
      //Serial.printf("\nMedia = %f\n", msd_mean);

      // 2. Variancia

      double msd_variance = 0;

      for (int i = (centralElement - w_border); i <= centralElement + w_border; i++)
      {
        msd_variance += pow((MPU_data[i][axis] - msd_mean), 2);
      }

      msd_variance /= w_size ;//- 1;
      MSD[centralElement - w_border][axis] = sqrt(msd_variance);
      //Serial.printf("\nDesvio padrao da janela = %f\n",MSD[centralElement - w_border][axis]);
    }
  }

  double media[6] = {0, 0, 0, 0, 0, 0};

  for (int position = 0; position < (nc - (2 * w_border)); position++)
  {
    for (int axis = 0; axis < 6; axis++)
      media[axis] += (MSD[position][axis]);
  }

  for (int axis = 0; axis < 6; axis++) {
    media[axis] /= (nc - (2 * w_border));
    //Serial.printf("\nMedia do sensor %i = %f\n", axis, media[axis]);
  }

  rms_acc = 0;
  rms_gyr = 0;

  for (int axis = 0; axis < 3; axis++)
  {
    rms_acc += pow(media[axis  ], 2);
    rms_gyr += pow(media[axis + 4], 2);
  }

  rms_acc = sqrt(rms_acc / 3);
  rms_gyr = sqrt(rms_gyr / 3);

  //rms -= 500;
  Serial.printf("RMS_ACC: %f,", rms_acc);
  Serial.printf("RMS_GYR: %f,", rms_gyr);
}

void dataClassification()
{
  result_class = 0;

  for (int i = 0; i < (n_classes - 1); i++)
  {
    if (rms_acc >= threshold[i])
    {
      result_class = i + 1;
    }
  }

  Serial.printf("CLASS: %i\n", result_class);
}
