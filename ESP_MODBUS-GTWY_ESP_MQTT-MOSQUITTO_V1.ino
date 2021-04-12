/*
 * Processo de controle e envio do dados de controle para o MQTT server
 */
#include <WiFi.h> // Instancia o objeto WiFiClient que será pametro para instancia do objeto da "ESPclientMQTT"
#include <PubSubClient.h> // Instancia o objeto que fara a comunicação MQTT com o server MQTT 
#include <PID_v1.h>
#include <analogWrite.h>
#include <MODBUS_ESP32.h> // Instancia os objetos para a requisição dos valores de PV ao Gateway via modbus

// Defines do MQTT
// Tópico MQTT para recepção do valor do Setpoint para controle do broker MQTT para ESP32
#define TOPICO_ESPSUBSCRIBE "Setpoint_changes"
// Tópico MQTT para envio do dados do controle, tópico esse que o Cliente no Node-Red estará inscrito
#define TOPICO_ESPPUBLISH "dados_controle"
// id mqtt (para identificação da sessão)
#define ID_MQTT "ESPClient"
// URL do broker MQTT que deseja utilizar
const char* BROKER_MQTT = "10.13.103.28"; 
const char* ssid_mosquitto = "clientesp";
const char* password_mosquitto = "2040ab";
// Porta do Broker MQTT
int BROKER_PORT = 1883;

// Configuração WiFi
#ifndef STASSID
#define STASSID "LampLABA"          //login rede do Lamp
#define STAPSK  "udtqcSSONDodtqq"   //senha rede do Lamp
#endif
const char* ssid_redeLamp = STASSID;
const char* password_redeLamp = STAPSK;
IPAddress ip(10, 13, 103, 52);
IPAddress gateway(10, 13, 96, 1);
IPAddress subnet(255, 255, 224, 0);
char* hostGW = "10.13.103.42";
uint16_t netID = 100;

// Objetos instaciados
WiFiClient canalWiFiMQTT;
WiFiClient canalMODBUS_GW;
PubSubClient ESPclientMQTT(canalWiFiMQTT);
ModBus_ESP32 MODBUS_GW(hostGW,&canalMODBUS_GW);

// Variáveis de controle
#define  BOMBAPIN 27
int modoPID = 0;                 // 0 - automático (PID), 1 - manual
int sampleTime = 100;
double PV_LDS01; // nivel lido pelo sensor LDS01
double PV_LDS02; // nivel lido pelo sensor LDS02
double SP = 80;
double MV; // tensaoPID
int tensaoPWM;
double KP = 1.6, KI = 0.15, KD = 0;
PID pidTank(&PV_LDS01,&MV,&SP,KP,KI,KD,DIRECT);

// variáveis utilizadas como parametro da requisição via modbus dos valores de PV dos dois sensores
const unsigned int adrrs_02 = 34;
const unsigned int adrrs_01 = 13;
const unsigned int len = 2;
const byte ID = 1;
int aux = 1;
int res1 = 1;
int res2 = 1;

// Variáveis utilizadas na Interrupção
volatile int contInterrup = 0; // utilizada pelo ISR para indicar a interrupção
int totalInterruptCounter;
hw_timer_t * timer = NULL; // define um ponteiro para uma estrutura do tipo hw_timer_t que o retorno da função "timerbegin()" usada para inicar o cronometro
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED; // usada nos processos críticos que envolvel "contInterrup"

// Função Interrupção
void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  contInterrup++;
  portEXIT_CRITICAL_ISR(&timerMux);
}

// Funções auxiliares
union Union2IntFloat{         // Função para conversão 2 Int para 1 Float
  short inteiros[2];
  char charArray[2];
  float finalFloat;
};
Union2IntFloat x1;
Union2IntFloat x2;
void mqttconnect() {
  /* Loop until reconnected */
  while (!ESPclientMQTT.connected()) {
    Serial.print("MQTT connecting ...");
    /* client ID */
    String clientId = "ESP32Client";
    /* connect now */
    if (ESPclientMQTT.connect(clientId.c_str(),ssid_mosquitto,password_mosquitto)) {
      Serial.println("connected");
      /* subscribe topic with default QoS 0*/
      ESPclientMQTT.subscribe(TOPICO_ESPSUBSCRIBE);
    } else {
      Serial.print("failed, status code =");
      Serial.print(ESPclientMQTT.state());
      Serial.println("try again in 5 seconds");
      /* Wait 5 seconds before retrying */
      delay(5000);
    }
  }
}
void mqtt_callback(char* topic, byte* payload, unsigned int length) 
{
  Serial.print("Message received: ");
  Serial.println(topic);

  Serial.print("payload: ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
  /* Caso haja alteração do valor do setpoint*/
  if(payload[0] != 80 && payload[0] > 0 && payload[0] < 300){
    SP = payload[0]; // Atribui o valor so SP para o valor desejado
  }
}
void verifica_conexoes_wifi_mqtt(void)
{ 
  /* se não há conexão com o Broker, a conexão é refeita */
  if (!ESPclientMQTT.connected()) 
      mqttconnect(); 
}
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  
  // Configuração com a rede WiFi
  WiFi.config(ip, gateway, subnet);
  WiFi.begin(ssid_redeLamp, password_redeLamp);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");  
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  // Inicializa o cliente MQTT
  ESPclientMQTT.setServer(BROKER_MQTT,BROKER_PORT);
  ESPclientMQTT.setCallback(mqtt_callback);
  // Conexão Modbus + GW
  do{
    aux = MODBUS_GW.AbrirCanal();
  }while(aux);

  // Configuração do controlador
  pinMode(BOMBAPIN, OUTPUT);
  pidTank.SetTunings(KP,KI,KD);
  pidTank.SetMode(AUTOMATIC);
  pidTank.SetSampleTime(sampleTime);

  // Inicializa variáveis da Interrupcao Timer
  timer = timerBegin(0, 80, true); // inicializar nosso cronometro - 80 é usado para indicar que o nosso cronometro eh incrementado 1,000,000 de vezes por seg
  timerAttachInterrupt(timer, &onTimer, true); // associa nosso cronometro a interrupção - true para indicar que a interrupção será do tipo borda
  timerAlarmWrite(timer,1000000, true); // define qndo o cronometro irá gerar um alarme para ser indicado uma interrupção  - 3000000 para cada seg - e true para resetar
  timerAlarmEnable(timer); // habilia nosso cronometro

}

void loop() {
  // put your main code here, to run repeatedly:
  if(contInterrup > 0){
    verifica_conexoes_wifi_mqtt();
    res1 = MODBUS_GW.ModBus_Request04(adrrs_01,len,ID,x1.inteiros);
    if(res1==0){
       PV_LDS01 = (double)(x1.finalFloat);  // valor retornado pelo GW
       
       ESPclientMQTT.publish(TOPICO_ESPPUBLISH,"ta funfando");
       Serial.println("------------------ PV 01 --------------: ");
       Serial.println(PV_LDS01);
       Serial.println("PV como um array de char");
       Serial.println(x1.charArray[0]);
       Serial.println(x1.charArray[1]);
       pidTank.Compute();
    
       Serial.println("------------------ MV --------------: ");
       Serial.println(MV);
        
       tensaoPWM = (int)MV;
       tensaoPWM = (int)map(tensaoPWM, 0, 255, 120, 254);
       analogWrite(BOMBAPIN, tensaoPWM); 
    
       Serial.println("------------------ SETPOINT --------------: ");
       Serial.println(SP);
    
       Serial.println("------------------ TENSÃO_PWM --------------: ");
       Serial.println(tensaoPWM);
       ESPclientMQTT.loop();
     }
  }
  portENTER_CRITICAL(&timerMux);
  contInterrup--; // a interrupção detectada foi tratada
  portEXIT_CRITICAL(&timerMux);
  totalInterruptCounter++;
  Serial.print("An interrupt as occurred. Total number: ");
  Serial.println(totalInterruptCounter);
}
