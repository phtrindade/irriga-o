/*-----------------------------------------------
*   Include de bibliotecas
-----------------------------------------------*/
#include <PubSubClient.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include "DHT.h"

/*-----------------------------------------------
*   Definição de GPIO
-----------------------------------------------*/
#define PIN_SENSOR_UMIDADE_1 34   //Analogico sensor de umidade 1
#define PIN_SENSOR_UMIDADE_2 39   //Analogico sensor de umidade 2
#define PIN_SENSOR_TEMP 23        //Digital port sensor de temperatura
#define PIN_MOTOR_1 25 
#define PIN_MOTOR_2 27
#define PIN_SENSOR_NIVEL 16    
#define PIN_LED_RESERVATORIO 13

/*-----------------------------------------------
* Constantes
-----------------------------------------------*/
#define TEMPO_MOTOR 3000

/*-----------------------------------------------
* Configuração DHT11
-----------------------------------------------*/
//Uncomment whatever type you're using!
#define DHTTYPE DHT11   // DHT 11 
//#define DHTTYPE DHT22   // DHT 22  (AM2302)
//#define DHTTYPE DHT21   // DHT 21 (AM2301)
// Initialize DHT sensor for normal 16mhz Arduino
DHT dht(PIN_SENSOR_TEMP, DHTTYPE);

/*-----------------------------------------------
* Informação para Wi-Fi e MQTT
-----------------------------------------------*/
// Configuração Wifi
const char* ssid = "Zheuss";//"REDEL-F71D-2";
const char* password = "99168178";//"S4F4A";
// Configuração Mqtt
const char* mqttServer = "tailor.cloudmqtt.com";
const int mqttPort = 15687;
const char* mqttUser = "gazejars";
const char* mqttPassword = "S0TRHi9kIggJ";
// Clientes Wi-Fi e MQTT
WiFiClient espClient;   
PubSubClient client(espClient);

/*-----------------------------------------------
* Timer
-----------------------------------------------*/
hw_timer_t * timer = NULL;

/*-----------------------------------------------
* Funções utilizadas no sistema
-----------------------------------------------*/
void conecta_wifi();
void conecta_mqtt();
void atualiza_sensores();
void alerta_nivel_baixo_reservatorio();
int convert_to_Percent();
void envia_dados();
void IRAM_ATTR isr();
void start_gpio();
void call_back(char* topic, byte* payload, unsigned int length);
void cb_timer();
void start_timer();
void liga_motor();

/*-----------------------------------------------
* Váriaveis utilizadas
-----------------------------------------------*/
// Sensor de Umidade
int sensor_umidade1=0; 
int sensor_umidade2=0;
int umidade1 = 0;
int umidade2 = 0;

// Sensor de Temepratura
float sensor_temp=0.0;

// Sensor de Nivel
int sensor_nivel = 0;

// Controle de envio de mensagem
boolean sinalizacao = false;
boolean envia = false;

// Variaveis de medição de tempo
unsigned long _startAtualiza;
unsigned long _endAtualiza;
unsigned long _startMotor1;
unsigned long _endMotor1;
unsigned long _startMotor2;
unsigned long _endMotor2;
unsigned long _startReservatorio;
unsigned long _endReservatorio;
unsigned long _startMqttConnect;
unsigned long _endMqttConnect;
unsigned long _startEnviaDados;
unsigned long _endEnviaDados;

// time 
int timeSeconds = 5; // tempo em segundos

/*-----------------------------------------------
*   SETUP do Código
-----------------------------------------------*/
void setup() {
    Serial.begin(115200);
    Serial.println("-------- SETUP --------");
    start_gpio();
    conecta_wifi();
    conecta_mqtt();
    atualiza_sensores();
    verifica_sensores();
    envia_dados();
    start_timer();
    digitalWrite(PIN_LED_RESERVATORIO, LOW);
}

/*-----------------------------------------------
*   LOOP do Código
-----------------------------------------------*/
void loop() {
    
    client.loop();
    
//    if(sensor_nivel == 0){
      if (sinalizacao) {
//        Serial.print("-------- Sinalização = ");
//        Serial.println(sinalizacao);
        
        _startAtualiza = ESP.getCycleCount();
        atualiza_sensores();
        _endAtualiza = ESP.getCycleCount();
        
        verifica_sensores();

        _startEnviaDados = ESP.getCycleCount();
        envia_dados();
        _endEnviaDados = ESP.getCycleCount();
        sinalizacao = false;
      }
    
      if (envia) {
        
        Serial.println("\n# # # Timer estourou, atualizando DADOS!!! # # #\n");
        
        _startAtualiza = ESP.getCycleCount();
        atualiza_sensores();
        _endAtualiza = ESP.getCycleCount();

        verifica_sensores();
        
//        Serial.print("-------- Envia = ");
//        Serial.println(envia);
       
        _startEnviaDados = ESP.getCycleCount();
        envia_dados();
        _endEnviaDados = ESP.getCycleCount();

        print_sensores();
        tempos_execucao();
        
        envia = false;
        
        start_timer();
      }
//    }else{
//      Serial.println("########## Sem água no reservatório ##########");
//    }

    //_startEnviaDados = ESP.getCycleCount();
    //_endEnviaDados = ESP.getCycleCount();

    _startAtualiza = 0;
    _endAtualiza = 0; 
    _startMotor1 = 0;
    _endMotor1 = 0;
    _startMotor2 = 0;
    _endMotor2 = 0;
    _startReservatorio = 0;
    _endReservatorio = 0;
    _startMqttConnect = 0;
    _endMqttConnect = 0;
    _startEnviaDados = 0;
    _endEnviaDados = 0;
}

/*-----------------------------------------------
*   TIMER
-----------------------------------------------*/
//contador
void cb_timer() {
  
  static unsigned int counter = 0;

  if (counter == timeSeconds) {
    counter = 0;
    stopTimer();
  }
  counter++;
}

//start
void start_timer() {

  //inicialização do timer. Parametros:
  /* 0 - seleção do timer a ser usado, de 0 a 3.
    80 - prescaler. O clock principal do ESP32 é 80MHz. Dividimos por 80 para ter 1us por tick.
    true - true para contador progressivo, false para regressivo
  */
  timer = timerBegin(0, 80, true);
  
  /*conecta à interrupção do timer
    - timer é a instância do hw_timer
    - endereço da função a ser chamada pelo timer
    - edge=true gera uma interrupção
  */
  timerAttachInterrupt(timer, &cb_timer, true);
  
  /* - o timer instanciado no inicio
     - o valor em us para 1s
     - auto-reload. true para repetir o alarme
  */
  timerAlarmWrite(timer, 1000000, true);

  //ativa o alarme
  timerAlarmEnable(timer);
}

//stop
void stopTimer() {
  timerEnd(timer);
  envia = true;
  timer = NULL;
}

/*-----------------------------------------------
*   OUTRAS FUNÇÕES
-----------------------------------------------*/

/*
 * Start GPIO 
*/
void start_gpio(){
    pinMode(PIN_SENSOR_UMIDADE_1, INPUT);
    pinMode(PIN_SENSOR_UMIDADE_2, INPUT);
    pinMode(PIN_SENSOR_TEMP, INPUT);
    pinMode(PIN_SENSOR_NIVEL, INPUT);
    pinMode(PIN_MOTOR_1, OUTPUT);
    pinMode(PIN_MOTOR_2, OUTPUT);
    pinMode(PIN_LED_RESERVATORIO, OUTPUT);
    dht.begin();

    attachInterrupt(digitalPinToInterrupt(PIN_SENSOR_NIVEL), isr, CHANGE);
}

void IRAM_ATTR isr() {

  detachInterrupt(digitalPinToInterrupt(PIN_SENSOR_NIVEL));
  
  sinalizacao = true;
  
  attachInterrupt(digitalPinToInterrupt(PIN_SENSOR_NIVEL), isr, CHANGE);
}

/*
 * Conecta Wifi
*/
void conecta_wifi(){
    WiFi.begin(ssid, password);
    
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print("Connecting to WiFi:");
        Serial.println(ssid);
        Serial.print(".");
    }
    Serial.println("WiFi conectado com sucesso!!!");
    Serial.println("");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
}

/*
 * Conecta MQTT
*/
void conecta_mqtt() {
    client.setServer(mqttServer, mqttPort);
    client.setCallback(callback);
    // Loop until we're reconnected
    while (!client.connected()) {
        Serial.println("Connecting to MQTT...");
        String clientId = "ESP32Client-";
        clientId += String(random(0xffff), HEX);
        if (client.connect(clientId.c_str(), mqttUser, mqttPassword )) {
            Serial.println("Conectado.");
        } else {
            Serial.print("Falha no estado ");
            Serial.print(client.state());
            delay(2000);
        }
    }
    // inscrevendo no topic nivelmessage para retornar atualizações da placa
    client.subscribe("esp/irrigacaoMessage");
}

/*
 * Callback 
*/
void callback(char* topic, byte* payload, unsigned int length) {

   Serial.print("Chegou mensagem no topico: ");
   Serial.println(topic);

   Serial.print("Messagem:");
   for (int i = 0; i < length; i++) {
       Serial.print((char)payload[i]);
   }

   if (payload[0] == '1'){
       Serial.println("Enviando dados do sensor");
       atualiza_sensores();
       print_sensores();
   }
   Serial.println();
   Serial.println(" — — — — — — — — — — — -");
}

/*
 * Atualiza sensores 
*/
void atualiza_sensores(){
    // Umidade
    sensor_umidade1 = analogRead(PIN_SENSOR_UMIDADE_1);
    sensor_umidade2 = analogRead(PIN_SENSOR_UMIDADE_2);
    umidade1 = convert_to_Percent(sensor_umidade1);
    umidade2 = convert_to_Percent(sensor_umidade2);
    // Temperatura
    sensor_temp = dht.readTemperature();
    // Check if any reads failed and exit early (to try again).
    if (isnan(sensor_temp)){
      Serial.println("# # # Falha ao ler dados do Sensor DHT! # # #");
      return;
    }
    // Nível
    sensor_nivel = digitalRead(PIN_SENSOR_NIVEL);
}


/*
 * Verifica sensores 
*/
void verifica_sensores(){
    if(sensor_nivel == 1){
      //Serial.println("Nivel do reservatório: BAIXO");
      
      alerta_nivel_baixo_reservatorio();
      
    } else{
        // Desliga LED de aviso de n[ivel baixo do reservatório
        digitalWrite(PIN_LED_RESERVATORIO, LOW);
        Serial.println("Nivel do reservatório: OK");
        
        if(umidade1 < 40){
          Serial.println("Planta 1 Precisa de Água");
          _startMotor1 = ESP.getCycleCount();
          liga_motor(1);
          _endMotor1 = ESP.getCycleCount();
        }
          if (umidade2 < 40){
            Serial.println("Planta 2 Precisa de Água");
            _startMotor2 = ESP.getCycleCount();
            liga_motor(2);
            _endMotor2 = ESP.getCycleCount();
          
        }
    }
}

/*
 * Envia dados 
*/
void envia_dados(){
   
    String mensagem_alarme;

    HTTPClient http; 
    String url = "http://192.168.137.95:3000/insere?";

    if(sensor_nivel == 1){
        mensagem_alarme = "BAIXO";
    } else{
        mensagem_alarme = "OK";
    }

    String mensagem = String(sensor_temp) + "-" + String(umidade1) + "-" +  String(umidade2) + "-" + mensagem_alarme;
    char message_buff[50];

    // Http
    //Padrão de concatenação - insere?temperatura=10&umidade1=10&umidade2=10&nivelReservatorio=OK
    url = url + "temperatura=" + String(sensor_temp) + "&umidade1=" + String(umidade1) + "&umidade2=" +  String(umidade2) + "&nivelReservatorio=" + mensagem_alarme;
      
    Serial.println("Enviando dados para banco de dados.");
    Serial.println(url);
    http.begin(url);
    int httpCode = http.GET();
    String payload = http.getString();
    Serial.println("Resposta do servidor");
    Serial.println(payload);
    http.end();

    // Mqtt
    mensagem.toCharArray(message_buff, mensagem.length()+ 1);
    client.publish("esp/irrigacao", message_buff);
}

/*
 * Alerta nível baixo do reservatório 
*/
void alerta_nivel_baixo_reservatorio(){
    digitalWrite(PIN_LED_RESERVATORIO, HIGH);
    Serial.println("\n---> Nivel do reservatório: BAIXO <---");
    delay(3000);
}

/*
 * Liga motores 
*/
void liga_motor(int value){
    if(value == 1){
      digitalWrite(PIN_MOTOR_1, HIGH);
      Serial.println("---> MOTOR '1' LIGADO <---");
      delay(TEMPO_MOTOR);
      digitalWrite(PIN_MOTOR_1, LOW);
      Serial.println("---> MOTOR '1' DESLIGADO <---");
      delay(TEMPO_MOTOR);
    }
    if(value == 2){
      digitalWrite(PIN_MOTOR_2, HIGH);
      Serial.println("---> MOTOR '2' LIGADO <---");
      delay(TEMPO_MOTOR);
      digitalWrite(PIN_MOTOR_2, LOW);
      Serial.println("---> MOTOR '2' DESLIGADO <---");
      delay(TEMPO_MOTOR);
    }
}

/*
 * Converte para percentual leitura do sensor de umidade
*/
int convert_to_Percent(int value){
  int percentValue = 0;
  percentValue = map(value, 4095, 1000, 0, 100);
  return percentValue;
}

/*
 * Print sensores 
*/
void print_sensores(){
    Serial.println("----------------------------------");
    Serial.println("Leituras do Sensores");
    Serial.println("----------------------------------");
    Serial.print("Umidade sensor 1: ");
    Serial.print(umidade1);
    Serial.println("%");
    Serial.print("Umidade sensor 2: ");
    Serial.print(umidade2);
    Serial.println("%");
    Serial.print("Temperatura Ambiente: ");
    Serial.println(sensor_temp);
    Serial.print("Nivel do Reservatorio: ");
    if(sensor_nivel == 0){
        Serial.println("OK");
    }else{
        Serial.println("BAIXO");
    }
}

/*
 * Print verifica tempo de execucao 
*/
void tempos_execucao(){
    Serial.println("----------------------------------");
    Serial.println("Tempo de Execução");
    Serial.println("----------------------------------");
    Serial.print("Tempo atualiza sensores: ");
    Serial.println(_endAtualiza-_startAtualiza);
    Serial.print("Tempo acionamento Motor 1: ");
    Serial.println(_endMotor1-_startMotor1);
    Serial.print("Tempo acionamento Motor 2: ");
    Serial.println(_endMotor2-_startMotor2);
    Serial.print("Tempo envio de dados: ");
    Serial.println(_endEnviaDados-_startEnviaDados);
}
