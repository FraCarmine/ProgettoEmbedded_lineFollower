#include <TaskScheduler.h>
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_TCS34725.h>
#include <QuickPID.h>

//--------------------Define..................................................

#define FREQMQTTRICEZIONE 100
#define FREQMQTTPUBLISH 2000
#define FREQRECONNECT 3000
#define FREQDRIVE 30
#define SX 0
#define CENTRO 1
#define DX 2
//0 sx 1 c 2 dx
#define TCAADDR 0x70 // Indirizzo I2C standard del multiplexer TCA9548A

//---------- SOGLIE CALIBRATE DAI LOG ----------
#define SOGLIA_BIANCO_MIN   100   // 
#define SOGLIA_NERO_MAX     60    //

// Percentuali R/G/B sul totale 
#define ROSSO_PERC_R_MIN    0.66f //Il bianco ha 60% il rosso vero ha 71%
#define VERDE_PERC_G_MIN    0.60f
#define BLU_PERC_B_MIN      0.60f



// --- PIN MOTORI (TB6612FNG) ---
#define PIN_STBY 13

// Motore Sinistro (A)
#define PIN_PWMA 32
#define PIN_AIN1 25
#define PIN_AIN2 26

// Motore Destro (B)
#define PIN_PWMB 23
#define PIN_BIN1 19
#define PIN_BIN2 18

// --- IMPOSTAZIONI VELOCITÀ ---
float velocitaMax = 250.0f;
int   minPWM      = 55;



//----------------------PROTOTIPI---------------------------------------------
void mqttRicezione();
void mqttPublish();
void setupWifi();
int mqttConnect(); //si occupa di connettersi a mqtt
void reconnect(); // Funzione della task che se viene persa connessione riconnette
void enableMqttTask(); //usato in reconnect per quando disattivo le task se cade connessione
void disableMqttTask(); //disbilito le task in caso non funzioni 
void mqttFirstConnect(); // chiama mqttConnect fino a quando non è connesso
void mqttSetup();
void mqttCallback(char* topic, byte* payload, unsigned int length);
void drive();
void impostaMotori(int velSx, int velDx); // Prototipo funzione motori
//----------------------------------------------------------------------------


//------------------- Configurazione Wi-Fi------------------------------------

#define SSID "R2-D2 7398"
#define PASSWORD "42Bx9!21"
#define PORT 1883
#define SERVER "192.168.137.1"

WiFiClient espClient;          // Oggetto WiFi
PubSubClient mqttClient(espClient); 
const String ID = "esp32_macchina";  //@TODO potrebbe essere un id univoco e quindi usare il mac

//----------------------------------------------------------------------------

//------------------------VARIABILI GLOBALI E PID-----------------------------------
String coloreTarget = "rosso";
String colAtt="";
String ultimaDirezione="dritto";

// Variabili per il PID
float pidSetpoint = 0; // Vogliamo che l'errore sia zero (sensore centrale sulla linea)
float pidInput = 0;    // L'errore attuale calcolato dai sensori
float pidOutput = 0;   // La correzione calcolata per sterzare



// Parametri PID (Da affinare per tracciato)
float Kp = 18.0; // Proporzionale (Sterzata brusca)
float Ki = 0.0;  // Integrale
float Kd = 2.0;  // Derivativo (Ammortizzatore per evitare oscillazioni) @TODO 

QuickPID mioPID(&pidInput, &pidOutput, &pidSetpoint, Kp, Ki, Kd, QuickPID::Action::direct);
//scheduler
Scheduler ts;

// Imposto la lettura a 2.4 millisecondi per non bloccare la task drive
Adafruit_TCS34725 tcs[3] = {
  Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_16X),
  Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_16X),
  Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_16X)
};
//----------------------------------------------------------------------------
void tcaSelect(uint8_t i) {//funzione consigliata per il multiplexer
  if (i > 7) return; 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}


//task
Task tMqttRicezione(FREQMQTTRICEZIONE * TASK_MILLISECOND, //FREQUENZA
           TASK_FOREVER,
           mqttRicezione,//nome funzione di handling
           &ts, //scheduler
           true // abilitazione
);

Task tMqttPublish(FREQMQTTPUBLISH * TASK_MILLISECOND,
                  TASK_FOREVER,
                  mqttPublish,
                  &ts,
                  true);

Task tReconnect(FREQRECONNECT *TASK_MILLISECOND,
                TASK_FOREVER,
                reconnect,
                &ts,
                true);

Task tDrive(FREQDRIVE * TASK_MILLISECOND,
            TASK_FOREVER,
            drive,
            &ts,
            true);





void setup() {
  Serial.begin(115200);
  setupWifi();
  Serial.println("Wifi pronto");
  mqttFirstConnect();
  Wire.begin();//inizio il i2c
  //controllo che sia tutto collegato correttamente
  for (int i = 0; i < 3; i++) {
    tcaSelect(i);
    if (tcs[i].begin()) {
      Serial.printf("Sensore su Canale %d\n", i);
    } else {
      Serial.printf("ERRORE: Sensore su Canale %d NON trovato!\n", i);
      return;
    }
  }
  // Configuro i pin dei motori come OUTPUT
  pinMode(PIN_STBY, OUTPUT);
  pinMode(PIN_PWMA, OUTPUT);
  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_PWMB, OUTPUT);
  pinMode(PIN_BIN1, OUTPUT);
  pinMode(PIN_BIN2, OUTPUT);

  //Disattivo lo standBy
  digitalWrite(PIN_STBY, HIGH);  

  // Setup del PID
  mioPID.SetMode(QuickPID::Control::automatic);
  mioPID.SetOutputLimits(-255, 255);
  mioPID.SetSampleTimeUs(0); // disabilita il timer interno
  
}

void loop() {
   ts.execute();
}

void mqttRicezione(){
  mqttClient.loop();
}

void mqttPublish(){
  mqttClient.publish("macchina/telemetry/status",      "online");
  mqttClient.publish("macchina/telemetry/colorTarget", coloreTarget.c_str());
  mqttClient.publish("macchina/telemetry/colorRead",   colAtt.c_str());  
}

void drive(){
  uint16_t r[3], g[3], b[3], c[3];
  double erroreAttuale = 0;


  // --- LEGGO CENTRO ---
  tcaSelect(CENTRO);
  tcs[CENTRO].getRawData(&r[CENTRO], &g[CENTRO], &b[CENTRO], &c[CENTRO]);
  String coloreCentro = riconosciColore(r[CENTRO], g[CENTRO], b[CENTRO], c[CENTRO]);
  Serial.printf("[CENTRO] C:%d R:%d G:%d B:%d → %s\n",c[CENTRO], r[CENTRO], g[CENTRO], b[CENTRO], coloreCentro.c_str());

  // --- LEGGO SX ---
  tcaSelect(SX);
  tcs[SX].getRawData(&r[SX], &g[SX], &b[SX], &c[SX]);
  String coloreSinistro = riconosciColore(r[SX], g[SX], b[SX], c[SX]);
  Serial.printf("[SX]     C:%d R:%d G:%d B:%d → %s\n", c[SX], r[SX], g[SX], b[SX], coloreSinistro.c_str());

  // --- LEGGO DX ---
  tcaSelect(DX);
  tcs[DX].getRawData(&r[DX], &g[DX], &b[DX], &c[DX]);
  String coloreDestro = riconosciColore(r[DX], g[DX], b[DX], c[DX]);
  Serial.printf("[DX]     C:%d R:%d G:%d B:%d → %s\n", c[DX], r[DX], g[DX], b[DX], coloreDestro.c_str());

  //aggiorno mqtt con i dati
  colAtt = "SX:" + coloreSinistro + " CX:" + coloreCentro + " DX:" + coloreDestro;
  

  //lo switch non vuole boolean ma numeri io uso bitmask
  uint8_t bitSx = (coloreSinistro == coloreTarget) ? 1 : 0;
  uint8_t bitCx = (coloreCentro == coloreTarget)   ? 1 : 0;
  uint8_t bitDx = (coloreDestro == coloreTarget)   ? 1 : 0;

  uint8_t statoSensori = (bitSx << 2) | (bitCx << 1) | bitDx; //SX-CX-DX

  switch (statoSensori) {
    case 7: // Binario 111 (Incrocio)
      erroreAttuale = 0;
      ultimaDirezione = "dritto";
      break;

    case 6: // Binario 110 leggermente a sx
      erroreAttuale = -5;
      ultimaDirezione = "sinistra";
      break;

    case 3: // Binario 011 leggermente a dx
      erroreAttuale = 5;
      ultimaDirezione = "destra";
      break;

    case 2: // Binario 010 Centro perfetto
      erroreAttuale = 0;
      ultimaDirezione = "dritto";
      break;

    case 4: // Binario 100 (Tutto a sinistra)
      erroreAttuale = -10;
      ultimaDirezione = "sinistra";
      break;

    case 1: // Binario 001 (Tutto a destra)
      erroreAttuale = 10;
      ultimaDirezione = "destra";
      break;

    case 0: // Binario 000 (Fuori Pista) Proseguo sulla stessa direzione
      if (ultimaDirezione == "sinistra") erroreAttuale = -25;
      else if (ultimaDirezione == "destra") erroreAttuale = 25;
      else erroreAttuale = 0;
      break;

    case 5: // Binario 101 (SX e DX)
      erroreAttuale = 0; 
      break;
  }

  //  CALCOLO PID
  pidInput = erroreAttuale;
  mioPID.Compute(); // Riempe la variabile pidOutput con la sterzata necessaria

  // ---CALCOLO VELOCITA ---
  // Più il PID sterza, più sottraggo velocità alla base per curvare
  int velocitaBaseDinamica = velocitaMax - abs(pidOutput);
  if(pidOutput == 0){
    velocitaBaseDinamica = 140;
  }
  if (velocitaBaseDinamica < 0) {
      velocitaBaseDinamica = 0; 
  }

  int velSinistra = velocitaBaseDinamica - pidOutput;
  int velDestra   = velocitaBaseDinamica + pidOutput;
  Serial.print("Velocita_SX:");
  Serial.print(velSinistra);
  Serial.print(",");
  Serial.print("Velocita_DX:");
  Serial.println(velDestra);
  impostaMotori(velSinistra, velDestra);
}

//gestione motori
void impostaMotori(int velSx, int velDx) {
  velSx = constrain(velSx, -255, 255);
  velDx = constrain(velDx, -255, 255);
  //anti stallo
  int pwmSx = abs(velSx);
  if (pwmSx > 0 && pwmSx < minPWM) pwmSx = minPWM; // Se è troppo debole, spingilo al minimo
  
  int pwmDx = abs(velDx);
  if (pwmDx > 0 && pwmDx < minPWM) pwmDx = minPWM; // Idem per la destra

  
  // --- MOTORE SINISTRO (A) ---
  if (velSx > 0) {
    digitalWrite(PIN_AIN1, HIGH);
    digitalWrite(PIN_AIN2, LOW);
  } else {
    digitalWrite(PIN_AIN1, LOW);
    digitalWrite(PIN_AIN2, HIGH); // Retromarcia
  }
  analogWrite(PIN_PWMA, pwmSx); // Uso il pwm corretto anti-stallo


  // --- MOTORE DESTRO (B) ---
  if (velDx > 0) {
    digitalWrite(PIN_BIN1, HIGH);
    digitalWrite(PIN_BIN2, LOW);
  } else {
    digitalWrite(PIN_BIN1, LOW);
    digitalWrite(PIN_BIN2, HIGH); // Retromarcia
  }
  analogWrite(PIN_PWMB, pwmDx); // Uso il pwm corretto anti-stallo
}

String riconosciColore(uint16_t r, uint16_t g, uint16_t b, uint16_t c) {
  
  // 1. Protezione buio totale
  if (c == 0) return "nero";

  // --- 2. TRUCCO DEL RAPPORTO (Perfetto per mezze linee e ombre) ---
  float rapportoRG = (float)r / (float)g;

  // Se il Rosso è almeno 1.45 volte il Verde, è per forza il nostro nastro!
  if (rapportoRG >= 1.45f) {
      return "rosso";
  }

 //percentuali 
  float percR = (float)r / c;
  float percG = (float)g / c;
  float percB = (float)b / c;

  if (percG >= VERDE_PERC_G_MIN && percG > percR && percG > percB) return "verde";
  if (percB >= BLU_PERC_B_MIN  && percB > percR && percB > percG) return "blu";

  // 4. SOLO ALLA FINE: se non è nessun colore, decido tra bianco o nero
  if (c >= SOGLIA_BIANCO_MIN) {
      return "bianco";
  }

  return "nero";
}





//---------------------------------------wifi---------------------
void reconnect(){
  static bool wifiTrying= false;
  if(WiFi.status() != WL_CONNECTED){
     if(!wifiTrying){
        WiFi.begin(SSID, PASSWORD);
        wifiTrying = true;
    }
    disableMqttTask();
  }
  else{
    wifiTrying = false;
    enableMqttTask(); //in caso si scolleghi il wifi ma rimanga attaccato mqtt disabilito task ma poi si ricollega subito e rimanevo con task disabilitate
    if(!mqttClient.connected()){
      /*for(int i=0; i<5 && (!mqttConnect())){
        delay(10); //@TODO valutare il tempo del delay e anche se disabilitare la task del drive e fermare i motori
      }*/
      if(mqttConnect()){
        enableMqttTask();   //se andato a buon fine riabilito le task
      }else{
        disableMqttTask(); //wifi si e mqtt no
      }
    }
  }
}

void disableMqttTask(){
  if (tMqttRicezione.isEnabled()) tMqttRicezione.disable(); // se abilitato la disabilito
  if (tMqttPublish.isEnabled())   tMqttPublish.disable(); //idem 
}

void enableMqttTask(){
  if (!tMqttRicezione.isEnabled()) tMqttRicezione.enable();
  if (!tMqttPublish.isEnabled())   tMqttPublish.enable();
}


void setupWifi(){
  Serial.print("connessione a  SSID");
  WiFi.begin(SSID, PASSWORD);
  while(WiFi.status() != WL_CONNECTED){
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Connected, IP address: ");
  Serial.println(WiFi.localIP());
}


void mqttFirstConnect() {
  mqttClient.setServer(SERVER, PORT);
  while (!mqttClient.connected()) {
    if(!mqttConnect()){
        Serial.println("ritento tra 200 millisecondi...");
        delay(200); // attesa prima di riprovare
      }
  }
}


int mqttConnect(){
  Serial.println("Provo connessione MQTT...");
    if (mqttClient.connect(ID.c_str())) {
      Serial.println("Connected (MQTT)");
      mqttSetup();  // fuznione per i subscribe e un messaggio per dire che sono connesso
      return 1;
    } else {
      Serial.print("Failed, rc=");
      Serial.print(mqttClient.state());
      return 0;
    }
}

void mqttSetup(){
  // Subscribe solo ai topic in ingresso
  mqttClient.subscribe("macchina/cmd/stop");
  mqttClient.subscribe("macchina/cmd/linea");
  mqttClient.subscribe("macchina/tuning/kp");
  mqttClient.subscribe("macchina/tuning/kd");
  mqttClient.subscribe("macchina/tuning/velMax");
  mqttClient.subscribe("macchina/tuning/minPWM");
  mqttClient.setCallback(mqttCallback);

  // Telemetry iniziale
  mqttClient.publish("macchina/telemetry/status", "online");
  mqttClient.publish("macchina/telemetry/ip", WiFi.localIP().toString().c_str());
}


void mqttCallback(char* topic, byte* payload, unsigned int length){
  String msg;
  for(int i = 0; i < length; i++) msg += (char)payload[i];
  msg.trim();

  if(strcmp(topic, "macchina/cmd/linea") == 0){
    if(!msg.equalsIgnoreCase(coloreTarget)){
      coloreTarget = msg;
      ultimaDirezione = "destra";
    }
  }
  else if(strcmp(topic, "macchina/cmd/stop") == 0){
    if(msg.equalsIgnoreCase("true")){
      impostaMotori(0, 0);
      tDrive.disable();
    } else {
      tDrive.enable();
    }
  }
  else if(strcmp(topic, "macchina/tuning/kp") == 0){
    float val = msg.toFloat();
    if(val > 0){ Kp = val; 
      mioPID.SetTunings(Kp, Ki, Kd); }
  }

  else if(strcmp(topic, "macchina/tuning/kd") == 0){
    float val = msg.toFloat();
    if(val >= 0){ Kd = val; 
      mioPID.SetTunings(Kp, Ki, Kd); }
  }

  else if(strcmp(topic, "macchina/tuning/velMax") == 0){
    int val = msg.toInt();
    if(val > 0 && val <= 255) velocitaMax = val;
  }
  else if(strcmp(topic, "macchina/tuning/minPWM") == 0){
    int val = msg.toInt();
    if(val >= 0 && val < 150) minPWM = val;
  }
}
