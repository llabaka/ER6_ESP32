#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <MFRC522.h>
#include <Arduino.h>
#include <driver/ledc.h>


// const char* ssid = "POCO F3"; // Cambié las comillas
// const char* password = "llabakawifi"; // Cambié las comillas

// PIN MFRC522
#define SS_PIN 5
#define RST_PIN 22
// LED
#define LED_PIN 26
#define RED_LED_PIN 25

//BUZZER
#define BUZZER_PIN 32

// Sustituir con datos de vuestra red
const char *ssid     = "AEG-IKASLE";
const char *password = "Ea25dneAEG";
const char *mqtt_server = "10.80.128.11";
const int mqtt_port = 1883; //MQTT insecure

WiFiClient espClient;
PubSubClient client(espClient);

byte nuidPICC[4] = {0, 0, 0, 0};
MFRC522::MIFARE_Key key;
MFRC522 rfid = MFRC522(SS_PIN, RST_PIN);

void setup()
{
  Serial.begin(115200);
  delay(10);
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Conectando a:\t");
  Serial.println(ssid); 

  // Esperar a que nos conectemos
  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(1000);
  Serial.print('.');
  }

  // Mostrar mensaje de exito y dirección IP asignada
  Serial.println();
  Serial.print("Conectado a:\t");
  Serial.println(WiFi.SSID()); 
  Serial.print("IP address:\t");
  Serial.println(WiFi.localIP());

    // Configuración del cliente MQTT
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(messageCallback);

  // Conectar al broker MQTT
  connectToMQTT();


  // Init SPI with specified pins
  Serial.println(F("Initializing SPI communication..."));
  SPI.begin(); // SCK=18, MISO=19, MOSI=25, SS=5

  // Init RFID
  Serial.println(F("Before RFID Initialization"));
  rfid.PCD_Init();
  Serial.println(F("After RFID Initialization"));
  
  // Delay to ensure RFID stabilizes
  delay(500); // Aumentar el delay para dar tiempo al módulo
  
  // Mostrar versión del lector
  rfid.PCD_DumpVersionToSerial();
  Serial.println(F("Scan PICC to see UID, SAK, type, and data blocks..."));

  // SETUP LED
  pinMode(LED_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);

  // SETUP BUZZER
  // pinMode(BUZZER_PIN, OUTPUT);
}

void connectToMQTT() {
  while (!client.connected()) {
    Serial.print("Intentando conectar al broker MQTT...");
    
    // Intentar conectarse
    if (client.connect("LANDER_ESP32")) { // "ESP32Client" puede ser cualquier ID de cliente único
      Serial.println("Conectado al broker MQTT");
      
      client.publish("testEsp32", "Hi, I'm ESP 32");
      client.subscribe("AnatiValidation");
      client.subscribe("AnatiValidationFailed");
    } else {
      Serial.print("Falló la conexión, rc=");
      Serial.print(client.state());
      Serial.println(" Intentando de nuevo en 5 segundos");
      delay(5000);
    }
  }
}

void loop() {
  client.loop();
  readRFID();
}


/**
   Helper routine to dump a byte array as hex values to Serial.
*/
void printHex(byte *buffer, byte bufferSize) {
  for (byte i = 0; i < bufferSize; i++) {
    Serial.print(buffer[i] < 0x10 ? " 0" : " ");
    Serial.print(buffer[i], HEX);
  }
}

/**
   Helper routine to dump a byte array as dec values to Serial.
*/
void printDec(byte *buffer, byte bufferSize) {
  for (byte i = 0; i < bufferSize; i++) {
    Serial.print(buffer[i] < 0x10 ? " 0" : " ");
    Serial.print(buffer[i], DEC);
  }
}

void readRFID(void ) { /* function readRFID */
  ////Read RFID card

  for (byte i = 0; i < 6; i++) {
    key.keyByte[i] = 0xFF;
  }
  // Look for new 1 cards
  if ( ! rfid.PICC_IsNewCardPresent())
    return;

  // Verify if the NUID has been readed
  if (  !rfid.PICC_ReadCardSerial())
    return;

  // Store NUID into nuidPICC array
  for (byte i = 0; i < 4; i++) {
    nuidPICC[i] = rfid.uid.uidByte[i];
  }

  Serial.print(F("RFID In dec: "));
  printDec(rfid.uid.uidByte, rfid.uid.size);
  Serial.println();
  // GESTION DE LED
  digitalWrite(LED_PIN, HIGH);
  tone(BUZZER_PIN, 2000); 
  delay(1000); // LED encendido por 1 segundo
  digitalWrite(LED_PIN, LOW);
  noTone(BUZZER_PIN);

  // LLAMADAS PARA escribir y leer la tarjeta
  // writeRFID();   // Escribir
  readRFIDData();

  // Halt PICC
  rfid.PICC_HaltA();

  // Stop encryption on PCD
  rfid.PCD_StopCrypto1();

}

void writeRFID(){
  byte dataBlock[16] = {"Cj8<f&~esD)}cw1"};   // Mensaje a escribir
  byte block = 4; // Bloque de la tarjeta que se escribe

  MFRC522::StatusCode status = rfid.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, block, &key, &(rfid.uid));
  if (status != MFRC522::STATUS_OK){
    Serial.print(F("Fallo en auntenticación"));
    Serial.println(rfid.GetStatusCodeName(status));
    return;
  }

  status = rfid.MIFARE_Write(block, dataBlock, 16);
  if(status != MFRC522::STATUS_OK){
    Serial.print(F("Fallo al escribrir: "));
    Serial.println(rfid.GetStatusCodeName(status));
    return;
  }
  Serial.println(F("Datos escritos en el bloque 4. "));

}

void readRFIDData(){
  byte buffer[18];
  byte size = sizeof(buffer);
  byte block = 4;  // Bloque de la tarjeta donde se leera

  
  MFRC522::StatusCode status = rfid.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, block, &key, &(rfid.uid));
  if (status != MFRC522::STATUS_OK){
    Serial.print(F("Fallo en auntenticación"));
    Serial.println(rfid.GetStatusCodeName(status));
    return;
  }

  status = rfid.MIFARE_Read(block, buffer, &size);
  if(status != MFRC522::STATUS_OK){
    Serial.print(F("Fallo al leer"));
    Serial.println(rfid.GetStatusCodeName(status));
    return;
  }

  Serial.print(F("Datos leidos del bloque 4: "));
  char message[17];
  for(byte i = 0; i < 16; i++){
    message[i] = buffer[i];
    Serial.write(buffer[i]);
  }

  message[16] = '\0'; 

  client.publish("testCardID", message);


  Serial.println();
}

// Definir la función callback que se llamará cuando llegue un mensaje
void messageCallback(char* topic, byte* payload, unsigned int length){

  Serial.print("Mensaje recibido en el tópico");
  Serial.print(topic);

  Serial.print("Mensaje: ");
  for(int i = 0; i < length; i++){
    Serial.print((char)payload[i]);
  }
  Serial.println();

  if(String(topic) == "AnatiValidation"){
    Serial.println("Ejecutando codigo segun el topic");
  }else if(String(topic) == "AnatiValidationFailed"){
    Serial.println("Ejecutando AnatiValidationFailed");
    // GESTION DE LED
    digitalWrite(RED_LED_PIN, HIGH);
    tone(BUZZER_PIN, 2000); 
    delay(200);
    noTone(BUZZER_PIN);
    delay(100);
    tone(BUZZER_PIN, 2000);
    delay(200);
    noTone(BUZZER_PIN);
    delay(1000); // LED encendido por 1 segundo
    digitalWrite(RED_LED_PIN, LOW);



  }
}
