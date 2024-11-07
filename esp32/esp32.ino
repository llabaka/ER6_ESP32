#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <MFRC522.h>
#include <Arduino.h>
#include <driver/ledc.h>
#include <ESP32Servo.h>
#include "certs.h"

// PIN MFRC522
#define SS_PIN 5
#define RST_PIN 22
// LED
#define GREENLED_PIN 26
#define RED_LED_PIN 25

// BUZZER
#define BUZZER_PIN 32

// SERVO
Servo myServo; // Crear objeto servo
#define SERVO_PIN 21
#define CLOSE_DOOR 0
#define OPEN_DOOR 180
int servoAngle = CLOSE_DOOR;

// Sustituir con datos de vuestra red
const char *ssid     = "AEG-IKASLE";
const char *password = "Ea25dneAEG";
const char *mqtt_server = "10.80.128.2";
const int mqtt_port = 8883; //MQTT insecure
 
WiFiClientSecure espClient;
PubSubClient client(espClient);

byte nuidPICC[4] = {0, 0, 0, 0};
MFRC522::MIFARE_Key key;
MFRC522 rfid = MFRC522(SS_PIN, RST_PIN);

char cardId[17] = ""; // Variable global para almacenar el cardId
bool isReading = false; // Estado de bloqueo para evitar lecturas duplicadas

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
  setup_ssl();
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
  pinMode(GREENLED_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);

  // SETUP BUZZER
  pinMode(BUZZER_PIN, OUTPUT);

  // SETUP SERVO
  myServo.attach(SERVO_PIN);
}

void connectToMQTT() {
  while (!client.connected()) {
    Serial.print("Intentando conectar al broker MQTT...");
    
    // Intentar conectarse
    if (client.connect("LANDER_ESP32")) { // "ESP32Client" puede ser cualquier ID de cliente único
      Serial.println("Conectado al broker MQTT");
      
      client.subscribe("OpenDoor");
      client.subscribe("AnatiValidationFailed");
      client.subscribe("AnatiCloseDoor");
    } else {
      Serial.print("Falló la conexión, rc=");
      Serial.print(client.state());
      Serial.println(" Intentando de nuevo en 5 segundos");
      delay(1000);
    }
  }
}

void loop() {
  client.loop();
  readRFID();

  // Revisa la conexión WiFi
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi desconectado. Intentando reconectar...");
    reconnectWiFi();
  }
  // Revisa la conexión MQTT
  if (!client.connected()) {
    connectToMQTT();
  }
}

void printHex(byte *buffer, byte bufferSize) {
  for (byte i = 0; i < bufferSize; i++) {
    Serial.print(buffer[i] < 0x10 ? " 0" : " ");
    Serial.print(buffer[i], HEX);
  }
}

void printDec(byte *buffer, byte bufferSize) {
  for (byte i = 0; i < bufferSize; i++) {
    Serial.print(buffer[i] < 0x10 ? " 0" : " ");
    Serial.print(buffer[i], DEC);
  }
}

void readRFID(void ) { 
  for (byte i = 0; i < 6; i++) {
    key.keyByte[i] = 0xFF;
  }
  
  if (!rfid.PICC_IsNewCardPresent())
    return;

  if (!rfid.PICC_ReadCardSerial())
    return;

  if (isReading) {
    return; // Si ya se está leyendo, no hacer nada
  }
  
  isReading = true; // Establecer estado de lectura

  for (byte i = 0; i < 4; i++) {
    nuidPICC[i] = rfid.uid.uidByte[i];
  }

  Serial.print(F("RFID In dec: "));
  printDec(rfid.uid.uidByte, rfid.uid.size);
  Serial.println();

  tone(BUZZER_PIN, 2000); 
  delay(500); // BUZZER sonará por 1 segundo
  noTone(BUZZER_PIN);

  readRFIDData();

  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();

  delay(2000); // Esperar un segundo antes de permitir otra lectura
  isReading = false; // Restablecer el estado de lectura
}

void writeRFID(){
  byte dataBlock[16] = {"Cj8<f&~esD)}cw1"};   // Mensaje a escribir
  byte block = 4; // Bloque de la tarjeta que se escribe

  MFRC522::StatusCode status = rfid.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, block, &key, &(rfid.uid));
  if (status != MFRC522::STATUS_OK){
    Serial.print(F("Fallo en autenticación"));
    Serial.println(rfid.GetStatusCodeName(status));
    return;
  }

  status = rfid.MIFARE_Write(block, dataBlock, 16);
  if(status != MFRC522::STATUS_OK){
    Serial.print(F("Fallo al escribir: "));
    Serial.println(rfid.GetStatusCodeName(status));
    return;
  }
  Serial.println(F("Datos escritos en el bloque 4."));
}

void readRFIDData(){
  byte buffer[18];
  byte size = sizeof(buffer);
  byte block = 4;

  MFRC522::StatusCode status = rfid.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, block, &key, &(rfid.uid));
  if (status != MFRC522::STATUS_OK){
    Serial.print(F("Fallo en autenticación"));
    Serial.println(rfid.GetStatusCodeName(status));
    return;
  }

  status = rfid.MIFARE_Read(block, buffer, &size);
  if(status != MFRC522::STATUS_OK){
    Serial.print(F("Fallo al leer"));
    Serial.println(rfid.GetStatusCodeName(status));
    return;
  }

  Serial.print(F("Datos leídos del bloque 4: "));
  for(byte i = 0; i < 16; i++){
    cardId[i] = buffer[i];  // Guarda el mensaje en cardId
    Serial.write(buffer[i]);
  }

  cardId[16] = '\0';  // Termina la cadena
  client.publish("testCardID", cardId);

  Serial.println();
}

void messageCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Mensaje recibido en el tópico: ");
  Serial.print(topic);

  for(int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  if(String(topic) == "OpenDoor") {

    Serial.println("El servo se ha empezado a mover");

    digitalWrite(GREENLED_PIN, HIGH);
    tone(BUZZER_PIN, 1000); 
    delay(500); // LED y BUZZER encendido por 1 segundo
    digitalWrite(GREENLED_PIN, LOW);
    noTone(BUZZER_PIN);

    // Mover el servo a 90 grados
    myServo.attach(21);     // Reasignar pin SERVO
    myServo.write(180);     // Mover el servo a 0 grados
    delay(1000);

    client.publish("DoorIsOpen", cardId);  // Usa cardId para publicar

  } else if(String(topic) == "AnatiValidationFailed") {
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

  } else if(String(topic) == "AnatiCloseDoor") {
    Serial.println("Cerrando la puerta...");

    // Mover el servo a 0 grados
    myServo.attach(21);      // Reasignar pin SERVO
    myServo.write(0);     // Mover el servo a 0 grados
    delay(1000);
    Serial.println("Puerta cerrada!");
  }
}

void reconnectWiFi() {
  WiFi.disconnect();
  WiFi.begin(ssid, password);
  
  // Espera hasta reconectarse
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  
  Serial.println();
  Serial.print("Reconectado al WiFi: ");
  Serial.println(WiFi.SSID());
  Serial.print("Nueva IP address: ");
  Serial.println(WiFi.localIP());
}

void setup_ssl(){
  espClient.setCACert(ca_cert);
  espClient.setCertificate(esp32_cert);
  espClient.setPrivateKey(esp32_key);
}