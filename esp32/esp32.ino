#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <MFRC522.h>
#include <Arduino.h>
#include <driver/ledc.h>
#include <ESP32Servo.h>

// const char* ssid = "POCO F3"; // Cambié las comillas
// const char* password = "llabakawifi"; // Cambié las comillas

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

const char* ca_cert = \
"-----BEGIN CERTIFICATE-----\n"
"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\n"
"-----END CERTIFICATE-----\n";

const char* esp32_cert = \
"-----BEGIN CERTIFICATE-----\n"
"MIIEezCCAmMCFBRnUEUm4GxwWA7G89XMG712XuDSMA0GCSqGSIb3DQEBCwUAMIGJ\n" \
"MQswCQYDVQQGEwJFVTERMA8GA1UECAwIR0lQVVpLT0ExETAPBgNVBAcMCERPTk9T\n" \
"VElBMQwwCgYDVQQKDANBRUcxEDAOBgNVBAsMB0tBT1RJS0ExEDAOBgNVBAMMB0tB\n" \
"T1RJS0ExIjAgBgkqhkiG9w0BCQEWE29za2FyLmNhbHZvQGFlZy5ldXMwHhcNMjQx\n" \
"MTA1MTE0NjI2WhcNMjUxMTA1MTE0NjI2WjBqMQswCQYDVQQGEwJTUDERMA8GA1UE\n" \
"CAwIR2lwdXprb2ExETAPBgNVBAcMCERvbm9zdGlhMRcwFQYDVQQKDA5BbmF0aWRh\n" \
"ZXBob2JpYTEMMAoGA1UECwwDQUVHMQ4wDAYDVQQDDAVFU1AzMjCCASIwDQYJKoZI\n" \
"hvcNAQEBBQADggEPADCCAQoCggEBAL/im+dE+Drk2Hk/6n6qsGyOllDTJFNyoU/2\n" \
"yKkKLG1YOyi0X8Unf0krZYEs++FMFGmZAmGNTts9XwDHBCiO9Zo19j9ctVaPMSdo\n" \
"1ggeFSsE9V2K/moTomStPLa27WZIp1dIUGy12aHUGzVutxqdIbuHNoiXDVlR4he+\n" \
"F/3HYy7RKx71BNZQm8syO9Yp8QWXPKIgqhgUELyKRJUjQ5XK9m0OpA3kywztSbG9\n" \
"ozyeLN8HMHm357DctZtbRzihGsQMSEW3oUMsusbAgZ3GwlR64KopPqGxnDNU3gLW\n" \
"xyofhMyE99FTA53HMwfo878av/CD8Q/gram6MSW5/FSIMKkODvsCAwEAATANBgkq\n" \
"hkiG9w0BAQsFAAOCAgEACpPMphz00rr26AEYVxiacYOJNm8Gns37CbuDsogUhOPj\n" \
"HYgmyV5XxhWsl5X69SP5kiwfD7LONVNwhKGVFPfjBmBOCkYqhx6EfgnVx7A/kQWf\n" \
"F66p2JqPvQZsEo2py8Ya/dCMPRx1F2XxM7wK2u4z9zI6pclbbZsCBEBuAKovlG/1\n" \
"Rxomrhl96neFAEzxXWwe2M2BZLMS0Xd4rfnKzzAqwn1YEjII/raM3ujPjlYrvJJ1\n" \
"zJcCkIcZisfeqjNdEBwX9whvVr+w7wRV0OC42jXCWPyxILUqQEEb7Wkn/idABI59\n" \
"Gb5o9FyUmCJKzIMaFzZWJAJrUcSFxWH0bPTskTqNwrjO6XcJpkwTQ/WO7qAXggnM\n" \
"48kuKqrLTRzfgKmO6W+mYWODpTFd+UQJjEZvCcM5mV5WZNsjeMP+cQ74numdJVWT\n" \
"lknEC5j5L75wE+WmT2KIdoPEQ9rWUREhs9ZWb3lH1uUBYJh0eOovE2MFJmOoQtAC\n" \
"L1+NOzx4oPwEClAZnikVZ3KT2DgC4avx71k5bEU/hpCBVAMJPI1YzWh7E6AFyiLd\n" \
"ADxkyF/tUrtssZNwYeXT66PPT8sb1QbOvYDvngxAhem4V+yke269WENpCcZLLnlV\n" \
"DQOFs7MIddLcHFD9pzJolKtCtMZPdST7VPeqQNPvZHW/n26o6tFBkMFL4bfhMlc=\n" \
"-----END CERTIFICATE-----\n";

const char* esp32_key = \
"-----BEGIN PRIVATE KEY-----\n"
"MIIEvwIBADANBgkqhkiG9w0BAQEFAASCBKkwggSlAgEAAoIBAQC/4pvnRPg65Nh5\n" \
"P+p+qrBsjpZQ0yRTcqFP9sipCixtWDsotF/FJ39JK2WBLPvhTBRpmQJhjU7bPV8A\n" \
"xwQojvWaNfY/XLVWjzEnaNYIHhUrBPVdiv5qE6JkrTy2tu1mSKdXSFBstdmh1Bs1\n" \
"brcanSG7hzaIlw1ZUeIXvhf9x2Mu0Sse9QTWUJvLMjvWKfEFlzyiIKoYFBC8ikSV\n" \
"I0OVyvZtDqQN5MsM7UmxvaM8nizfBzB5t+ew3LWbW0c4oRrEDEhFt6FDLLrGwIGd\n" \
"xsJUeuCqKT6hsZwzVN4C1scqH4TMhPfRUwOdxzMH6PO/Gr/wg/EP4K2pujElufxU\n" \
"iDCpDg77AgMBAAECggEACk4BoFxgVk1W4MAReEbzXYkUwndsUnzr887lfMp6k06z\n" \
"Mp7pK73QxJ2rgFHpnvJGgFtCuPltPDHiCbkmXIU85FC0jjeM136uHZcwM1RmP56v\n" \
"DJx7yDeQt4ZkJc3SFvWc4v+Trhrf+qRY57gv5iJ9HsSGpbQy99KBaVQAltTf2zIx\n" \
"pJty4V9anowLjO1nHu8tST5ESHa2YVWmwOmELJHf80dQjULtNMxGwewFE74cdc+M\n" \
"tDpJE611jfhWKF251mGfAfo3WQRqUrYqMAo1c0zHaoG9WL1Z+RWRpYn1AS86QGOU\n" \
"NA9NFjaqOp/DqgFlBcJ+rbr/DOcmz/cPoLoP3oChmQKBgQDlKViq3ikPXB3E3VvE\n" \
"EtH7eqQDhj8qEn8NQt/avz4N7pSnHNR7RKNlMyZIfk0iX2fRyF7xPub2phnsgCI/\n" \
"+CQLRIdZb41qvKFl8WU9k1xFDZ86cmPuuJ420HICVyIKs43PyrKtuTxXya8GD941\n" \
"IgDabGtq8oX8PAUonjVJ/kpraQKBgQDWW6epvThQr+neqU57MIDK2jMJzaidfJ3r\n" \
"40hQIy0J3pR6FMeKP0CT/vPzEsBQlj5DnTyAB/E4JN62gDSAhXa9RS6VflF+ulRJ\n" \
"X8Jn4H05SSumZCIFBhNgc6G4REMXa6TiJr1Y37kxx6DLemy6lYVZCBPq0c+q69Mw\n" \
"LVhZcTqOwwKBgQCLvNGnp77L8fTpJb4eaweGXLuEtqjvo8W7tWrBfdp+LlyRJqBe\n" \
"5nToce9HR1ULv7eUEaXrX9sAzjqCn4PDFDIOeOQ74i0OyCV2/2Mn0CL0rKDKyBQt\n" \
"7n/zltnemXlVozW3Xrfj/U9RjNdgT+7E4Y45ouFBns+bBpJyuT5vd1Oz+QKBgQDT\n" \
"w6hXX7+Ktr7sYYZO3RPfUsCpJqs1Ki85IkgGIzoTTfiQwoZ+ZQ5/JpgJwrSK6GKK\n" \
"mYNzWGVNed8rnGxBq5gqU3Y56ZjJAXrTIe8EgBo1xbuBndqd6+qolpNlXsqJEKbL\n" \
"ZEoayqpCK10Gp+NSUPRziC9lA/GMgX0ZUzK86cdX0QKBgQChupqqScDyOUlNrAjI\n" \
"eoizjepa0590rxE2w91KoPwwOtJaAQXY0HL8uX4qc+nTLtCPOJUKZu3mdgGVqCXY\n" \
"qsDn5dVF3Bg1ZeprVwlFNtxxLgqUI1JpcPfTRGlrDO5U1oHPiP68oF9xXMHT38i3\n" \
"pd0W8MoEyrYMe+F+xO21aJrNpw==\n"
"-----END PRIVATE KEY-----\n";

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
  myServo.write(180);    // Mover el servo a 0 grados
  delay(1000);
  myServo.write(0);    // Mover el servo a 0 grados
  delay(1000);
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