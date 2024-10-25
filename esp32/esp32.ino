#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <MFRC522.h>

// const char* ssid = "POCO F3"; // Cambié las comillas
// const char* password = "llabakawifi"; // Cambié las comillas

// PIN MFRC522
#define SS_PIN 5
#define RST_PIN 22


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
}

void connectToMQTT() {
  while (!client.connected()) {
    Serial.print("Intentando conectar al broker MQTT...");
    
    // Intentar conectarse
    if (client.connect("LANDER_ESP32")) { // "ESP32Client" puede ser cualquier ID de cliente único
      Serial.println("Conectado al broker MQTT");
      
      client.subscribe("test");
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

  // Halt PICC
  rfid.PICC_HaltA();

  // Stop encryption on PCD
  rfid.PCD_StopCrypto1();

}