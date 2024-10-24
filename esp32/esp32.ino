#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>

// const char* ssid = "POCO F3"; // Cambié las comillas
// const char* password = "llabakawifi"; // Cambié las comillas

// Sustituir con datos de vuestra red
const char *ssid     = "AEG-IKASLE";
const char *password = "Ea25dneAEG";
const char *mqtt_server = "10.80.128.11";
const int mqtt_port = 1883; //MQTT insecure

WiFiClient espClient;
PubSubClient client(espClient);

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
}