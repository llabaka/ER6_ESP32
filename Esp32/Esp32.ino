#include <WiFi.h>

const char* ssid = "POCO F3";
const char* password = "llabakawifi";

// AEG IKASLE WIFI (STATIC MAC)
// const char* ssid = "AEG-IKASLE";
// const char* password = "Wb_AEG_A3";

void setup(){
    Serial.begin(115200);
    delay(1000);

    WiFi.mode(WIFI_STA); //Optional
    WiFi.begin(ssid, password);
    Serial.println("\nConnecting");

    while(WiFi.status() != WL_CONNECTED){
        Serial.print(".");
        delay(100);
    }

    Serial.println("\nConnected to the WiFi network");
    Serial.print("Local ESP32 IP: ");
    Serial.println(WiFi.localIP());
}

void loop(){}