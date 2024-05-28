// Petit thermomètre enregistreur avec deux sondes de température, une interne au esp32-c3 
// et une externe avec un 1-wire DS18B20 avec un esp32-c3-super-mini
//
// ATTENTION, ce code a été testé sur un esp32-c3. Pas testé sur les autres boards !
//
#define zVERSION  "zf240528.2354"
#define zHOST     "thi4"            // ATTENTION, tout en minuscule !

/*
Utilisation:

Astuce:

Afin d'économiser la résistance et de simplifier au maximum le câblage, 
j'utilise la pull up interne de l'esp32-c3 comme pull up et les deux pins 
adjacentes pour la masse et l'alimentation pour le DS18B20. Il faudra 
donc configurer ces pins en conséquence !

Installation:

Pour les esp32-c3 super mini, il faut:
 * choisir comme board ESP32C3 Dev Module
 * disabled USB CDC On Boot et utiliser USBSerial. au lieu de Serial. pour la console !
 * changer le schéma de la partition à Minimal SPIFFS (1.9MB APP with OTA/190kB SPIFFS)

Pour le WiFiManager, il faut installer cette lib depuis le lib manager sur Arduino:
https://github.com/tzapu/WiFiManager

Pour MQTT, il faut installer la lib (home-assistant-integration):
https://github.com/dawidchyrzynski/arduino-home-assistant

Pour JSON, il faut installer cette lib:
https://github.com/bblanchon/ArduinoJson

Sources:
https://www.espboards.dev/blog/esp32-inbuilt-temperature-sensor
https://forum.fritzing.org/t/need-esp32-c3-super-mini-board-model/20561
https://www.aliexpress.com/item/1005006005040320.html
https://randomnerdtutorials.com/esp32-useful-wi-fi-functions-arduino
https://dronebotworkshop.com/wifimanager/
https://lastminuteengineers.com/esp32-ota-web-updater-arduino-ide/
https://github.com/dawidchyrzynski/arduino-home-assistant/blob/main/examples/sensor-integer/sensor-integer.ino
https://chat.mistral.ai/    pour toute la partie API REST et wifiAuto ᕗ
*/




// #define DEBUG true
// #undef DEBUG



// General
const int ledPin = 8;             // the number of the LED pin
const int buttonPin = 9;          // the number of the pushbutton pin
int zDelay1Interval = 60000;       // Délais en mili secondes pour le zDelay1


// Sonar Pulse
#include "zSonarpulse.h"


// WIFI
#include "zWifi.h"


// OTA WEB server
#include "otaWebServer.h"


// MQTT
#include "zMqtt.h"


// Temperature sensor
#include "zTemperature.h"


// Deep Sleep
#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
// #define TIME_TO_SLEEP  300      /* Time ESP32 will go to sleep (in seconds) */
#define TIME_TO_SLEEP  300      /* Time ESP32 will go to sleep (in seconds) */
RTC_DATA_ATTR int bootCount = 0;


void setup() {
  // Pulse deux fois pour dire que l'on démarre
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW); delay(zSonarPulseOn); digitalWrite(ledPin, HIGH); delay(zSonarPulseOff);
  digitalWrite(ledPin, LOW); delay(zSonarPulseOn); digitalWrite(ledPin, HIGH); delay(zSonarPulseOff);
  delay(zSonarPulseWait);

  // Il faut lire la température tout de suite au début avant que le MCU ne puisse chauffer !
  // initTempSensor();
  initTempSensor();
  initDS18B20Sensor();
  delay(200);
  readSensor();

  // start serial console
  USBSerial.begin(19200);
  USBSerial.setDebugOutput(true);       //pour voir les messages de debug des libs sur la console série !
  delay(3000);                          //le temps de passer sur la Serial Monitor ;-)
  USBSerial.println("\n\n\n\n**************************************\nCa commence !"); USBSerial.println(zHOST ", " zVERSION);

  //Increment boot number and print it every reboot
  ++bootCount;
  sensorValue4 = bootCount;
  USBSerial.println("Boot number: " + String(bootCount));

  // First we configure the wake up source
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  USBSerial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) + " Seconds");

  // start WIFI
  zStartWifi();
  sensorValue3 = WiFi.RSSI();

  // start OTA server
  otaWebServer();

  // Connexion au MQTT
  USBSerial.println("\n\nConnect MQTT !\n");
  ConnectMQTT();

  // go go go
  USBSerial.println("\nC'est parti !\n");

  // Envoie toute la sauce !
  zEnvoieTouteLaSauce();
  USBSerial.println("\nC'est envoyé !\n");

  // // On va dormir !
  // USBSerial.println("Going to sleep now");
  // delay(200);
  // USBSerial.flush(); 
  // esp_deep_sleep_start();
  // USBSerial.println("This will never be printed");
}


void loop() {
  // Envoie toute la sauce !
  zEnvoieTouteLaSauce();

  // Délais non bloquant pour le sonarpulse et l'OTA
  zDelay1(zDelay1Interval);
}


// Envoie toute la sauce !
void zEnvoieTouteLaSauce(){

  // Lit les températures
  readSensor();

  // Envoie les mesures au MQTT
  sendSensorMqtt();

  // Graphe sur l'Arduino IDE les courbes des mesures
  USBSerial.print("sensor1:");
  USBSerial.print(sensorValue1);
  USBSerial.print(",sensor2:");
  USBSerial.print(sensorValue2);
  USBSerial.print(",sensor3:");
  USBSerial.print(sensorValue3);
  USBSerial.print(",sensor4:");
  USBSerial.print(sensorValue4);
  USBSerial.print(",sensor5:");
  USBSerial.println(sensorValue5);
}


// Délais non bloquant pour le sonarpulse et l'OTA
void zDelay1(long zDelayMili){
  long zDelay1NextMillis = zDelayMili + millis(); 
  while(millis() < zDelay1NextMillis ){
    // OTA loop
    server.handleClient();
    // Un petit coup sonar pulse sur la LED pour dire que tout fonctionne bien
    sonarPulse();
  }
}

