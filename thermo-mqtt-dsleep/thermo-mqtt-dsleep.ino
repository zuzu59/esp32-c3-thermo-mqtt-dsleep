// Juste un test de faire un thermomètre enregistreur très minimaliste avec le capteur 
// de température interne d'un esp32-c3-super-mini
// Envoie aussi le résultat des senseurs sur le mqtt pour home assistant (pas en fonction actuellement !)
// ATTENTION, ce code a été testé sur un esp32-c3 super mini. Pas testé sur les autres bords !
//
// zf240417.1554
//
// Utilisation:
//
// Installation:
// 
// Il faut disabled USB CDC On Boot et utiliser USBSerial. au lieu de Serial. pour la console !
//
// Il faut installer cette lib pour le senseur de température interne
// https://github.com/espressif/esp-idf/blob/master/components/driver/test_apps/legacy_rtc_temp_driver/main/test_rtc_temp_driver.c

// Pour MQTT, il faut installer la lib (home-assistant-integration):
// https://github.com/dawidchyrzynski/arduino-home-assistant
//
// Sources:
// https://www.espboards.dev/blog/esp32-inbuilt-temperature-sensor
// https://forum.fritzing.org/t/need-esp32-c3-super-mini-board-model/20561
// https://www.digikey.fr/fr/resources/conversion-calculators/conversion-calculator-voltage-divider
// https://raw.githubusercontent.com/zuzu59/esp32-c3-thermo-mqtt-dsleep/master/fonction_conversion_ADC.txt
//


// #define DEBUG true
// #undef DEBUG



// General
const int ledPin = 8;    // the number of the LED pin
const int buttonPin = 9;  // the number of the pushbutton pin
int sensorPin = A0;   // select the input pin for battery meter
// int sensorValue = 0;  // variable to store the value coming from the sensor



float sensorValue1 = 0;  // variable to store the value coming from the sensor 1
float sensorValue2 = 0;  // variable to store the value coming from the sensor 2
#define TEMP_CELSIUS 0


// Temperature sensor internal
#include "driver/temp_sensor.h"


// WIFI
#include <WiFi.h>
#include "secrets.h"

static void ConnectWiFi() {
    USBSerial.printf("WIFI_SSID: %s\nWIFI_PASSWORD: %s\n", WIFI_SSID, WIFI_PASSWORD);
    WiFi.mode(WIFI_STA); //Optional
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    WiFi.setTxPower(WIFI_POWER_8_5dBm);  //c'est pour le Lolin esp32-c3 mini V1 ! https://www.wemos.cc/en/latest/c3/c3_mini_1_0_0.html
    int txPower = WiFi.getTxPower();
    USBSerial.print("TX power: ");
    USBSerial.println(txPower);
    USBSerial.println("Connecting");
    while(WiFi.status() != WL_CONNECTED){
        USBSerial.print(".");
        delay(100);
    }
    USBSerial.println("\nConnected to the WiFi network");
    USBSerial.print("Local ESP32 IP: ");
    USBSerial.println(WiFi.localIP());
}


// MQTT
#include <ArduinoHA.h>
#define DEVICE_NAME     "Ti1"
#define SENSOR_NAME1     "Temperature"
#define SENSOR_NAME2     "Battery"

#define PUBLISH_INTERVAL  1000 // how often image should be published to HA (milliseconds)

WiFiClient client;
HADevice device(DEVICE_NAME);                // c'est le IDS du device, il doit être unique !
HAMqtt mqtt(client, device);
unsigned long lastUpdateAt = 0;

// You should define your own ID.
HASensorNumber Sensor1(SENSOR_NAME1, HASensorNumber::PrecisionP1);   // c'est le nom du sensor sur MQTT ! (PrecisionP1=x.1, PrecisionP2=x.01, ...)
HASensorNumber Sensor2(SENSOR_NAME2, HASensorNumber::PrecisionP2);   // c'est le nom du sensor sur MQTT ! (PrecisionP1=x.1, PrecisionP2=x.01, ...)

static void ConnectMQTT() {
   device.setName(DEVICE_NAME);                // c'est le nom du device sur Home Assistant !
    // device.setSoftwareVersion("1.0.0");
    mqtt.setDataPrefix(DEVICE_NAME);             // c'est le nom du device sur MQTT !

    Sensor1.setIcon("mdi:thermometer");
    Sensor1.setName(SENSOR_NAME1);           // c'est le nom du sensor sur Home Assistant !
    Sensor1.setUnitOfMeasurement("°");

    Sensor2.setIcon("mdi:battery-charging-wireless-outline");
    Sensor2.setName(SENSOR_NAME2);           // c'est le nom du sensor sur Home Assistant !
    Sensor2.setUnitOfMeasurement("V");

    mqtt.begin(BROKER_ADDR, BROKER_USERNAME, BROKER_PASSWORD);
    USBSerial.println("MQTT connected");
}


// Redirection de la console
#define CONSOLE(...)                                                                                                   \
  do                                                                                                                   \
  {                                                                                                                    \
    USBSerial.print(__VA_ARGS__);                                                                                         \
  } while (0)

#define CONSOLELN(...)                                                                                                 \
  do                                                                                                                   \
  {                                                                                                                    \
    USBSerial.println(__VA_ARGS__);                                                                                       \
  } while (0)

#define CONSOLEF(...)                                                                                                 \
  do                                                                                                                   \
  {                                                                                                                    \
    USBSerial.printf(__VA_ARGS__);                                                                                       \
  } while (0)




// Temperature sensor internal initialising
void initTempSensor(){
    temp_sensor_config_t temp_sensor = TSENS_CONFIG_DEFAULT();
    temp_sensor.dac_offset = TSENS_DAC_L2;  // TSENS_DAC_L2 is default; L4(-40°C ~ 20°C), L2(-10°C ~ 80°C), L1(20°C ~ 100°C), L0(50°C ~ 125°C)
    temp_sensor_set_config(temp_sensor);
    temp_sensor_start();
}


// Lit les senseurs
void readSensor(){
    temp_sensor_read_celsius(&sensorValue1);
    // fonction de conversion bit to volts de l'ADC avec le diviseur résistif et de la diode !
    // voir https://raw.githubusercontent.com/zuzu59/esp32-c3-thermo-mqtt-dsleep/master/fonction_conversion_ADC.txt
    // 0.001034 * (ADC - 2380) + 3.6
    uint16_t reading = analogRead(sensorPin);
    sensorValue2 = 0.001034 * (reading - 2380) + 3.6;            // 2960 pour 4.2V et 2380 pour 3.6V
}


// Envoie les senseurs au mqtt
void sendSensorMqtt(){
    mqtt.loop();
    Sensor1.setValue(sensorValue1);
    Sensor2.setValue(sensorValue2);
}



void setup() {
    USBSerial.begin(19200);
    USBSerial.setDebugOutput(true);       //pour voir les messages de debug des libs sur la console série !
    delay(3000);  //le temps de passer sur la Serial Monitor ;-)
    USBSerial.println("\n\n\n\n**************************************\nCa commence !\n");

    pinMode(ledPin, OUTPUT);
    digitalWrite(ledPin, LOW);
    delay(500); 
    digitalWrite(ledPin, HIGH);

    // Temperature sensor internal initialise
    initTempSensor();





    USBSerial.println("Connect WIFI !");
    ConnectWiFi();
    digitalWrite(ledPin, HIGH);
    delay(500); 

    USBSerial.println("\n\nConnect MQTT !\n");
    ConnectMQTT();

    USBSerial.println("\nC'est parti !\n");
}


void loop() {
    digitalWrite(ledPin, LOW);
    delay(100); 
    digitalWrite(ledPin, HIGH);

    readSensor();
    sendSensorMqtt();

    USBSerial.printf("sensor1:%f,sensor2:%f\n", sensorValue1, sensorValue2);
    delay(PUBLISH_INTERVAL);
}

