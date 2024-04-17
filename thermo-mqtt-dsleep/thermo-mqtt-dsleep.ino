// Juste un test de faire un thermomètre enregistreur très minimaliste avec le capteur 
// de température interne d'un esp32-c3-super-mini
// Envoie aussi le résultat des senseurs sur le mqtt pour home assistant (pas en fonction actuellement !)
// ATTENTION, ce code a été testé sur un esp32-c3 super mini. Pas testé sur les autres bords !
//
#define zVERSION "zf240417.0044"


// il faut ajouter dans le mqtt le nombre de boot !


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
// https://randomnerdtutorials.com/esp32-deep-sleep-arduino-ide-wake-up-sources/
// https://randomnerdtutorials.com/esp32-useful-wi-fi-functions-arduino
//


// #define DEBUG true
// #undef DEBUG



// General
const int ledPin = 8;    // the number of the LED pin
const int buttonPin = 9;  // the number of the pushbutton pin
int sensorPin = A0;   // select the input pin for battery meter
float sensorValue1 = 0;  // variable to store the value coming from the sensor 1
float sensorValue2 = 0;  // variable to store the value coming from the sensor 2
float sensorValue3 = 0;  // variable to store the value coming from the sensor 3
float sensorValue4 = 0;  // variable to store the value coming from the sensor 3


// Deep Sleep
#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  300      /* Time ESP32 will go to sleep (in seconds) */
RTC_DATA_ATTR int bootCount = 0;


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


    sensorValue3 = WiFi.RSSI();
    USBSerial.print("RRSI: ");
    USBSerial.println(sensorValue3);


}


// MQTT
#include <ArduinoHA.h>
#define DEVICE_NAME      "thi1"
#define SENSOR_NAME1     "Temperature"
#define SENSOR_NAME2     "Battery"
#define SENSOR_NAME3     "RSSI"
#define SENSOR_NAME4     "bootCount"

WiFiClient client;
HADevice device(DEVICE_NAME);                // c'est le ID du device, il doit être unique !
HAMqtt mqtt(client, device);
unsigned long lastUpdateAt = 0;

// c'est le ID du sensor, il doit être unique !
HASensorNumber Sensor1(DEVICE_NAME SENSOR_NAME1, HASensorNumber::PrecisionP1);   // c'est le nom du sensor sur MQTT ! (PrecisionP1=x.1, PrecisionP2=x.01, ...)
HASensorNumber Sensor2(DEVICE_NAME SENSOR_NAME2, HASensorNumber::PrecisionP2);   // c'est le nom du sensor sur MQTT ! (PrecisionP1=x.1, PrecisionP2=x.01, ...)
HASensorNumber Sensor3(DEVICE_NAME SENSOR_NAME3);   // c'est le nom du sensor sur MQTT !
HASensorNumber Sensor4(DEVICE_NAME SENSOR_NAME4);   // c'est le nom du sensor sur MQTT !

static void ConnectMQTT() {
    device.setName(DEVICE_NAME);                // c'est le nom du device sur Home Assistant !
    mqtt.setDataPrefix(DEVICE_NAME);             // c'est le nom du device sur MQTT !
    device.setSoftwareVersion(zVERSION);
    device.setManufacturer("espressif");
    device.setModel("esp32-c3 super mini");

    Sensor1.setIcon("mdi:thermometer");
    Sensor1.setName(SENSOR_NAME1);           // c'est le nom du sensor sur Home Assistant !
    Sensor1.setUnitOfMeasurement("°C");

    Sensor2.setIcon("mdi:battery-charging-wireless-outline");
    Sensor2.setName(SENSOR_NAME2);           // c'est le nom du sensor sur Home Assistant !
    Sensor2.setUnitOfMeasurement("V");

    Sensor3.setIcon("mdi:wifi-strength-1");
    Sensor3.setName(SENSOR_NAME3);           // c'est le nom du sensor sur Home Assistant !
    Sensor3.setUnitOfMeasurement("dBm");

    Sensor4.setIcon("mdi:counter");
    Sensor4.setName(SENSOR_NAME4);           // c'est le nom du sensor sur Home Assistant !
    Sensor4.setUnitOfMeasurement("sum");

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
    Sensor3.setValue(sensorValue3);
    Sensor4.setValue(sensorValue4);
}


void setup() {
    pinMode(ledPin, OUTPUT);
    digitalWrite(ledPin, LOW);

    // Il faut lire la température tout de suite au début avant que le MCU ne puisse chauffer !
    initTempSensor();
    readSensor();

    USBSerial.begin(19200);
    USBSerial.setDebugOutput(true);       //pour voir les messages de debug des libs sur la console série !
    // delay(3000);  //le temps de passer sur la Serial Monitor ;-)
    USBSerial.println("\n\n\n\n**************************************\nCa commence !\n");

    //Increment boot number and print it every reboot
    ++bootCount;
    sensorValue4 = bootCount;
    USBSerial.println("Boot number: " + String(bootCount));

    // First we configure the wake up source
    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
    USBSerial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) + " Seconds");

    // digitalWrite(ledPin, HIGH);
    USBSerial.println("Connect WIFI !");
    ConnectWiFi();
    // digitalWrite(ledPin, LOW);
    delay(200); 

    USBSerial.println("\n\nConnect MQTT !\n");
    ConnectMQTT();

    sendSensorMqtt();
    USBSerial.printf("sensor1: %f,sensor2: %f\n", sensorValue1, sensorValue2);
    USBSerial.println("\nC'est envoyé !\n");

    USBSerial.println("Going to sleep now");
    delay(200);
    USBSerial.flush(); 
    esp_deep_sleep_start();
    USBSerial.println("This will never be printed");
}


void loop() {
}

