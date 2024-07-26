// zf240726.2228

// Sources:
// https://microcontrollerslab.com/esp32-htu21d-temperature-humidity-sensor-tutorial/
// https://www.reddit.com/r/esp32/comments/1crwakg/built_in_temperature_sensor_on_esp32c3_red_as/?rdt=63263
// https://github.com/dawidchyrzynski/arduino-home-assistant/blob/main/examples/sensor-integer/sensor-integer.ino


// Temperature sensor HTU21D
#include <Wire.h>
#include "Adafruit_HTU21DF.h"

const int vccPin = 21;       // the number of the VCC pin
const int gndPin = 20;       // the number of the GND pin
const int SdaPin = 10;       // the number of the SDA pin ! the default pin on esp32-c3 super mini is 8 !
const int SclPin = 9;        // the number of the SCL pin ! the default pin on esp32-c3 super mini is 9 !

// Setup a HTU21D instance
Adafruit_HTU21DF htu = Adafruit_HTU21DF();

// Temperature sensor HTU21D initialising
void initHTU21DSensor(){
  pinMode(gndPin, OUTPUT);   // gnd
  digitalWrite(gndPin, LOW);
  pinMode(vccPin, OUTPUT );   // vcc
  digitalWrite(vccPin, HIGH);
    // Start the HTU21D sensor
      Wire.begin(SdaPin,SclPin);
      htu.begin();
}


RTC_DATA_ATTR float tempInternal1 = 0;
RTC_DATA_ATTR float tempInternal2 = 0;


// Lit les senseurs
void readSensor(){
    // lit la température interne
    sensorValue1 = temperatureRead();
    sensorValue1 = sensorValue1 - 7.0;        // Enlève des ° en trop, je ne sais pas pourquoi ? zf240526.1142, zf240530.0908

    // moyenne glissante
    sensorValue1 = (sensorValue1 + tempInternal1 + tempInternal2) / 3;
    tempInternal1 = tempInternal2;
    tempInternal2 = sensorValue1;

    // lit la température et l'humidité du HTU21D
    sensorValue5 = htu.readTemperature();
    sensorValue2 = htu.readHumidity();
}
