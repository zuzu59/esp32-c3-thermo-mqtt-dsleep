# esp32-c3-thermo-mqtt-dsleep
Juste un test de faire un thermomètre enregistreur très minimaliste avec le capteur de température interne d'un esp32-c3-super-mini

zf240418.1201

## Buts
Simplement faire un thermomètre enregistreur très minimaliste avec le capteur de température interne d'un esp32-c3-super-mini


## Problématiques
Comme le capteur de température se trouve à l'intérieur du esp32-c3, il mesure la température du chip !<br>
Si on veut pouvoir mesurer la température ambiante, il faudra absolument travailler en mode dsleep.

Et quand on travaille en mode dsleep, il faut envoyer le résultat sur un brocker MQTT via WIFI pour le recording !


## Moyens
J'ai utilisé pour ce projet un esp32-c3 super mini

https://grabcad.com/library/esp32-c3-supermini-1

https://fr.aliexpress.com/item/1005006170575141.html

https://www.studiopieters.nl/esp32-c3-pinout/


## Sources
https://www.espboards.dev/blog/esp32-inbuilt-temperature-sensor

https://forum.fritzing.org/t/need-esp32-c3-super-mini-board-model/20561

https://www.digikey.fr/fr/resources/conversion-calculators/conversion-calculator-voltage-divider


