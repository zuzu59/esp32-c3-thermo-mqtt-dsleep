//
// WIFI
//
// zf241104.1758
//
// Sources:
// https://randomnerdtutorials.com/esp32-useful-wi-fi-functions-arduino
// https://dronebotworkshop.com/wifimanager/


// Choix de la connexion WIFI, qu'une seule possibilité !
// #define zWifiNormal true
#define zWifiAuto true
// #define zWifiManager true        // plus testé !  zf240910.1512



// References:
//
// Available ESP32 RF power parameters: 
// 19.5dBm WIFI_POWER_19dBm (19.5dBm output, highest supply current ~150mA)
// 19dBm WIFI_POWER_18_5dBm 
// 18.5dBm WIFI_POWER_17dBm 
// 17dBm WIFI_POWER_15dBm 
// 15dBm WIFI_POWER_13dBm 
// 13dBm WIFI_POWER_11dBm 
// 11dBm WIFI_POWER_8_5dBm 
// 8dBm WIFI_POWER_7dBm 
// 7dBm WIFI_POWER_5dBm 
// 5dBm WIFI_POWER_2dBm 
// 2dBm WIFI_POWER_MINUS_1dBm (-1dBm (For -1dBm output, lowest supply current ~120mA)


// Général
const long zIntervalzWifi_Check_Connection =   60000;             // Interval en mili secondes pour le check de la connexion WIFI
unsigned long zPrevious_MilliszWifi_Check_Connection = 0;       // Compteur de temps pour le check de la connexion WIFI
float rrsiLevel = 0;      // variable to store the RRSI level
int watchCount = 0;
IPAddress zSubnet(255, 255, 255, 0);


// WIFI
#include <WiFi.h>
#include <HTTPClient.h>
#include <ESPmDNS.h>

#define yWIFI_SSID WIFI_SSID3
#define yWIFI_PASSWORD WIFI_PASSWORD3

WiFiClient client;
HTTPClient http;


void zWifiTrouble(){
  Serial.println("\nOn a un problème avec le WIFI !");
  delay(200);
  Serial.flush(); 
  WiFi.disconnect();
  if (zDSLEEP == 1){
    // On part en dsleep pour économiser la batterie !
    esp_deep_sleep_start();
  }
  // Power off the ESP32-CAM
  Serial.println("\nOn fait un power OFF de la caméra !\n");
  pinMode(32, OUTPUT);
  digitalWrite(32, HIGH);
  delay(2000); // Wait for 2 seconds
  esp_restart();
}


void zWifiBegin(const char* zWIFI_SSID, const char* zWIFI_PASSWORD){
#ifdef zIpStatic
  WiFi.config(zLocal_IP, zGateway, zSubnet);
#else
  WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);
#endif
  WiFi.setHostname(zHOST);
  WiFi.persistent(false);           // pour ne pas user l'EEPROM !
  Serial.print("Connecting on ");
  Serial.print(zWIFI_SSID);
  WiFi.begin(zWIFI_SSID, zWIFI_PASSWORD);
#ifdef lowTxPower
  WiFi.setTxPower(WIFI_POWER_8_5dBm);  // diminution de la puissance à cause de la réflexion de l'antenne sur le HTU21D directement soudé sur le esp32-c3 super mini zf240725.1800
#endif  
  int connAttempts = 0;
  while (WiFi.status() != WL_CONNECTED && connAttempts < 60) {
    delay(500);
    Serial.print(".");
    connAttempts++;
  }
  Serial.println("");
  if (WiFi.status() == WL_CONNECTED) {
  } else {
    Serial.println("Failed to connect");
    zWifiTrouble();
  }
}


#ifdef zWifiAuto
  #include <vector>
  struct WifiCredentials {
    String ssid;
    String password;
  };
  std::vector<WifiCredentials> wifi_creds;

  void connectToBestWifi() {
      // Ajouter vos informations d'identification Wi-Fi qui se trouvent dans secrets.h autant de fois que vous le souhaitez
    WifiCredentials creds1 = {WIFI_SSID1, WIFI_PASSWORD1};
    wifi_creds.push_back(creds1);
    WifiCredentials creds2 = {WIFI_SSID2, WIFI_PASSWORD2};
    wifi_creds.push_back(creds2);
    WifiCredentials creds3 = {WIFI_SSID3, WIFI_PASSWORD3};
    wifi_creds.push_back(creds3);
    WifiCredentials creds4 = {WIFI_SSID4, WIFI_PASSWORD4};
    wifi_creds.push_back(creds4);
    WifiCredentials creds5 = {WIFI_SSID5, WIFI_PASSWORD5};
    wifi_creds.push_back(creds5);
    WifiCredentials creds6 = {WIFI_SSID6, WIFI_PASSWORD6};
    wifi_creds.push_back(creds6);
    WifiCredentials creds7 = {WIFI_SSID7, WIFI_PASSWORD7};
    wifi_creds.push_back(creds7);
    WifiCredentials creds8 = {WIFI_SSID8, WIFI_PASSWORD8};
    wifi_creds.push_back(creds8);
    WifiCredentials creds9 = {WIFI_SSID9, WIFI_PASSWORD9};
    wifi_creds.push_back(creds9);
    WifiCredentials creds10 = {WIFI_SSID10, WIFI_PASSWORD10};
    wifi_creds.push_back(creds10);

    int best_rssi = -1000;
    String best_ssid;
    String best_password;
    // Scanner les réseaux Wi-Fi
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);
     Serial.println("On scanne les AP WIFI...");
    int n = WiFi.scanNetworks();
    Serial.print("Number SSID scanned: ");
    Serial.println(n);
    for (int i = 0; i < n; i++) {
      Serial.print("SSID scanned: ");
      Serial.println(WiFi.SSID(i));
      Serial.print("RSSI: ");
      Serial.println(WiFi.RSSI(i));
      for (const auto &cred : wifi_creds) {
        if (WiFi.SSID(i) == cred.ssid) {
          int rssi = WiFi.RSSI(i);
          if (rssi > best_rssi) {
            best_rssi = rssi;
            best_ssid = cred.ssid;
            best_password = cred.password;
          }
        }
      }
    }
    // Se connecter au réseau Wi-Fi avec le meilleur RSSI
    if (!best_ssid.isEmpty()) {
      zWifiBegin(best_ssid.c_str(), best_password.c_str());
      // WiFi.begin(best_ssid.c_str(), best_password.c_str());

      // WiFi.setTxPower(WIFI_POWER_8_5dBm);  // diminution de la puissance à cause de la réflexion de l'antenne sur le HTU21D directement soudé sur le esp32-c3 super mini zf240725.1800

      // Serial.print("Connecting to ");
      // Serial.println(best_ssid);
      // int connAttempts = 0;
      // while (WiFi.status() != WL_CONNECTED && connAttempts < 60) {
      //   delay(500);
      //   Serial.print(".");
      //   connAttempts++;
      // }
      // Serial.println("");
      // if (WiFi.status() == WL_CONNECTED) {
      // } else {
      //   Serial.println("Failed to connect");
      //   zWifiTrouble();
      // }
    } else {
      Serial.println("No known networks found");
      zWifiTrouble();
    }
  }
#endif


#ifdef zWifiManager
  #include <WiFiManager.h>

  static void ConnectWiFi() {
    WiFi.mode(WIFI_STA);
    Serial.println("Connexion en WIFI Manager");
    // si le bouton FLASH de l'esp32-c3 est appuyé dans les 3 secondes après le boot, la config WIFI sera effacée !
    pinMode(buttonPin, INPUT_PULLUP);
    if ( digitalRead(buttonPin) == LOW) {
      WiFiManager wm; wm.resetSettings();
      Serial.println("Config WIFI effacée !"); delay(1000);

      // Power off the ESP32-CAM
      Serial.println("\nOn fait un power OFF de la caméra !\n");
      digitalWrite(32, HIGH);
      delay(2000); // Wait for 2 seconds

      // ESP.restart();
      esp_restart();

    }
    WiFiManager wm;
    bool res;
    res = wm.autoConnect("esp32_wifi_config",""); // pas de password pour l'ap esp32_wifi_config
    if(!res) {
      zWifiTrouble();
    }
  }
#endif


#ifdef zWifiNormal
  static void ConnectWiFi() {
  Serial.println("Connexion en WIFI Normal avec secrets.h");
  Serial.printf("WIFI_SSID: %s\nWIFI_PASSWORD: %s\n", WIFI_SSID, WIFI_PASSWORD);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  zWifiBegin(yWIFI_SSID, yWIFI_PASSWORD);



  // WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  // // WiFi.setTxPower(WIFI_POWER_8_5dBm);  //c'est pour le Lolin esp32-c3 mini V1 ! https://www.wemos.cc/en/latest/c3/c3_mini_1_0_0.html
  // Serial.println("Connecting");
  // long zWifiTiemeout = 10000 + millis(); 
  // while(WiFi.status() != WL_CONNECTED){
  //   Serial.print("."); delay(100);
  //   if(millis() > zWifiTiemeout ){
  //     zWifiTrouble();
  //   }
  // }
}
#endif


// start mDNS
void zStartmDNS(){
  /*use mdns for host name resolution*/
  if (!MDNS.begin(zHOST)) {         //http://xxx.local
    Serial.println("Error setting up MDNS responder!");
    while (1) {
      delay(1000);
    }
  }
  Serial.println("mDNS responder started");
}


// Browse les services mDNS
void browseService(const char *service, const char *proto) {
  Serial.printf("Browsing for service _%s._%s.local. ... ", service, proto);
  int n = MDNS.queryService(service, proto);
  if (n == 0) {
    Serial.println("no services found");
  } else {
    Serial.print(n);
    Serial.println(" service(s) found");
    for (int i = 0; i < n; ++i) {
      // Print details for each service found
      Serial.print("  ");
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.print(MDNS.hostname(i));
      Serial.print(" (");
      Serial.print(MDNS.address(i));
      Serial.print(":");
      Serial.print(MDNS.port(i));
      Serial.println(")");
    }
  }
  Serial.println();
}


// Scanne les services mDNS
void zScanServices() {
  Serial.println("zScanServices !");
  browseService("http", "tcp");
  delay(1000);
  browseService("arduino", "tcp");
  delay(1000);
  browseService("workstation", "tcp");
  delay(1000);
  browseService("smb", "tcp");
  delay(1000);
  browseService("afpovertcp", "tcp");
  delay(1000);
  browseService("ftp", "tcp");
  delay(1000);
  browseService("ipp", "tcp");
  delay(1000);
  browseService("printer", "tcp");
  delay(10000);
}


// start WIFI
void zStartWifi(){
  digitalWrite(ledPin, HIGH);
  #ifdef zWifiNormal
    ConnectWiFi();
  #endif
  #ifdef zWifiManager
    ConnectWiFi();
  #endif
  #ifdef zWifiAuto
    connectToBestWifi();
  #endif
  Serial.println("\nConnecté au réseau WiFi !");
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  int zRSSI = WiFi.RSSI();
  Serial.print("RSSI: ");
  Serial.println(zRSSI);

  int txPower = WiFi.getTxPower();
  Serial.print("TX power: ");
  Serial.println(txPower);  

  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
  digitalWrite(ledPin, LOW);

  zStartmDNS();
}


// Check for WIFI
void zWifi_Check_Connection(){
  unsigned long currentMillis = millis();
  if(currentMillis - zPrevious_MilliszWifi_Check_Connection >= zIntervalzWifi_Check_Connection || zPrevious_MilliszWifi_Check_Connection > currentMillis){
    zPrevious_MilliszWifi_Check_Connection = currentMillis;
    Serial.println("\nzWifi_Check_Connection !");

    // Vérifie si on est toujours connecté au WIFI ?
    if(WiFi.status() != WL_CONNECTED){
      // Wifi disconnected
      Serial.println("WIFI Disconnected !");
      ESP.restart();
    }

    zStartmDNS();

    // // Scanne les services mDNS
    // zScanServices();
  }
}

