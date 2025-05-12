#include <LCD_I2C.h>
#include <HCSR04.h>
#include <U8g2lib.h>
#include <WiFiEspAT.h>
#include <PubSubClient.h>

#include "Alarme.h"
#include "ViseurAutomatique.h"


#define RED_PIN 44
#define GREEN_PIN 45
#define BLUE_PIN 46
#define BUZZER_PIN 3

#define TRIGGER_PIN 9
#define ECHO_PIN 10

#define IN_1 5
#define IN_2 6
#define IN_3 7
#define IN_4 8

#define CLK_PIN 30
#define DIN_PIN 34
#define CS_PIN 32

#define LIGHT_PIN A1

// Initialisation du client MQTT
#define DEVICE_NAME "2066931"
#define MQTT_PORT 1883
#define MQTT_USER "etdshawi"
#define MQTT_PASS "shawi123"
#define HAS_SECRETS 0
#define AT_BAUD_RATE 115200

#if HAS_SECRETS
#include "arduino_secret.h"
/////// SVP par soucis de sécurité, mettez vos informations dans le fichier arduino_secrets.h

// Nom et mot de passe du réseau wifi
const char ssid[] = SECRET_SSID;
const char pass[] = SECRET_PASS;

#else
const char ssid[] = "TechniquesInformatique-Etudiant";  // your network SSID (name)
const char pass[] = "shawi123";                         // your network password (use for WPA, or use as key for WEP)

#endif


HCSR04 hc(TRIGGER_PIN, ECHO_PIN);
LCD_I2C lcd(0x27, 16, 2);

U8G2_MAX7219_8X8_F_4W_SW_SPI u8g2(
  U8G2_R0,        // Rotation
  CLK_PIN,        // clock
  DIN_PIN,        // data
  CS_PIN,         // cs
  U8X8_PIN_NONE,  // dc
  U8X8_PIN_NONE   // reset
);

//Variables
unsigned long currentTime = 0;

float dist = 0;

Alarm alarm(RED_PIN, GREEN_PIN, BLUE_PIN, BUZZER_PIN, dist);
ViseurAutomatique viseur(IN_1, IN_2, IN_3, IN_4, dist);

WiFiClient espClient;
PubSubClient client(espClient);


const char* mqttServer = "216.128.180.194";
int blinkRate = 500;
int status = WL_IDLE_STATUS;  // Status du module wifi
int connectionAttemps = 0;

char ligne[2][16] = { "                ", "                " };


float minLum = 1023;
float maxLum = 0;
unsigned long timer = 0;
bool tempoDrawing = false;

enum CommandeState { WAITING,
                     VALID,
                     INVALID,
                     IMPOSSIBLE };
CommandeState commandState = WAITING;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while (!Serial)
    ;
  pinMode(LED_BUILTIN, OUTPUT);

  MQTTSetup();

  lcd.begin();

  alarm.setDistance(15);
  alarm.setColourA(255, 0, 0);
  alarm.setColourB(0, 0, 255);
  viseur.setAngleMin(10);
  viseur.setAngleMax(170);

  u8g2Setup();
  lcdSetup();

  alarm.turnOn();

}

void loop() {
  // put your main code here, to run repeatedly:
  currentTime = millis();

  client.loop();

  periodicTask();
  periodicTaskLCD();

  lcdTask(currentTime);
  getDistTask(currentTime);

  alarm.update();
  viseur.update();

  commandStateManager();
}
void lcdSetup() {
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("2066931");
  lcd.setCursor(0, 1);
  lcd.print("Hope 4 the best");
  delay(2000);
  lcd.clear();
}

void getDistTask(unsigned long ct) {
  static unsigned long lastTime = 0;
  int rate = 100;
  static int lastDist = 0;

  if (ct - lastTime >= rate) {

    dist = hc.dist();
    lastTime = ct;

    if (dist > 0) {
      lastDist = dist;
    } else {
      dist = lastDist;
    }
  }
}
void lcdTask(unsigned long ct) {
  static unsigned long lastTime = 0;
  int rate = 100;
  char strDist[10];

  dtostrf(dist, 6, 2, strDist);

  snprintf(ligne[0], sizeof(ligne[0]), "Dist: %s cm", strDist);
  snprintf(ligne[1], sizeof(ligne[1]), "Obj : %-10s", viseur.getEtatTexte());

  if (ct - lastTime >= rate) {

    lcd.setCursor(0, 0);
    lcd.print(ligne[0]);

    lcd.setCursor(0, 1);
    lcd.print(ligne[1]);
    lastTime = ct;
  }
}

void serialEvent() {

  String tampon = Serial.readStringUntil('\n');

  Serial.println("Réception : " + tampon);

  String commande;
  String arg1, arg2;

  analyserCommande(tampon, commande, arg1, arg2);
  processCommandTask(commande, arg1, arg2);
}
void analyserCommande(const String& tampon, String& commande, String& arg1, String& arg2) {
  commande = "";
  arg1 = "";
  arg2 = "";

  int firstSep = tampon.indexOf(';');
  int secondSep = tampon.indexOf(';', firstSep + 1);

  if (firstSep == -1) {
    // Pas de point-virgule, c'est peut-être "stop" ou autre commande sans paramètre
    commande = tampon;
    return;
  }

  // Extraire la commande
  commande = tampon.substring(0, firstSep);

  // Extraire arg1
  if (secondSep != -1) {
    arg1 = tampon.substring(firstSep + 1, secondSep);

    arg2 = tampon.substring(secondSep + 1);
  } else {
    // Il y a une seule valeur après la commande
    arg1 = tampon.substring(firstSep + 1);
  }
}
void processCommandTask(String commande, String arg1, String arg2) {

  if (commande == "cfg") {

    if (arg1 == "alm") {


      commandState = VALID;
      alarm.setDistance(arg2.toInt());
      Serial.print("Distance d'alarme configurée à : ");
      Serial.println(alarm.getDistance());


    } else if (arg1 == "lim_inf") {

      if (arg2.toInt() > viseur.getDistanceMaxSuivi()) {

        Serial.println("La distance minimale ne peut pas être plus grande que la distance maximale");
        commandState = IMPOSSIBLE;

      } else {

        commandState = VALID;
        viseur.setDistanceMinSuivi(arg2.toInt());
        Serial.print("Distance minimale du moteur configurée à : ");
        Serial.println(viseur.getDistanceMinSuivi());
      }
    }



    else if (arg1 == "lim_sup") {
      if (arg2.toInt() < viseur.getDistanceMinSuivi()) {


        Serial.println("La distance maximale ne peut pas être plus petite que la distance minimale");
        commandState = IMPOSSIBLE;

      } else {

        commandState = VALID;
        viseur.setDistanceMaxSuivi(arg2.toInt());
        Serial.print("Distance maximale du moteur configurée à : ");
        Serial.println(viseur.getDistanceMaxSuivi());
      }
    } else {
      Serial.println("Commande reçu non valide");
      commandState = INVALID;
    }
  }

  else if (commande == "g_dist" || commande == "gDist") {

    commandState = VALID;
    Serial.print("Distance de l'objet : ");
    Serial.println(dist);

  } else {

    Serial.println("Commande reçu non valide");
    commandState = INVALID;
  }
}

void u8g2Setup() {
  u8g2.begin();

  u8g2.setContrast(15);
  u8g2.setFont(u8g2_font_4x6_tr);

  u8g2.clearBuffer();
  u8g2.sendBuffer();
}
void drawCommandValid() {


  u8g2.clearBuffer();

  u8g2.drawLine(6, 7, 1, 2);
  u8g2.drawLine(2, 1, 3, 0);


  u8g2.sendBuffer();
}
void drawCommandImpossible() {


  u8g2.clearBuffer();

  u8g2.drawCircle(4, 3, 3, U8G2_DRAW_ALL);
  u8g2.drawLine(2, 1, 6, 5);

  u8g2.sendBuffer();

  if (currentTime >= timer) {


    commandState = WAITING;
  }
}
void drawCommandInvalid() {
  static unsigned long lastTime = 0;
  int rate = 3000;

  u8g2.clearBuffer();

  u8g2.drawLine(0, 0, 7, 7);
  u8g2.drawLine(7, 0, 0, 7);

  u8g2.sendBuffer();

  if (currentTime >= timer) {


    commandState = WAITING;
  }
}
void commandStateManager() {

  switch (commandState) {
    case WAITING:
      u8g2.clearBuffer();
      u8g2.sendBuffer();
      timer = currentTime + 3000;
      break;
    case VALID:
      timer = currentTime + 3000;
      drawCommandValid();
      break;
    case INVALID:
      drawCommandInvalid();
      break;
    case IMPOSSIBLE:
      drawCommandImpossible();
      break;
  }
}
void MQTTSetup() {


  wifiInit();
  client.setServer(mqttServer, MQTT_PORT);
  client.setCallback(mqttEvent);

  if (!client.connect(DEVICE_NAME, MQTT_USER, MQTT_PASS)) {
    Serial.println("Incapable de se connecter sur le serveur MQTT");
    Serial.print("client.state : ");
    Serial.println(client.state());
  } else {
    Serial.println("Connecté sur le serveur MQTT");
  }


  client.subscribe("etd/18/motor", 0);
  if (client.subscribe("etd/18/motor", 0)) {
    Serial.println("Abonnement au client 'motor' réussi");
  } else {
    Serial.println("Erreur d'abonnement au topic 'motor'.");
  }

  client.subscribe("etd/18/color", 0);
  if (client.subscribe("etd/18/color", 0)) {
    Serial.println("Abonnement au client 'color' réussi");
  } else {
    Serial.println("Erreur d'abonnement au topic 'color'.");
  }
}
void wifiInit() {
  Serial1.begin(AT_BAUD_RATE);
  WiFi.init(&Serial1);

  Serial.println();
  Serial.print("Tentative de connexion à SSID: ");
  Serial.println(ssid);

  while (status != WL_CONNECTED && connectionAttemps < 5) {
    Serial.print(".");

    // Connecter au ssid
    status = WiFi.begin(ssid, pass);
    connectionAttemps++;
  }
  if (status == WL_CONNECTED) {
    Serial.println();
    Serial.println("Connecté au réseau WiFi.");
    printWifiStatus();
  } else {
    WiFi.disconnect();
    Serial.println();
    Serial.println("La connexion au réseau WiFi a échoué.");
    blinkRate = 100;
  }
  // Cela peut prendre un certain temps pour que le module wifi soit prêt
  // Voir 1 minute dans la documentation
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println();
    Serial.println("La communication avec le module WiFi a échoué!");
    // ne pas continuer
    errorState(2, 1);
  }
}
void printWifiStatus() {

  // imprime le SSID du réseau auquel vous êtes connecté:
  char ssid[33];
  WiFi.SSID(ssid);
  Serial.print("SSID: ");
  Serial.println(ssid);

  // imprime le BSSID du réseau auquel vous êtes connecté:
  uint8_t bssid[6];
  WiFi.BSSID(bssid);
  Serial.print("BSSID: ");
  printMacAddress(bssid);

  uint8_t mac[6];
  WiFi.macAddress(mac);
  Serial.print("MAC: ");
  printMacAddress(mac);

  // imprime l'adresse IP de votre carte:
  IPAddress ip = WiFi.localIP();
  Serial.print("Adresse IP: ");
  Serial.println(ip);

  // imprime la force du signal reçu:
  long rssi = WiFi.RSSI();
  Serial.print("force du signal (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}


void printMacAddress(byte mac[]) {
  for (int i = 5; i >= 0; i--) {
    if (mac[i] < 16) {
      Serial.print("0");
    }
    Serial.print(mac[i], HEX);
    if (i > 0) {
      Serial.print(":");
    }
  }
  Serial.println();
}

void errorState(int codeA, int codeB) {
  const int rate = 100;
  const int pauseBetween = 500;
  const int pauseAfter = 1000;

  // On ne sort jamais de cette fonction
  while (true) {
    for (int i = 0; i < codeA; i++) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(rate);
      digitalWrite(LED_BUILTIN, LOW);
      delay(rate);
    }
    delay(pauseBetween);
    for (int i = 0; i < codeB; i++) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(rate);
      digitalWrite(LED_BUILTIN, LOW);
      delay(rate);
    }
    delay(pauseAfter);

    Serial.print("Erreur : ");
    Serial.print(codeA);
    Serial.print(".");
    Serial.println(codeB);
  }
}
void mqttEvent(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message recu [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  // On traite les messages reçus
  char message[length + 1];
  memcpy(message, payload, length);
  message[length] = '\0';

  if (strcmp(topic, "etd/18/motor") == 0) {
    toggleMotor(message);
  }
  if (strcmp(topic, "etd/18/color") == 0) {
    changeLedColor(message);
  }
}
void periodicTask() {
  static unsigned long lastTime = 0;
  static char message[200] = "";
  static char szLum[6];
  static char szDist[10];
  static char szAngle[10];
  const unsigned int rate = 2500;

  static float lum = 0;


  if (currentTime - lastTime < rate) return;

  lastTime = currentTime;

  // On lit la température et l'humidité
  lum = analogRead(LIGHT_PIN);
  if (lum > maxLum) {
    maxLum = lum;
  } else if (lum < minLum) {
    minLum = lum;
  }
  float lumMap = mapFloat(lum, minLum, maxLum, 0, 100);


  // On convertit les valeurs en chaîne de caractères
  dtostrf(lumMap, 4, 1, szLum);
  dtostrf(dist, 4, 1, szDist);
  dtostrf(viseur.getAngle(), 4, 1, szAngle);

  // On construit le message
  if (viseur.getEtatTexte() == "INACTIF") {
    sprintf(message, "{\"number\":%s, \"name\":%s, \"lum\":%s, \"uptime\":%lu, \"dist\":%s}", "\"2066931\"", "\"Tomy Béland\"", szLum, currentTime / 1000, szDist);
  } else {
    sprintf(message, "{\"number\":%s, \"name\":%s, \"lum\":%s, \"uptime\":%lu, \"dist\":%s, \"angle\":%s }", "\"2066931\"", "\"Tomy Béland\"", szLum, currentTime / 1000, szDist, szAngle);
  }

  Serial.print("Envoie : ");
  Serial.println(message);

  // On publie le message
  if (!client.publish("etd/18/data", message)) {
    reconnect();
    Serial.println("Incapable d'envoyer le message!");
  } else {
    Serial.println("Message envoyé");
  }
}
void periodicTaskLCD() {
  static unsigned long lastTime = 0;
  const unsigned int rate = 1100;
  static char message[200] = "";

  sprintf(message, "{\"line1\":\"%s\", \"line2\":\"%s\" }", ligne[0], ligne[1]);


  if (currentTime - lastTime >= rate) {
    // On publie le message
    Serial.print("Envoie : ");
    Serial.println(message);

    if (!client.publish("etd/18/data", message)) {
      reconnect();
      Serial.println("Incapable d'envoyer le message!");
    } else {
      Serial.println("Message envoyé");
    }
    lastTime = currentTime;
  }
}
bool reconnect() {
  bool result = client.connect(DEVICE_NAME, MQTT_USER, MQTT_PASS);
  if (!result) {
    Serial.println("Incapable de se connecter sur le serveur MQTT");
  }
  return result;
}

void toggleMotor(char* message) {
  String strMessage = String(message);
  strMessage.trim();

  int firstStep = strMessage.indexOf(':');
  int lastStep = strMessage.indexOf('}');

  String strMotor = strMessage.substring(firstStep + 1, lastStep);
  strMotor.trim();
  int motor = strMotor.toInt();

  if (motor == 1) {
    viseur.activer();
  } else if (motor == 0) {
    viseur.desactiver();
  }
}
void changeLedColor(char* message) {
  String strRGB = String(message);
  Serial.println("Je change la couleur");

  int firstStep = strRGB.indexOf('#');

  strRGB = strRGB.substring(firstStep + 1);

  Serial.println(strRGB);

  String strRed = strRGB.substring(0, 2);
  int r = hex2rgb(strRed);
  String strGreen = strRGB.substring(2, 4);
  int g = hex2rgb(strGreen);
  String strBlue = strRGB.substring(4, 6);
  int b = hex2rgb(strBlue);

  alarm.setColourA(r, g, b);
}
int hex2rgb(String hex) {
  return (int)strtol(hex.c_str(), NULL, 16);
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
