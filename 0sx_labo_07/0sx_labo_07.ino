#include <LCD_I2C.h>
#include <HCSR04.h>
#include <U8g2lib.h>

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

float minStep;
float maxStep;


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
  lcd.begin();

  minStep = viseur.getMinStep();
  maxStep = viseur.getMaxStep();
  alarm.setDistance(15);
  alarm.setColourA(255, 0, 0);
  alarm.setColourB(0, 0, 255);
  viseur.setAngleMin(10);
  viseur.setAngleMax(170);

  u8g2Setup();
  lcdSetup();

  alarm.turnOn();
  viseur.activer();
}

void loop() {
  // put your main code here, to run repeatedly:
  currentTime = millis();

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

  if (ct - lastTime >= rate) {
    lcd.setCursor(0, 0);
    lcd.print("Dist : ");
    lcd.print(dist);
    lcd.print(" cm  ");
    lcd.setCursor(0, 1);
    lcd.print("Obj  : ");
    lcd.print(viseur.getEtatTexte());

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
