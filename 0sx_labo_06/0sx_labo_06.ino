#include <LCD_I2C.h>
#include <AccelStepper.h>
#include <HCSR04.h>
#include <U8g2lib.h>

#include "led_rgb.h"

#define BUZZER_PIN 3

#define TRIGGER_PIN 9
#define ECHO_PIN 10

#define MOTOR_INTERFACE_TYPE 4
#define IN_1 5
#define IN_2 6
#define IN_3 7
#define IN_4 8

#define CLK_PIN 30
#define DIN_PIN 34
#define CS_PIN 32

HCSR04 hc(TRIGGER_PIN, ECHO_PIN);
AccelStepper myStepper(MOTOR_INTERFACE_TYPE, IN_1, IN_3, IN_2, IN_4);
LCD_I2C lcd(0x27, 16, 2);
Led led;
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
unsigned long previousTime = 0;
int deltaTime = 0;

int dist = 0;
const int stepPerTurn = 2038;
int currentPosition = 0;

const int maxSpeed = 500;
const int speed = 500;
const int acceleration = 200;

int alertDist = 15;
int minDist = 30;
int maxDist = 60;
int minAngle = 10;
int maxAngle = 170;
float minStep;
float maxStep;

unsigned long lastTimeObjectDetected = 0;
unsigned long timer = 0;


bool tempoDrawing = false;

enum DistanceState { TROP_PRES,
                     NORMAL,
                     TROP_LOIN };
enum Alert { ON,
             OFF };
enum Couleur { ROUGE,
               BLEU,
               BLANC };
enum CommandeState { WAITING,
                     VALID,
                     INVALID,
                     IMPOSSIBLE };

DistanceState distanceState = NORMAL;
Couleur couleur[3] = { ROUGE, BLEU, BLANC };
Alert alert = OFF;
CommandeState commandState = WAITING;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);


  lcd.begin();

  u8g2Setup();
  led.setup();
  pinMode(BUZZER_PIN, OUTPUT);

  myStepper.setMaxSpeed(maxSpeed);
  myStepper.setSpeed(speed);
  myStepper.setAcceleration(acceleration);

  minStep = minAngle / 360.0 * stepPerTurn;
  maxStep = maxAngle / 360.0 * stepPerTurn;

  lcdSetup();
}

void loop() {
  // put your main code here, to run repeatedly:
  currentTime = millis();

  lcdTask(currentTime);

  getDistTask(currentTime);

  stateManager();
  alertStateManager();
  commandStateManager();

  //serialTask(currentTime);

  myStepper.run();
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
void stateManager() {

  switch (distanceState) {

    case TROP_PRES:
      tooCloseState(currentTime);
      break;

    case NORMAL:
      normalState(currentTime);
      break;

    case TROP_LOIN:
      tooFarState(currentTime);
      break;
  }

  if (dist < minDist) {
    distanceState = TROP_PRES;

  } else if (dist > maxDist) {
    distanceState = TROP_LOIN;

  } else {
    distanceState = NORMAL;
  }
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

    lastTime = ct;
  }
}

void alertStateManager() {

  switch (alert) {
    case ON:
      alertOnState(currentTime);
      break;
    case OFF:
      alertOffState();
      break;
  }
}
void colorManager(Couleur couleur[], int compteur) {
  const int ledOn = 255;
  const int ledOff = 0;
  const int ledWhite = 127;


  if (couleur[compteur] == ROUGE) {
    led.SetRgb(ledOn, ledOff, ledOff);
  } else if (couleur[compteur] == BLEU) {
    led.SetRgb(ledOff, ledOff, ledOn);
  } else if (couleur[compteur] == BLANC) {
    led.SetRgb(ledWhite, ledWhite, ledWhite);
  }
}

void ledTask(unsigned long ct) {
  static unsigned long lastTime = 0;
  int rate = 200;
  static int compteur = 0;
  colorManager(couleur, compteur);

  if (ct - lastTime >= rate) {

    compteur++;
    if (compteur == 3) {
      compteur = 0;
    }
    lastTime = ct;
  }
}
void ledOffTask() {
  const int ledOff = 0;

  led.SetRgb(ledOff, ledOff, ledOff);
}
void buzzerTask() {
  int frequency = 500;

  tone(BUZZER_PIN, frequency);
}
void buzzerOffTask() {

  noTone(BUZZER_PIN);
}

void alertOnState(unsigned long ct) {

  int rate = 3000;
  bool transition = dist > alertDist;

  if (transition) {
    if ((ct - lastTimeObjectDetected >= rate)) {

      alert = OFF;
    }
  } else {
    lastTimeObjectDetected = ct;
  }

  ledTask(ct);
  buzzerTask();
}
void alertOffState() {
  bool transition = dist < alertDist;

  if (transition) {
    alert = ON;
  }
  ledOffTask();
  buzzerOffTask();
}

void tooCloseState(unsigned long ct) {
  static unsigned long lastTime = 0;
  int rate = 100;
  int minPosition = map(minDist, minDist, maxDist, minStep, maxStep);

  if (ct - lastTime >= rate) {

    lcd.setCursor(6, 1);
    lcd.print(" trop pres");

    myStepper.moveTo(minPosition);

    if (myStepper.distanceToGo() == 0) {
      myStepper.disableOutputs();
    }

    lastTime = ct;
  }
}
void tooFarState(unsigned long ct) {
  static unsigned long lastTime = 0;
  int rate = 100;
  int maxPosition = map(maxDist, minDist, maxDist, minStep, maxStep);


  if (ct - lastTime >= rate) {

    lcd.setCursor(6, 1);
    lcd.print(" trop loin");

    myStepper.moveTo(maxPosition);

    if (myStepper.distanceToGo() == 0) {
      myStepper.disableOutputs();
    }

    lastTime = ct;
  }
}
void normalState(unsigned long ct) {
  static unsigned long lastTime = 0;
  int rate = 100;

  pointeurTask(ct);

  if (ct - lastTime >= rate) {

    lcd.print(currentPosition);  //position en degrés
    lcd.print(" deg    ");

    lastTime = ct;
  }
}

void pointeurTask(unsigned long ct) {
  static unsigned long lastTime = 0;
  int rate = 100;


  static int stepsToMove = 0;

  if (ct - lastTime >= rate) {
    currentPosition = map(dist, minDist, maxDist, minAngle, maxAngle);
    stepsToMove = map(dist, minDist, maxDist, minStep, maxStep);

    lastTime = ct;
  }

  if (myStepper.distanceToGo() == 0) {
    myStepper.disableOutputs();
  }

  myStepper.moveTo(stepsToMove);
}
void serialTask(unsigned long ct) {

  static unsigned long lastTime = 0;
  int rate = 100;

  if (ct - lastTime >= rate) {

    Serial.print("etd:2066931,dist:");
    Serial.print(dist);
    Serial.print(",deg:");
    Serial.println(currentPosition);

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
      alertDist = arg2.toInt();
      Serial.print("Distance d'alarme configurée à : ");
      Serial.println(alertDist);


    } else if (arg1 == "lim_inf") {

      if (arg2.toInt() > maxDist) {

        Serial.println("La distance minimale ne peut pas être plus grande que la distance maximale");
        commandState = IMPOSSIBLE;

      } else {

        commandState = VALID;
        minDist = arg2.toInt();
        Serial.print("Distance minimale du moteur configurée à : ");
        Serial.println(minDist);
      }
    }



    else if (arg1 == "lim_sup") {
      if (arg2.toInt() < minDist) {


        Serial.println("La distance maximale ne peut pas être plus petite que la distance minimale");
        commandState = IMPOSSIBLE;

      } else {

        commandState = VALID;
        maxDist = arg2.toInt();
        Serial.print("Distance maximale du moteur configurée à : ");
        Serial.println(maxDist);
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
