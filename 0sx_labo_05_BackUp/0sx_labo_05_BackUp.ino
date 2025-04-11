#include <LCD_I2C.h>
#include <AccelStepper.h>
#include <HCSR04.h>

#include "led_rgb.h"

#define BUZZER_PIN 3

#define TRIGGER_PIN 9
#define ECHO_PIN 10

#define MOTOR_INTERFACE_TYPE 4
#define IN_1 5
#define IN_2 6
#define IN_3 7
#define IN_4 8

HCSR04 hc(TRIGGER_PIN, ECHO_PIN);
AccelStepper myStepper(MOTOR_INTERFACE_TYPE, IN_1, IN_3, IN_2, IN_4);
LCD_I2C lcd(0x27, 16, 2);
Led led;

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

const int alertDist = 15;
int minDist = 30;
int maxDist = 60;
int minAngle = 10;
int maxAngle = 170;
float minStep;
float maxStep;

unsigned long lastTimeObjectDetected = 0;

enum DistanceState { TROP_PRES,
                NORMAL,
                TROP_LOIN };
enum Alert {ON, OFF};
enum Couleur { ROUGE,
               BLEU,
               BLANC };

DistanceState distanceState = NORMAL;
Couleur couleur[3] = { ROUGE, BLEU, BLANC };
Alert alert = OFF;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  lcd.begin();

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

  serialTask(currentTime);

  myStepper.run();
}
void lcdSetup() {
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("2066931");
  lcd.setCursor(0, 1);
  lcd.print("Labo 4B");
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

    if(dist > 0) {
      lastDist = dist;
    } 
    else {
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
  }
  else {
    lcd.setCursor(6, 1);
    lcd.print(" ALERTE   ");
    lastTimeObjectDetected = ct;
  }

  ledTask(ct);
  buzzerTask();

  
}
void alertOffState() {
  bool transition = dist < alertDist;

  if(transition) {
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
