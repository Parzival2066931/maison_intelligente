#include <LCD_I2C.h>
#include <AccelStepper.h>
#include <HCSR04.h>

#include "led_rgb.h"
#define NB_PIN 3

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


enum Distance {TROP_TROP_PRES, TROP_PRES, NORMAL, TROP_LOIN};
enum Couleur {ROUGE, BLEU, BLANC};

Distance distance = NORMAL;
Couleur couleur[3] = {ROUGE, BLEU, BLANC};

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  lcd.begin();

  for(int i = 0; i < NB_PIN; i++) {
    pinMode(led.GetPin(i), OUTPUT);
  }

  myStepper.setMaxSpeed(maxSpeed);
  myStepper.setSpeed(speed);
  myStepper.setAcceleration(acceleration);

  minStep = minAngle/360.0 * stepPerTurn;
  maxStep = maxAngle/360.0 * stepPerTurn;

  lcdSetup();
}

void loop() {
  // put your main code here, to run repeatedly:
  currentTime = millis();
  


  lcdTask(currentTime);

  getDistTask(currentTime);

  stateManager();

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
  
  if(dist < alertDist) {
    distance = TROP_TROP_PRES;

  }
  else if(dist < minDist) {
    distance = TROP_PRES;

  }
  else if(dist > maxDist) {
    distance = TROP_LOIN;

  }
  else {
    distance = NORMAL;

  }

  switch(distance) {

    case TROP_TROP_PRES:
      tooTooCloseState(currentTime);
      break;

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
  
}
void getDistTask(unsigned long ct) {
  static unsigned long lastTime = 0;
  int rate = 100;

  if(ct - lastTime >= rate) {

    dist = hc.dist();

    lastTime = ct;
  }
}
void lcdTask(unsigned long ct) {
  static int count = 0;
  static unsigned long lastTime = 0;
  int rate = 100;

  if(ct - lastTime >= rate) {
    lcd.setCursor(0, 0);
    lcd.print("Dist : ");
    lcd.print(dist);
    lcd.print(" cm  ");
    lcd.setCursor(0, 1);
    lcd.print("Obj  : ");

    lastTime = ct;
  }
}
Led colorManager(Couleur couleur[], int i) {

  if(couleur[i] == ROUGE) {
    led.SetRgb(255, 0, 0);
  }
  else if(couleur[i] == BLEU) {
    led.SetRgb(0, 0, 255);
  }
  else if(couleur[i] == BLANC) {
    led.SetRgb(255, 255, 255);
  }
  return led.SetRgb(led.GetRed(), led.GetGreen(), led.GetBlue());//Pas le droit à revoir!
}
void ledTask(unsigned long ct) {
  static unsigned long lastTime = 0;
  int rate = 1000;
  static int compteur = 0;
  

  if(ct - lastTime >= rate) {
    
    compteur++;
    if(compteur == 3) {
      compteur = 0;
    }
  }
  analogWrite(led.GetPin(compteur), colorManager(couleur, compteur));
}
void buzzerTask() {

}
void tooTooCloseState(unsigned long ct) {
  ledTask(currentTime);
  buzzerTask();

  lcd.setCursor(6, 1);
  lcd.print(" ALERTE   ");
}
void tooCloseState(unsigned long ct) {
  static unsigned long lastTime = 0;
  int rate = 100;
  int minPosition = map(minDist, minDist, maxDist, minStep, maxStep);

  if(ct - lastTime >= rate) {

    lcd.setCursor(6, 1);
    lcd.print(" trop pres");

    
    myStepper.moveTo(minPosition);

    if(myStepper.distanceToGo() == 0) {
      myStepper.disableOutputs();

    }

    lastTime = ct;
  }
  
  
}
void tooFarState(unsigned long ct) {
  static unsigned long lastTime = 0;
  int rate = 100;
  int maxPosition = map(maxDist, minDist, maxDist, minStep, maxStep);


  if(ct - lastTime >= rate) {

    lcd.setCursor(6, 1);
    lcd.print(" trop loin");

    myStepper.moveTo(maxPosition);
    
    if(myStepper.distanceToGo() == 0) {
      myStepper.disableOutputs();

    }

    lastTime = ct;
  }
}
void normalState(unsigned long ct) {
  static unsigned long lastTime = 0;
  int rate = 100;

  pointeurTask(ct);

  if(ct - lastTime >= rate) {

    lcd.print(currentPosition); //position en degrés
    lcd.print(" deg    ");

    lastTime = ct;
  }
}
void pointeurTask(unsigned long ct) {
  static unsigned long lastTime = 0;
  int rate = 100;
  
  
  static int stepsToMove = 0;

  if(ct - lastTime >= rate) {
    currentPosition = map(dist, minDist, maxDist, minAngle, maxAngle);
    stepsToMove = map(dist, minDist, maxDist, minStep, maxStep);

    lastTime = ct;
  }

  if(myStepper.distanceToGo() == 0) {
    myStepper.disableOutputs();
  }

  myStepper.moveTo(stepsToMove);
  
}
void serialTask(unsigned long ct) {

  static unsigned long lastTime = 0;
  int rate = 100;

  if(ct - lastTime >= rate) {

    Serial.print("etd:2066931,dist:");
    Serial.print(dist);
    Serial.print(",deg:");
    Serial.println(currentPosition);

    lastTime = ct;
  }
  

  

}

