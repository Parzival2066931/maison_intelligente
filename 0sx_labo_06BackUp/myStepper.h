#include <Arduino.h>
#include <AccelStepper.h>

#define MOTOR_INTERFACE_TYPE 4
#define IN_1 5
#define IN_2 6
#define IN_3 7
#define IN_4 8

class Moteur {
  private:
    AccelStepper myStepper(MOTOR_INTERFACE_TYPE, IN_1, IN_3, IN_2, IN_4);
    int maxSpeed;
    int acceleration;
    int speed;

  public:
    Moteur() {
      maxSpeed = 500;
      acceleration = 200;
      speed = 200;
    }

    void setup(int maxSpeed, int acceleration, int speed) {
      this->maxSpeed = maxSpeed;
      this->acceleration = acceleration;
      this->speed = speed;

      myStepper.setMaxSpeed(this->maxSpeed);
      myStepper.setSpeed(this->speed);
      myStepper.setAcceleration(this->acceleration);
    }
    void run() {
      myStepper.run();
    }
    bool destination() {
      if(myStepper.distanceToGo() == 0) {
        return true;
      }
      else {
        return false;
      }
    }
    

};