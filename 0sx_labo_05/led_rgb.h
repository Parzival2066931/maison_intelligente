#include <Arduino.h>

#define NB_PIN 3
#define FIRST_PIN 44


class Led {

  private:

    unsigned int r;
    unsigned int g;
    unsigned int b;
    int pin[3];
  
  public:

    Led() {
      r = 0;
      g = 0;
      b = 0;
      
      for(int i = 0; i < NB_PIN; i++) {
        pin[i] = FIRST_PIN + i;
      }
    }

    Led(unsigned int r, unsigned int g, unsigned int b) {
      this->r = r;
      this->g = g;
      this->b = b;

      for(int i = 0; i < NB_PIN; i++) {
        pin[i] = FIRST_PIN + i;
      }
    }

    void SetRgb(unsigned int r, unsigned int g, unsigned int b) {
      this->r = r;
      this->g = g;
      this->b = b;
    }

    unsigned int GetRed() {return r;}
    unsigned int GetGreen() {return g;}
    unsigned int GetBlue() {return b;}


    int GetPin(int i) {return pin[i];}

};