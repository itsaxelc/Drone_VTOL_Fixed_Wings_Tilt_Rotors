#include<Arduino.h>
#include "configs/configs.h"

void loopRate(int freq) {
  //DESCRIPTION: Cette fonction régule la fréquence de la boucle principale à la fréquence spécifiée (Ici c'est 2KHz)

  float invFreq = 1.0/freq*1000000.0;
  unsigned long checker = micros();
  
  //Sit in loop until appropriate time has passed
  while (invFreq > (checker - current_time)) {
    checker = micros();
  }
}

void loopBlink() {
  //DESCRIPTION: cette fonction fait clignotter la led builtin pour indiquer que la boucle loop() tourne

  if (current_time - blink_counter > blink_delay) {
    blink_counter = micros();
    digitalWrite(13, blinkAlternate); //Pin 13 est la broche de la LED
    
    if (blinkAlternate == 1) {
      blinkAlternate = 0;
      blink_delay = 100000;
      }
    else if (blinkAlternate == 0) {
      blinkAlternate = 1;
      blink_delay = 2000000;
      }
  }
}