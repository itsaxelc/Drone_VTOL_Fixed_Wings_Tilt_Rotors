#include<Arduino.h>
#include "configs/configs.h"

void failSafe() {
  //DESCRIPTION: Si la radio fournit des valeurs invalides (bruit/valeurs corrompues), cette fonction remet toutes les commandes à leurs valeurs par défaut

  unsigned long static last_valid_signal = 0;
  #define HOLD_DURATION 5000000 // 5 secondes en µs
  unsigned minVal = 1000;
  unsigned maxVal = 2000;
  int check1 = 0;
  int check2 = 0;
  int check3 = 0;
  int check4 = 0;
  int check5 = 0;
  int check6 = 0;

  //Déclencheurs des critères de défaillance
  if (channel_1_pwm > maxVal || channel_1_pwm < minVal) check1 = 1;
  if (channel_2_pwm > maxVal || channel_2_pwm < minVal) check2 = 1;
  if (channel_3_pwm > maxVal || channel_3_pwm < minVal) check3 = 1;
  if (channel_4_pwm > maxVal || channel_4_pwm < minVal) check4 = 1;
  if (channel_5_pwm > maxVal || channel_5_pwm < minVal) check5 = 1;
  if (channel_6_pwm > maxVal || channel_6_pwm < minVal) check6 = 1;

  if (!(check1 + check2 + check3 + check4 + check5 + check6)){
    last_valid_signal = micros(); // signal OK, on mémorise
  }
  //En cas de problème, mettre les valeurs de sécurité
  else if (micros() - last_valid_signal > HOLD_DURATION){
    // Signal perdu depuis plus de 2s → coupure
    channel_1_pwm = channel_1_fs;
    channel_2_pwm = channel_2_fs;
    channel_3_pwm = channel_3_fs;
    channel_4_pwm = channel_4_fs;
    channel_5_pwm = channel_5_fs;
    channel_6_pwm = channel_6_fs;
  }
  // Sinon : on garde les dernières valeurs valides → Hold
}