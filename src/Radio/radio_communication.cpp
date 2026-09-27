#include<Arduino.h>
#include "configs/configs.h"
#include "radio_communication.h"

void radioSetup() {
  // Mode recepteur PPM 
  #if defined USE_PPM_RX
    // Déclaration de pin d'interruption
    pinMode(PPM_Pin, INPUT_PULLUP);
    delay(20);
    // On attribue une interruption à la pin, on connecte à son ISR et on met sur RISING egde parce que le PPM se détermine par le RISING edge
    attachInterrupt(digitalPinToInterrupt(PPM_Pin), getPPM, RISING);

  //Mode récepteur PWM
  #elif defined USE_PWM_RX
    // Déclaration de pins d'interruption de chaque canal
    pinMode(ch1Pin, INPUT_PULLUP);
    pinMode(ch2Pin, INPUT_PULLUP);
    pinMode(ch3Pin, INPUT_PULLUP);
    pinMode(ch4Pin, INPUT_PULLUP);
    pinMode(ch5Pin, INPUT_PULLUP);
    pinMode(ch6Pin, INPUT_PULLUP);
    delay(20);
    // On attribue une interruption à chaque pin, on connecte à leur ISR et on met sur CHANGE parce que le PWM se détermine à la fois par le RISING et le FALLING edge
    attachInterrupt(digitalPinToInterrupt(ch1Pin), getCh1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ch2Pin), getCh2, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ch3Pin), getCh3, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ch4Pin), getCh4, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ch5Pin), getCh5, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ch6Pin), getCh6, CHANGE);
    delay(20);
    
  #else
    #error No RX type defined... // Directive préprocesseur qui stoppe la compilation et affiche le message d’erreur qu'il contient
  #endif
}

unsigned long getRadioPWM(int ch_num) {
  // Description: Obtenir les radio commandes actuelles par les ISR
  unsigned long returnPWM = 0;
  
  if (ch_num == 1) {
    returnPWM = channel_1_raw;
  }
  else if (ch_num == 2) {
    returnPWM = channel_2_raw;
  }
  else if (ch_num == 3) {
    returnPWM = channel_3_raw;
  }
  else if (ch_num == 4) {
    returnPWM = channel_4_raw;
  }
  else if (ch_num == 5) {
    returnPWM = channel_5_raw;
  }
  else if (ch_num == 6) {
    returnPWM = channel_6_raw;
  }
  
  return returnPWM;
}


//Implémentation des fonctions ISR pour lire les canaux PWM ou PPM selon configuration

void getPPM(){
  unsigned long now=micros(); // On capture le temps (l'instant) du front actuel pour éviter les petits décallages
  unsigned long dt_ppm; // intervalle entre les fronts

  dt_ppm = now-time_ms; // Calcul de l'intervalle entre les fronts actuel et précédent
  time_ms = now; // On stocke le temps (l'instant) du front actuel

  if (dt_ppm > 5000) ppm_counter = 0; // Longue durée indiquant qu'un frame vient de se terminer, c'est cette durée qui est supérieure à la longueur d'un intervalle entre 2 rising edge qui permet de délimiter les frames
    
  if (ppm_counter == 1) channel_1_raw = dt_ppm; // Première impulsion
  
  if (ppm_counter == 2) channel_2_raw = dt_ppm; // Seconde impulsion
  
  if (ppm_counter == 3) channel_3_raw = dt_ppm; // Troisième impulsion
  
  if (ppm_counter == 4) channel_4_raw = dt_ppm; // Quatrième impulsion
  
  if (ppm_counter == 5) channel_5_raw = dt_ppm; // Cinquième impulsion
  
  if (ppm_counter == 6) channel_6_raw = dt_ppm; // Sixième impulsion
    
  ppm_counter = ppm_counter + 1;
}

// ISRs de PWM pour chaque canal
/*
  1: Lecture du pin
  2: Stockage du l'instant de RISING edge
  3: Calcul de la valeur channel_x_raw avec l'instant du FALLING edge
*/
void getCh1(){
  int trigger = digitalRead(ch1Pin);

  if(trigger == 1) rising_edge_start_1 = micros();

  else if(trigger == 0) channel_1_raw = micros() - rising_edge_start_1;
}

void getCh2() {
  int trigger = digitalRead(ch2Pin);

  if(trigger == 1) rising_edge_start_2 = micros();

  else if(trigger == 0) channel_2_raw = micros() - rising_edge_start_2;
}

void getCh3() {
  int trigger = digitalRead(ch3Pin);

  if(trigger == 1) rising_edge_start_3 = micros();

  else if(trigger == 0) channel_3_raw = micros() - rising_edge_start_3;
}

void getCh4() {
  int trigger = digitalRead(ch4Pin);

  if(trigger == 1) rising_edge_start_4 = micros();

  else if(trigger == 0) channel_4_raw = micros() - rising_edge_start_4;
}

void getCh5() {
  int trigger = digitalRead(ch5Pin);

  if(trigger == 1) rising_edge_start_5 = micros();

  else if(trigger == 0) channel_5_raw = micros() - rising_edge_start_5;
}

void getCh6(){
  int trigger = digitalRead(ch6Pin);

  if(trigger == 1) rising_edge_start_6 = micros();

  else if(trigger == 0) channel_6_raw = micros() - rising_edge_start_6;
}

void getDesState() {
  // DESCRIPTION : Cette fonction génère les variables d’état désirées des roll, des pitch et des yaw à partir des commandes PWM de la radio

  thro_des = (channel_1_pwm - 1000.0)/1000.0; //Between 0 and 1
  roll_des = (channel_2_pwm - 1500.0)/500.0; //Between -1 and 1
  pitch_des = (channel_3_pwm - 1500.0)/500.0; //Between -1 and 1
  yaw_des = (channel_4_pwm - 1500.0)/500.0; //Between -1 and 1
  
  //On limite la valeur aux bornes normalisées
  thro_des = constrain(thro_des, 0.0, 1.0); //Between 0 and 1
  roll_des = constrain(roll_des, -1.0, 1.0)*maxRoll; //Between -maxRoll and +maxRoll
  pitch_des = constrain(pitch_des, -1.0, 1.0)*maxPitch; //Between -maxPitch and +maxPitch
  yaw_des = constrain(yaw_des, -1.0, 1.0)*maxYaw; //Between -maxYaw and +maxYaw

  yaw_des = (1.0-transition_fader)*(yaw_des-ff_roll_yaw_mix_amount*roll_des/maxRoll) + (transition_fader)*(yaw_des); // ff + hover (forward flight combiné avec hover)
  pitch_des = (1.0-transition_fader)*(pitch_des+ff_pitch_trim) + (transition_fader)*(pitch_des); // ff + hover (forward flight combiné avec hover)

}