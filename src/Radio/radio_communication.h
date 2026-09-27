#pragma once

void radioSetup();

unsigned long getRadioPWM(int ch_num);
//Implémentation des fonctions ISR pour lire les canaux PWM ou PPM selon configuration

void getPPM();

// ISRs de PWM pour chaque canal
/*
  1: Lecture du pin
  2: Stockage du l'instant de RISING edge
  3: Calcul de la valeur channel_x_raw avec l'instant du FALLING edge
*/
void getCh1();

void getCh2();

void getCh3();

void getCh4();

void getCh5();

void getCh6();

void getDesState();