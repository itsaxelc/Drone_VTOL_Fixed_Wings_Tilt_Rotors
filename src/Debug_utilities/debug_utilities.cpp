#include <Arduino.h>
#include "../configs/configs.h"
#include "debug_utilities.h"
// F("") : macro Arduino qui stocke une chaîne de caractères constante en mémoire Flash plutôt qu'en RAM, permettant de garder la RAM libre pour les variables importantes

void printRadioData() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F(" CH1 (us): "));
    Serial.print(channel_1_pwm);
    Serial.print(F(" CH2 (us): "));
    Serial.print(channel_2_pwm);
    Serial.print(F(" CH3 (us): "));
    Serial.print(channel_3_pwm);
    Serial.print(F(" CH4 (us): "));
    Serial.print(channel_4_pwm);
    Serial.print(F(" CH5 (us): "));
    Serial.print(channel_5_pwm);
    Serial.print(F(" CH6 (us): "));
    Serial.println(channel_6_pwm);
  }
}

// Commandes utilisateur normalisées
void printDesiredState() {
  if (current_time - print_counter > 10000){
    print_counter = micros();
    Serial.print(F("thro_des (between 0 and 1): "));
    Serial.print(thro_des);
    Serial.print(F(" roll_des (between 0 and 1): "));
    Serial.print(roll_des);
    Serial.print(F(" pitch_des (between 0 and 1): "));
    Serial.print(pitch_des);
    Serial.print(F(" yaw_des (between 0 and 1): "));
    Serial.println(yaw_des);
  }
}

void printGyroData() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("GyroX (deg/s): "));
    Serial.print(GyroX);
    Serial.print(F(" GyroY (deg/s): "));
    Serial.print(GyroY);
    Serial.print(F(" GyroZ (deg/s): "));
    Serial.println(GyroZ);
  }
}

void printAccelData() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("AccX (en g): "));
    Serial.print(AccX);
    Serial.print(F(" AccY (en g): "));
    Serial.print(AccY);
    Serial.print(F(" AccZ (en g): "));
    Serial.println(AccZ);
  }
}

void printMagData(){
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("MagX (uT): "));
    Serial.print(MagX);
    Serial.print(F(" MagY (uT): "));
    Serial.print(MagY);
    Serial.print(F(" MagZ (uT): "));
    Serial.println(MagZ);
  }
}

void printRollPitchYaw() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("roll (deg): "));
    Serial.print(roll_IMU);
    Serial.print(F(" pitch (deg): "));
    Serial.print(pitch_IMU);
    Serial.print(F(" yaw (deg): "));
    Serial.println(yaw_IMU);
  }
}

void printPIDoutput() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("roll_PID: "));
    Serial.print(roll_PID);
    Serial.print(F(" pitch_PID: "));
    Serial.print(pitch_PID);
    Serial.print(F(" yaw_PID: "));
    Serial.println(yaw_PID);
  }
}

void printMotorCommands() {
  // Commandes ESC OneShot125 avec une largeur d’impulsion comprise entre 125us et 250 us
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("m1_command (us): "));
    Serial.print(m1_command_PWM);
    Serial.print(F(" m2_command (us): "));
    Serial.print(m2_command_PWM);
  }
}

void printServoCommands() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F(" s1_command (deg): "));
    Serial.print(s1_command_PWM);
    Serial.print(F(" s2_command (deg): "));
    Serial.print(s2_command_PWM);
  }
}

void printLoopRate() {
  if(current_time-print_counter > 10000){
    print_counter = micros();
    Serial.print(F("dt = "));
    Serial.println(dt*1000000.0);
  }
}