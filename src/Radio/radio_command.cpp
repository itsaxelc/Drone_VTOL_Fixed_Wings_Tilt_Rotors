#include<Arduino.h>
#include "configs/configs.h"
#include "radio_communication.h"


void scaleCommands() {
  //DESCRIPTION: Fonction de mise à l’échelle des commandes d’actionneurs normalisées vers les valeurs du protocole ESC/servo.

  // Mis à l’échelle entre 125 µs et 250 µs pour le protocole OneShot125
  m1_command_PWM = m1_command_scaled*125 + 125;
  m2_command_PWM = m2_command_scaled*125 + 125;

  // Contraindre les commandes des moteurs dans les limites du protocole OneShot125
  m1_command_PWM = constrain(m1_command_PWM, 125, 250);
  m2_command_PWM = constrain(m2_command_PWM, 125, 250);

  // Mis à l’échelle entre 0 et 180 pour les servos
  s1_command_PWM = s1_command_scaled*180;
  s2_command_PWM = s2_command_scaled*180;
  s3_command_PWM = s3_command_scaled*180;
  
  // Contraindre les commandes des servos dans les limites de la bibliothèque Servo
  s1_command_PWM = constrain(s1_command_PWM, 0, 180);
  s2_command_PWM = constrain(s2_command_PWM, 0, 180);
  s3_command_PWM = constrain(s3_command_PWM, 0, 180);

}

void getCommands() {
  //DESCRIPTION : Cette fonction récupère les commandes radio les plus récentes sous forme de signaux PWM par canal X, où X correspond au numéro du canal
  #if defined USE_PPM_RX || defined USE_PWM_RX
    channel_1_pwm = getRadioPWM(1);
    channel_2_pwm = getRadioPWM(2);
    channel_3_pwm = getRadioPWM(3);
    channel_4_pwm = getRadioPWM(4);
    channel_5_pwm = getRadioPWM(5);
    channel_6_pwm = getRadioPWM(6);
  #endif
  
  // On filtre passe-bas les commandes critiques et on met à jour les valeurs précédentes
  float b = 0.7; //Lower=slower, higher=noiser
  channel_1_pwm = (1.0 - b)*channel_1_pwm_prev + b*channel_1_pwm;
  channel_2_pwm = (1.0 - b)*channel_2_pwm_prev + b*channel_2_pwm;
  channel_3_pwm = (1.0 - b)*channel_3_pwm_prev + b*channel_3_pwm;
  channel_4_pwm = (1.0 - b)*channel_4_pwm_prev + b*channel_4_pwm;
  channel_1_pwm_prev = channel_1_pwm;
  channel_2_pwm_prev = channel_2_pwm;
  channel_3_pwm_prev = channel_3_pwm;
  channel_4_pwm_prev = channel_4_pwm;
}