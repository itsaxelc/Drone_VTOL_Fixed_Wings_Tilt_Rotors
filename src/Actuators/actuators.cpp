#include<Arduino.h>
#include "configs/configs.h"
#include "IMU/imu.h"
#include "IMU/madgwick.h"
#include "Helpers/timing.h"
#include "Radio/radio_command.h"
#include "Radio/radio_communication.h"
#include "Safety/failsafe.h"
#include "actuators.h"


void commandMotors() {
  // Description : Fonction qui envoie les impulsions de commandes aux broches des moteurs par le protocole OneShot125

  int wentLow = 0;
  int pulseStart, timer;
  int flagM1 = 0;
  int flagM2 = 0;
  
  //Write all motor pins high
  digitalWrite(m1Pin, HIGH);
  digitalWrite(m2Pin, HIGH);
  pulseStart = micros();

  //On met chaque broche moteur à l’état bas une fois que la durée d’impulsion correcte est atteinte
  // Ya pas d'ordre, le moteur qui reçois 150us et celui qui reçois 200us vont chacun se couper au moment où chaque durée est atteinte, ya pas de m1 puis m2
  while(wentLow < 2 ){ // Boucle pour commander tous les 2 moteurs, puis terminer
    timer = micros();
    if ((m1_command_PWM <= timer - pulseStart) && (flagM1==0)) {
      digitalWrite(m1Pin, LOW);
      wentLow = wentLow + 1;
      flagM1 = 1;
    }
    if ((m2_command_PWM <= timer - pulseStart) && (flagM2==0)) {
      digitalWrite(m2Pin, LOW);
      wentLow = wentLow + 1;
      flagM2 = 1;
    }
  }
}

void controlMixer() {
  // DESCRIPTION : Mélange les commandes mises à l’échelle issues du contrôleur PID vers les actionneurs selon la configuration du véhicule

  // transition_fader : 1 = hover (rotors verticaux), 0 = cruise (rotors horizontaux)

  m1_command_scaled = thro_des - roll_PID - (1.0 - transition_fader)*yaw_PID; // moteur droit
  m2_command_scaled = thro_des + roll_PID + (1.0 - transition_fader)*yaw_PID; // moteur gauche

  // Scaling
  float tilt_max = max(m1_command_scaled, m2_command_scaled);
  float tilt_min = min(m1_command_scaled, m2_command_scaled);

  float scale=1;

  if (tilt_max > 1.0 || tilt_min < 0.0) {
    float scale_max = (1.0 - thro_des) / (tilt_max - thro_des);
    float scale_min = (thro_des - 0.0) / (thro_des - tilt_min);
    scale = min(scale_max, scale_min);
  }

  m1_command_scaled = thro_des + (m1_command_scaled - thro_des) * scale;
  m2_command_scaled = thro_des + (m2_command_scaled - thro_des) * scale;

  m1_command_scaled = constrain(m1_command_scaled, 0.0, 1.0);
  m2_command_scaled = constrain(m2_command_scaled, 0.0, 1.0);

  // Inclinaison de base des rotors : planning direct par transition_fader, pas par le PID 
  float tilt_cmd = tilt_servo_cruise_pos + transition_fader*(tilt_servo_hover_pos-tilt_servo_cruise_pos);

  // Pitch en hover : inclinaison COMMUNE des deux rotors, nulle en cruise 
  float tilt_pitch_common = transition_fader*k_pitch_tilt_collective*pitch_PID;

  // Yaw en hover : inclinaison DIFFÉRENTIELLE des deux rotors, nulle en cruise 
  float tilt_yaw_diff = transition_fader*k_yaw_tilt_diff*yaw_PID;

  float s1_raw=tilt_cmd + tilt_pitch_common + tilt_yaw_diff; // tilt rotor droit
  float s2_raw=tilt_cmd - tilt_pitch_common + tilt_yaw_diff; // tilt rotor gauche

  tilt_max = max(s1_raw, s2_raw);
  tilt_min = min(s1_raw, s2_raw);

  if (tilt_max > 1.0 || tilt_min < 0.0) {
    float scale_max = (tilt_max > tilt_cmd) ? (1.0 - tilt_cmd) / (tilt_max - tilt_cmd) : 1.0;
    float scale_min = (tilt_min < tilt_cmd) ? (tilt_cmd - 0.0) / (tilt_cmd - tilt_min) : 1.0;
    float tilt_scale = min(scale_max, scale_min);
    s1_command_scaled = tilt_cmd + (s1_raw - tilt_cmd) * tilt_scale;
    s2_command_scaled = tilt_cmd + (s2_raw - tilt_cmd) * tilt_scale;
  }
  
  else {
    s1_command_scaled = s1_raw;
    s2_command_scaled = s2_raw;
  }

  s1_command_scaled = constrain(s1_command_scaled,0.0,1.0);
  s2_command_scaled = constrain(s2_command_scaled,0.0,1.0);

  // Pitch en cruise : gouverne de profondeur, nulle en hover
  s3_command_scaled = constrain(elevator_trim + (1.0 - transition_fader)*pitch_PID, 0.0, 1.0);
}

void armMotors(){
  // Description : Fonction qui envoie plusieurs impulsions de commande aux moteurs, utilisé pour armer les moteurs dans la fonction setup()

  for (int i = 0; i <= 50; i++) {
    commandMotors();
    delay(2);
  }
}

void calibrateESCs(){
  // Description : Utilisé dans la fonction setup() pour permettre la procédure standard de calibration des ESC avec la radio.
  while (true) {
    prev_time = current_time;      
    current_time = micros();      
    dt = (current_time - prev_time)/1000000.0;
    
    digitalWrite(13, HIGH); //LED on pour indiquer qu'on est pas dans main loop

    getCommands(); // Récupère les commandes radio actuellement disponibles
    failSafe(); //Évite les failures en cas de mauvaise connexion du récepteur ; utilise par défaut les valeurs de failsafe définies dans le setup
    getDesState(); //Convertit les commandes brutes en valeurs normalisées en fonction des limites de contrôle saturées
    getIMUdata(); // Récupère les données brutes de gyro, acceléromètre, et magnétomètre de l'IMU et des filtres passe-bas (LP filters) pour éliminer le bruit 
    Madgwick(GyroX, -GyroY, -GyroZ, -AccX, AccY, AccZ, MagY, -MagX, MagZ, dt); //Mise à jour de roll_IMU, pitch_IMU, et yaw_IMU (degrés)
    getDesState(); //Convertit les commandes brutes en valeurs normalisées en fonction des limites de contrôle saturées
      
    m1_command_scaled = thro_des;
    m2_command_scaled = thro_des;
    s1_command_scaled = thro_des;
    s2_command_scaled = thro_des;
    s3_command_scaled = thro_des;
  
    scaleCommands(); //Mise à l'échelle des commandes moteurs dans la plage de 125 à 250 (oneshot125 protocol) et des commandes servo PWM de 0 à 180 (for servo library)
  
    throttleCut(); // Met directement les commandes moteur à low selon l'état de CH5
    
    servo1.write(s1_command_PWM); 
    servo2.write(s2_command_PWM);
    servo3.write(s3_command_PWM);
    commandMotors(); // Envoie des impulsions de commandes à chaque pin de moteur en utilisant le OneShot125 protocol
      
    //printRadioData(); // Affiche les valeurs de PWM radio (plage: 1000 à 2000)
      
    loopRate(2000); //Do not exceed 2000Hz, all filter parameters tuned to 2000Hz by default
   }
}

void throttleCut() {
  // Description : Cette fonction définit directement les sorties des actionneurs à leur valeur minimale si elle est activée

  if ((channel_5_pwm > 1800) || (armedFly == false)) {
    armedFly = false;
    // 125 (minimum de OneShot125) peut encore faire tourner légèrement le moteur, mais 120=garantie de coupure totale
    m1_command_PWM = 120;
    m2_command_PWM = 120;

  }
}

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