#include<Arduino.h>
#include "../configs/configs.h"

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