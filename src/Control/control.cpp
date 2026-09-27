#include<Arduino.h>
#include "configs/configs.h"
#include "control.h"

void controlANGLE(){
  //Description : Cette fonction contient une implémentation avancée d’un contrôleur d’angle PID en cascade et génère des variables d’axes stabilisés : PID de roulis (roll), de tangage (pitch) et de lacet (yaw)
  /* 
    Un contrôleur P-I sur l’angle sert de boucle externe, qui génère des commandes de vitesse angulaire demandées pour la boucle interne du contrôleur PID de taux
    Ces commandes de vitesse demandées sont filtrées par un filtre passe-bas du premier ordre, qui agit comme un amortissement du contrôleur d’angle
  */

  //Outer loop - PID on angle
  float roll_des_ol, pitch_des_ol;
  //Roll
  error_roll = roll_des - roll_IMU;
  integral_roll_ol = integral_roll_prev_ol + error_roll*dt;
  if (channel_1_pwm < 1060) {   // On empêche l'accumulation du terme intégral lorsque la puissance des gaz est trop faible
    integral_roll_ol = 0; // On remet donc integral_roll_ol = 0;
  }
  integral_roll_ol = constrain(integral_roll_ol, -i_limit, i_limit); // On sature l'intégrateur pour éviter une accumulation dangereuse
  derivative_roll = (roll_IMU - roll_IMU_prev)/dt;
  //Le terme D est désactivé, il est souvent pas nécessaire 
  roll_des_ol = Kp_roll_angle*error_roll + Ki_roll_angle*integral_roll_ol;// - Kd_roll_angle*derivative_roll; // La partie dérivée est inutile ici

  //Pitch
  error_pitch = pitch_des - pitch_IMU;
  integral_pitch_ol = integral_pitch_prev_ol + error_pitch*dt;
  if (channel_1_pwm < 1060) {   //On empêche l'accumulation du terme intégral lorsque la puissance des gaz est trop faible
    integral_pitch_ol = 0;
  }
  integral_pitch_ol = constrain(integral_pitch_ol, -i_limit, i_limit); //On sature l'intégrateur pour éviter une accumulation dangereuse.
  derivative_pitch = (pitch_IMU - pitch_IMU_prev)/dt;
  pitch_des_ol = Kp_pitch_angle*error_pitch + Ki_pitch_angle*integral_pitch_ol;// - Kd_pitch_angle*derivative_pitch; // La partie dérivée est inutile ici

  // Application du gain de la boucle, des contraintes et du filtre passe-bas pour le damping
  //La multiplication par Kl convertit la sortie du PI angle en une consigne de vitesse angulaire utilisable par la boucle interne
  float Kl = 30.0; // 10=>correction lente; 50=>correction rapide; 100=>correction trop rapide, risque d'oscillations
  roll_des_ol = Kl*roll_des_ol;
  pitch_des_ol = Kl*pitch_des_ol;
  roll_des_ol = constrain(roll_des_ol, -240.0, 240.0);
  pitch_des_ol = constrain(pitch_des_ol, -240.0, 240.0);
  //Filtrage passe-bas
  roll_des_ol = (1.0 - B_loop_roll)*roll_des_prev + B_loop_roll*roll_des_ol;
  pitch_des_ol = (1.0 - B_loop_pitch)*pitch_des_prev + B_loop_pitch*pitch_des_ol;

  //Inner loop - PID on rate
  //Roll
  error_roll = roll_des_ol-GyroX;
  integral_roll_il = integral_roll_prev_il + error_roll*dt;
  if (channel_1_pwm < 1060){   //On empêche l'accumulation du terme intégral lorsque la puissance des gaz est trop faible
    integral_roll_il = 0;
  }
  integral_roll_il=constrain(integral_roll_il,-i_limit,i_limit); //On sature l'intégrateur pour éviter une accumulation dangereuse.
  derivative_roll = (error_roll-error_roll_prev)/dt;
  roll_PID = .01*(Kp_roll_rate*error_roll + Ki_roll_rate*integral_roll_il + Kd_roll_rate*derivative_roll); //Multiplié par 0,01 afin de ramener la valeur dans la plage comprise entre −1 et 1

  //Pitch
  error_pitch = pitch_des_ol - GyroY;
  integral_pitch_il = integral_pitch_prev_il + error_pitch*dt;
  if (channel_1_pwm < 1060) {  // On empêche l'accumulation du terme intégral lorsque la puissance des gaz est trop faible
    integral_pitch_il = 0;
  }
  integral_pitch_il = constrain(integral_pitch_il, -i_limit, i_limit); //On sature l'intégrateur pour éviter une accumulation dangereuse.
  derivative_pitch = (error_pitch - error_pitch_prev)/dt; 
  pitch_PID = .01*(Kp_pitch_rate*error_pitch + Ki_pitch_rate*integral_pitch_il + Kd_pitch_rate*derivative_pitch); //Multiplié par 0,01 afin de ramener la valeur dans la plage comprise entre −1 et 1
  
  //Yaw
  error_yaw = yaw_des - GyroZ;
  integral_yaw = integral_yaw_prev + error_yaw*dt;
  if (channel_1_pwm < 1060) {   // On empêche l'accumulation du terme intégral lorsque la puissance des gaz est trop faible
    integral_yaw = 0;
  }
  integral_yaw = constrain(integral_yaw, -i_limit, i_limit); // On sature l'intégrateur pour éviter une accumulation dangereuse
  derivative_yaw = (error_yaw - error_yaw_prev)/dt; 
  yaw_PID = .01*(Kp_yaw*error_yaw + Ki_yaw*integral_yaw + Kd_yaw*derivative_yaw); //Multiplié par 0,01 afin de ramener la valeur dans la plage comprise entre −1 et 1
  
  //Mise à jour des variables de roll
  integral_roll_prev_ol = integral_roll_ol;
  integral_roll_prev_il = integral_roll_il;
  error_roll_prev = error_roll;
  roll_IMU_prev = roll_IMU;
  roll_des_prev = roll_des_ol;
  //Mise à jour des variables de pitch
  integral_pitch_prev_ol = integral_pitch_ol;
  integral_pitch_prev_il = integral_pitch_il;
  error_pitch_prev = error_pitch;
  pitch_IMU_prev = pitch_IMU;
  pitch_des_prev = pitch_des_ol;
  //Mise à jour des variables de yaw 
  error_yaw_prev = error_yaw;
  integral_yaw_prev = integral_yaw;

  roll_PID = constrain(roll_PID,-1.0,1.0);
  pitch_PID = constrain(pitch_PID,-1.0,1.0);
  yaw_PID = constrain(yaw_PID,-1.0,1.0);

}

void controlRATE() {
  // DESCRIPTION : Cette fonction contient une implémentation simple d’un contrôleur PID de taux (rate PID) et génère des variables d’axes stabilisés : PID de roulis (roll), de tangage (pitch) et de lacet (yaw)
  /*
    L’erreur est calculée à partir de la différence entre les vitesses angulaires désirées (roll des, pitch des et yaw des) et les vitesses angulaires mesurées (GyroX, GyroY et GyroZ)

  */  //Roll
  error_roll = roll_des - GyroX;
  integral_roll = integral_roll_prev + error_roll*dt;
  if (channel_1_pwm < 1060) {   // On empêche l'accumulation du terme intégral lorsque la puissance des gaz est trop faible
    integral_roll = 0;
  }
  integral_roll = constrain(integral_roll, -i_limit, i_limit); //On sature l'intégrateur pour éviter une accumulation dangereuse.
  derivative_roll = (error_roll - error_roll_prev)/dt; 
  roll_PID = .01*(Kp_roll_rate*error_roll + Ki_roll_rate*integral_roll + Kd_roll_rate*derivative_roll); //Multiplié par 0,01 afin de ramener la valeur dans la plage comprise entre −1 et 1.
  //Pitch
  error_pitch = pitch_des - GyroY;
  integral_pitch = integral_pitch_prev + error_pitch*dt;
  if (channel_1_pwm < 1060) {   // On empêche l'accumulation du terme intégral lorsque la puissance des gaz est trop faible
    integral_pitch = 0;
  }
  integral_pitch = constrain(integral_pitch, -i_limit, i_limit); //On sature l'intégrateur pour éviter une accumulation dangereuse.
  derivative_pitch = (error_pitch - error_pitch_prev)/dt; 
  pitch_PID = .01*(Kp_pitch_rate*error_pitch + Ki_pitch_rate*integral_pitch + Kd_pitch_rate*derivative_pitch); //Multiplié par 0,01 afin de ramener la valeur dans la plage comprise entre −1 et 1.
  //Yaw, stablize on rate from GyroZ
  error_yaw = yaw_des - GyroZ;
  integral_yaw = integral_yaw_prev + error_yaw*dt;
  if (channel_1_pwm < 1060) {   // On empêche l'accumulation du terme intégral lorsque la puissance des gaz est trop faible
    integral_yaw = 0;
  }
  integral_yaw = constrain(integral_yaw, -i_limit, i_limit); //On sature l'intégrateur pour éviter une accumulation dangereuse.
  derivative_yaw = (error_yaw - error_yaw_prev)/dt; 
  yaw_PID = .01*(Kp_yaw*error_yaw + Ki_yaw*integral_yaw + Kd_yaw*derivative_yaw); //Multiplié par 0,01 afin de ramener la valeur dans la plage comprise entre −1 et 1

 //Mise à jour des variables de roll
  error_roll_prev = error_roll;
  integral_roll_prev = integral_roll;
  GyroX_prev = GyroX;
  //Mise à jour des variables de pitch
  error_pitch_prev = error_pitch;
  integral_pitch_prev = integral_pitch;
  GyroY_prev = GyroY;
  //Mise à jour des variables de yaw 
  error_yaw_prev = error_yaw;
  integral_yaw_prev = integral_yaw;
}