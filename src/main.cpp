#include <Arduino.h>
#include "configs/configs.h"
#include "Actuators/actuators.h"
#include "Control/control.h"
#include "Debug_utilities/debug_utilities.h"
#include "Helpers/timing.h"
#include "IMU/imu.h"
#include "IMU/madgwick.h"
#include "Radio/radio_command.h"
#include "Radio/radio_communication.h"
#include "Safety/failsafe.h"

void setup() {
  Serial.begin(500000); //USB serial
  delay(500);
  
  //Initialisation de tous les pins
  pinMode(13, OUTPUT); //Pin 13 LED blinker on board
  pinMode(m1Pin, OUTPUT);
  pinMode(m2Pin, OUTPUT);

  servo1.attach(servo1Pin, 1000, 2000); // (Pin, min PWM value, max PWM value)
  servo2.attach(servo2Pin, 1000, 2000);
  servo3.attach(servo3Pin, 1000, 2000);

  //Allumer la LED intégrée pour signaler le démarrage
  digitalWrite(13, HIGH);

  delay(5);

  //Initialisation de la radio communication
  radioSetup();
  
  //Initialise les canaux radio à des valeurs par défaut (sûres) avant d’entrer dans la boucle principale
  channel_1_pwm = channel_1_fs;
  channel_2_pwm = channel_2_fs;
  channel_3_pwm = channel_3_fs;
  channel_4_pwm = channel_4_fs;
  channel_5_pwm = channel_5_fs;
  channel_6_pwm = channel_6_fs;

  //Initialisation de la communication IMU
  IMUinit();

  delay(5);

  // Calcul de l’erreur de l’IMU pour mettre à zéro les mesures de l’accéléromètre et du gyroscope en mettant le véhicule à plat au moment de la mise sous tension
  // calculate_IMU_error(); // Les paramètres de calibration sont affichés sur le moniteur série. On les copie dans la section des variables utilisateur, puis on commente cette ligne définitivement

  // On arme les canaux des servos
  // On commande l’angle des servos de 0 à 180 degrés (1000 à 2000 µs PWM)
  // On met ces valeurs à 90 pour les servos pour éviter de potentiels chocs mécaniques au démarrage
  servo1.write(90); 
  servo2.write(90); 
  servo3.write(90); 
  
  delay(5);

  // calibrateESCs();  // Hélices retirées, on décommente cette ligne pour calibrer les ESCs 
  // ESCs calibré en mettant le manche des gaz au maximum, en allumant le système, puis en ramenant le throttle à zéro après les bips.
  // Le code ne continuera pas au-delà de cette ligne si cette fonction est décommentée !

  // On arme les moteurs OneShot125
  // On commande les OneShot125 ESC avec une largeur d'impulsion dans la plage 125 à 250us
  m1_command_PWM = 125; 
  m2_command_PWM = 125;
  armMotors(); // On boucle dans commandMotors() jusqu'à ce que les ESCs s'arment
  
  // On indique l'entrée dans la boucle principale avec 3 clignotements rapides
  setupBlink(3,160,70); //numBlinks, upTime (ms), downTime (ms)

  // Si on utilise l’IMU MPU9250, on décommente pour effectuer une calibration unique du magnétomètre (peut devoir être refaite en cas de changement d’environnement ou de lieu)
  // calibrateMagnetometer(); // Génère les erreurs et facteurs d’échelle du magnétomètre à copier dans la section des variables utilisateur
}

void loop() {
  // Surveiller le temps actuel et calculer le temps écoulé depuis la dernière boucle
  prev_time = current_time;      
  current_time = micros();      
  dt = (current_time - prev_time)/1000000.0;

  loopBlink(); //Indiquer que nous sommes dans la boucle principale avec un court clignotement toutes les 1,5 secondes

  // Affichage des données à 100 Hz ( On décommente une seule option à la fois pour le débogage)
  // printRadioData();     // Affiche les valeurs PWM de la radio (attendu : 1000 à 2000)
  // printDesiredState();  // Affiche l’état désiré du véhicule en degrés ou degrés/s (attendu : +/- maxAXIS pour roll, pitch, yaw ; 0 à 1 pour le throttle)
  // printGyroData();      // Affiche les données gyroscope filtrées de l’IMU (attendu : ~ -250 à 250, 0 au repos)
  // printAccelData();     // Affiche les données accéléromètre filtrées de l’IMU (attendu : ~ -2 à 2 ; x,y = 0 à plat, z = 1 à plat)
  // printMagData();       // Affiche les données magnétomètre filtrées de l’IMU (attendu : ~ -300 à 300)
  // printRollPitchYaw();  // Affiche les angles roll, pitch et yaw en degrés issus du filtre Madgwick (0 à plat)
  // printPIDoutput();     // Affiche les sorties PID calculées et la consigne (attendu : ~ -1 à 1)
  // printMotorCommands(); // Affiche les valeurs envoyées aux moteurs (attendu : 120 à 250)
  // printServoCommands(); // Affiche les valeurs envoyées aux servos (attendu : 0 à 180)
  // printLoopRate();      // Affiche le temps entre deux boucles en microsecondes

  // On récupère le statut d'armement
  armedStatus(); // On vérifie que le throttle cut est low et que throttle est low

  // On récupère l'état du drone
  getIMUdata(); // Récupère les données brutes du gyroscope, de l’accéléromètre et du magnétomètre depuis l’IMU et applique des filtres passe-bas pour réduire le bruit

  Madgwick(GyroX,-GyroY,-GyroZ,-AccX, AccY,AccZ,MagY,-MagX,MagZ,dt); // Met à jour les estimations d’angles roll_IMU, pitch_IMU et yaw_IMU (en degrés)

  // Calculer les états désirés
  getDesState(); // Convertit les commandes brutes en valeurs normalisées basées sur les limites de contrôle saturées

  channel_6_pwm = constrain(channel_6_pwm,1000.0,2000.0);

  if(channel_6_pwm>1800 && transition_fader<1.0) transition_fader = transition_fader + dt/faderate;
  
  else if (channel_6_pwm<1200 && transition_fader>0.0) transition_fader = transition_fader - dt/faderate;

  transition_fader = constrain(transition_fader, 0.0, 1.0);
  
  //Serial.println(transition_fader);

  // Atténuer progressivement les gains PID
  // (1.0 - transition_fader)*(forward flight) + (transition_fader)*(hover)
  Kp_roll_angle = (1.0 - transition_fader)*(kp_roll_angle_ff) + (transition_fader)*(kp_roll_angle_hover);
  Kd_roll_angle = (1.0 - transition_fader)*(kd_roll_angle_ff) + (transition_fader)*(kd_roll_angle_hover);
  Kp_pitch_angle = (1.0 - transition_fader)*(kp_pitch_angle_ff) + (transition_fader)*(kp_pitch_angle_hover);
  Kd_pitch_angle = (1.0 - transition_fader)*(kd_pitch_angle_ff) + (transition_fader)*(kd_pitch_angle_hover);
  Kp_yaw = (1.0 - transition_fader)*(kp_yaw_ff) + (transition_fader)*(kp_yaw_hover);
  Kd_yaw = (1.0 - transition_fader)*(kd_yaw_ff) + (transition_fader)*(kd_yaw_hover);

  // Contrôleur PID :
  controlANGLE(); // Stabilisation sur une consigne d'angle en utilisant une structure en cascade. Le contrôleur de vitesse angulaire (rate controller) doit d'abord être bien réglé !
  //controlRATE();   // Stabilisation sur une consigne de vitesse angulaire

  // Mélange des commandes et mise à l'échelle vers les valeurs PWM
  controlMixer(); // Combine les sorties PID pour générer les commandes des actionneurs (la configuration spécifique du mélange des moteurs/servos se fait ici)

  scaleCommands(); // Convertit les commandes moteurs dans la plage 125 à 250 (protocole OneShot125) et les commandes servos dans la plage 0 à 180 (bibliothèque Servo)

  // Vérification du throttle cut
  throttleCut(); // Force directement les commandes moteurs à leur valeur minimale selon l'état du canal 5

  // Commande des actionneurs
  commandMotors(); // Envoie les impulsions OneShot125 aux moteurs

  // Envoie les positions aux servos
  servo1.write(s1_command_PWM);
  servo2.write(s2_command_PWM);
  servo3.write(s3_command_PWM);

  // Lecture des commandes pour la prochaine itération
  getCommands(); // Lit les commandes radio actuellement disponibles
  failSafe();    // Active les valeurs de secours en cas de perte ou de problème de liaison radio

  // Régulation de la fréquence de boucle
  loopRate(2000); // Limite la boucle à 2000 Hz. Tous les paramètres des filtres ont été réglés pour fonctionner à 2000 Hz
}