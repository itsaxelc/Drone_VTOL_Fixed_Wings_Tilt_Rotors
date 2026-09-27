// On décommente seulement le type de récepteur utilisé
#define USE_PWM_RX
//#define USE_PPM_RX


// On décommente juste la version d'IMU utilisée
#define USE_MPU6050_I2C // Défaut
//#define USE_MPU9250_SPI

// On décommente une seule des plages de mesure (full scale) du gyroscope(deg/sec)
#define GYRO_250DPS //Default
//#define GYRO_500DPS
//#define GYRO_1000DPS
//#define GYRO_2000DPS

//On décommente une seule des plages de mesure (full scale) de l'accéléromètre (G's)
#define ACCEL_2G //Default
//#define ACCEL_4G
//#define ACCEL_8G
//#define ACCEL_16G


// Les librairies requises
#include <Wire.h>     // Communication I2c
#include <SPI.h>      // Communication SPI
#include <PWMServo.h> // Pour commander les actuateurs

// Inclure la bonne librairie MPU
#if defined USE_MPU6050_I2C
  #include "../lib/MPU6050/MPU6050.h"
  MPU6050 mpu6050;
#elif defined USE_MPU9250_SPI
  #include "src/MPU9250/MPU9250.h"
  MPU9250 mpu9250(SPI2,36);
#else
  #error No MPU defined... 
#endif

// Configuration de la sélection des plages de mesure et du facteur d'échelle du gyroscope et de l'accéléromètre

//Plage de mesure
#if defined USE_MPU6050_I2C
  #define GYRO_FS_SEL_250    MPU6050_GYRO_FS_250
  #define GYRO_FS_SEL_500    MPU6050_GYRO_FS_500
  #define GYRO_FS_SEL_1000   MPU6050_GYRO_FS_1000
  #define GYRO_FS_SEL_2000   MPU6050_GYRO_FS_2000
  #define ACCEL_FS_SEL_2     MPU6050_ACCEL_FS_2
  #define ACCEL_FS_SEL_4     MPU6050_ACCEL_FS_4
  #define ACCEL_FS_SEL_8     MPU6050_ACCEL_FS_8
  #define ACCEL_FS_SEL_16    MPU6050_ACCEL_FS_16
#elif defined USE_MPU9250_SPI
  #define GYRO_FS_SEL_250    mpu9250.GYRO_RANGE_250DPS
  #define GYRO_FS_SEL_500    mpu9250.GYRO_RANGE_500DPS
  #define GYRO_FS_SEL_1000   mpu9250.GYRO_RANGE_1000DPS                                                        
  #define GYRO_FS_SEL_2000   mpu9250.GYRO_RANGE_2000DPS
  #define ACCEL_FS_SEL_2     mpu9250.ACCEL_RANGE_2G
  #define ACCEL_FS_SEL_4     mpu9250.ACCEL_RANGE_4G
  #define ACCEL_FS_SEL_8     mpu9250.ACCEL_RANGE_8G
  #define ACCEL_FS_SEL_16    mpu9250.ACCEL_RANGE_16G
#endif

//Facteurs d'échelle
#if defined GYRO_250DPS
  #define GYRO_SCALE GYRO_FS_SEL_250
  #define GYRO_SCALE_FACTOR 131.0
#elif defined GYRO_500DPS
  #define GYRO_SCALE GYRO_FS_SEL_500
  #define GYRO_SCALE_FACTOR 65.5
#elif defined GYRO_1000DPS
  #define GYRO_SCALE GYRO_FS_SEL_1000
  #define GYRO_SCALE_FACTOR 32.8
#elif defined GYRO_2000DPS
  #define GYRO_SCALE GYRO_FS_SEL_2000
  #define GYRO_SCALE_FACTOR 16.4
#endif

#if defined ACCEL_2G
  #define ACCEL_SCALE ACCEL_FS_SEL_2
  #define ACCEL_SCALE_FACTOR 16384.0
#elif defined ACCEL_4G
  #define ACCEL_SCALE ACCEL_FS_SEL_4
  #define ACCEL_SCALE_FACTOR 8192.0
#elif defined ACCEL_8G
  #define ACCEL_SCALE ACCEL_FS_SEL_8
  #define ACCEL_SCALE_FACTOR 4096.0
#elif defined ACCEL_16G
  #define ACCEL_SCALE ACCEL_FS_SEL_16
  #define ACCEL_SCALE_FACTOR 2048.0
#endif


// Valeurs de sécurité (failsafe) radio pour chaque canal en cas de données récepteur erronées
unsigned long channel_1_fs = 1000; //Gaz
unsigned long channel_2_fs = 1500; //Roll
unsigned long channel_3_fs = 1500; //Pitch
unsigned long channel_4_fs = 1500; //Yaw
unsigned long channel_5_fs = 2000; // train (gear), supérieur à 1500 = coupure des gaz (throttle cut)
unsigned long channel_6_fs = 1000; //aux1 affecté à SwA (2 positions) pour la transition de hover vers cruise,  <1500 => hover, >1500 => cruise, mode=hover par défaut

// Paramètres des filtres-Réglages par défaut optimisés pour une boucle à 2 kHz
float B_madgwick = 0.04;  //Paramètre du filtre de Madgwick
float B_accel = 0.2;     //Paramètre de filtre passe-bas de l'accéléromètre (MPU6050 défaut: 0.14, MPU9250 défaut: 0.2)
float B_gyro = 0.2;       //Paramètre de filtre passe-bas du gyroscope (MPU6050 défaut: 0.1, MPU9250 défaut: 0.17)
float B_mag = 1.0;        //Paramètre de filtre passe-bas du magnétomètre

// Paramètres de calibration du magnétomètre
// Si on utilise le MPU9250, on décommente calibrateMagnetometer() dans void setup() pour avoir ces valeurs, sinon on considère erreur nulle et échelle conservatrice
float MagErrorX = 0.0;
float MagErrorY = 0.0; 
float MagErrorZ = 0.0;
float MagScaleX = 1.0;
float MagScaleY = 1.0;
float MagScaleZ = 1.0;

// Paramètres de calibration de l'IMU- On calibre l'IMU en utilisant calculate_IMU_error() dans void setup() pour obtenir ces valeurs, puis on décommente calculate_IMU_error()
float AccErrorX = 0.02;
float AccErrorY = -0.01;
float AccErrorZ = -0.08;
float GyroErrorX = 5.56;
float GyroErrorY = 0.08;
float GyroErrorZ = -0.51;

//Paramètres du contrôleur (On note les valeurs par défaut avant de modifier)
float i_limit = 25.0;     // Niveau de saturation de l'intégrateur, principalement pour la sécurité (25.0 par défaut)
float maxRoll = 50.0;     // Max roll angle in degrees for angle mode (maximum ~70 degrees), deg/sec for rate mode
float maxPitch = 50.0;    // Max pitch angle in degrees for angle mode (maximum ~70 degrees), deg/sec for rate mode
float maxYaw = 220.0;     // Max yaw rate in deg/sec

float Kp_roll_angle = 0.2;    //Roll P-gain - angle mode 
float Ki_roll_angle = 0.3;    //Roll I-gain - angle mode
float Kd_roll_angle = 0.05;   //Roll D-gain - angle mode
float B_loop_roll = 0.9;      // Terme de damping pour roll controlANGLE(), plus bas signifie plus de damping (Doit être entre 0 et 1), "terme d’amortissement du roulis" est une partie du contrôleur qui sert à s’opposer à la vitesse de rotation en roulis
float Kp_pitch_angle = 0.2;   //Pitch P-gain - angle mode
float Ki_pitch_angle = 0.3;   //Pitch I-gain - angle mode
float Kd_pitch_angle = 0.05;  //Pitch D-gain - angle mode (aucun effet sur controlANGLE2)
float B_loop_pitch = 0.9;     //Terme de damping pour pitch controlANGLE(), plus bas signifie plus de damping (Doit être entre 0 et 1), "terme d’amortissement du tangage" est une partie du contrôleur qui sert à s’opposer à la vitesse de rotation en tangage

float Kp_roll_rate = 0.15;    //Roll P-gain - rate mode
float Ki_roll_rate = 0.2;     //Roll I-gain - rate mode
float Kd_roll_rate = 0.0002;  //Roll D-gain - rate mode (Trop augmenter cette valeur pourrait faire surchauffer les moteurs)
float Kp_pitch_rate = 0.15;   //Pitch P-gain - rate mode
float Ki_pitch_rate = 0.2;    //Pitch I-gain - rate mode
float Kd_pitch_rate = 0.0002; //Pitch D-gain - rate mode (Trop augmenter cette valeur pourrait faire surchauffer les moteurs)

float Kp_yaw = 0.3;           //Yaw P-gain
float Ki_yaw = 0.05;          //Yaw I-gain
float Kd_yaw = 0.00015;       //Yaw D-gain (Trop augmenter cette valeur pourrait faire surchauffer les moteurs)

                                      
//Déclaration des pins
//NOTE: Pin 13 est réservé pour LED builtin, pins 18 et 19 sont réservés pour le MPU6050 pour les configurations par défaut
const int ch1Pin = 15; //Gaz
const int ch2Pin = 16; //Aileron
const int ch3Pin = 17; //Profondeur
const int ch4Pin = 20; //Direction
const int ch5Pin = 21; //train (coupure des gaz)
const int ch6Pin = 22; //aux1 (cannal aux libre pour gestion de transition)
const int PPM_Pin = 23;
//Pins de sortie OneShot125 ESC :
const int m1Pin = 0;
const int m2Pin = 1;

//Servo PWM :
const int servo1Pin = 6;
const int servo2Pin = 7;
const int servo3Pin = 8;

//Création d'objets "servo" pour contrôler les servomoteurs
PWMServo servo1;  
PWMServo servo2;
PWMServo servo3;

//========================================================================================================================//

//Déclaration des variables globales

//General stuff
float dt;
unsigned long current_time, prev_time;
unsigned long print_counter = micros();
unsigned long blink_counter, blink_delay;
bool blinkAlternate;

//Radio communication:
unsigned long channel_1_pwm, channel_2_pwm, channel_3_pwm, channel_4_pwm, channel_5_pwm, channel_6_pwm;
unsigned long channel_1_pwm_prev, channel_2_pwm_prev, channel_3_pwm_prev, channel_4_pwm_prev;

//IMU:
float AccX, AccY, AccZ;
float AccX_prev, AccY_prev, AccZ_prev;
float GyroX, GyroY, GyroZ;
float GyroX_prev, GyroY_prev, GyroZ_prev;
float MagX = 0, MagY = 0, MagZ = 0;
float MagX_prev, MagY_prev, MagZ_prev;
float roll_IMU, pitch_IMU, yaw_IMU;
float roll_IMU_prev, pitch_IMU_prev;
float q0 = 1.0f; //Initialisation du quaternion pour le filtre de Madgwick
float q1 = 0.0f;
float q2 = 0.0f;
float q3 = 0.0f;

//Variables pour les commandes utilisateurs normalisées:
float thro_des, roll_des, pitch_des, yaw_des;

// Variables des contrôleurs PID:
float error_roll, error_roll_prev, roll_des_prev, integral_roll, integral_roll_il, integral_roll_ol, integral_roll_prev, integral_roll_prev_il, integral_roll_prev_ol, derivative_roll, roll_PID = 0;
float error_pitch, error_pitch_prev, pitch_des_prev, integral_pitch, integral_pitch_il, integral_pitch_ol, integral_pitch_prev, integral_pitch_prev_il, integral_pitch_prev_ol, derivative_pitch, pitch_PID = 0;
float error_yaw, error_yaw_prev, integral_yaw, integral_yaw_prev, derivative_yaw, yaw_PID = 0;

//Variables pour le mixeur de commandes
float m1_command_scaled, m2_command_scaled;
int m1_command_PWM, m2_command_PWM;
float s1_command_scaled, s2_command_scaled, s3_command_scaled;
int s1_command_PWM, s2_command_PWM, s3_command_PWM;

// Variables qui stockent les instants de front montant de chaque canal PWM pour calculer les largeurs d'impulsions
unsigned long rising_edge_start_1, rising_edge_start_2, rising_edge_start_3, rising_edge_start_4, rising_edge_start_5, rising_edge_start_6; 
// Variables qui stockent les largeurs d'impulsion de chaque canal PWM
unsigned long channel_1_raw, channel_2_raw, channel_3_raw, channel_4_raw, channel_5_raw, channel_6_raw;
// Compteur qui permet de passer d'une valeur d'impulsion à une autre dans le PPM
int ppm_counter = 0;
// Variable qui stocke la durée entre les appels de fonction getPPM, durée qui permet de récupérer
unsigned long time_ms = 0;


// variable pour le statut du vol
bool armedFly = false; // False=Pas autorisé à voler

// Variables spécifiques au tilt rotor et la transition
float transition_fader = 1.0; // 1.0=hover, 0.0=forward flight
float faderate = 3.0; // temps en secondes pour passer des limites minimales aux maximales (0 à 1) ou inversement (1 à 0)

// Gains de controle
// hover
float kp_pitch_angle_hover=1.17;
float kd_pitch_angle_hover = 0.09;
float kp_roll_angle_hover =  0.36;
float kd_roll_angle_hover = 0.07;
float kp_yaw_hover = 0.45;
float kd_yaw_hover = 0.0003;
// forward flight
float kp_pitch_angle_ff = 0.25;
float kd_pitch_angle_ff = 0.05;
float kp_roll_angle_ff = 0.5;
float kd_roll_angle_ff = 0.06;
float kp_yaw_ff = 0.2;
float kd_yaw_ff = 0.00017;

// Tilt-rotor : positions servo (échelle 0 à 1) aux deux extrêmes mécaniques
float tilt_servo_cruise_pos = 0.0; // rotors à l'horizontale (croisière) - À CALIBRER au banc
float tilt_servo_hover_pos = 0.5;  // rotors à la verticale (hover) - À CALIBRER au banc

float elevator_trim = 0.5; // position neutre de la gouverne de profondeur

float k_pitch_tilt_collective = 0.15; // gain de l'inclinaison commune pour le tangage en hover - à calibrer
float k_yaw_tilt_diff = 0.1;          // gain de l'inclinaison différentielle pour le lacet en hover - à calibrer

float ff_roll_yaw_mix_amount = 30; // En vol de croisière une commande de roulis induit également un certain lacet afin d’obtenir des virages coordonnés
float ff_pitch_trim = 5; // Angle de trim en tangage en degrés à commander en vol de croisière