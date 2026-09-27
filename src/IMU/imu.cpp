#include<Arduino.h>
#include "configs/configs.h"
#include "IMU/madgwick.h"
#include "Helpers/timing.h"
#include "imu.h"

void IMUinit() {
  //DESCRIPTION: Fonction d'initialisation del'IMU
  
  #if defined USE_MPU6050_I2C
    Wire.begin();
    Wire.setClock(1000000);
    
    mpu6050.initialize();
    
    if (mpu6050.testConnection() == false) {
      Serial.println("MPU6050 initialization unsuccessful");
      Serial.println("Check MPU6050 wiring");
      while(1) {}
    }

    mpu6050.setFullScaleGyroRange(GYRO_SCALE);
    mpu6050.setFullScaleAccelRange(ACCEL_SCALE);
    
  #elif defined USE_MPU9250_SPI
    int status = mpu9250.begin();    

    if (status < 0) {
      Serial.println("MPU9250 initialization unsuccessful");
      Serial.println("Check MPU9250 wiring or try cycling power");
      Serial.print("Status: ");
      Serial.println(status);
      while(1) {}
    }

    mpu9250.setGyroRange(GYRO_SCALE);
    mpu9250.setAccelRange(ACCEL_SCALE);
    mpu9250.setMagCalX(MagErrorX, MagScaleX);
    mpu9250.setMagCalY(MagErrorY, MagScaleY);
    mpu9250.setMagCalZ(MagErrorZ, MagScaleZ);
    mpu9250.setSrd(0); // Configure le gyroscope et l'accéléromètre à une fréquence de lecture de 1khz, et le magnétomètre à 100hz
  #endif
}

void getIMUdata() {
  //DESCRIPTION : Fonction qui récupère l’ensemble des données de l’IMU et filtre passe-bas les données du gyroscope, de l’accéléromètre et du magnétomètre

  int16_t AcX,AcY,AcZ,GyX,GyY,GyZ,MgX,MgY,MgZ;

  #if defined USE_MPU6050_I2C
    mpu6050.getMotion6(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ);
  #elif defined USE_MPU9250_SPI
    mpu9250.getMotion9(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ, &MgX, &MgY, &MgZ);
  #endif

 // Accéléromètre
  AccX = AcX / ACCEL_SCALE_FACTOR; //G's
  AccY = AcY / ACCEL_SCALE_FACTOR;
  AccZ = AcZ / ACCEL_SCALE_FACTOR;
  //Corrige les outputs avec les valeurs d'erreurs calculées
  AccX = AccX - AccErrorX;
  AccY = AccY - AccErrorY;
  AccZ = AccZ - AccErrorZ;
  //Filtrer  passe-bas les données de l’accéléromètre
  AccX = (1.0 - B_accel)*AccX_prev + B_accel*AccX;
  AccY = (1.0 - B_accel)*AccY_prev + B_accel*AccY;
  AccZ = (1.0 - B_accel)*AccZ_prev + B_accel*AccZ;
  AccX_prev = AccX;
  AccY_prev = AccY;
  AccZ_prev = AccZ;

  // Gyroscope
  GyroX = GyX / GYRO_SCALE_FACTOR; //deg/sec
  GyroY = GyY / GYRO_SCALE_FACTOR;
  GyroZ = GyZ / GYRO_SCALE_FACTOR;
  //Corrige les outputs avec les valeurs d'erreurs calculées
  GyroX = GyroX - GyroErrorX;
  GyroY = GyroY - GyroErrorY;
  GyroZ = GyroZ - GyroErrorZ;
  //Filtrer  passe-bas les données du gyroscope
  GyroX = (1.0 - B_gyro)*GyroX_prev + B_gyro*GyroX;
  GyroY = (1.0 - B_gyro)*GyroY_prev + B_gyro*GyroY;
  GyroZ = (1.0 - B_gyro)*GyroZ_prev + B_gyro*GyroZ;
  GyroX_prev = GyroX;
  GyroY_prev = GyroY;
  GyroZ_prev = GyroZ;

  MgX=MgY=MgZ=0;
  //Magnetometer
  MagX = MgX/6.0; //uT
  MagY = MgY/6.0;
  MagZ = MgZ/6.0;
  //Corrige les outputs avec les valeurs d'erreurs calculées
  MagX = (MagX - MagErrorX)*MagScaleX;
  MagY = (MagY - MagErrorY)*MagScaleY;
  MagZ = (MagZ - MagErrorZ)*MagScaleZ;
  //Filtrer  passe-bas les données du magnétomètre
  MagX = (1.0 - B_mag)*MagX_prev + B_mag*MagX;
  MagY = (1.0 - B_mag)*MagY_prev + B_mag*MagY;
  MagZ = (1.0 - B_mag)*MagZ_prev + B_mag*MagZ;
  MagX_prev = MagX;
  MagY_prev = MagY;
  MagZ_prev = MagZ;
}

void calculate_IMU_error() {
//Description: Cette fonction calcule l'erreur de l'accéléromètre et du gyroscope de la centrale IMU au démarrage
//Remarque: le drone doit être mis sous tension sur une surface plane

  int16_t AcX,AcY,AcZ,GyX,GyY,GyZ,MgX,MgY,MgZ;
  AccErrorX = 0.0;
  AccErrorY = 0.0;
  AccErrorZ = 0.0;
  GyroErrorX = 0.0;
  GyroErrorY= 0.0;
  GyroErrorZ = 0.0;
  
  //Lire les valeurs de l'IMU 12000 fois
  int c = 0;
  while(c<12000){
    #if defined USE_MPU6050_I2C
      mpu6050.getMotion6(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ);
    #elif defined USE_MPU9250_SPI
      mpu9250.getMotion9(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ, &MgX, &MgY, &MgZ);
    #endif
    
    AccX = AcX/ACCEL_SCALE_FACTOR;
    AccY = AcY/ACCEL_SCALE_FACTOR;
    AccZ = AcZ/ACCEL_SCALE_FACTOR;
    GyroX = GyX/GYRO_SCALE_FACTOR;
    GyroY = GyY/GYRO_SCALE_FACTOR;
    GyroZ = GyZ/GYRO_SCALE_FACTOR;
    
    //Sommer toutes les erreurs lues
    AccErrorX = AccErrorX + AccX;
    AccErrorY = AccErrorY + AccY;
    AccErrorZ = AccErrorZ + AccZ;
    GyroErrorX = GyroErrorX + GyroX;
    GyroErrorY = GyroErrorY + GyroY;
    GyroErrorZ = GyroErrorZ + GyroZ;
    c++;
  }
  //Diviser la somme des erreurs par 12000 pour obtenir l'erreur
  AccErrorX  = AccErrorX / c;
  AccErrorY  = AccErrorY / c;
  AccErrorZ  = AccErrorZ / c - 1.0; //A l'arrêt sur un axe horizontal, l'axe Z devrait lire 1g (la gravité), soustraire 1.0 permet d'obtenir l'erreur de l'axe Z
  GyroErrorX = GyroErrorX / c;
  GyroErrorY = GyroErrorY / c;
  GyroErrorZ = GyroErrorZ / c;

  while (true)
  {
    Serial.print("float AccErrorX = ");
    Serial.print(AccErrorX);
    Serial.println(";");
    Serial.print("float AccErrorY = ");
    Serial.print(AccErrorY);
    Serial.println(";");
    Serial.print("float AccErrorZ = ");
    Serial.print(AccErrorZ);
    Serial.println(";");
    
    Serial.print("float GyroErrorX = ");
    Serial.print(GyroErrorX);
    Serial.println(";");
    Serial.print("float GyroErrorY = ");
    Serial.print(GyroErrorY);
    Serial.println(";");
    Serial.print("float GyroErrorZ = ");
    Serial.print(GyroErrorZ);
    Serial.println(";");
    /*
    On récupère les valeurs d'erreurs calculées affichées précédemment,
    on les copie dans la section «user specified variables» (en début de code),
    puis on commente l’appel à calculate_IMU_error() dans void setup() pour ne pas recalibrer à chaque démarrage.
    */
  }
}

void calibrateAttitude() {
  // DESCRIPTION : Cette fobnction est utilisée pour échauffer la boucle principale afin de permettre au filtre de Madgwick de converger avant l’envoi de commandes aux actionneurs
  // On met le drone sous tension sur une surface horizontale !

  // Initialisation de l’IMU et du filtre de Madgwick dans une boucle principale simulée
  for (int i = 0; i <= 10000; i++) {
    prev_time = current_time;      
    current_time = micros();      
    dt = (current_time - prev_time)/1000000.0; 
    getIMUdata();
    Madgwick(GyroX, -GyroY, -GyroZ, -AccX, AccY, AccZ, MagY, -MagX, MagZ, dt);
    loopRate(2000); // Ne pas dépasser 2KHz
  }
}

void calibrateMagnetometer() {
  #if defined USE_MPU9250_SPI 
    float success;
    Serial.println("Beginning magnetometer calibration in");
    Serial.println("3...");
    delay(1000);
    Serial.println("2...");
    delay(1000);
    Serial.println("1...");
    delay(1000);
    Serial.println("Rotate the IMU about all axes until complete.");
    Serial.println(" ");
    success = mpu9250.calibrateMag();
    if(success) {
      Serial.println("Calibration Successful!");
      Serial.println("Please comment out the calibrateMagnetometer() function and copy these values into the code:");
      Serial.print("float MagErrorX = ");
      Serial.print(mpu9250.getMagBiasX_uT());
      Serial.println(";");
      Serial.print("float MagErrorY = ");
      Serial.print(mpu9250.getMagBiasY_uT());
      Serial.println(";");
      Serial.print("float MagErrorZ = ");
      Serial.print(mpu9250.getMagBiasZ_uT());
      Serial.println(";");
      Serial.print("float MagScaleX = ");
      Serial.print(mpu9250.getMagScaleFactorX());
      Serial.println(";");
      Serial.print("float MagScaleY = ");
      Serial.print(mpu9250.getMagScaleFactorY());
      Serial.println(";");
      Serial.print("float MagScaleZ = ");
      Serial.print(mpu9250.getMagScaleFactorZ());
      Serial.println(";");
      Serial.println(" ");
      Serial.println("If you are having trouble with your attitude estimate at a new flying location, repeat this process as needed.");
    }
    else {
      Serial.println("Calibration Unsuccessful. Please reset the board and try again.");
    }
  
    while(1); // Arrête le code ici pour ne pas entrer dans la boucle principale jusqu'à ce que la fonction soit décommentée
  #endif
  Serial.println("Error: MPU9250 not selected. Cannot calibrate non-existent magnetometer.");
  while(1); // Arrête le code ici pour ne pas entrer dans la boucle principale jusqu'à ce que la fonction soit décommentée
}