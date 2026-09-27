#pragma once

#define USE_PWM_RX
//#define USE_PPM_RX

#define USE_MPU6050_I2C
//#define USE_MPU9250_SPI

#define GYRO_250DPS
//#define GYRO_500DPS
//#define GYRO_1000DPS
//#define GYRO_2000DPS

#define ACCEL_2G
//#define ACCEL_4G
//#define ACCEL_8G
//#define ACCEL_16G

#include <Wire.h>
#include <SPI.h>
#include <PWMServo.h>

#if defined USE_MPU6050_I2C
  #include "../lib/MPU6050/MPU6050.h"
  extern MPU6050 mpu6050;               // <-- was: MPU6050 mpu6050;
#elif defined USE_MPU9250_SPI
  #include "src/MPU9250/MPU9250.h"
  extern MPU9250 mpu9250;
#else
  #error No MPU defined... 
#endif

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

const int ch1Pin = 15;
const int ch2Pin = 16;
const int ch3Pin = 17;
const int ch4Pin = 20;
const int ch5Pin = 21;
const int ch6Pin = 22;
const int PPM_Pin = 23;
const int m1Pin = 0;
const int m2Pin = 1;
const int servo1Pin = 6;
const int servo2Pin = 7;
const int servo3Pin = 8;

// ---- Everything below: declarations only ----

extern unsigned long channel_1_fs, channel_2_fs, channel_3_fs, channel_4_fs, channel_5_fs, channel_6_fs;

extern float B_madgwick, B_accel, B_gyro, B_mag;

extern float MagErrorX, MagErrorY, MagErrorZ, MagScaleX, MagScaleY, MagScaleZ;
extern float AccErrorX, AccErrorY, AccErrorZ, GyroErrorX, GyroErrorY, GyroErrorZ;

extern float i_limit, maxRoll, maxPitch, maxYaw;

extern float Kp_roll_angle, Ki_roll_angle, Kd_roll_angle, B_loop_roll;
extern float Kp_pitch_angle, Ki_pitch_angle, Kd_pitch_angle, B_loop_pitch;
extern float Kp_roll_rate, Ki_roll_rate, Kd_roll_rate;
extern float Kp_pitch_rate, Ki_pitch_rate, Kd_pitch_rate;
extern float Kp_yaw, Ki_yaw, Kd_yaw;

// Pins can stay as `const int` in the header — const globals have internal linkage
// by default in C++, so no ODR problem there. Leave ch1Pin...servo3Pin as-is.

extern PWMServo servo1, servo2, servo3;

extern float dt;
extern unsigned long current_time, prev_time;
extern unsigned long print_counter;
extern unsigned long blink_counter, blink_delay;
extern bool blinkAlternate;

extern unsigned long channel_1_pwm, channel_2_pwm, channel_3_pwm, channel_4_pwm, channel_5_pwm, channel_6_pwm;
extern unsigned long channel_1_pwm_prev, channel_2_pwm_prev, channel_3_pwm_prev, channel_4_pwm_prev;

extern float AccX, AccY, AccZ;
extern float AccX_prev, AccY_prev, AccZ_prev;
extern float GyroX, GyroY, GyroZ;
extern float GyroX_prev, GyroY_prev, GyroZ_prev;
extern float MagX, MagY, MagZ;
extern float MagX_prev, MagY_prev, MagZ_prev;
extern float roll_IMU, pitch_IMU, yaw_IMU;
extern float roll_IMU_prev, pitch_IMU_prev;
extern float q0, q1, q2, q3;

extern float thro_des, roll_des, pitch_des, yaw_des;

extern float error_roll, error_roll_prev, roll_des_prev, integral_roll, integral_roll_il, integral_roll_ol, integral_roll_prev, integral_roll_prev_il, integral_roll_prev_ol, derivative_roll, roll_PID;
extern float error_pitch, error_pitch_prev, pitch_des_prev, integral_pitch, integral_pitch_il, integral_pitch_ol, integral_pitch_prev, integral_pitch_prev_il, integral_pitch_prev_ol, derivative_pitch, pitch_PID;
extern float error_yaw, error_yaw_prev, integral_yaw, integral_yaw_prev, derivative_yaw, yaw_PID;

extern float m1_command_scaled, m2_command_scaled;
extern int m1_command_PWM, m2_command_PWM;
extern float s1_command_scaled, s2_command_scaled, s3_command_scaled;
extern int s1_command_PWM, s2_command_PWM, s3_command_PWM;

extern unsigned long rising_edge_start_1, rising_edge_start_2, rising_edge_start_3, rising_edge_start_4, rising_edge_start_5, rising_edge_start_6;
extern unsigned long channel_1_raw, channel_2_raw, channel_3_raw, channel_4_raw, channel_5_raw, channel_6_raw;
extern int ppm_counter;
extern unsigned long time_ms;

extern bool armedFly;

extern float transition_fader, faderate;

extern float kp_pitch_angle_hover, kd_pitch_angle_hover, kp_roll_angle_hover, kd_roll_angle_hover, kp_yaw_hover, kd_yaw_hover;
extern float kp_pitch_angle_ff, kd_pitch_angle_ff, kp_roll_angle_ff, kd_roll_angle_ff, kp_yaw_ff, kd_yaw_ff;

extern float tilt_servo_cruise_pos, tilt_servo_hover_pos;
extern float elevator_trim;
extern float k_pitch_tilt_collective, k_yaw_tilt_diff;
extern float ff_roll_yaw_mix_amount, ff_pitch_trim;