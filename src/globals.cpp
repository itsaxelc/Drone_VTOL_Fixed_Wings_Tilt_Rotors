#include "configs/configs.h"   // adjust to whatever you name/keep the header as
#include "../lib/MPU6050/MPU6050.h"

MPU6050 mpu6050;
PWMServo servo1;
PWMServo servo2;
PWMServo servo3;

unsigned long channel_1_fs = 1000;
unsigned long channel_2_fs = 1500;
unsigned long channel_3_fs = 1500;
unsigned long channel_4_fs = 1500;
unsigned long channel_5_fs = 2000;
unsigned long channel_6_fs = 1000;

float B_madgwick = 0.04;
float B_accel = 0.2;
float B_gyro = 0.2;
float B_mag = 1.0;

float MagErrorX = 0.0, MagErrorY = 0.0, MagErrorZ = 0.0;
float MagScaleX = 1.0, MagScaleY = 1.0, MagScaleZ = 1.0;

float AccErrorX = 0.02, AccErrorY = -0.01, AccErrorZ = -0.08;
float GyroErrorX = 5.56, GyroErrorY = 0.08, GyroErrorZ = -0.51;

float i_limit = 25.0;
float maxRoll = 50.0;
float maxPitch = 50.0;
float maxYaw = 220.0;

float Kp_roll_angle = 0.2, Ki_roll_angle = 0.3, Kd_roll_angle = 0.05, B_loop_roll = 0.9;
float Kp_pitch_angle = 0.2, Ki_pitch_angle = 0.3, Kd_pitch_angle = 0.05, B_loop_pitch = 0.9;
float Kp_roll_rate = 0.15, Ki_roll_rate = 0.2, Kd_roll_rate = 0.0002;
float Kp_pitch_rate = 0.15, Ki_pitch_rate = 0.2, Kd_pitch_rate = 0.0002;
float Kp_yaw = 0.3, Ki_yaw = 0.05, Kd_yaw = 0.00015;

float dt;
unsigned long current_time, prev_time;
unsigned long print_counter = micros();
unsigned long blink_counter, blink_delay;
bool blinkAlternate;

unsigned long channel_1_pwm, channel_2_pwm, channel_3_pwm, channel_4_pwm, channel_5_pwm, channel_6_pwm;
unsigned long channel_1_pwm_prev, channel_2_pwm_prev, channel_3_pwm_prev, channel_4_pwm_prev;

float AccX, AccY, AccZ;
float AccX_prev, AccY_prev, AccZ_prev;
float GyroX, GyroY, GyroZ;
float GyroX_prev, GyroY_prev, GyroZ_prev;
float MagX = 0, MagY = 0, MagZ = 0;
float MagX_prev, MagY_prev, MagZ_prev;
float roll_IMU, pitch_IMU, yaw_IMU;
float roll_IMU_prev, pitch_IMU_prev;
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

float thro_des, roll_des, pitch_des, yaw_des;

float error_roll, error_roll_prev, roll_des_prev, integral_roll, integral_roll_il, integral_roll_ol, integral_roll_prev, integral_roll_prev_il, integral_roll_prev_ol, derivative_roll, roll_PID = 0;
float error_pitch, error_pitch_prev, pitch_des_prev, integral_pitch, integral_pitch_il, integral_pitch_ol, integral_pitch_prev, integral_pitch_prev_il, integral_pitch_prev_ol, derivative_pitch, pitch_PID = 0;
float error_yaw, error_yaw_prev, integral_yaw, integral_yaw_prev, derivative_yaw, yaw_PID = 0;

float m1_command_scaled, m2_command_scaled;
int m1_command_PWM, m2_command_PWM;
float s1_command_scaled, s2_command_scaled, s3_command_scaled;
int s1_command_PWM, s2_command_PWM, s3_command_PWM;

unsigned long rising_edge_start_1, rising_edge_start_2, rising_edge_start_3, rising_edge_start_4, rising_edge_start_5, rising_edge_start_6;
unsigned long channel_1_raw, channel_2_raw, channel_3_raw, channel_4_raw, channel_5_raw, channel_6_raw;
int ppm_counter = 0;
unsigned long time_ms = 0;

bool armedFly = false;

float transition_fader = 1.0;
float faderate = 3.0;

float kp_pitch_angle_hover = 1.17, kd_pitch_angle_hover = 0.09;
float kp_roll_angle_hover = 0.36, kd_roll_angle_hover = 0.07;
float kp_yaw_hover = 0.45, kd_yaw_hover = 0.0003;
float kp_pitch_angle_ff = 0.25, kd_pitch_angle_ff = 0.05;
float kp_roll_angle_ff = 0.5, kd_roll_angle_ff = 0.06;
float kp_yaw_ff = 0.2, kd_yaw_ff = 0.00017;

float tilt_servo_cruise_pos = 0.0;
float tilt_servo_hover_pos = 0.5;

float elevator_trim = 0.5;

float k_pitch_tilt_collective = 0.15;
float k_yaw_tilt_diff = 0.1;

float ff_roll_yaw_mix_amount = 30;
float ff_pitch_trim = 5;