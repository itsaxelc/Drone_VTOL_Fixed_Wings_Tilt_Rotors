#pragma once

void Madgwick(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float invSampleFreq);

void Madgwick6DOF(float gx, float gy, float gz, float ax, float ay, float az, float invSampleFreq);

float invSqrt(float x);