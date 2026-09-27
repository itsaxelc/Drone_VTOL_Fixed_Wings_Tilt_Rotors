#include<Arduino.h>
#include<math.h>
#include "configs/configs.h"
#include "madgwick.h"

// =========================
// Fonctions communes
// =========================

static void gyroToRadians(float& gx, float& gy, float& gz)
{
    // degrees/s -> radians/s

    gx *= DEG_TO_RAD;
    gy *= DEG_TO_RAD;
    gz *= DEG_TO_RAD;
}


static void computeQuaternionDerivative(float gx, float gy, float gz,
            float& qDot1, float& qDot2, float& qDot3, float& qDot4)
{
    // dq/dt = 0.5 * q ⊗ omega_q

    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);
}


static bool normalizeVector(float& x, float& y, float& z)
{
    float norm = x * x + y * y + z * z;

    if (norm == 0.0f)
        return false;

    float recipNorm = invSqrt(norm);

    x *= recipNorm;
    y *= recipNorm;
    z *= recipNorm;

    return true;
}


static void normalizeQuaternion()
{
    float recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);

    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}


static void integrateQuaternion(float qDot1, float qDot2, float qDot3, float qDot4,
                                float invSampleFreq)
{
    q0 += qDot1 * invSampleFreq;
    q1 += qDot2 * invSampleFreq;
    q2 += qDot3 * invSampleFreq;
    q3 += qDot4 * invSampleFreq;
}


static void updateEulerAngles()
{
    roll_IMU = atan2(q0 * q1 + q2 * q3, 0.5f - q1 * q1 - q2 * q2) * RAD_TO_DEG;

    pitch_IMU = -asin(-2.0f * (q1 * q3 - q0 * q2)) * RAD_TO_DEG;

    yaw_IMU = -atan2(q1 * q2 + q0 * q3, 0.5f - q2 * q2 - q3 * q3) * RAD_TO_DEG;
}


// =========================
// Spécifique 9DOF
// =========================

static void computeMagneticReference(float mx, float my, float mz, 
  float& _2bx, float& _2bz, float& _4bx, float& _4bz)
{
    float _2q0 = 2.0f * q0;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;

    float _2q0mx = 2.0f * q0 * mx;
    float _2q0my = 2.0f * q0 * my;
    float _2q0mz = 2.0f * q0 * mz;
    float _2q1mx = 2.0f * q1 * mx;

    float q0q0 = q0 * q0;
    float q1q1 = q1 * q1;
    float q2q2 = q2 * q2;
    float q3q3 = q3 * q3;

    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q3 = q2 * q3;

    float hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 
              + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;

    float hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + 
              _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;

    float hz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + 
              _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;

    _2bx = sqrtf(hx * hx + hy * hy);
    _2bz = hz;

    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;
}


static void computeGradient9DOF(float ax, float ay, float az, float mx, float my, float mz, 
  float _2bx, float _2bz, float _4bx, float _4bz, float& s0, float& s1, float& s2, float& s3)
{
    float _2q0 = 2.0f * q0;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;

    float _2q0q2 = 2.0f * q0 * q2;
    float _2q2q3 = 2.0f * q2 * q3;

    float q0q0 = q0 * q0;
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;

    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;

    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;

    float q3q3 = q3 * q3;

    s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay)
         - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx)
        + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
        + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);

    s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay)
        - 4.0f * q1 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az)
        + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx)
        + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
        + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);

    s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay)
        - 4.0f * q2 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az)
        + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx)
        + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
        + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);

    s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay)
        + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx)
        + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
        + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
}


// =========================
// Spécifique 6DOF
// =========================

static void computeGradient6DOF(
    float ax, float ay, float az,
    float& s0, float& s1,
    float& s2, float& s3)
{
    float _2q0 = 2.0f * q0;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;

    float _4q0 = 4.0f * q0;
    float _4q1 = 4.0f * q1;
    float _4q2 = 4.0f * q2;

    float _8q1 = 8.0f * q1;
    float _8q2 = 8.0f * q2;

    float q0q0 = q0 * q0;
    float q1q1 = q1 * q1;
    float q2q2 = q2 * q2;
    float q3q3 = q3 * q3;

    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;

    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1
        + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;

    s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2
        + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;

    s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
}


// =========================
// Fonctions communes de correction
// =========================

static bool normalizeGradient(
    float& s0, float& s1,
    float& s2, float& s3)
{
    float norm = s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3;

    if (norm == 0.0f) return false;

    float recipNorm = invSqrt(norm);

    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    return true;
}


static void applyFeedback(float& qDot1, float& qDot2, float& qDot3, float& qDot4,
                          float s0, float s1, float s2, float s3)
{
    qDot1 -= B_madgwick * s0;
    qDot2 -= B_madgwick * s1;
    qDot3 -= B_madgwick * s2;
    qDot4 -= B_madgwick * s3;
}

void Madgwick(float gx, float gy, float gz, float ax, float ay, float az,
              float mx, float my, float mz,
              float invSampleFreq)
{
    // Si le magnétomètre n'est pas disponible
    #if defined USE_MPU6050_I2C
        Madgwick6DOF(gx, gy, gz,ax, ay, az, invSampleFreq);
        return;
    #endif

    // Si les données magnétiques sont invalides
    if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f))
    {
        Madgwick6DOF(gx, gy, gz, ax, ay, az, invSampleFreq);
        return;
    }

    // Gyroscope : degrees/s -> radians/s
    gyroToRadians(gx, gy, gz);

    // Propagation gyroscope
    float qDot1, qDot2, qDot3, qDot4;

    computeQuaternionDerivative(gx, gy, gz, qDot1, qDot2, qDot3, qDot4);

    // Accéléromètre valide ?
    if (normalizeVector(ax, ay, az))
    {
        // Normalisation magnétomètre
        if (normalizeVector(mx, my, mz))
        {
            float _2bx, _2bz;
            float _4bx, _4bz;

            computeMagneticReference(mx, my, mz, _2bx, _2bz, _4bx, _4bz);

            float s0, s1, s2, s3;

            computeGradient9DOF(ax, ay, az, mx, my, mz, _2bx, _2bz, _4bx, _4bz, s0, s1, s2, s3);

            if (normalizeGradient(s0, s1, s2, s3))
            {
                applyFeedback( qDot1, qDot2, qDot3, qDot4, s0, s1, s2, s3);
            }
        }
    }

    // Intégration
    integrateQuaternion( qDot1, qDot2, qDot3, qDot4, invSampleFreq);

    // Normalisation
    normalizeQuaternion();

    // Quaternion -> angles
    updateEulerAngles();
}

void Madgwick6DOF(float gx, float gy, float gz, float ax, float ay, float az, float invSampleFreq)
{
    // Gyroscope : degrees/s -> radians/s
    gyroToRadians(gx, gy, gz);

    // Propagation gyroscope
    float qDot1, qDot2, qDot3, qDot4;

    computeQuaternionDerivative(gx, gy, gz, qDot1, qDot2, qDot3, qDot4);

    // Correction accéléromètre
    if (normalizeVector(ax, ay, az))
    {
        float s0, s1, s2, s3;

        computeGradient6DOF(ax, ay, az, s0, s1, s2, s3);

        if (normalizeGradient(s0, s1, s2, s3))
        {
            applyFeedback(qDot1, qDot2, qDot3, qDot4, s0, s1, s2, s3);
        }
    }

    // Intégration
    integrateQuaternion(qDot1, qDot2, qDot3, qDot4, invSampleFreq);

    // Normalisation
    normalizeQuaternion();

    // Quaternion -> angles
    updateEulerAngles();
}

float invSqrt(float x) {
  return 1.0/sqrtf(x);
}