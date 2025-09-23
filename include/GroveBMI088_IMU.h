// Grove BMI088 IMU implementation.
// Inherits from IMUInterface. All code inline for header-only.

#ifndef GROVE_BMI088_IMU_H
#define GROVE_BMI088_IMU_H

#include <Arduino.h>
#include <Wire.h>
#include "BMI088.h" // Seeed Grove BMI088 library
#include "IMUInterface.h"
#include <string.h> // For memset

class GroveBMI088_IMU : public IMUInterface
{
public:
    GroveBMI088_IMU() : bmi088(BMI088_ACC_ALT_ADDRESS, BMI088_GYRO_ALT_ADDRESS), // Default addresses (accel 0x18, gyro 0x68)
                        latestAccelerometerX(0.0f), latestAccelerometerY(0.0f), latestAccelerometerZ(0.0f),
                        latestGyroscopeX(0.0f), latestGyroscopeY(0.0f), latestGyroscopeZ(0.0f),
                        latestTemperature(0.0f), latestTimestampMilliseconds(0),
                        sampleCount(0)
    {
        resetCovarianceAccumulators();
    }

    bool begin() override
    {
        // Init I2C (use Wire; adjust if using Wire1 on RP2350)
        Wire.begin();

        // Init BMI088 (handles both accel and gyro internally)
        bmi088.initialize();

        // Check connection
        return bmi088.isConnection();
    }

    void readSensorData() override
    {
        // Read accel (in m/s²)
        float ax, ay, az;
        bmi088.getAcceleration(&ax, &ay, &az);
        latestAccelerometerX = ax;
        latestAccelerometerY = ay;
        latestAccelerometerZ = az;

        // Read gyro (in rad/s)
        float gx, gy, gz;
        bmi088.getGyroscope(&gx, &gy, &gz);
        latestGyroscopeX = gx;
        latestGyroscopeY = gy;
        latestGyroscopeZ = gz;

        // Read temperature (°C) - library returns int16_t, convert
        latestTemperature = static_cast<float>(bmi088.getTemperature()) / 100.0f; // Assuming library scale

        latestTimestampMilliseconds = millis();

        // Accumulate for accel covariance
        sumAccelX += latestAccelerometerX;
        sumAccelY += latestAccelerometerY;
        sumAccelZ += latestAccelerometerZ;
        sumAccelXX += latestAccelerometerX * latestAccelerometerX;
        sumAccelYY += latestAccelerometerY * latestAccelerometerY;
        sumAccelZZ += latestAccelerometerZ * latestAccelerometerZ;
        sumAccelXY += latestAccelerometerX * latestAccelerometerY;
        sumAccelXZ += latestAccelerometerX * latestAccelerometerZ;
        sumAccelYZ += latestAccelerometerY * latestAccelerometerZ;

        // Accumulate for gyro covariance
        sumGyroX += latestGyroscopeX;
        sumGyroY += latestGyroscopeY;
        sumGyroZ += latestGyroscopeZ;
        sumGyroXX += latestGyroscopeX * latestGyroscopeX;
        sumGyroYY += latestGyroscopeY * latestGyroscopeY;
        sumGyroZZ += latestGyroscopeZ * latestGyroscopeZ;
        sumGyroXY += latestGyroscopeX * latestGyroscopeY;
        sumGyroXZ += latestGyroscopeX * latestGyroscopeZ;
        sumGyroYZ += latestGyroscopeY * latestGyroscopeZ;

        sampleCount++;
    }

    float getAccelerometerX() const override { return latestAccelerometerX; }
    float getAccelerometerY() const override { return latestAccelerometerY; }
    float getAccelerometerZ() const override { return latestAccelerometerZ; }
    float getGyroscopeX() const override { return latestGyroscopeX; }
    float getGyroscopeY() const override { return latestGyroscopeY; }
    float getGyroscopeZ() const override { return latestGyroscopeZ; }
    float getTemperature() const override { return latestTemperature; }

    // Orientation not supported by BMI088; use defaults
    float getOrientationX() const override { return 0.0f; }
    float getOrientationY() const override { return 0.0f; }
    float getOrientationZ() const override { return 0.0f; }
    float getOrientationW() const override { return 1.0f; }

    void computeCovariances() override
    {
        if (sampleCount < 2)
        {
            // Not enough samples; zero matrices
            memset(accelCovMatrix, 0, sizeof(accelCovMatrix));
            memset(gyroCovMatrix, 0, sizeof(gyroCovMatrix));
            return;
        }

        // Accel means
        float meanAccelX = sumAccelX / sampleCount;
        float meanAccelY = sumAccelY / sampleCount;
        float meanAccelZ = sumAccelZ / sampleCount;

        // Accel covariances
        accelCovMatrix[0] = (sumAccelXX / sampleCount) - (meanAccelX * meanAccelX); // Var(X)
        accelCovMatrix[1] = (sumAccelXY / sampleCount) - (meanAccelX * meanAccelY); // Cov(X,Y)
        accelCovMatrix[2] = (sumAccelXZ / sampleCount) - (meanAccelX * meanAccelZ); // Cov(X,Z)
        accelCovMatrix[3] = accelCovMatrix[1];                                      // Cov(Y,X) = Cov(X,Y)
        accelCovMatrix[4] = (sumAccelYY / sampleCount) - (meanAccelY * meanAccelY); // Var(Y)
        accelCovMatrix[5] = (sumAccelYZ / sampleCount) - (meanAccelY * meanAccelZ); // Cov(Y,Z)
        accelCovMatrix[6] = accelCovMatrix[2];                                      // Cov(Z,X) = Cov(X,Z)
        accelCovMatrix[7] = accelCovMatrix[5];                                      // Cov(Z,Y) = Cov(Y,Z)
        accelCovMatrix[8] = (sumAccelZZ / sampleCount) - (meanAccelZ * meanAccelZ); // Var(Z)

        // Gyro means
        float meanGyroX = sumGyroX / sampleCount;
        float meanGyroY = sumGyroY / sampleCount;
        float meanGyroZ = sumGyroZ / sampleCount;

        // Gyro covariances
        gyroCovMatrix[0] = (sumGyroXX / sampleCount) - (meanGyroX * meanGyroX); // Var(X)
        gyroCovMatrix[1] = (sumGyroXY / sampleCount) - (meanGyroX * meanGyroY); // Cov(X,Y)
        gyroCovMatrix[2] = (sumGyroXZ / sampleCount) - (meanGyroX * meanGyroZ); // Cov(X,Z)
        gyroCovMatrix[3] = gyroCovMatrix[1];                                    // Cov(Y,X)
        gyroCovMatrix[4] = (sumGyroYY / sampleCount) - (meanGyroY * meanGyroY); // Var(Y)
        gyroCovMatrix[5] = (sumGyroYZ / sampleCount) - (meanGyroY * meanGyroZ); // Cov(Y,Z)
        gyroCovMatrix[6] = gyroCovMatrix[2];                                    // Cov(Z,X)
        gyroCovMatrix[7] = gyroCovMatrix[5];                                    // Cov(Z,Y)
        gyroCovMatrix[8] = (sumGyroZZ / sampleCount) - (meanGyroZ * meanGyroZ); // Var(Z)
    }

    float getAccelerometerCovariance() const override { return accelCovMatrix[0]; } // Example: return Var(X); update if needed
    float getGyroscopeCovariance() const override { return gyroCovMatrix[0]; }      // Example: return Var(X)

    // New getters for full matrices (used in JSON formatter)
    const float *getAccelCovMatrix() const override { return accelCovMatrix; }
    const float *getGyroCovMatrix() const override { return gyroCovMatrix; }

    unsigned long getTimestampMilliseconds() const override { return latestTimestampMilliseconds; }

    // Optional: Reset accumulators to prevent overflow in long runs
    void resetCovarianceAccumulators()
    {
        sumAccelX = sumAccelY = sumAccelZ = 0.0f;
        sumAccelXX = sumAccelYY = sumAccelZZ = 0.0f;
        sumAccelXY = sumAccelXZ = sumAccelYZ = 0.0f;
        sumGyroX = sumGyroY = sumGyroZ = 0.0f;
        sumGyroXX = sumGyroYY = sumGyroZZ = 0.0f;
        sumGyroXY = sumGyroXZ = sumGyroYZ = 0.0f;
        sampleCount = 0;
        memset(accelCovMatrix, 0, sizeof(accelCovMatrix));
        memset(gyroCovMatrix, 0, sizeof(gyroCovMatrix));
    }

private:
    BMI088 bmi088; // Single instance for both accel and gyro

    float latestAccelerometerX, latestAccelerometerY, latestAccelerometerZ;
    float latestGyroscopeX, latestGyroscopeY, latestGyroscopeZ;
    float latestTemperature;
    unsigned long latestTimestampMilliseconds;

    // Accumulators for covariance
    float sumAccelX, sumAccelY, sumAccelZ;
    float sumAccelXX, sumAccelYY, sumAccelZZ;
    float sumAccelXY, sumAccelXZ, sumAccelYZ;
    float sumGyroX, sumGyroY, sumGyroZ;
    float sumGyroXX, sumGyroYY, sumGyroZZ;
    float sumGyroXY, sumGyroXZ, sumGyroYZ;
    int sampleCount;

    // Covariance matrices (3x3, row-major)
    float accelCovMatrix[9];
    float gyroCovMatrix[9];
};

#endif // GROVE_BMI088_IMU_H
