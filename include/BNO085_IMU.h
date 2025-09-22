// BNO085 IMU implementation.
// Inherits from IMUInterface. All code inline for header-only.

#ifndef BNO085_IMU_H
#define BNO085_IMU_H

#include <Arduino.h>
#include <Adafruit_BNO08x.h> // Adafruit library for BNO085/BNO08x
#include "IMUInterface.h"
#include <string.h> // For memset

class BNO085_IMU : public IMUInterface
{
public:
    BNO085_IMU() : latestAccelerometerX(0.0f), latestAccelerometerY(0.0f), latestAccelerometerZ(0.0f),
                   latestGyroscopeX(0.0f), latestGyroscopeY(0.0f), latestGyroscopeZ(0.0f),
                   latestTemperature(0.0f), latestTimestampMilliseconds(0),
                   latestOrientationX(0.0f), latestOrientationY(0.0f), latestOrientationZ(0.0f), latestOrientationW(1.0f),
                   sampleCount(0)
    {
        resetCovarianceAccumulators();
    }

    bool begin() override
    {
        if (!sensorInstance.begin_I2C())
        { // Default address 0x4A; change to 0x4B if jumpered
            return false;
        }
        // Enable required reports (100Hz default rate)
        sensorInstance.enableReport(SH2_LINEAR_ACCELERATION);
        sensorInstance.enableReport(SH2_GYROSCOPE_CALIBRATED);
        sensorInstance.enableReport(SH2_GAME_ROTATION_VECTOR); // For orientation quaternion (no mag, smooth)
        sensorInstance.enableReport(SH2_TEMPERATURE);
        delay(500); // Allow initial calibration/stabilization
        return true;
    }

    void readSensorData() override
    {
        sh2_SensorValue_t sensorValue;

        // Poll for new data (BNO08x updates asynchronously)
        while (sensorInstance.getSensorEvent(&sensorValue))
        {
            switch (sensorValue.sensorId)
            {
            case SH2_LINEAR_ACCELERATION:
                latestAccelerometerX = sensorValue.un.linearAcceleration.x;
                latestAccelerometerY = sensorValue.un.linearAcceleration.y;
                latestAccelerometerZ = sensorValue.un.linearAcceleration.z;
                break;
            case SH2_GYROSCOPE_CALIBRATED:
                latestGyroscopeX = sensorValue.un.gyroscope.x;
                latestGyroscopeY = sensorValue.un.gyroscope.y;
                latestGyroscopeZ = sensorValue.un.gyroscope.z;
                break;
            case SH2_GAME_ROTATION_VECTOR:
                latestOrientationX = sensorValue.un.gameRotationVector.i;
                latestOrientationY = sensorValue.un.gameRotationVector.j;
                latestOrientationZ = sensorValue.un.gameRotationVector.k;
                latestOrientationW = sensorValue.un.gameRotationVector.real;
                break;
            case SH2_TEMPERATURE:
                latestTemperature = sensorValue.un.temperature.value;
                break;
            default:
                break;
            }
        }

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
    float getOrientationX() const override { return latestOrientationX; }
    float getOrientationY() const override { return latestOrientationY; }
    float getOrientationZ() const override { return latestOrientationZ; }
    float getOrientationW() const override { return latestOrientationW; }

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
    Adafruit_BNO08x sensorInstance;

    float latestAccelerometerX, latestAccelerometerY, latestAccelerometerZ;
    float latestGyroscopeX, latestGyroscopeY, latestGyroscopeZ;
    float latestTemperature;
    unsigned long latestTimestampMilliseconds;

    float latestOrientationX, latestOrientationY, latestOrientationZ, latestOrientationW;

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

#endif // BNO085_IMU_H
