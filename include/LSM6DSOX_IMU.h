// LSM6DSOX IMU implementation.
// Inherits from IMUInterface. All code inline for header-only.

#ifndef LSM6DSOX_IMU_H
#define LSM6DSOX_IMU_H

#include <Arduino.h>
#include <Adafruit_LSM6DSOX.h> // Specific for LSM6DSOX
#include "IMUInterface.h"

class LSM6DSOX_IMU : public IMUInterface
{
public:
    LSM6DSOX_IMU() : latestAccelerometerX(0.0), latestAccelerometerY(0.0), latestAccelerometerZ(0.0),
                     latestGyroscopeX(0.0), latestGyroscopeY(0.0), latestGyroscopeZ(0.0),
                     latestTemperature(0.0), latestTimestampMilliseconds(0),
                     sampleCount(0)
    {
        resetCovarianceAccumulators();
    }

    bool begin() override
    {
        return sensorInstance.begin_I2C(); // Init I2C
    }

    void readSensorData() override
    {
        sensors_event_t accelEvent, gyroEvent, tempEvent;
        sensorInstance.getEvent(&accelEvent, &gyroEvent, &tempEvent);

        latestAccelerometerX = accelEvent.acceleration.x;
        latestAccelerometerY = accelEvent.acceleration.y;
        latestAccelerometerZ = accelEvent.acceleration.z;

        latestGyroscopeX = gyroEvent.gyro.x;
        latestGyroscopeY = gyroEvent.gyro.y;
        latestGyroscopeZ = gyroEvent.gyro.z;

        latestTemperature = tempEvent.temperature;

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
    const float *getAccelCovMatrix() const { return accelCovMatrix; }
    const float *getGyroCovMatrix() const { return gyroCovMatrix; }

    unsigned long getTimestampMilliseconds() const override { return latestTimestampMilliseconds; }

    // Optional: Reset accumulators to prevent overflow in long runs
    void resetCovarianceAccumulators()
    {
        sumAccelX = sumAccelY = sumAccelZ = 0.0;
        sumAccelXX = sumAccelYY = sumAccelZZ = 0.0;
        sumAccelXY = sumAccelXZ = sumAccelYZ = 0.0;
        sumGyroX = sumGyroY = sumGyroZ = 0.0;
        sumGyroXX = sumGyroYY = sumGyroZZ = 0.0;
        sumGyroXY = sumGyroXZ = sumGyroYZ = 0.0;
        sampleCount = 0;
        memset(accelCovMatrix, 0, sizeof(accelCovMatrix));
        memset(gyroCovMatrix, 0, sizeof(gyroCovMatrix));
    }

private:
    Adafruit_LSM6DSOX sensorInstance;

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

#endif // LSM6DSOX_IMU_H
