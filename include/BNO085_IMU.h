// BNO085 IMU implementation.
// Inherits from IMUInterface. All code inline for header-only.

#ifndef BNO085_IMU_H
#define BNO085_IMU_H

#include <Arduino.h>
#include <Adafruit_BNO08x.h> // Adafruit library for BNO085/BNO08x
#include "IMUInterface.h"
#include "IMUCommon.h"
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
        accelAccumulator.reset();
        gyroAccumulator.reset();
        orientAccumulator.reset();
        memset(accelCovMatrix, 0, sizeof(accelCovMatrix));
        memset(gyroCovMatrix, 0, sizeof(gyroCovMatrix));
        memset(orientCovMatrix, 0, sizeof(orientCovMatrix));
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

        // Configure covariance accumulator defaults
        accelAccumulator.setWindowSize(200);
        gyroAccumulator.setWindowSize(200);
        orientAccumulator.setWindowSize(200);
        accelAccumulator.setVarianceEpsilon(1e-9f);
        gyroAccumulator.setVarianceEpsilon(1e-9f);
        orientAccumulator.setVarianceEpsilon(1e-9f);
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

        // Accumulate sensor and orientation samples
        accelAccumulator.addSample(latestAccelerometerX, latestAccelerometerY, latestAccelerometerZ);
        gyroAccumulator.addSample(latestGyroscopeX, latestGyroscopeY, latestGyroscopeZ);
        // Track the vector part (i,j,k) of the quaternion for simple covariance
        orientAccumulator.addSample(latestOrientationX, latestOrientationY, latestOrientationZ);
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
        // Compute covariance matrices using the unbiased sample estimator:
        //   mean_x = (1/n) * Σ x_i
        //   cov_xy = (1/(n-1)) * Σ (x_i - mean_x) (y_i - mean_y)
        // Implementation uses sum and cross-product accumulators for numerical efficiency.
        accelAccumulator.computeCovMatrix(accelCovMatrix);
        gyroAccumulator.computeCovMatrix(gyroCovMatrix);
        // Orientation covariance based on quaternion vector part (i,j,k). If there are
        // insufficient samples orientCovMatrix will be zeroed by the accumulator.
        orientAccumulator.computeCovMatrix(orientCovMatrix);
    }

    float getAccelerometerCovariance() const override { return accelCovMatrix[0]; } // Example: return Var(X); update if needed
    float getGyroscopeCovariance() const override { return gyroCovMatrix[0]; }      // Example: return Var(X)

    // New getters for full matrices (used in JSON formatter)
    const float *getAccelCovMatrix() const override { return accelCovMatrix; }
    const float *getGyroCovMatrix() const override { return gyroCovMatrix; }
    // Orientation is provided by this sensor (game rotation vector). Indicate support.
    bool hasOrientation() const override { return true; }
    // Provide a 3x3 orientation covariance matrix computed over the quaternion
    // vector part (i,j,k). This is a pragmatic choice: many drivers do not provide
    // a full orientation covariance; using the vector part gives some notion of
    // variability while remaining simple.
    const float *getOrientationCovMatrix() const override { return orientCovMatrix; }

    unsigned long getTimestampMilliseconds() const override { return latestTimestampMilliseconds; }

    // Optional: Reset accumulators to prevent overflow in long runs
    void resetCovarianceAccumulators()
    {
        accelAccumulator.reset();
        gyroAccumulator.reset();
        orientAccumulator.reset();
        sampleCount = 0;
        memset(accelCovMatrix, 0, sizeof(accelCovMatrix));
        memset(gyroCovMatrix, 0, sizeof(gyroCovMatrix));
        memset(orientCovMatrix, 0, sizeof(orientCovMatrix));
    }

private:
    Adafruit_BNO08x sensorInstance;

    float latestAccelerometerX, latestAccelerometerY, latestAccelerometerZ;
    float latestGyroscopeX, latestGyroscopeY, latestGyroscopeZ;
    float latestTemperature;
    unsigned long latestTimestampMilliseconds;

    float latestOrientationX, latestOrientationY, latestOrientationZ, latestOrientationW;

    // Accumulators for covariance (reuse common helper)
    CovarianceAccumulator accelAccumulator;
    CovarianceAccumulator gyroAccumulator;
    // Track quaternion vector part for orientation covariance (i,j,k)
    CovarianceAccumulator orientAccumulator;
    int sampleCount;

    // Covariance matrices (3x3, row-major)
    float accelCovMatrix[9];
    float gyroCovMatrix[9];
    float orientCovMatrix[9];
};

#endif // BNO085_IMU_H
