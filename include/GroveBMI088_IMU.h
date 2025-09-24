// Grove BMI088 IMU implementation.
// Inherits from IMUInterface. All code inline for header-only.

#ifndef GROVE_BMI088_IMU_H
#define GROVE_BMI088_IMU_H

#include <Arduino.h>
#include <Wire.h>
#include "BMI088.h"
#include "IMUInterface.h"
#include "IMUCommon.h"
#include <string.h> // For memset

// This header uses the Bolderflight BMI088 library API.
// The library transforms sensor data into a right-handed coordinate system
// with Z positive down and returns accel in m/s^2 and gyro in rad/s.

class GroveBMI088_IMU : public IMUInterface
{
private:
    Bmi088 *bmi088; // Pointer to Bmi088 instance (allocated in ctor)
public:
    GroveBMI088_IMU() : // Bolderflight BMI088 will be allocated in constructor body
                        latestAccelerometerX(0.0f), latestAccelerometerY(0.0f), latestAccelerometerZ(0.0f),
                        latestGyroscopeX(0.0f), latestGyroscopeY(0.0f), latestGyroscopeZ(0.0f),
                        latestTemperature(0.0f), latestTimestampMilliseconds(0),
                        offsetAccelX(0.0f), offsetAccelY(0.0f), offsetAccelZ(0.0f),
                        offsetGyroX(0.0f), offsetGyroY(0.0f), offsetGyroZ(0.0f),
                        sampleCount(0)
    {
    // Allocate Bolderflight BMI088 instance with default addresses; we'll probe other pairs in begin() if needed
    bmi088 = new Bmi088(Wire, 0x18, 0x68);

        accelAccumulator.reset();
        gyroAccumulator.reset();
        memset(accelCovMatrix, 0, sizeof(accelCovMatrix));
        memset(gyroCovMatrix, 0, sizeof(gyroCovMatrix));
    }

    ~GroveBMI088_IMU()
    {
        if (bmi088) {
            delete bmi088;
            bmi088 = nullptr;
        }
    }

    bool begin() override
    {
        // Init I2C (use Wire; adjust if using Wire1 on RP2350)
        Wire.begin();

        // Try to initialize with the default address pair. If that fails,
        // probe other common address pairs used by Grove/boards (0x18/0x19 accel and 0x68/0x69 gyro variants).
        const uint8_t accel_addrs[] = {0x18, 0x19};
        const uint8_t gyro_addrs[] = {0x68, 0x69};

        Serial.println("BMI088: Starting probe for I2C addresses...");

        bool init_ok = false;
        for (size_t ai = 0; ai < sizeof(accel_addrs); ++ai) {
            for (size_t gi = 0; gi < sizeof(gyro_addrs); ++gi) {
                uint8_t a = accel_addrs[ai];
                uint8_t g = gyro_addrs[gi];
                Serial.print("BMI088: Trying accel=0x");
                Serial.print(a, HEX);
                Serial.print(" gyro=0x");
                Serial.println(g, HEX);

                // Re-create bmi088 instance for this address pair
                delete bmi088;
                bmi088 = new Bmi088(Wire, a, g);
                int status = bmi088->begin();
                if (status >= 0) {
                    init_ok = true;
                    Serial.print("BMI088: Found device at accel=0x");
                    Serial.print(a, HEX);
                    Serial.print(" gyro=0x");
                    Serial.println(g, HEX);
                    break;
                }
                // small delay before next probe
                delay(10);
            }
            if (init_ok) break;
        }

        if (!init_ok) {
            Serial.println("BMI088: No device detected on I2C with common address pairs.");
            return false;
        }

        // Set ranges and ODR to sensible defaults using the enums defined in BMI088.h
        bmi088->setRange(Bmi088::ACCEL_RANGE_24G, Bmi088::GYRO_RANGE_500DPS);
        bmi088->setOdr(Bmi088::ODR_400HZ);

        // Configure covariance accumulator defaults: rolling window and epsilon.
        // These values are conservative starting points for ground robotics.
        accelAccumulator.setWindowSize(200); // e.g. ~2s at 100Hz
        gyroAccumulator.setWindowSize(200);
        accelAccumulator.setVarianceEpsilon(1e-9f);
        gyroAccumulator.setVarianceEpsilon(1e-9f);

        // Calibrate offsets while device is still
        Serial.println("BMI088: Calibrating offsets...");
        calibrateOffsets();
        Serial.println("BMI088: Calibration complete.");

        return true;
    }

    void readSensorData() override
    {
        // Read synchronized sensor values using Bolderflight BMI088
        bmi088->readSensor();

        float ax = bmi088->getAccelX_mss();
        float ay = bmi088->getAccelY_mss();
        float az = bmi088->getAccelZ_mss();

        // Apply offsets and smoothing
        ax = ax - offsetAccelX;
        ay = ay - offsetAccelY;
        az = az - offsetAccelZ;
        latestAccelerometerX = lowPassFilter(latestAccelerometerX, ax, 0.8f);
        latestAccelerometerY = lowPassFilter(latestAccelerometerY, ay, 0.8f);
        latestAccelerometerZ = lowPassFilter(latestAccelerometerZ, az, 0.8f);

        float gx = bmi088->getGyroX_rads();
        float gy = bmi088->getGyroY_rads();
        float gz = bmi088->getGyroZ_rads();

        gx = gx - offsetGyroX;
        gy = gy - offsetGyroY;
        gz = gz - offsetGyroZ;
        latestGyroscopeX = lowPassFilter(latestGyroscopeX, gx, 0.8f);
        latestGyroscopeY = lowPassFilter(latestGyroscopeY, gy, 0.8f);
        latestGyroscopeZ = lowPassFilter(latestGyroscopeZ, gz, 0.8f);

        latestTemperature = bmi088->getTemperature_C();
        latestTimestampMilliseconds = millis();

        accelAccumulator.addSample(latestAccelerometerX, latestAccelerometerY, latestAccelerometerZ);
        gyroAccumulator.addSample(latestGyroscopeX, latestGyroscopeY, latestGyroscopeZ);
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
        // Compute covariance matrices using the unbiased sample estimator
        // (cov = Σ(x-mean)(y-mean) / (n-1)). The accumulator uses sum and
        // cross-product accumulators to compute this efficiently.
        accelAccumulator.computeCovMatrix(accelCovMatrix);
        gyroAccumulator.computeCovMatrix(gyroCovMatrix);
    }

    float getAccelerometerCovariance() const override { return accelCovMatrix[0]; } // Example: return Var(X); update if needed
    float getGyroscopeCovariance() const override { return gyroCovMatrix[0]; }      // Example: return Var(X)

    // New getters for full matrices (used in JSON formatter)
    const float *getAccelCovMatrix() const override { return accelCovMatrix; }
    const float *getGyroCovMatrix() const override { return gyroCovMatrix; }

    // BMI088 does not provide orientation/quaternion information.
    bool hasOrientation() const override { return false; }
    const float *getOrientationCovMatrix() const override { return nullptr; }

    unsigned long getTimestampMilliseconds() const override { return latestTimestampMilliseconds; }

    // Optional: Reset accumulators to prevent overflow in long runs
    void resetCovarianceAccumulators()
    {
        accelAccumulator.reset();
        gyroAccumulator.reset();
        sampleCount = 0;
        memset(accelCovMatrix, 0, sizeof(accelCovMatrix));
        memset(gyroCovMatrix, 0, sizeof(gyroCovMatrix));
    }

    float latestAccelerometerX, latestAccelerometerY, latestAccelerometerZ;
    float latestGyroscopeX, latestGyroscopeY, latestGyroscopeZ;
    float latestTemperature;
    unsigned long latestTimestampMilliseconds;

    float offsetAccelX, offsetAccelY, offsetAccelZ; // Calibration offsets
    float offsetGyroX, offsetGyroY, offsetGyroZ;

    // Accumulators for covariance
    CovarianceAccumulator accelAccumulator;
    CovarianceAccumulator gyroAccumulator;
    int sampleCount;

    // Covariance matrices (3x3, row-major)
    float accelCovMatrix[9];
    float gyroCovMatrix[9];

    // Simple exponential moving average low-pass filter
    float lowPassFilter(float prev, float curr, float alpha)
    {
        return prev * (1 - alpha) + curr * alpha;
    }

    // Calibrate offsets (call in begin(), assume sensor is still)
    void calibrateOffsets()
    {
        offsetAccelX = offsetAccelY = offsetAccelZ = 0.0f;
        offsetGyroX = offsetGyroY = offsetGyroZ = 0.0f;
        const int calSamples = 100;
        for (int i = 0; i < calSamples; i++)
        {
            bmi088->readSensor();
            float ax = bmi088->getAccelX_mss();
            float ay = bmi088->getAccelY_mss();
            float az = bmi088->getAccelZ_mss();

            float gx = bmi088->getGyroX_rads();
            float gy = bmi088->getGyroY_rads();
            float gz = bmi088->getGyroZ_rads();

            offsetAccelX += ax;
            offsetAccelY += ay;
            offsetAccelZ += az;
            offsetGyroX += gx;
            offsetGyroY += gy;
            offsetGyroZ += gz;
            delay(10); // Short delay between samples
        }
        offsetAccelX /= calSamples;
        offsetAccelY /= calSamples;
        // Subtract gravity from Z (assume Z up in mapped frame)
        offsetAccelZ = (offsetAccelZ / calSamples) - 9.81f;
        offsetGyroX /= calSamples;
        offsetGyroY /= calSamples;
        offsetGyroZ /= calSamples;
    }
};

#endif // GROVE_BMI088_IMU_H
