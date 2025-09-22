// Abstract base for IMU sensors.
// Common interface for data reading and covariances.

#ifndef IMU_INTERFACE_H
#define IMU_INTERFACE_H

#include <Arduino.h>

class IMUInterface
{
public:
    virtual bool begin() = 0;
    virtual void readSensorData() = 0;
    virtual float getAccelerometerX() const = 0;
    virtual float getAccelerometerY() const = 0;
    virtual float getAccelerometerZ() const = 0;
    virtual float getGyroscopeX() const = 0;
    virtual float getGyroscopeY() const = 0;
    virtual float getGyroscopeZ() const = 0;
    virtual float getTemperature() const = 0;
    virtual void computeCovariances() = 0;
    virtual float getAccelerometerCovariance() const = 0;
    virtual float getGyroscopeCovariance() const = 0;
    virtual const float *getAccelCovMatrix() const = 0;
    virtual const float *getGyroCovMatrix() const = 0;
    virtual float getOrientationX() const = 0; // Quaternion X
    virtual float getOrientationY() const = 0; // Quaternion Y
    virtual float getOrientationZ() const = 0; // Quaternion Z
    virtual float getOrientationW() const = 0; // Quaternion W
    virtual unsigned long getTimestampMilliseconds() const = 0;
    virtual ~IMUInterface() {}
};

#endif // IMU_INTERFACE_H
