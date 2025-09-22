// Formatter for ROS IMU JSON.
// All code inline for header-only.

#ifndef IMU_MESSAGE_FORMATTER_H
#define IMU_MESSAGE_FORMATTER_H

#include <Arduino.h>
#include <ArduinoJson.h>
#include "IMUInterface.h"

class IMUMessageFormatter
{
public:
    static String formatAsRosImuJson(const IMUInterface &imu)
    {
        JsonDocument doc;

        unsigned long ts = imu.getTimestampMilliseconds();
        doc["header"]["stamp"]["secs"] = ts / 1000;
        doc["header"]["stamp"]["nsecs"] = (ts % 1000) * 1000000;
        doc["header"]["frame_id"] = "imu_link";

        // Orientation (from game rotation vector)
        const auto &orient = doc["orientation"].to<JsonObject>();
        orient["x"] = imu.getOrientationX();
        orient["y"] = imu.getOrientationY();
        orient["z"] = imu.getOrientationZ();
        orient["w"] = imu.getOrientationW();

        const auto &orientCov = doc["orientation_covariance"].to<JsonArray>();
        orientCov.add(-1.0); // Unknown; BNO doesn't provide
        for (int i = 1; i < 9; ++i)
            orientCov.add(0.0);

        // Linear acceleration
        const auto &linAccel = doc["linear_acceleration"].to<JsonObject>();
        linAccel["x"] = imu.getAccelerometerX();
        linAccel["y"] = imu.getAccelerometerY();
        linAccel["z"] = imu.getAccelerometerZ();

        const auto &linCov = doc["linear_acceleration_covariance"].to<JsonArray>();
        const float *accelMatrix = imu.getAccelCovMatrix();
        for (int i = 0; i < 9; ++i)
            linCov.add(accelMatrix[i]);

        // Angular velocity
        const auto &angVel = doc["angular_velocity"].to<JsonObject>();
        angVel["x"] = imu.getGyroscopeX();
        angVel["y"] = imu.getGyroscopeY();
        angVel["z"] = imu.getGyroscopeZ();

        const auto &angCov = doc["angular_velocity_covariance"].to<JsonArray>();
        const float *gyroMatrix = imu.getGyroCovMatrix();
        for (int i = 0; i < 9; ++i)
            angCov.add(gyroMatrix[i]);

        String json;
        serializeJson(doc, json);
        return json;
    }
};

#endif // IMU_MESSAGE_FORMATTER_H
