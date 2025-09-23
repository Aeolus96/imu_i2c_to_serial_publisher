// Main entry point for XIAO RP2350 IMU reader.
// Handles setup, loop, and IMU instance selection.

#include <Arduino.h>
#include "IMUInterface.h"
#include "IMUMessageFormatter.h"

// SELECT THE IMU SENSOR BY INCLUDING THE APPROPRIATE HEADER AND INSTANTIATING IT:
// #include "LSM6DSOX_IMU.h"
// LSM6DSOX_IMU activeImuSensor;

// #include "BNO085_IMU.h"
// BNO085_IMU activeImuSensor;

#include "GroveBMI088_IMU.h"
GroveBMI088_IMU activeImuSensor;

// Reference to active IMU (interface for modularity)
IMUInterface &imu = activeImuSensor;

void setup()
{
    Serial.begin(115200);
    while (!Serial)
        delay(10);
    Serial.println("Starting IMU...");

    if (!imu.begin())
    {
        Serial.println("IMU init failed!");
        while (true)
            delay(1000);
    }
    Serial.println("IMU ready.");
}

void loop()
{
    // Read sensor data
    imu.readSensorData();

    // Compute dynamic covariances
    imu.computeCovariances();

    // Format as ROS IMU JSON
    String imuJson = IMUMessageFormatter::formatAsRosImuJson(imu);

    // Output to Serial
    Serial.println(imuJson);

    delay(200); // Adjust rate
}
