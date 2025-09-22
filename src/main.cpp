// Main entry point for XIAO RP2350 IMU reader.
// Handles setup, loop, and IMU instance selection.

#include <Arduino.h>
#include "IMUInterface.h"
// #include "LSM6DSOX_IMU.h"
#include "BNO085_IMU.h" // New include for BNO085 support
#include "IMUMessageFormatter.h"

// Instance of the IMU sensor - comment/uncomment to switch
// LSM6DSOX_IMU activeImuSensor;
BNO085_IMU activeImuSensor; // Example: Switch to BNO085

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
