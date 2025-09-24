imu_i2c_to_serial_publisher
==========================

Overview
--------
This project reads IMU sensors over I2C, accumulates statistics, and publishes JSON messages compatible with ROS `sensor_msgs/Imu` (via a separate Python converter). The code is header-only for the IMU drivers and uses a shared `CovarianceAccumulator` helper.

Design choices (covariance and filtering)
----------------------------------------
- Covariance estimator: uses the unbiased sample covariance (divide by n-1). This is the standard statistical estimator for sample covariance and is appropriate when inferring variance from measured samples.

  Formulas:
  - mean_x = (1/n) * Σ x_i
  - cov_xy = (1/(n-1)) * Σ_i (x_i - mean_x)(y_i - mean_y)
  - internally computed using sums and cross-products:
    Σ (x_i - mean_x)(y_i - mean_y) = Σ x_i y_i - n * mean_x * mean_y

- Rolling window: Each IMU driver configures a rolling window for covariance (default 200 samples). When enabled, the accumulator maintains a FIFO buffer of the most recent N samples so covariance reflects recent noise characteristics rather than lifetime statistics.
  - Default window size: 200 samples (approx ~2s at 100Hz). Adjust with `setWindowSize(N)` on the accumulators.

- Numerical robustness:
  - Diagonal covariance elements (variances) are clamped to zero if negative or non-finite to avoid invalid values in downstream systems.
  - A small epsilon threshold (`setVarianceEpsilon`) is available to zero extremely small variances if desired (default 1e-9).

ROS compatibility notes
----------------------
- JSON message structure follows the `sensor_msgs/Imu` layout. Orientation covariance of `[-1,0,...]` indicates the IMU does not provide orientation (per REP145).
- Units:
  - linear_acceleration: m/s^2; covariance elements are in (m/s^2)^2.
  - angular_velocity: rad/s; covariance elements are in (rad/s)^2.
- Scientific notation is acceptable in the JSON (e.g., `1.23e-6`) and will be parsed correctly by standard JSON and ROS converters.

Files of interest
-----------------
- `include/IMUCommon.h` — CovarianceAccumulator (rolling window, unbiased estimator, clamping).
- `include/IMUMessageFormatter.h` — JSON formatter with diagonal clamping before serialization.
- `include/GroveBMI088_IMU.h`, `include/LSM6DSOX_IMU.h`, `include/BNO085_IMU.h` — IMU drivers. Each sets default window and epsilon in `begin()`.

Recommendations
---------------
- Tune `setWindowSize()` per your publish rate and how quickly you want covariance to adapt. Larger windows yield smoother but slower-changing covariance estimates.
- If your estimator downstream is sensitive to small variances, consider increasing `varianceEpsilon` to a reasonable floor such as 1e-8 or 1e-6 depending on sensor noise.

Building & running
------------------
Use PlatformIO from the project root:

```bash
pio run
pio run --target upload
```

After flashing, open your serial monitor and run the Python converter to translate JSON messages into ROS `sensor_msgs/Imu` messages.

Feedback & contribution
-----------------------
If you'd like different defaults, additional logging, or unit tests for the accumulator, open an issue or send a PR.
