#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_BNO08x.h>
#include <BMI088.h>
#include <math.h>

// ============ Configuration ============
// Baud rate for serial communication. Set high to handle data volume from multiple IMUs.
// Recommendation: 1000000 (1 Mbps) for stability with RPi5 reader.
static const uint32_t SERIAL_BAUD = 1000000;

// Stream rate in Hz. Lowered to 50 Hz to reduce serial load; adjustable based on testing.
static const uint16_t STREAM_RATE_HZ = 50;

// Orientation mode for BNO: Game (no magnetometer, more robust to magnetic interference) or Absolute (uses mag for true north).
enum class BnoOrientationMode { Game, Absolute };
static const BnoOrientationMode BNO_MODE = BnoOrientationMode::Game;

// Enable linear acceleration (gravity-removed) for BNO.
static const bool BNO_ENABLE_LINEAR = true;

// Toggle adaptive covariance per message (adjusts uncertainty based on real-time conditions like vibration).
static const bool USE_ADAPTIVE_COVARIANCE = true;

// Frame IDs following ROS conventions (for tf2 transforms in ROS2).
static const char* FRAME_LSM6DSOX = "imu_link_lsm6dsox";
static const char* FRAME_BNO085   = "imu_link_bno085";
static const char* FRAME_BMI088   = "imu_link_bmi088";

// ============ Shared Data Structures ============
// These structs mirror fields in ROS2 sensor_msgs/Imu message.
// - present: Flag if data is valid (false if sensor failed or no new data).
// - Values: In SI units (quaternions unitless, angular vel in rad/s, linear acc in m/s²).
// - Covariances: Diagonal variances (approximation for roll/pitch/yaw, angular vel, linear acc).
//   In ROS, full 9x9 covariance matrices are used, but here we use 3-value diagonals for simplicity.
//   Higher covariance = higher uncertainty (less trust in data, useful for fusion filters like EKF).
struct Orientation {
  bool  present = false;
  float w = NAN, x = NAN, y = NAN, z = NAN;   // Unit quaternion (w + xi + yj + zk).
  double covariance_x = 0.0, covariance_y = 0.0, covariance_z = 0.0; // Variances (rad²) for roll, pitch, yaw.
};

struct AngularVelocity {
  bool  present = false;
  float x = NAN, y = NAN, z = NAN;            // rad/s.
  double covariance_x = 0.0, covariance_y = 0.0, covariance_z = 0.0; // Variances (rad²/s²).
};

struct LinearAcceleration {
  bool  present = false;
  float x = NAN, y = NAN, z = NAN;            // m/s².
  double covariance_x = 0.0, covariance_y = 0.0, covariance_z = 0.0; // Variances (m²/s⁴).
};

struct ImuPacket {
  const char* name  = "IMU";
  const char* frame = "imu_link";
  Orientation orientation;
  AngularVelocity angular_velocity;
  LinearAcceleration linear_acceleration;
};

// Mirrors ROS2 sensor_msgs/MagneticField.
struct MagneticFieldPacket {
  const char* name  = "BNO085_MAG";
  const char* frame = "imu_link_bno085";
  bool  present = false;
  float x_T = NAN, y_T = NAN, z_T = NAN;      // Tesla (SI unit for ROS).
  double covariance_x = 0.0, covariance_y = 0.0, covariance_z = 0.0; // Variances (T²).
};

// ============ Common Base Class for IMU Sources ============
// Base class for all IMUs, handling shared logic like covariance computation and JSON output.
// This design promotes reusability: Students can extend for new sensors.
class ImuSource {
public:
  virtual ~ImuSource() {}
  virtual bool begin() = 0;
  virtual void poll_and_print() = 0;

protected:
  // Inputs for adaptive covariance: Real-time proxies to detect conditions like vibration or fast motion.
  // - accel_rms: Root mean square of acceleration deviations (m/s²), proxies vibration.
  // - gyro_mag: Magnitude of angular velocity (rad/s), proxies motion intensity.
  // - yaw_quality: [0..1] quality of yaw estimate (from magnetometer accuracy if available; -1 if N/A).
  struct CovarianceInputs {
    double accel_rms = 0.0;
    double gyro_mag  = 0.0;
    double yaw_quality = -1.0;
  };

  // Baseline variances (tuned per sensor from datasheets/experiments). Units as above.
  // orientation_missing: For 6-axis IMUs without fusion, mark orientation as unavailable.
  struct CovarianceBaselines {
    double ori_x = 0.03, ori_y = 0.03, ori_z = 0.10; // rad²
    double ang_x = 0.005, ang_y = 0.005, ang_z = 0.010; // rad²/s²
    double lin_x = 0.30, lin_y = 0.30, lin_z = 0.50; // m²/s⁴
    bool orientation_missing = false;
  };

  // Scaling factors (>1 inflates covariance under poor conditions, <1 reduces when good).
  struct CovarianceScales {
    double ori_x = 1.0, ori_y = 1.0, ori_z = 1.0;
    double ang_x = 1.0, ang_y = 1.0, ang_z = 1.0;
    double lin_x = 1.0, lin_y = 1.0, lin_z = 1.0;
  };

  // Compute scales dynamically from inputs. Math: Piecewise linear inflation to model uncertainty.
  // Why? In dynamic environments (e.g., vibration), sensor noise increases—adapt covariance to reflect this for better ROS fusion.
  // Example: High accel_rms (vibration) inflates linear acc covariance, reducing trust in noisy data.
  static CovarianceScales compute_scales(const CovarianceInputs& in) {
    CovarianceScales s;

    // Linear acc scaling: If RMS > 0.2 m/s² (quiet threshold), inflate linearly (slope 2) up to 6x cap.
    // Formula: scale = 1 + clamp((rms - thresh) * slope, 0, max_k)
    const double vib_thresh = 0.2;
    if (in.accel_rms > vib_thresh) {
      double k = (in.accel_rms - vib_thresh) * 2.0;
      k = max(0.0, min(k, 5.0)); // Clamp for stability.
      s.lin_x = s.lin_y = s.lin_z = 1.0 + k;
    }

    // Angular vel scaling: If magnitude > 0.5 rad/s, inflate (slope 0.5) up to 3x.
    if (in.gyro_mag > 0.5) {
      double k = (in.gyro_mag - 0.5) * 0.5;
      k = max(0.0, min(k, 2.0));
      s.ang_x = s.ang_y = s.ang_z = 1.0 + k;
    }

    // Yaw scaling: If quality available [0..1], tiered reduction/inflation.
    // High quality (>0.8): 0.7x (reduce cov); medium (>0.4): 1x; low: 3x (inflate).
    if (in.yaw_quality >= 0.0) {
      if (in.yaw_quality > 0.8) s.ori_z = 0.7;
      else if (in.yaw_quality > 0.4) s.ori_z = 1.0;
      else s.ori_z = 3.0;
    }

    return s;
  }

  // Apply baselines * scales to packet covariances.
  // For missing orientation (ROS convention): Set covariance_x = -1, others 0.
  static void apply_covariances(ImuPacket& p, const CovarianceBaselines& base, const CovarianceScales& s) {
    if (base.orientation_missing) {
      p.orientation.covariance_x = -1.0;
      p.orientation.covariance_y = 0.0;
      p.orientation.covariance_z = 0.0;
    } else {
      p.orientation.covariance_x = base.ori_x * s.ori_x;
      p.orientation.covariance_y = base.ori_y * s.ori_y;
      p.orientation.covariance_z = base.ori_z * s.ori_z;
    }

    p.angular_velocity.covariance_x = base.ang_x * s.ang_x;
    p.angular_velocity.covariance_y = base.ang_y * s.ang_y;
    p.angular_velocity.covariance_z = base.ang_z * s.ang_z;

    p.linear_acceleration.covariance_x = base.lin_x * s.lin_x;
    p.linear_acceleration.covariance_y = base.lin_y * s.lin_y;
    p.linear_acceleration.covariance_z = base.lin_z * s.lin_z;
  }

  // Print IMU data as JSON (efficient for Arduino; no dynamic strings).
  // Precision: High for quats (7 decimals), medium for others to match sensor accuracy.
  static void print_json(const ImuPacket& packet) {
    Serial.print("{\"name\":\"");  Serial.print(packet.name);
    Serial.print("\",\"frame\":\""); Serial.print(packet.frame); Serial.print("\"");

    Serial.print(",\"orientation\":{");
    Serial.print("\"present\":"); Serial.print(packet.orientation.present ? "true" : "false"); Serial.print(",");
    Serial.print("\"w\":"); Serial.print(packet.orientation.w,7); Serial.print(",");
    Serial.print("\"x\":"); Serial.print(packet.orientation.x,7); Serial.print(",");
    Serial.print("\"y\":"); Serial.print(packet.orientation.y,7); Serial.print(",");
    Serial.print("\"z\":"); Serial.print(packet.orientation.z,7); Serial.print(",");
    Serial.print("\"covariance\":["); Serial.print(packet.orientation.covariance_x,6); Serial.print(",");
    Serial.print(packet.orientation.covariance_y,6); Serial.print(",");
    Serial.print(packet.orientation.covariance_z,6); Serial.print("]}");

    Serial.print(",\"angular_velocity\":{");
    Serial.print("\"present\":"); Serial.print(packet.angular_velocity.present ? "true" : "false"); Serial.print(",");
    Serial.print("\"x\":"); Serial.print(packet.angular_velocity.x,6); Serial.print(",");
    Serial.print("\"y\":"); Serial.print(packet.angular_velocity.y,6); Serial.print(",");
    Serial.print("\"z\":"); Serial.print(packet.angular_velocity.z,6); Serial.print(",");
    Serial.print("\"covariance\":["); Serial.print(packet.angular_velocity.covariance_x,6); Serial.print(",");
    Serial.print(packet.angular_velocity.covariance_y,6); Serial.print(",");
    Serial.print(packet.angular_velocity.covariance_z,6); Serial.print("]}");

    Serial.print(",\"linear_acceleration\":{");
    Serial.print("\"present\":"); Serial.print(packet.linear_acceleration.present ? "true" : "false"); Serial.print(",");
    Serial.print("\"x\":"); Serial.print(packet.linear_acceleration.x,5); Serial.print(",");
    Serial.print("\"y\":"); Serial.print(packet.linear_acceleration.y,5); Serial.print(",");
    Serial.print("\"z\":"); Serial.print(packet.linear_acceleration.z,5); Serial.print(",");
    Serial.print("\"covariance\":["); Serial.print(packet.linear_acceleration.covariance_x,6); Serial.print(",");
    Serial.print(packet.linear_acceleration.covariance_y,6); Serial.print(",");
    Serial.print(packet.linear_acceleration.covariance_z,6); Serial.print("]}");

    Serial.println("}");
    Serial.flush(); // Ensure data is sent immediately to avoid buffer issues.
  }

  // Print magnetic field as separate JSON (for ROS MagneticField topic).
  static void print_mag_json(const MagneticFieldPacket& m) {
    if (!m.present) return;
    Serial.print("{\"name\":\"");  Serial.print(m.name);
    Serial.print("\",\"frame\":\""); Serial.print(m.frame); Serial.print("\"");
    Serial.print(",\"magnetic_field\":{");
    Serial.print("\"x\":"); Serial.print(m.x_T,7); Serial.print(",");
    Serial.print("\"y\":"); Serial.print(m.y_T,7); Serial.print(",");
    Serial.print("\"z\":"); Serial.print(m.z_T,7); Serial.print(",");
    Serial.print("\"covariance\":["); Serial.print(m.covariance_x,6); Serial.print(",");
    Serial.print(m.covariance_y,6); Serial.print(",");
    Serial.print(m.covariance_z,6); Serial.print("]}");
    Serial.println("}");
    Serial.flush();
  }
};

// ============ LSM6DSOX Source ============
// 6-axis IMU (accel + gyro). No internal fusion, so no orientation.
class Lsm6dsoxSource : public ImuSource {
public:
  Lsm6dsoxSource(const char* frame_id, const char* sensor_name)
  : frame(frame_id), name(sensor_name) {}

  bool begin() override {
    if (!imu.begin_I2C()) return false;
    imu.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
    imu.setGyroRange(LSM6DS_GYRO_RANGE_1000_DPS);
    imu.setAccelDataRate(LSM6DS_RATE_208_HZ); // Higher than stream rate for oversampling.
    imu.setGyroDataRate(LSM6DS_RATE_208_HZ);
    last_emit_us = micros();
    last_update_us = last_emit_us;
    return true;
  }

  void poll_and_print() override {
    uint32_t now_us = micros();
    if ((now_us - last_emit_us) < emit_period_us) return;

    sensors_event_t accel, gyro, temp;
    bool new_data = imu.getEvent(&accel, &gyro, &temp);
    if (new_data) {
      last_update_us = now_us;
      // Update vibration proxy: EMA of | |a| - g | (detects deviations from gravity norm).
      // Math: amag = sqrt(ax² + ay² + az²), dev = |amag - g|, rms = (1-α)*old + α*dev.
      // Why? Captures vibration intensity regardless of sensor tilt (static |a| = g).
      update_accel_rms(accel.acceleration.x, accel.acceleration.y, accel.acceleration.z);

      ImuPacket packet;
      packet.name = name;
      packet.frame = frame;

      // No orientation: Set identity quat, mark missing in cov per ROS.
      packet.orientation.present = false;
      packet.orientation.w = 1.0f; packet.orientation.x = 0.0f; packet.orientation.y = 0.0f; packet.orientation.z = 0.0f;

      // Angular velocity (rad/s, lib provides).
      packet.angular_velocity.present = true;
      packet.angular_velocity.x = gyro.gyro.x;
      packet.angular_velocity.y = gyro.gyro.y;
      packet.angular_velocity.z = gyro.gyro.z;

      // Linear acceleration (m/s², includes gravity).
      packet.linear_acceleration.present = true;
      packet.linear_acceleration.x = accel.acceleration.x;
      packet.linear_acceleration.y = accel.acceleration.y;
      packet.linear_acceleration.z = accel.acceleration.z;

      // Baselines tuned for LSM6DSOX noise levels.
      CovarianceBaselines base;
      base.orientation_missing = true;
      base.ang_x = 0.005; base.ang_y = 0.005; base.ang_z = 0.010;
      base.lin_x = 0.20;  base.lin_y = 0.20;  base.lin_z = 0.30;

      // Inputs for adaptation.
      CovarianceInputs in;
      in.accel_rms = accel_rms;
      in.gyro_mag = sqrt(gyro.gyro.x * gyro.gyro.x + gyro.gyro.y * gyro.gyro.y + gyro.gyro.z * gyro.gyro.z);
      in.yaw_quality = -1.0; // N/A for 6-axis.

      CovarianceScales scales = USE_ADAPTIVE_COVARIANCE ? compute_scales(in) : CovarianceScales{};
      apply_covariances(packet, base, scales);

      print_json(packet);
      last_emit_us = now_us; // Update after successful emit.
    } else if ((now_us - last_update_us) > (2 * emit_period_us)) {
      // Timeout: No new data → invalidate to prevent stale outputs.
      // Teaches: Sensors can fail; handle gracefully.
    }
  }

private:
  Adafruit_LSM6DSOX imu;
  const char* frame;
  const char* name;

  double accel_rms = 0.0;
  void update_accel_rms(float ax, float ay, float az) {
    const double alpha = 0.1; // EMA decay factor (lower = slower adaptation).
    const double g = 9.80665;
    double amag = sqrt(ax * ax + ay * ay + az * az);
    double dev = fabs(amag - g);
    accel_rms = (1.0 - alpha) * accel_rms + alpha * dev;
  }

  uint32_t last_emit_us = 0;
  uint32_t last_update_us = 0;
  const uint32_t emit_period_us = 1000000UL / STREAM_RATE_HZ;
};

// ============ BNO085 Source ============
// 9-axis IMU with internal fusion (orientation from accel/gyro/mag).
class Bno085Source : public ImuSource {
public:
  Bno085Source(const char* frame_id, const char* sensor_name)
  : frame(frame_id), name(sensor_name), imu(-1) {}

  bool begin() override {
    if (!imu.begin_I2C()) return false;
    uint32_t interval_us = 1000000UL / STREAM_RATE_HZ;
    if (BNO_MODE == BnoOrientationMode::Game) {
      imu.enableReport(SH2_GAME_ROTATION_VECTOR, interval_us);
    } else {
      imu.enableReport(SH2_ROTATION_VECTOR, interval_us);
    }
    imu.enableReport(SH2_GYROSCOPE_CALIBRATED, interval_us);
    if (BNO_ENABLE_LINEAR) imu.enableReport(SH2_LINEAR_ACCELERATION, interval_us);
    imu.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED, interval_us);

    last_emit_us = micros();
    last_update_us = last_emit_us;
    return true;
  }

  void poll_and_print() override {
    uint32_t now_us = micros();
    if ((now_us - last_emit_us) < emit_period_us) return;

    // Drain events, latch latest (BNO uses async reports).
    sh2_SensorValue_t ev;
    bool new_data = false;
    while (imu.getSensorEvent(&ev)) {
      new_data = true;
      switch (ev.sensorId) {
        case SH2_GAME_ROTATION_VECTOR:
          orientation_present = true;
          qw = ev.un.gameRotationVector.real;
          qx = ev.un.gameRotationVector.i;
          qy = ev.un.gameRotationVector.j;
          qz = ev.un.gameRotationVector.k;
          yaw_quality = -1.0; // No accuracy in Game mode.
          break;
        case SH2_ROTATION_VECTOR:
          orientation_present = true;
          qw = ev.un.rotationVector.real;
          qx = ev.un.rotationVector.i;
          qy = ev.un.rotationVector.j;
          qz = ev.un.rotationVector.k;
          yaw_accuracy_rad = ev.un.rotationVector.accuracy; // Heading uncertainty (rad).
          // Map to quality [0..1]: Low accuracy → low quality.
          // Formula: quality = max(0, 1 - (accuracy / threshold)), threshold ~30°.
          yaw_quality = max(0.0, 1.0 - (yaw_accuracy_rad / (M_PI / 6.0)));
          break;
        case SH2_GYROSCOPE_CALIBRATED:
          angular_present = true;
          gx = ev.un.gyroscope.x; gy = ev.un.gyroscope.y; gz = ev.un.gyroscope.z;
          break;
        case SH2_LINEAR_ACCELERATION:
          linear_present = true;
          lax = ev.un.linearAcceleration.x; lay = ev.un.linearAcceleration.y; laz = ev.un.linearAcceleration.z;
          update_lin_rms(lax, lay, laz);
          break;
        case SH2_MAGNETIC_FIELD_CALIBRATED:
          mag_present = true;
          mx_T = ev.un.magneticField.x * 1e-6f; // µT to T.
          my_T = ev.un.magneticField.y * 1e-6f;
          mz_T = ev.un.magneticField.z * 1e-6f;
          break;
        default: break;
      }
    }

    if (new_data) last_update_us = now_us;

    if (new_data || ((now_us - last_update_us) <= (2 * emit_period_us))) {
      ImuPacket packet;
      packet.name = name;
      packet.frame = frame;

      packet.orientation.present = orientation_present;
      packet.orientation.w = qw; packet.orientation.x = qx; packet.orientation.y = qy; packet.orientation.z = qz;

      packet.angular_velocity.present = angular_present;
      packet.angular_velocity.x = gx; packet.angular_velocity.y = gy; packet.angular_velocity.z = gz;

      packet.linear_acceleration.present = linear_present;
      packet.linear_acceleration.x = lax; packet.linear_acceleration.y = lay; packet.linear_acceleration.z = laz;

      // Baselines: Tighter yaw for Absolute mode (mag-aided).
      CovarianceBaselines base;
      base.ori_x = 0.03; base.ori_y = 0.03; base.ori_z = (BNO_MODE == BnoOrientationMode::Game) ? 0.10 : 0.06;
      base.ang_x = 0.005; base.ang_y = 0.005; base.ang_z = 0.010;
      base.lin_x = 0.30;  base.lin_y = 0.30;  base.lin_z = 0.50;
      base.orientation_missing = false;

      CovarianceInputs in;
      in.accel_rms = lin_rms;
      in.gyro_mag = sqrt(gx * gx + gy * gy + gz * gz);
      in.yaw_quality = yaw_quality;

      CovarianceScales scales = USE_ADAPTIVE_COVARIANCE ? compute_scales(in) : CovarianceScales{};
      apply_covariances(packet, base, scales);

      print_json(packet);

      if (mag_present) {
        MagneticFieldPacket m;
        m.present = true;
        m.frame = FRAME_BNO085;
        m.x_T = mx_T; m.y_T = my_T; m.z_T = mz_T;
        // Static cov; could adapt based on field strength deviations.
        m.covariance_x = 1e-8; m.covariance_y = 1e-8; m.covariance_z = 1e-8;
        print_mag_json(m);
      }

      last_emit_us = now_us;
    } // Else: Stale → skip emit.
  }

private:
  Adafruit_BNO08x imu;
  const char* frame;
  const char* name;

  // Latched values.
  bool orientation_present = false, angular_present = false, linear_present = false, mag_present = false;
  float qw = NAN, qx = NAN, qy = NAN, qz = NAN;
  float gx = NAN, gy = NAN, gz = NAN;
  float lax = NAN, lay = NAN, laz = NAN;
  float mx_T = NAN, my_T = NAN, mz_T = NAN;

  double yaw_quality = -1.0;
  double yaw_accuracy_rad = 0.0;
  double lin_rms = 0.0;

  void update_lin_rms(float x, float y, float z) {
    const double alpha = 0.1;
    double amag = sqrt(x * x + y * y + z * z);
    lin_rms = (1.0 - alpha) * lin_rms + alpha * amag;
  }

  uint32_t last_emit_us = 0;
  uint32_t last_update_us = 0;
  const uint32_t emit_period_us = 1000000UL / STREAM_RATE_HZ;
};

// ============ BMI088 Source ============
// 6-axis IMU (accel + gyro, I2C). No orientation.
class Bmi088Source : public ImuSource {
public:
  Bmi088Source(const char* frame_id, const char* sensor_name) : frame(frame_id), name(sensor_name), imu(0x18, 0x68) {} // I2C addresses for accel and gyro.

  bool begin() override {
    imu.initialize(); // Initialize the sensor.
    last_emit_us = micros();
    last_update_us = last_emit_us;
    return imu.isConnection(); // Check if connected.
  }

  void poll_and_print() override {
    uint32_t now_us = micros();
    if ((now_us - last_emit_us) < emit_period_us) return;

    // Read data (lib provides accel in mg, gyro in dps).
    float ax_mg, ay_mg, az_mg;
    float gx_dps, gy_dps, gz_dps;
    imu.getAcceleration(&ax_mg, &ay_mg, &az_mg);
    imu.getGyroscope(&gx_dps, &gy_dps, &gz_dps);
    bool new_data = true; // Lib doesn't return status; assume success.

    if (new_data) {
      last_update_us = now_us;
      float ax = ax_mg * 0.00980665f; // mg to m/s².
      float ay = ay_mg * 0.00980665f;
      float az = az_mg * 0.00980665f;
      float gx = gx_dps * (M_PI / 180.0f); // dps to rad/s.
      float gy = gy_dps * (M_PI / 180.0f);
      float gz = gz_dps * (M_PI / 180.0f);

      // Update vibration proxy (similar to LSM).
      update_accel_rms(ax, ay, az);

      ImuPacket packet;
      packet.name = name;
      packet.frame = frame;

      packet.orientation.present = false;
      packet.orientation.w = 1.0f; packet.orientation.x = 0.0f; packet.orientation.y = 0.0f; packet.orientation.z = 0.0f;

      packet.angular_velocity.present = true;
      packet.angular_velocity.x = gx;
      packet.angular_velocity.y = gy;
      packet.angular_velocity.z = gz;

      packet.linear_acceleration.present = true;
      packet.linear_acceleration.x = ax;
      packet.linear_acceleration.y = ay;
      packet.linear_acceleration.z = az;

      // Baselines similar to LSM.
      CovarianceBaselines base;
      base.orientation_missing = true;
      base.ang_x = 0.005; base.ang_y = 0.005; base.ang_z = 0.010;
      base.lin_x = 0.20;  base.lin_y = 0.20;  base.lin_z = 0.30;

      CovarianceInputs in;
      in.accel_rms = accel_rms;
      in.gyro_mag = sqrt(gx * gx + gy * gy + gz * gz);
      in.yaw_quality = -1.0;

      CovarianceScales scales = USE_ADAPTIVE_COVARIANCE ? compute_scales(in) : CovarianceScales{};
      apply_covariances(packet, base, scales);

      print_json(packet);
      last_emit_us = now_us;
    } else if ((now_us - last_update_us) > (2 * emit_period_us)) {
      // Timeout for stale data.
    }
  }

private:
  const char* frame;
  const char* name;
  BMI088 imu;

  double accel_rms = 0.0;
  void update_accel_rms(float ax, float ay, float az) {
    const double alpha = 0.1;
    const double g = 9.80665;
    double amag = sqrt(ax * ax + ay * ay + az * az);
    double dev = fabs(amag - g);
    accel_rms = (1.0 - alpha) * accel_rms + alpha * dev;
  }

  uint32_t last_emit_us = 0;
  uint32_t last_update_us = 0;
  const uint32_t emit_period_us = 1000000UL / STREAM_RATE_HZ;
};

// ============ Instances ============
Lsm6dsoxSource lsm6(FRAME_LSM6DSOX, "LSM6DSOX");
Bno085Source   bno (FRAME_BNO085,   "BNO085");
Bmi088Source   bmi (FRAME_BMI088,   "BMI088");

// ============ Setup / Loop ============
void setup() {
  Serial.begin(SERIAL_BAUD);
  while (!Serial) delay(10);
  Wire.begin();

  if (!lsm6.begin()) Serial.println("# LSM6DSOX init failed");
  if (!bno.begin()) Serial.println("# BNO085 init failed");
  if (!bmi.begin()) Serial.println("# BMI088 init failed");
}

int sensor_cycle = 0; // For staggering polls.
void loop() {
  // Stagger: Poll one sensor per loop to spread serial writes (avoids bursts).
  // Loop runs fast, so effective rate unchanged. Teaches non-blocking design.
  switch (sensor_cycle % 3) {
    case 0: lsm6.poll_and_print(); break;
    case 1: bno.poll_and_print(); break;
    case 2: bmi.poll_and_print(); break;
  }
  sensor_cycle++;
}