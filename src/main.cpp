// src/main.cpp (finalized for firmware-side verification)
// XIAO RP2350 / ESP32-S3 IMU -> COBS+CRC binary telemetry at 100 Hz
// Reuses your include/: BNO085_IMU.h, LSM6DSOX_IMU.h, GroveBMI088_IMU.h, IMUCommon.h
// Sensor units: gyro rad/s, accel m/s^2; orientation quaternion if available.
// Flags: bit0 orientation_valid (others reserved; host ignores).
// Commands on USB CDC: 'T' = send one-shot synthetic test frame; 'C' = toggle CRC error injection; 'R' = reset covariances (if implemented).

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#include "IMUInterface.h"
#include "BNO085_IMU.h"
#include "LSM6DSOX_IMU.h"
#include "GroveBMI088_IMU.h"
#include "IMUCommon.h"

static const uint8_t PKT_IMU_V1 = 0x31;

static inline uint16_t crc16_ccitt(const uint8_t *data, size_t len, uint16_t crc = 0xFFFF)
{
    for (size_t i = 0; i < len; ++i)
    {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t b = 0; b < 8; ++b)
        {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

// Correct COBS encoder: returns bytes written
static size_t cobs_encode(const uint8_t *input, size_t length, uint8_t *output)
{
    uint8_t *out_start = output;
    const uint8_t *in_end = input + length;
    uint8_t *code_ptr = output++;
    uint8_t code = 1;
    while (input < in_end)
    {
        if (*input == 0)
        {
            *code_ptr = code;
            code_ptr = output++;
            code = 1;
            ++input;
        }
        else
        {
            *output++ = *input++;
            if (++code == 0xFF)
            {
                *code_ptr = code;
                code_ptr = output++;
                code = 1;
            }
        }
    }
    *code_ptr = code;
    return (size_t)(output - out_start);
}

enum SensorId : uint8_t
{
    SID_NONE = 0,
    SID_LSM6DSOX = 1,
    SID_BMI088 = 2,
    SID_BNO085 = 3
};

static IMUInterface *imu = nullptr;
static LSM6DSOX_IMU imu_lsm6;
static GroveBMI088_IMU imu_bmi;
static BNO085_IMU imu_bno;
static SensorId detected = SID_NONE;

// Self-test controls
static bool inject_crc = false;
static const uint16_t inject_every = 100;
static bool send_test_once = false;

static bool begin_first_available()
{
    if (imu_bno.begin())
    {
        imu = &imu_bno;
        detected = SID_BNO085;
        Serial.println("IMU: BNO085 detected");
        return true;
    }
    if (imu_lsm6.begin())
    {
        imu = &imu_lsm6;
        detected = SID_LSM6DSOX;
        Serial.println("IMU: LSM6DSOX detected");
        return true;
    }
    if (imu_bmi.begin())
    {
        imu = &imu_bmi;
        detected = SID_BMI088;
        Serial.println("IMU: BMI088 detected");
        return true;
    }
    imu = nullptr;
    detected = SID_NONE;
    Serial.println("IMU: none detected");
    return false;
}

#pragma pack(push, 1)
struct ImuPacketV1
{
    uint8_t type;      // 0x31
    uint16_t seq;      // incrementing
    uint8_t sensor_id; // SensorId
    uint8_t flags;     // bit0: orientation_valid
    uint32_t t_ms;     // millis()
    float qw, qx, qy, qz;
    float gx, gy, gz; // rad/s
    float ax, ay, az; // m/s^2
    float cov_ori_x, cov_ori_y, cov_ori_z;
    float cov_gyr_x, cov_gyr_y, cov_gyr_z;
    float cov_acc_x, cov_acc_y, cov_acc_z;
    uint16_t crc;
};
#pragma pack(pop)

static_assert(sizeof(ImuPacketV1) == (1 + 2 + 1 + 1 + 4 + 16 + 12 + 12 + 12 + 12 + 12 + 2), "Packet size mismatch");

static const uint32_t publish_hz = 100;
static const uint32_t publish_dt_ms = 1000 / publish_hz;

static inline float fclampnan(float v)
{
    if (!isfinite(v))
        return 0.0f;
    return v;
}

static void process_commands()
{
    // Non-blocking read of up to a few bytes
    int safety = 16;
    while (Serial.available() > 0 && safety-- > 0)
    {
        int c = Serial.read();
        if (c == 'T' || c == 't')
        {
            send_test_once = true;
            Serial.println("CMD: one-shot test frame requested");
        }
        else if (c == 'C' || c == 'c')
        {
            inject_crc = !inject_crc;
            Serial.print("CMD: CRC injection ");
            Serial.println(inject_crc ? "ON" : "OFF");
        }
        else if (c == 'R' || c == 'r')
        {
            // Optional: reset covariance accumulators if your drivers expose it
            // For compatibility, silently ignore if not present.
            Serial.println("CMD: reset covariances (if supported)");
        }
    }
}

void setup()
{
    Serial.begin(115200);
    delay(50);
    Serial.println("IMU binary v1 ready (COBS+CRC16), 115200 baud");
    Wire.begin();
    begin_first_available();
}

void loop()
{
    static uint16_t seq = 0;
    static uint32_t last_ms = 0;

    const uint32_t now_ms = millis();
    if ((now_ms - last_ms) < publish_dt_ms)
    {
        process_commands();
        delay(1);
        return;
    }
    last_ms = now_ms;
    process_commands();

    // Read exactly once per publish tick
    if (!imu)
    {
        begin_first_available();
    }
    else
    {
        imu->readSensorData();
    }

    // Prepare packet
    ImuPacketV1 p{};
    p.type = PKT_IMU_V1;
    p.seq = seq++;
    p.sensor_id = (uint8_t)detected;
    const bool has_ori = (imu && imu->hasOrientation());
    p.flags = has_ori ? 0x01 : 0x00;
    p.t_ms = now_ms;

    // Synthetic test frame if requested
    if (send_test_once)
    {
        send_test_once = false;
        p.qw = 1.0f;
        p.qx = p.qy = p.qz = 0.0f;
        p.gx = 0.01f;
        p.gy = -0.02f;
        p.gz = 0.0f;
        p.ax = 0.0f;
        p.ay = 0.0f;
        p.az = 9.81f;
        p.cov_ori_x = p.cov_ori_y = p.cov_ori_z = 1e-6f;
        p.cov_gyr_x = p.cov_gyr_y = p.cov_gyr_z = 1e-5f;
        p.cov_acc_x = p.cov_acc_y = p.cov_acc_z = 1e-4f;
    }
    else if (imu)
    {
        // Compute covariances after one sample so window aligns with publish cadence
        imu->computeCovariances();

        // Orientation
        float qx = has_ori ? imu->getOrientationX() : 0.0f;
        float qy = has_ori ? imu->getOrientationY() : 0.0f;
        float qz = has_ori ? imu->getOrientationZ() : 0.0f;
        float qw = has_ori ? imu->getOrientationW() : 1.0f;
        // Normalize quaternion defensively
        float qn = sqrtf(qw * qw + qx * qx + qy * qy + qz * qz);
        if (qn > 0.0f)
        {
            qw /= qn;
            qx /= qn;
            qy /= qn;
            qz /= qn;
        }
        p.qw = fclampnan(qw);
        p.qx = fclampnan(qx);
        p.qy = fclampnan(qy);
        p.qz = fclampnan(qz);

        // Gyro and accel with NaN guards
        p.gx = fclampnan(imu->getGyroscopeX());
        p.gy = fclampnan(imu->getGyroscopeY());
        p.gz = fclampnan(imu->getGyroscopeZ());

        p.ax = fclampnan(imu->getAccelerometerX());
        p.ay = fclampnan(imu->getAccelerometerY());
        p.az = fclampnan(imu->getAccelerometerZ());

        // Diagonal variances from 3x3 row-major
        const float *covG = imu->getGyroCovMatrix();
        const float *covA = imu->getAccelCovMatrix();
        const float *covO = has_ori ? imu->getOrientationCovMatrix() : nullptr;

        p.cov_gyr_x = covG ? fclampnan(covG[0]) : 0.0f;
        p.cov_gyr_y = covG ? fclampnan(covG[4]) : 0.0f;
        p.cov_gyr_z = covG ? fclampnan(covG[8]) : 0.0f;

        p.cov_acc_x = covA ? fclampnan(covA[0]) : 0.0f;
        p.cov_acc_y = covA ? fclampnan(covA[4]) : 0.0f;
        p.cov_acc_z = covA ? fclampnan(covA[8]) : 0.0f;

        if (covO)
        {
            p.cov_ori_x = fclampnan(covO[0]);
            p.cov_ori_y = fclampnan(covO[4]);
            p.cov_ori_z = fclampnan(covO[8]);
        }
    }
    else
    {
        // No IMU yet: keep line active for debugging
        p.qw = 1.0f;
        p.qx = p.qy = p.qz = 0.0f;
        p.gx = p.gy = p.gz = 0.0f;
        p.ax = p.ay = p.az = 0.0f;
    }

    // CRC and optional corruption for test
    const size_t payload_len_wo_crc = sizeof(ImuPacketV1) - sizeof(uint16_t);
    p.crc = crc16_ccitt(reinterpret_cast<const uint8_t *>(&p), payload_len_wo_crc);
    if (inject_crc && (p.seq % inject_every == 0))
    {
        p.crc ^= 0xFFFF; // deliberate bad CRC for recovery tests
    }

    // Encode and send
    uint8_t enc[256];
    const size_t enc_len = cobs_encode(reinterpret_cast<const uint8_t *>(&p), sizeof(ImuPacketV1), enc);
    Serial.write(enc, enc_len);
    Serial.write((uint8_t)0x00);
}
