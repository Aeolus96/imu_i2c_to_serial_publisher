// src/main.cpp
// XIAO RP2350 / ESP32-S3 IMU -> COBS+CRC16 binary telemetry at 100 Hz.
// Packet: [0x31][seq u16][sensor_id u8][flags u8][t_ms u32]
//         [quat w,x,y,z f32][gyro x,y,z f32][acc x,y,z f32]
//         [diag_ori 3*f32][diag_gyro 3*f32][diag_acc 3*f32][crc16 u16] + COBS + 0x00.
//
// Changes:
// - Read ax/ay/az and gyro into locals immediately after readSensorData()
//   THEN call computeCovariances(); publish the locals so any driver-side
//   filtering/cov updates cannot mutate the values being sent (fixes BMI088 2 g Z).
// - ‘D’ command prints one-line accel dump in m/s^2 for face checks.
// - Same schema, flags, and driver includes; no Z-only gravity tweak.

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
    uint8_t type;
    uint16_t seq;
    uint8_t sensor_id;
    uint8_t flags; // bit0: orientation_valid
    uint32_t t_ms;
    float qw, qx, qy, qz;
    float gx, gy, gz; // rad/s
    float ax, ay, az; // m/s^2
    float cov_ori_x, cov_ori_y, cov_ori_z;
    float cov_gyr_x, cov_gyr_y, cov_gyr_z;
    float cov_acc_x, cov_acc_y, cov_acc_z;
    uint16_t crc;
};
#pragma pack(pop)

#if defined(__cplusplus) && (__cplusplus >= 201103L)
static_assert(sizeof(ImuPacketV1) == (1 + 2 + 1 + 1 + 4 + 16 + 12 + 12 + 12 + 12 + 12 + 2), "Packet size mismatch");
#endif

static const uint32_t publish_hz = 100;
static const uint32_t publish_dt_ms = 1000 / publish_hz;

static bool inject_crc = false;
static bool send_test_once = false;

static inline float fclampnan(float v) { return isfinite(v) ? v : 0.0f; }

static void process_commands(float ax, float ay, float az)
{
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
            Serial.println("CMD: reset covariances (if supported)");
        }
        else if (c == 'D' || c == 'd')
        {
            Serial.print("ACC mps2: ");
            Serial.print(ax, 6);
            Serial.print(", ");
            Serial.print(ay, 6);
            Serial.print(", ");
            Serial.println(az, 6);
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
        process_commands(0.0f, 0.0f, 0.0f);
        delay(1);
        return;
    }
    last_ms = now_ms;

    if (!imu)
    {
        begin_first_available();
    }

    // 1) Read one fresh sample
    float raw_qw = 1.0f, raw_qx = 0.0f, raw_qy = 0.0f, raw_qz = 0.0f;
    float raw_gx = 0.0f, raw_gy = 0.0f, raw_gz = 0.0f;
    float raw_ax = 0.0f, raw_ay = 0.0f, raw_az = 0.0f;
    bool has_ori = false;

    if (imu)
    {
        imu->readSensorData();

        // Snapshot values BEFORE computing covariances (critical for BMI088)
        has_ori = imu->hasOrientation();
        if (has_ori)
        {
            raw_qx = imu->getOrientationX();
            raw_qy = imu->getOrientationY();
            raw_qz = imu->getOrientationZ();
            raw_qw = imu->getOrientationW();
            float qn = sqrtf(raw_qw * raw_qw + raw_qx * raw_qx + raw_qy * raw_qy + raw_qz * raw_qz);
            if (qn > 0.0f)
            {
                raw_qw /= qn;
                raw_qx /= qn;
                raw_qy /= qn;
                raw_qz /= qn;
            }
        }
        raw_gx = imu->getGyroscopeX();
        raw_gy = imu->getGyroscopeY();
        raw_gz = imu->getGyroscopeZ();
        raw_ax = imu->getAccelerometerX();
        raw_ay = imu->getAccelerometerY();
        raw_az = imu->getAccelerometerZ();

        // 2) Only now compute covariances (do not touch the snapshot)
        imu->computeCovariances();

        // Guard against NaNs
        raw_qw = fclampnan(raw_qw);
        raw_qx = fclampnan(raw_qx);
        raw_qy = fclampnan(raw_qy);
        raw_qz = fclampnan(raw_qz);
        raw_gx = fclampnan(raw_gx);
        raw_gy = fclampnan(raw_gy);
        raw_gz = fclampnan(raw_gz);
        raw_ax = fclampnan(raw_ax);
        raw_ay = fclampnan(raw_ay);
        raw_az = fclampnan(raw_az);
    }

    process_commands(raw_ax, raw_ay, raw_az);

    // Build packet
    ImuPacketV1 p{};
    p.type = PKT_IMU_V1;
    p.seq = seq++;
    p.sensor_id = (uint8_t)detected;
    p.flags = has_ori ? 0x01 : 0x00;
    p.t_ms = now_ms;

    p.qw = raw_qw;
    p.qx = raw_qx;
    p.qy = raw_qy;
    p.qz = raw_qz;
    p.gx = raw_gx;
    p.gy = raw_gy;
    p.gz = raw_gz;
    p.ax = raw_ax;
    p.ay = raw_ay;
    p.az = raw_az;

    const float *covG = imu ? imu->getGyroCovMatrix() : nullptr;
    const float *covA = imu ? imu->getAccelCovMatrix() : nullptr;
    const float *covO = (imu && has_ori) ? imu->getOrientationCovMatrix() : nullptr;

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

    // CRC (optional injection for recovery tests)
    const size_t payload_len_wo_crc = sizeof(ImuPacketV1) - sizeof(uint16_t);
    p.crc = crc16_ccitt(reinterpret_cast<const uint8_t *>(&p), payload_len_wo_crc);
    if (inject_crc && (p.seq % 100 == 0))
    {
        p.crc ^= 0xFFFF;
    }

    // COBS encode + delimiter
    uint8_t enc[256];
    const size_t enc_len = cobs_encode(reinterpret_cast<const uint8_t *>(&p), sizeof(ImuPacketV1), enc);
    Serial.write(enc, enc_len);
    Serial.write((uint8_t)0x00);
}
