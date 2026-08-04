#pragma once
// -----------------------------------------------------------------------------
// Framed serial telemetry for the browser bench.
//
//   $H,<field>,<field>,...*CS     schema, resent every TELEM_HEADER_MS
//   $D,<val>,<val>,...*CS         one sample, fields in schema order
//   $E,<free text>                event / log line (shown in the monitor only)
//
// CS is an NMEA-style XOR of every character between '$' and '*', hex, 2 digits.
// The bench keys charts off these exact field names, so keep them if you can:
//   ax ay az   FUSED accel, whatever ISM6HG256X::read() handed back   m/s^2
//   asrc       0 = low-g sourced that sample, 1 = high-g
//   lx ly lz   pure readLowG()   m/s^2
//   hx hy hz   pure readHighG()  m/s^2
//   gx gy gz   gyro             rad/s
//   qw qx qy qz  AHRS quaternion (send this, NOT euler - no gimbal lock, and
//                the bench derives roll/pitch/yaw itself)
//   kf_alt kf_vel kf_acc, vaccel, baro_alt, pres_pa, state, brake, dt0..dtN
// -----------------------------------------------------------------------------
#include <Arduino.h>
#include <stdio.h>
#include <string.h>
#include "sensors/sensor_types.h"   // IMUData / BaroData

#ifndef TELEM_HEADER_MS
#define TELEM_HEADER_MS 2000
#endif
#ifndef TELEM_MAX_TASKS
#define TELEM_MAX_TASKS 6
#endif

struct TelemExtras {
    float qw, qx, qy, qz;   // FusionAhrsGetQuaternion()
    float vaccel_ms2;       // get_vertical_acceleration_ms2()
    float kf_alt_m;
    float kf_vel_ms;
    float kf_acc_ms2;
    uint8_t state;          // flight state machine enum
    float brake_cmd;        // 0..1, drives the airbrake tabs in the 3D view
};

// ---------------------------------------------------------------- internals --
static inline uint8_t telem_xor(const char* s, size_t n) {
    uint8_t c = 0;
    for (size_t i = 0; i < n; i++) c ^= (uint8_t)s[i];
    return c;
}

// Writes "$<body>*CS\r\n" only if the USB/UART FIFO can take the whole frame.
// Dropping a frame is always better than blocking a flight task on a host that
// is not draining the CDC endpoint.
static inline void telem_emit(const char* body, size_t n) {
    if (!Serial) return;
    if (Serial.availableForWrite() < (int)(n + 6)) return;
    char tail[7];
    snprintf(tail, sizeof(tail), "*%02X\r\n", telem_xor(body, n));
    Serial.write('$');
    Serial.write((const uint8_t*)body, n);
    Serial.write((const uint8_t*)tail, strlen(tail));
}

static inline void telem_event(const char* msg) {
    char b[160];
    int n = snprintf(b, sizeof(b), "E,%s", msg);
    if (n > 0) telem_emit(b, (size_t)min(n, (int)sizeof(b) - 1));
}

// -------------------------------------------------------------------- header --
static inline void telem_header(uint8_t nTasks) {
    char b[320];
    int n = snprintf(b, sizeof(b),
        "H,t_us,ax,ay,az,asrc,lx,ly,lz,hx,hy,hz,gx,gy,gz,imu_c,"
        "pres_pa,baro_alt,baro_c,qw,qx,qy,qz,vaccel,"
        "kf_alt,kf_vel,kf_acc,state,brake");
    if (nTasks > TELEM_MAX_TASKS) nTasks = TELEM_MAX_TASKS;
    for (uint8_t i = 0; i < nTasks && n > 0 && n < (int)sizeof(b) - 8; i++)
        n += snprintf(b + n, sizeof(b) - n, ",dt%u", (unsigned)i);
    if (n > 0) telem_emit(b, (size_t)min(n, (int)sizeof(b) - 1));
}

// ---------------------------------------------------------------------- data --
// imu    : the fused sample the AHRS and Kalman filter actually consumed
// lowG   : pure readLowG()  - does not perturb the fallback state machine
// highG  : pure readHighG()
// srcHi  : lastAccelSource() == HighG
static inline void telem_sample(const IMUData& imu,
                                const AccelData& lowG, const AccelData& highG,
                                bool srcHi,
                                const BaroData& baro, const TelemExtras& x,
                                const float* dts, uint8_t nTasks) {
    static uint32_t lastHdr = 0;
    uint32_t now = millis();
    if ((uint32_t)(now - lastHdr) >= TELEM_HEADER_MS) {
        lastHdr = now;
        telem_header(nTasks);
    }

    char b[512];
    int n = snprintf(b, sizeof(b),
        "D,%llu,"
        "%.4f,%.4f,%.4f,%u,"       // fused accel + source flag
        "%.4f,%.4f,%.4f,"          // low-g channel
        "%.3f,%.3f,%.3f,"          // high-g channel
        "%.5f,%.5f,%.5f,%.2f,"     // gyro + imu temp
        "%.1f,%.3f,%.2f,"          // baro
        "%.5f,%.5f,%.5f,%.5f,"     // quaternion
        "%.3f,%.3f,%.3f,%.3f,"     // vaccel + kalman
        "%u,%.3f",                 // state + brake
        (unsigned long long)imu.timestamp_us,
        imu.accelX_ms2, imu.accelY_ms2, imu.accelZ_ms2, (unsigned)(srcHi ? 1 : 0),
        lowG.accelX_ms2, lowG.accelY_ms2, lowG.accelZ_ms2,
        highG.accelX_ms2, highG.accelY_ms2, highG.accelZ_ms2,
        imu.gyroX_rps, imu.gyroY_rps, imu.gyroZ_rps, imu.temperature_c,
        baro.pressure_pa, baro.altitude_m, baro.temperature_c,
        x.qw, x.qx, x.qy, x.qz,
        x.vaccel_ms2, x.kf_alt_m, x.kf_vel_ms, x.kf_acc_ms2,
        (unsigned)x.state, x.brake_cmd);

    if (nTasks > TELEM_MAX_TASKS) nTasks = TELEM_MAX_TASKS;
    for (uint8_t i = 0; i < nTasks && n > 0 && n < (int)sizeof(b) - 12; i++)
        n += snprintf(b + n, sizeof(b) - n, ",%.4f", dts[i]);

    if (n > 0) telem_emit(b, (size_t)min(n, (int)sizeof(b) - 1));
}