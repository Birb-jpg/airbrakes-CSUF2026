#include "sensors/ism6hg256x.h"

// Driver reports sensitivity in mg/LSB and mdps/LSB; the IMUData contract is SI.
constexpr float MG_TO_MS2 = 9.80665f / 1000.0f;
constexpr float MDPS_TO_RPS = (float)(PI / 180.0 / 1000.0);

// 240 Hz is the first ISM6HG256X ODR step at or above the 208 Hz IMU task rate.
constexpr float SAMPLE_RATE_HZ = 240.0f;
constexpr int32_t ACCEL_RANGE_G = 16;
constexpr int32_t GYRO_RANGE_DPS = 2000;

constexpr int32_t HIGH_G_RANGE_G = 64;
constexpr float HIGH_G_RATE_HZ = 480.0f;  // lowest ODR the high-g channel offers

// int16 counts; ~97.7% of the +-16 g low-g range.
constexpr int16_t LOW_G_SATURATION_COUNTS = 32000;
// Fall back to low-g only well inside its range, so the switch does not chatter.
constexpr float LOW_G_RETURN_MS2 = 13.0f * 9.80665f;


ISM6HG256X::ISM6HG256X(int8_t CS_PIN, SPIClass* spi, uint32_t spi_speed)
    : raw_imu_(spi, CS_PIN, spi_speed), cs_pin_(CS_PIN) {}

ISM6HG256X::ISM6HG256X(TwoWire* i2c, uint8_t address)
    : raw_imu_(i2c, address), cs_pin_(-1) {}


bool ISM6HG256X::begin() {
    if (raw_imu_.begin() != ISM6HG256X_OK) return false;

    uint8_t id = 0;
    if (raw_imu_.ReadID(&id) != ISM6HG256X_OK || id != ISM6HG256X_ID) return false;

    // Everything is configured while all three channels are still disabled, on
    // purpose: Set_HG_X_OutputDataRate() branches on the *low-g* enable flag, so
    // setting the high-g ODR after Enable_X() writes the register without
    // caching acc_hg_odr, and Enable_HG_X() then clobbers it with its default.
    // Configuring first takes the caching path and Enable_HG_X() applies it.

    // Accel, low-g
    if (raw_imu_.Set_X_FullScale(ACCEL_RANGE_G) != ISM6HG256X_OK) return false;
    if (raw_imu_.Set_X_OutputDataRate(SAMPLE_RATE_HZ) != ISM6HG256X_OK) return false;

    // Accel, high-g. Free-runs faster than the IMU task; reads take the latest sample.
    if (raw_imu_.Set_HG_X_FullScale(HIGH_G_RANGE_G) != ISM6HG256X_OK) return false;
    if (raw_imu_.Set_HG_X_OutputDataRate(HIGH_G_RATE_HZ) != ISM6HG256X_OK) return false;

    // Gyro
    if (raw_imu_.Set_G_FullScale(GYRO_RANGE_DPS) != ISM6HG256X_OK) return false;
    if (raw_imu_.Set_G_OutputDataRate(SAMPLE_RATE_HZ) != ISM6HG256X_OK) return false;

    if (raw_imu_.Enable_X() != ISM6HG256X_OK) return false;
    if (raw_imu_.Enable_HG_X() != ISM6HG256X_OK) return false;
    if (raw_imu_.Enable_G() != ISM6HG256X_OK) return false;

    if (raw_imu_.Get_X_Sensitivity(&acc_sensitivity_) != ISM6HG256X_OK) return false;
    if (raw_imu_.Get_HG_X_Sensitivity(&hg_sensitivity_) != ISM6HG256X_OK) return false;
    if (raw_imu_.Get_G_Sensitivity(&gyro_sensitivity_) != ISM6HG256X_OK) return false;

    return true;
}

bool ISM6HG256X::read(IMUData& out) {
    ISM6HG256X_AxesRaw_t accel_raw;
    ISM6HG256X_AxesRaw_t gyro_raw;

    // Raw counts rather than Get_X_Axes()/Get_G_Axes(), which truncate to whole
    // mg / mdps and would throw away roughly half a milli-g of resolution.
    if (raw_imu_.Get_X_AxesRaw(&accel_raw) != ISM6HG256X_OK) return false;
    if (raw_imu_.Get_G_AxesRaw(&gyro_raw) != ISM6HG256X_OK) return false;

    out.gyroX_rps = gyro_raw.x * gyro_sensitivity_ * MDPS_TO_RPS - gyro_bias_x_;
    out.gyroY_rps = gyro_raw.y * gyro_sensitivity_ * MDPS_TO_RPS - gyro_bias_y_;
    out.gyroZ_rps = gyro_raw.z * gyro_sensitivity_ * MDPS_TO_RPS - gyro_bias_z_;

    // The two channels differ in filtering and latency, so the whole vector comes
    // from one of them -- never a per-axis mix, which would not be a coherent
    // measurement. The extra high-g burst only happens once the low-g clips.
    const bool low_g_clipped = isSaturated(accel_raw);
    bool use_high_g = low_g_clipped;

    AccelData high_g;
    if (accel_source_ == AccelSource::HighG || low_g_clipped) {
        if (!readHighG(high_g)) return false;
        const float peak = fmaxf(fmaxf(fabsf(high_g.accelX_ms2), fabsf(high_g.accelY_ms2)),
                                 fabsf(high_g.accelZ_ms2));
        // Hysteresis: only hand back to the low-g channel well inside its range.
        use_high_g = low_g_clipped || peak >= LOW_G_RETURN_MS2;
    }

    if (use_high_g) {
        out.accelX_ms2 = high_g.accelX_ms2;
        out.accelY_ms2 = high_g.accelY_ms2;
        out.accelZ_ms2 = high_g.accelZ_ms2;
    } else {
        out.accelX_ms2 = accel_raw.x * acc_sensitivity_ * MG_TO_MS2;
        out.accelY_ms2 = accel_raw.y * acc_sensitivity_ * MG_TO_MS2;
        out.accelZ_ms2 = accel_raw.z * acc_sensitivity_ * MG_TO_MS2;
    }
    accel_source_ = use_high_g ? AccelSource::HighG : AccelSource::LowG;

    // Temperature is a nice-to-have; a failed read should not drop the sample.
    if (!readTemperature(out.temperature_c)) out.temperature_c = NAN;
    out.timestamp_us = micros();

    return true;
}

bool ISM6HG256X::readLowG(AccelData& out) {
    ISM6HG256X_AxesRaw_t raw;
    if (raw_imu_.Get_X_AxesRaw(&raw) != ISM6HG256X_OK) return false;

    out.accelX_ms2 = raw.x * acc_sensitivity_ * MG_TO_MS2;
    out.accelY_ms2 = raw.y * acc_sensitivity_ * MG_TO_MS2;
    out.accelZ_ms2 = raw.z * acc_sensitivity_ * MG_TO_MS2;
    out.timestamp_us = micros();
    return true;
}

bool ISM6HG256X::readHighG(AccelData& out) {
    ISM6HG256X_AxesRaw_t raw;
    if (raw_imu_.Get_HG_X_AxesRaw(&raw) != ISM6HG256X_OK) return false;

    out.accelX_ms2 = raw.x * hg_sensitivity_ * MG_TO_MS2;
    out.accelY_ms2 = raw.y * hg_sensitivity_ * MG_TO_MS2;
    out.accelZ_ms2 = raw.z * hg_sensitivity_ * MG_TO_MS2;
    out.timestamp_us = micros();
    return true;
}

bool ISM6HG256X::calibrate(int samples, int delay_ms) {
    // Clear first so read() reports unbiased rates while averaging.
    gyro_bias_x_ = 0.0f;
    gyro_bias_y_ = 0.0f;
    gyro_bias_z_ = 0.0f;

    float sx = 0, sy = 0, sz = 0;
    IMUData sample;
    for (int i = 0; i < samples; i++) {
        if (!read(sample)) return false;
        sx += sample.gyroX_rps;
        sy += sample.gyroY_rps;
        sz += sample.gyroZ_rps;
        delay(delay_ms);
    }
    gyro_bias_x_ = sx / samples;
    gyro_bias_y_ = sy / samples;
    gyro_bias_z_ = sz / samples;
    return true;
}

bool ISM6HG256X::dataReady() {
    uint8_t accel_ready = 0;
    uint8_t gyro_ready = 0;
    if (raw_imu_.Get_X_DRDY_Status(&accel_ready) != ISM6HG256X_OK) return false;
    if (raw_imu_.Get_G_DRDY_Status(&gyro_ready) != ISM6HG256X_OK) return false;
    return accel_ready != 0 && gyro_ready != 0;
}

bool ISM6HG256X::readTemperature(float& out) {
    uint8_t lo = 0;
    uint8_t hi = 0;
    if (raw_imu_.Read_Reg(ISM6HG256X_OUT_TEMP_L, &lo) != ISM6HG256X_OK) return false;
    if (raw_imu_.Read_Reg(ISM6HG256X_OUT_TEMP_H, &hi) != ISM6HG256X_OK) return false;

    int16_t raw = (int16_t)(((uint16_t)hi << 8) | lo);
    out = (raw / 256.0f) + 25.0f;  // 256 LSB/degC, zeroed at 25 degC
    return true;
}

bool ISM6HG256X::isSaturated(const ISM6HG256X_AxesRaw_t& raw) {
    return abs(raw.x) >= LOW_G_SATURATION_COUNTS ||
           abs(raw.y) >= LOW_G_SATURATION_COUNTS ||
           abs(raw.z) >= LOW_G_SATURATION_COUNTS;
}
