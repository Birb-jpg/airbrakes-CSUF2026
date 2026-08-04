#pragma once
#include "sensors/interfaces/IImu.h"
#include "sensor_types.h"
#include <SPI.h>
#include <Wire.h>
#include <ISM6HG256XSensor.h>

/// @brief ST ISM6HG256XTR 6-DOF IMU (accelerometer + gyroscope).
///
/// The part has two independent accelerometer channels -- a low-g one (used for
/// normal flight) and a high-g one (used when the low-g channel clips). Both run
/// at once; read() picks between them automatically, and readLowG()/readHighG()
/// expose each channel directly.
class ISM6HG256X : public IImu {
    public:
        /// @brief Which accelerometer channel filled the last read().
        enum class AccelSource : uint8_t { LowG, HighG };

        /// @brief SPI constructor. The bus must be SPI.begin()'d before begin() is
        ///        called; the driver drives CS itself on every transaction.
        /// @param CS_PIN Chip select pin.
        /// @param spi SPI peripheral the sensor is wired to.
        /// @param spi_speed SPI clock in Hz (10 MHz max per datasheet).
        explicit ISM6HG256X(int8_t CS_PIN, SPIClass* spi = &SPI, uint32_t spi_speed = 2000000);
        /// @brief I2C constructor.
        explicit ISM6HG256X(TwoWire* i2c, uint8_t address = ISM6HG256X_I2C_ADD_L);
        bool begin() override;
        /// @brief Reads the gyro plus whichever accelerometer channel is in range.
        bool read(IMUData& out) override;
        /// @brief Estimates the gyro zero-rate bias by averaging readings taken at rest.
        /// @param samples The number of readings to take.
        /// @param delay_ms The delay between each reading in milliseconds.
        /// @return True if calibration was successful, false otherwise.
        bool calibrate(int samples, int delay_ms = 10) override;
        bool dataReady() override;

        /// @brief Reads the low-g accelerometer regardless of saturation state.
        bool readLowG(AccelData& out);
        /// @brief Reads the high-g accelerometer regardless of saturation state.
        bool readHighG(AccelData& out);
        /// @brief Which channel the most recent read() drew from.
        AccelSource lastAccelSource() const { return accel_source_; }
    private:
        bool readTemperature(float& out);
        static bool isSaturated(const ISM6HG256X_AxesRaw_t& raw);

        ISM6HG256XSensor raw_imu_;
        int8_t cs_pin_;
        // Cached at begin() so read() does not re-read the full-scale registers
        // on every sample. mg/LSB and mdps/LSB respectively.
        float_t acc_sensitivity_ = ISM6HG256X_ACC_SENSITIVITY_FS_16G;
        float_t hg_sensitivity_ = ISM6HG256X_ACC_SENSITIVITY_FS_64G;
        float_t gyro_sensitivity_ = ISM6HG256X_GYRO_SENSITIVITY_FS_2000DPS;
        AccelSource accel_source_ = AccelSource::LowG;
        float gyro_bias_x_ = 0.0f;
        float gyro_bias_y_ = 0.0f;
        float gyro_bias_z_ = 0.0f;
};
