#include "sensors/bmp581.h"
#include "SPI.h"


BMP581::BMP581(int8_t CS_PIN) : cs_pin_(CS_PIN) {}


bool BMP581::begin() {
    bool ok = (cs_pin_ < 0) ? raw_baro_.begin() : raw_baro_.begin(cs_pin_,&SPI);
    if (!ok) return false;
    raw_baro_.setTemperatureOversampling(BMP5XX_OVERSAMPLING_1X);
    raw_baro_.setPressureOversampling(BMP5XX_OVERSAMPLING_8X);
    raw_baro_.setIIRFilterCoeff(BMP5XX_IIR_FILTER_BYPASS);
    raw_baro_.setOutputDataRate(BMP5XX_ODR_50_HZ);
    raw_baro_.setPowerMode(BMP5XX_POWERMODE_NORMAL);
    raw_baro_.enablePressure(true);
    return true;
}

bool BMP581::read(BaroData& out) {
    if (!raw_baro_.performReading()) return false;
    out.pressure_pa = raw_baro_.pressure * 100.0f;
    out.temperature_c = raw_baro_.temperature;
    out.altitude_m = 44330.0f * (1.0f - powf(out.pressure_pa / pad_pressure_pa_, 1.0f / 5.257f));
    out.timestamp_us  = micros();
    return true;
}

bool BMP581::calibrate(int samples, int delay_ms) {
    float sum = 0.0f;
    for (int i = 0; i < samples; i++) {
        if (!raw_baro_.performReading()) return false;
        sum += raw_baro_.pressure * 100.0f;  // hPa -> Pa
        delay(delay_ms);
    }
    pad_pressure_pa_ = sum / samples;
    Serial.print("Pad pressure reference: ");
    Serial.print(pad_pressure_pa_);
    Serial.println(" Pa");
    Serial.println("Calibration complete.");
    return true;
}
bool BMP581::dataReady() {
    return raw_baro_.dataReady();
}

