#include "config.h"

#include "scheduler.h"
#include "sensors/bmp581.h"
#include "sensors/ism6hg256x.h"
#include "sensors/rocket_ahrs.h"
#include "sensors/kalman_filter.h"
#include "sensors/ism6hg256x.h"
// #include "telemetry.h"
// #include <Fusion.h>

Scheduler<3> sensorScheduler;

// ISM6HG256X imu(IMU_CS_PIN);
ISM6HG256X imu(IMU_CS_PIN, &SPI); // 2 MHz SPI clock
IMUData imuData;
BMP581 baro(BARO_CS_PIN);
BaroData baroData;
LinearKalmanFilter kalmanFilter;

void queueHelper(bool ok, const char* name);
void imuTask();
void baroTask(); 
void loggerTask();

constexpr uint32_t imuPeriod = 1e6 / 240; // 240 Hz -> ~4.2 ms, matches the ISM6HG256X ODR
constexpr uint32_t baroPeriod = 1e6 / 50; // 50 Hz -> 20 ms
constexpr uint32_t logPeriod = 1e6 / 10; // 10 Hz -> 100 ms


void setup() {
  // Init protocols
  SPI.begin(); 
  Serial.begin(115200);
  while (!Serial) { delay(100); }
  Serial.println("Serial available");
  // Init sensors
  queueHelper(imu.begin(), "IMU Initialization");
  queueHelper(baro.begin(), "Barometer Initialization");
  delay(1000); // wait for sensors to settle
  // Calibrate sensors
  queueHelper(imu.calibrate(200, 5), "IMU Calibration");
  queueHelper(baro.calibrate(200, 5), "Barometer Calibration");  
  // Init AHRS
  queueHelper(setup_fusion(), "AHRS Initialization");
  uint32_t now = micros();
  sensorScheduler.addTask(imuTask, imuPeriod, now); // task 0
  sensorScheduler.addTask(baroTask, baroPeriod, now); // task 1
  sensorScheduler.addTask(loggerTask, logPeriod, now); // task 2 etc etc
  // Tick the AHRS
  uint32_t converge_start = micros();
  while (micros() - converge_start < 3000000) {  // 3 seconds
      uint32_t now = micros();
      sensorScheduler.tick(now);
  }
  Serial.println("Vertical Bias Calibration Starting");
  calibrate_vertical_bias(200, []() { sensorScheduler.tick(micros()); });
  Serial.println("Vertical Bias Calibration Complete");
  kalmanFilter.reset();
  Serial.println("Setup complete!");
}

void imuTask() {
  imu.read(imuData);
  update_fusion(imuData.accelX_ms2, imuData.accelY_ms2, imuData.accelZ_ms2,
                imuData.gyroX_rps, imuData.gyroY_rps, imuData.gyroZ_rps,
                sensorScheduler.getTaskDtSeconds(0));
  kalmanFilter.predict(get_vertical_acceleration_ms2(), sensorScheduler.getTaskDtSeconds(0));
}

void baroTask() {
  baro.read(baroData);
  kalmanFilter.update(baroData.altitude_m);
}

void loggerTask() {
  // Serial.printf("| IMU | Accel: %f, %f, %f Gyro: %f, %f, %f Temperature: %f C | Baro | Pressure: %f Pa, Temperature: %f C, Altitude: %f m ",
  //               imuData.accelX_ms2,
  //               imuData.accelY_ms2,
  //               imuData.accelZ_ms2,
  //               imuData.gyroX_rps,
  //               imuData.gyroY_rps,
  //               imuData.gyroZ_rps,
  //               imuData.temperature_c,
  //               baroData.pressure_pa,
  //               baroData.temperature_c,
  //               baroData.altitude_m);
  FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(ahrs_get()));
  // Example precision limiting (.2 for 2 decimals, .3 for 3 decimals)
  const char* accelSource = (imu.lastAccelSource() == ISM6HG256X::AccelSource::HighG) ? "HIGH_G" : "LOW_G";
  Serial.printf("AHRS: %.2f, %.2f, %.2f | Vert: %.3f m/s^2 (%s) ", euler.angle.roll, euler.angle.pitch, euler.angle.yaw, get_vertical_acceleration_ms2(), accelSource);
  Serial.printf("| LKF: Alt: %.2f m, Vel: %.2f m/s ", kalmanFilter.get_altitude(), kalmanFilter.get_velocity());
  for (int i = 0; i < sensorScheduler.taskCount(); i++) {
    float dt = sensorScheduler.getTaskDtSeconds(i);
    Serial.printf("Task %d dt: %.4f s, ", i, dt); 
  }
  Serial.println();
}

// void loggerTask() {
//     AccelData lo;
//     AccelData hi;
//     imu.readLowG(lo);
//     imu.readHighG(hi);
//     bool srcHi = (imu.lastAccelSource() == ISM6HG256X::AccelSource::HighG);
//     FusionQuaternion q = FusionAhrsGetQuaternion(ahrs_get());
//     TelemExtras x = {
//         q.element.w, q.element.x, q.element.y, q.element.z,
//         get_vertical_acceleration_ms2(),
//         kalmanFilter.get_altitude(),
//         kalmanFilter.get_velocity(),
//         // kalmanFilter.get_acceleration(),   // drop if you don't expose this
//         (uint8_t)0,
//         0                           // 0..1, or just 0.0f for now
//     };
//     float dts[TELEM_MAX_TASKS];
//     uint8_t n = min((int)sensorScheduler.taskCount(), TELEM_MAX_TASKS);
//     for (uint8_t i = 0; i < n; i++) dts[i] = sensorScheduler.getTaskDtSeconds(i);

//     telem_sample(imuData, lo, hi, srcHi, baroData, x, dts, n);
// }


void loop() {
  sensorScheduler.tick(micros());
}

void queueHelper(bool ok, const char* name) {
    if (ok) { Serial.printf("%s successful\n", name); return; }
    Serial.printf("%s failed\n", name);
    while (1) delay(1000);
}
