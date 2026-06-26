#include "config.h"

#include "scheduler.h"
#include "sensors/bmp581.h"
#include "sensors/lsm6dsox.h"
#include "sensors/rocket_ahrs.h"
#include "sensors/kalman_filter.h"

// #include <Fusion.h>

Scheduler<3> sensorScheduler;

LSM6DSOX imu(IMU_CS_PIN);
IMUData imuData;
BMP581 baro(BARO_CS_PIN);
BaroData baroData;
LinearKalmanFilter kalmanFilter;

void queueHelper(bool ok, const char* name);
void imuTask();
void baroTask(); 
void loggerTask();

constexpr uint32_t imuPeriod = 1e6 / 208; // 208 Hz -> ~4.8 ms
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
  Serial.printf("AHRS: %.2f, %.2f, %.2f | Vert: %.3f m/s^2 ", euler.angle.roll, euler.angle.pitch, euler.angle.yaw, get_vertical_acceleration_ms2());
  Serial.printf("| LKF: Alt: %.2f m, Vel: %.2f m/s ", kalmanFilter.get_altitude(), kalmanFilter.get_velocity());
  for (int i = 0; i < sensorScheduler.taskCount(); i++) {
    float dt = sensorScheduler.getTaskDtSeconds(i);
    Serial.printf("Task %d dt: %.4f s, ", i, dt); 
  }
  Serial.println();
}



void loop() {
  sensorScheduler.tick(micros());
}

void queueHelper(bool ok, const char* name) {
    if (ok) { Serial.printf("%s successful\n", name); return; }
    Serial.printf("%s failed\n", name);
    while (1) delay(1000);
}
