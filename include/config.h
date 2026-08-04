#pragma once
#include <Arduino.h>


#define SERIAL_BAUD_RATE 115200

// ---------------------------------------------------------------------------
// SPI0 -- sensor bus (MAG, BARO, IMU). These are the core's default SPI0 pins,
// so no SPI.setRX()/setSCK()/setTX() remap is needed before SPI.begin().
// ---------------------------------------------------------------------------
#define SENSOR_SPI_RX_PIN 16   // MISO
#define SENSOR_SPI_SCK_PIN 18
#define SENSOR_SPI_TX_PIN 19   // MOSI

#define IMU_CS_PIN 17          // ISM6HG256XTR
#define MAG_CS_PIN 21
#define BARO_CS_PIN 27         // BMP581 -- net is labelled CSN_BMP "GPIO028", but the pad is GPIO27

// ---------------------------------------------------------------------------
// SPI1 -- radio + SD bus (SX1262, microSD). Not the core's default SPI1 pins,
// so SPI1 must be remapped with setRX()/setSCK()/setTX() before SPI1.begin().
// ---------------------------------------------------------------------------
#define RADIO_SD_SPI_RX_PIN 8  // MISO
#define RADIO_SD_SPI_SCK_PIN 10
#define RADIO_SD_SPI_TX_PIN 11 // MOSI

#define SD_CS_PIN 9
#define RADIO_CS_PIN 12        // SX1262
#define RADIO_DIO1_PIN 13
#define RADIO_BUSY_PIN 14
#define RADIO_RESET_PIN 15

// ---------------------------------------------------------------------------
// Stepper driver
//
// NOTE: GPIO20/22/23 are SPI0's *second* pin group -- the same peripheral the
// sensor bus above uses. SPI0 can only be routed to one pin group at a time, so
// the stepper cannot get its own SPI0 instance on these pins. See the comment in
// main.cpp / driver bring-up before wiring this up.
// ---------------------------------------------------------------------------
#define STEPPER_SPI_RX_PIN 20  // MISO
#define STEPPER_SPI_SCK_PIN 22
#define STEPPER_SPI_TX_PIN 23  // MOSI

#define STEPPER_CS_PIN 28
#define STEPPER_ENABLE_PIN 24
#define STEPPER_DIR_PIN 25
#define STEPPER_STEP_PIN 26

// ---------------------------------------------------------------------------
// Status LED
// ---------------------------------------------------------------------------
#define LED_PIN 33             // WS2812B-2020
