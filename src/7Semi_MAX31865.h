/*
  MAX31865_7Semi.h - MAX31865 RTD-to-Digital Converter Driver
  ------------------------------------------------------------
  Author  : 7Semi
  Version : 1.0
  Date    : 12-Sep-2025

  Description:
    This Arduino library provides an easy interface to the MAX31865 IC,
    which reads temperature from RTD sensors such as PT100 and PT1000.

  Features:
    - Supports 2-wire, 3-wire, and 4-wire RTD configurations
    - Compatible with both Hardware and Software SPI
    - Supports 50Hz/60Hz noise filters
    - One-shot or continuous auto-conversion mode
    - Built-in fault detection and decoding
    - Programmable thresholds for RTD range checking
    - Accurate Callendar–Van Dusen temperature conversion
    - Customizable reference resistor, R0, and CVD coefficients

  Usage:
    Include this header in your Arduino project to interface with
    a MAX31865 IC via SPI. Call `begin()` during setup and use the
    available read functions to get resistance or temperature data.

  Dependencies:
    - SPI.h
    - Arduino.h

  Wiring (example for HW SPI):
    MAX31865 -> Arduino
    CS       -> D10 (or any GPIO)
    SCK      -> SCK
    MISO     -> MISO
    MOSI     -> MOSI

*/

#pragma once
#ifndef _SEMI_MAX31865_H_
#define _SEMI_MAX31865_H_

#include <Arduino.h>
#include <SPI.h>
#include <math.h>

// =========================
// MAX31865 Register Map
// =========================
#define REG_CONFIG   0x00
#define REG_RTD_MSB  0x01
#define REG_RTD_LSB  0x02
#define REG_HFT_MSB  0x03
#define REG_HFT_LSB  0x04
#define REG_LFT_MSB  0x05
#define REG_LFT_LSB  0x06
#define REG_FAULT    0x07

// =========================
// Configuration Register Bits
// =========================
#define CFG_VBIAS     (1 << 7)  // Enable bias voltage
#define CFG_MODE_AUTO (1 << 6)  // Auto conversion mode
#define CFG_1SHOT     (1 << 5)  // One-shot conversion
#define CFG_3WIRE     (1 << 4)  // 3-wire configuration
#define CFG_FTCTL1    (1 << 3)  // Fault detection cycle control
#define CFG_FTCTL0    (1 << 2)
#define CFG_FAULTCLR  (1 << 1)  // Clear faults
#define CFG_FILT50    (1 << 0)  // 50Hz filter (0 = 60Hz)

// =========================
// Fault Register Bits
// =========================
#define FLT_RTDHIGH    (1 << 7)
#define FLT_RTDLOW     (1 << 6)
#define FLT_REFINHIGH  (1 << 5)
#define FLT_REFINLOW   (1 << 4)
#define FLT_RTDINLOW   (1 << 3)
#define FLT_OVUV       (1 << 2)

// =========================
// Wire Mode Enum
// =========================
enum WireMode : uint8_t {
  WIRES_2 = 0,
  WIRES_4 = 0,  // Treated the same as 2-wire
  WIRES_3 = 1
};

// =========================
// Filter Frequency Enum
// =========================
enum Filter : uint8_t {
  FILTER_60HZ = 0,
  FILTER_50HZ = 1
};

// =========================
// MAX31865 Class Declaration
// =========================
class MAX31865_7Semi {
public:

  // Callendar–Van Dusen Coefficients for temperature calculation
  struct CVD {
    float A = 3.9083e-3f;
    float B = -5.775e-7f;
    float C = -4.183e-12f; // Only used for < 0°C
  };

  // Faults structure for decoding the fault register
  struct FaultStatus {
    bool rtdHigh, rtdLow;
    bool refInHigh, refInLow;
    bool rtdInLow, overUnderVolt;
  };

  // =========================
  // Constructors
  // =========================
  MAX31865_7Semi(uint8_t csPin, SPIClass &spi = SPI); // Hardware SPI
  MAX31865_7Semi(uint8_t csPin, uint8_t sckPin, uint8_t misoPin, uint8_t mosiPin); // Software SPI

  // =========================
  // Initialization
  // =========================
  bool begin(WireMode wires = WIRES_2,
             Filter filter = FILTER_50HZ,
             bool autoConvert = true,
             bool vbiasOn = true,
             uint32_t spiHz = 1000000);

  // =========================
  // Configuration
  // =========================
  void setWireMode(WireMode wires);
  void setFilter(Filter f);
  void setAutoConvert(bool on);
  void setVbias(bool on);
  void oneshot();                 // Start one-shot conversion
  void clearFaults();             // Clear fault flags
  void runFaultDetectCycle(uint8_t cycle = 0b11); // Run manual fault detection

  // =========================
  // Threshold Configuration
  // =========================
  void setHighThreshold(uint16_t value);
  void setLowThreshold(uint16_t value);
  float getHighThreshold();
  float getLowThreshold();

  // =========================
  // Reading Data
  // =========================
  uint16_t readRaw();             // Raw RTD value
  float    readResistance();      // Resistance in ohms
  float    readTemperatureC();    // Temperature in Celsius

  // =========================
  // Fault Handling
  // =========================
  bool        readFault();        // Check if fault exists
  FaultStatus readFaultStatus();  // Decode fault register
  uint8_t     readFaultReg();     // Raw fault register

  // =========================
  // Sensor Model Configuration
  // =========================
  void setReferenceResistor(float rref_ohm); // Rref (default 430Ω)
  void setR0(float r0_ohm);                  // R0 for PT100/PT1000 (default 100Ω)
  void setCVD(const CVD &cvd);               // Set custom CVD coefficients

  // =========================
  // Utility
  // =========================
  uint16_t tempToRaw(float tempC, float Rref, float R0); // Convert °C to raw value

private:
  // =========================
  // Internal State
  // =========================
  bool        _soft        = false;
  SPIClass   *_spi         = nullptr;
  SPISettings _spiSet      = SPISettings(1000000, MSBFIRST, SPI_MODE1);

  uint8_t _sck  = 255;
  uint8_t _miso = 255;
  uint8_t _mosi = 255;
  uint8_t _cs   = 255;

  uint8_t _configCache = 0;
  float   _Rref        = 430.0f; // Reference resistor
  float   _R0          = 100.0f; // Nominal RTD resistance at 0°C
  CVD     _cvd;

  uint32_t _halfPeriod_ns = 500; // Delay for software SPI

  // =========================
  // SPI Helpers
  // =========================
  inline void csLow()  { if (_cs != 255) digitalWrite(_cs, LOW);  }
  inline void csHigh() { if (_cs != 255) digitalWrite(_cs, HIGH); }

  void     write8(uint8_t reg, uint8_t val);
  uint8_t  read8(uint8_t reg);
  uint16_t read16(uint8_t reg);

  uint8_t  spiTransfer(uint8_t b);
  uint8_t  softSpiTransfer(uint8_t b);
  void     clockIdle();
  void     halfPeriodDelay();

  float    cvdResistanceToC(float R) const; // Convert resistance to temperature
};

#endif // _SEMI_MAX31865_H_
