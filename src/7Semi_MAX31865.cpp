/*
  7Semi_MAX31865.cpp - Source for MAX31865_7Semi Driver
  -----------------------------------------------------
  Author  : 7Semi
  Version : 1.0
  Date    : 12-Sep-2025

  Description:
    This file implements the MAX31865 RTD-to-digital converter interface with support
    for both hardware and software SPI on Arduino platforms.

  Supported Features:
    - PT100/PT1000 sensors with 2, 3, or 4-wire configurations
    - Hardware SPI and bit-banged Software SPI
    - 50Hz/60Hz filtering, auto or one-shot conversion
    - Configurable high/low resistance thresholds
    - Fault detection and diagnostics
    - Temperature conversion using Callendar–Van Dusen equation (IEC 60751)
*/

#include "7Semi_MAX31865.h"

// -----------------------------------------------------------------------------
// Constructors (Keep member init order consistent with header)
// -----------------------------------------------------------------------------

// Constructor for hardware SPI
MAX31865_7Semi::MAX31865_7Semi(uint8_t csPin, SPIClass &spi)
  : _soft(false),
    _spi(&spi),
    _spiSet(SPISettings(1000000, MSBFIRST, SPI_MODE1)),
    _sck(255), _miso(255), _mosi(255),
    _cs(csPin),
    _configCache(0),
    _Rref(430.0f), _R0(100.0f),
    _cvd(),
    _halfPeriod_ns(500)
{}

// Constructor for software (bit-banged) SPI
MAX31865_7Semi::MAX31865_7Semi(uint8_t csPin, uint8_t sckPin, uint8_t misoPin, uint8_t mosiPin)
  : _soft(true),
    _spi(nullptr),
    _spiSet(SPISettings(1000000, MSBFIRST, SPI_MODE1)),
    _sck(sckPin), _miso(misoPin), _mosi(mosiPin),
    _cs(csPin),
    _configCache(0),
    _Rref(430.0f), _R0(100.0f),
    _cvd(),
    _halfPeriod_ns(500)
{}

// -----------------------------------------------------------------------------
// Initialization
// -----------------------------------------------------------------------------

bool MAX31865_7Semi::begin(WireMode wires, Filter filter, bool autoConvert, bool vbiasOn, uint32_t spiHz) {
  pinMode(_cs, OUTPUT);
  csHigh(); // Ensure CS is high before SPI begins

  if (_soft) {
    // Software SPI setup
    pinMode(_sck, OUTPUT);
    pinMode(_mosi, OUTPUT);
    pinMode(_miso, INPUT);
    clockIdle();

    if (spiHz == 0) spiHz = 250000;  // Default for bit-bang
    _halfPeriod_ns = (uint32_t)(5e8 / spiHz); // ns per half SPI cycle
  } else {
    // Hardware SPI setup
    _spiSet = SPISettings(spiHz ? spiHz : 1000000, MSBFIRST, SPI_MODE1);
    _spi->begin();
  }

  // Build configuration byte
  _configCache = 0;
  if (vbiasOn) _configCache |= CFG_VBIAS;
  if (autoConvert) _configCache |= CFG_MODE_AUTO;
  if (wires == WIRES_3) _configCache |= CFG_3WIRE;
  if (filter == FILTER_50HZ) _configCache |= CFG_FILT50;

  write8(REG_CONFIG, _configCache);
  clearFaults();
  return true;
}

// -----------------------------------------------------------------------------
// Configuration Setters
// -----------------------------------------------------------------------------

void MAX31865_7Semi::setWireMode(WireMode wires) {
  if (wires == WIRES_3) _configCache |= CFG_3WIRE;
  else _configCache &= ~CFG_3WIRE;
  write8(REG_CONFIG, _configCache);
}

void MAX31865_7Semi::setFilter(Filter f) {
  if (f == FILTER_50HZ) _configCache |= CFG_FILT50;
  else _configCache &= ~CFG_FILT50;
  write8(REG_CONFIG, _configCache);
}

void MAX31865_7Semi::setAutoConvert(bool on) {
  if (on) _configCache |= CFG_MODE_AUTO;
  else _configCache &= ~CFG_MODE_AUTO;
  write8(REG_CONFIG, _configCache);
}

void MAX31865_7Semi::setVbias(bool on) {
  if (on) _configCache |= CFG_VBIAS;
  else _configCache &= ~CFG_VBIAS;
  write8(REG_CONFIG, _configCache);
}

void MAX31865_7Semi::oneshot() {
  write8(REG_CONFIG, _configCache | CFG_1SHOT);
}

void MAX31865_7Semi::clearFaults() {
  write8(REG_CONFIG, _configCache | CFG_FAULTCLR);
}

void MAX31865_7Semi::runFaultDetectCycle(uint8_t c) {
  uint8_t cfg = (_configCache & ~(CFG_FTCTL1 | CFG_FTCTL0)) | ((c & 3) << 2);
  write8(REG_CONFIG, cfg);
  write8(REG_CONFIG, _configCache); // Restore
}

// -----------------------------------------------------------------------------
// Thresholds (Set in raw units, but calculate from °C)
// -----------------------------------------------------------------------------

void MAX31865_7Semi::setHighThreshold(uint16_t value) {
  value = tempToRaw(value, _Rref, _R0) << 1;
  write8(REG_HFT_MSB, (value >> 8) & 0xFF);
  write8(REG_HFT_LSB, value & 0xFF);
}

void MAX31865_7Semi::setLowThreshold(uint16_t value) {
  value = tempToRaw(value, _Rref, _R0) << 1;
  write8(REG_LFT_MSB, (value >> 8) & 0xFF);
  write8(REG_LFT_LSB, value & 0xFF);
}

float MAX31865_7Semi::getHighThreshold() {
  uint8_t v = read8(REG_RTD_MSB);
  v &= 0xFD;
  write8(REG_RTD_MSB, v);
  uint16_t th = read16(REG_HFT_MSB) >> 1;
  return cvdResistanceToC((th / 32767.0f) * _Rref);
}

float MAX31865_7Semi::getLowThreshold() {
  uint8_t v = read8(REG_RTD_MSB);
  v &= 0xFD;
  write8(REG_RTD_MSB, v);
  uint16_t th = read16(REG_LFT_MSB) >> 1;
  return cvdResistanceToC((th / 32767.0f) * _Rref);
}

// -----------------------------------------------------------------------------
// Sensor Readings
// -----------------------------------------------------------------------------

uint16_t MAX31865_7Semi::readRaw() {
  uint16_t raw = read16(REG_RTD_MSB);
  return raw >> 1;
}

float MAX31865_7Semi::readResistance() {
  return (readRaw() / 32768.0f) * _Rref;
}

float MAX31865_7Semi::readTemperatureC() {
  return cvdResistanceToC(readResistance());
}

// -----------------------------------------------------------------------------
// Fault Handling
// -----------------------------------------------------------------------------

bool MAX31865_7Semi::readFault() {
  return (read16(REG_RTD_MSB) & 0x0001) != 0;
}

uint8_t MAX31865_7Semi::readFaultReg() {
  return read8(REG_FAULT);
}

MAX31865_7Semi::FaultStatus MAX31865_7Semi::readFaultStatus() {
  FaultStatus s{};
  uint8_t f = read8(REG_FAULT);
  s.rtdHigh       = f & FLT_RTDHIGH;
  s.rtdLow        = f & FLT_RTDLOW;
  s.refInHigh     = f & FLT_REFINHIGH;
  s.refInLow      = f & FLT_REFINLOW;
  s.rtdInLow      = f & FLT_RTDINLOW;
  s.overUnderVolt = f & FLT_OVUV;
  return s;
}

// -----------------------------------------------------------------------------
// Model Parameters
// -----------------------------------------------------------------------------

void MAX31865_7Semi::setReferenceResistor(float r) { _Rref = r; }
void MAX31865_7Semi::setR0(float r0)               { _R0 = r0; }
void MAX31865_7Semi::setCVD(const CVD &cvd)        { _cvd = cvd; }

// -----------------------------------------------------------------------------
// SPI Register Access
// -----------------------------------------------------------------------------

void MAX31865_7Semi::write8(uint8_t reg, uint8_t val) {
  csLow();
  spiTransfer(reg | 0x80); // Set MSB for write
  spiTransfer(val);
  csHigh();
}

uint8_t MAX31865_7Semi::read8(uint8_t reg) {
  csLow();
  spiTransfer(reg & 0x7F); // Clear MSB for read
  uint8_t val = spiTransfer(0x00);
  csHigh();
  return val;
}

uint16_t MAX31865_7Semi::read16(uint8_t reg) {
  csLow();
  spiTransfer(reg & 0x7F);
  uint16_t msb = spiTransfer(0x00);
  uint16_t lsb = spiTransfer(0x00);
  csHigh();
  return (msb << 8) | lsb;
}

// -----------------------------------------------------------------------------
// SPI Transfer Handling
// -----------------------------------------------------------------------------

uint8_t MAX31865_7Semi::spiTransfer(uint8_t b) {
  if (_soft) return softSpiTransfer(b);
  _spi->beginTransaction(_spiSet);
  uint8_t result = _spi->transfer(b);
  _spi->endTransaction();
  return result;
}

void MAX31865_7Semi::clockIdle() {
  if (_soft) digitalWrite(_sck, LOW);
}

void MAX31865_7Semi::halfPeriodDelay() {
  uint32_t us = (_halfPeriod_ns + 999) / 1000;
  if (us) delayMicroseconds(us);
  else __asm__ __volatile__("nop\n\t");
}

// Bit-bang SPI (Mode1: CPOL=0, CPHA=1)
uint8_t MAX31865_7Semi::softSpiTransfer(uint8_t b) {
  uint8_t rx = 0;
  for (uint8_t i = 0; i < 8; ++i) {
    digitalWrite(_mosi, (b & 0x80) ? HIGH : LOW);
    b <<= 1;
    halfPeriodDelay();

    digitalWrite(_sck, HIGH);
    halfPeriodDelay();

    rx <<= 1;
    if (digitalRead(_miso)) rx |= 1;

    digitalWrite(_sck, LOW);
  }
  return rx;
}

// -----------------------------------------------------------------------------
// Temperature Conversion (Callendar–Van Dusen)
// -----------------------------------------------------------------------------

float MAX31865_7Semi::cvdResistanceToC(float R) const {
  const float A = _cvd.A, B = _cvd.B, C = _cvd.C;
  float a = B * _R0, b = A * _R0, c = _R0 - R;
  float disc = b * b - 4 * a * c;

  // Try quadratic solution first (valid ≥ 0°C)
  if (disc >= 0.0f) {
    float T = (-b + sqrtf(disc)) / (2 * a);
    if (T >= -0.5f) return T;
  }

  // Else, Newton-Raphson for sub-zero
  float T = -20.0f;
  for (int i = 0; i < 25; ++i) {
    float T2 = T * T, T3 = T2 * T;
    float f  = _R0 * (1 + A*T + B*T2 + C*(T - 100.0f)*T3) - R;
    float df = _R0 * (A + 2*B*T + C*(4*T3 - 300.0f*T2));
    if (fabsf(df) < 1e-12f) break;
    float dT = -f / df;
    T += dT;
    if (fabsf(dT) < 1e-4f) break;
  }
  return T;
}

// Convert temperature in °C to raw RTD ADC value
uint16_t MAX31865_7Semi::tempToRaw(float tempC, float Rref, float R0) {
  const float A = _cvd.A, B = _cvd.B, C = _cvd.C;
  float Rt;

  if (tempC >= 0.0f) {
    Rt = R0 * (1 + A * tempC + B * tempC * tempC);
  } else {
    float t2 = tempC * tempC;
    float t3 = t2 * tempC;
    Rt = R0 * (1 + A * tempC + B * t2 + C * (tempC - 100.0f) * t3);
  }

  return (uint16_t)roundf((Rt / Rref) * 32768.0f);
}
