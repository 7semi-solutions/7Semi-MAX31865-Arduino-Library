/*******************************************************
 * @file FaultDetect.ino
 *
 * @brief Beginner example demonstrating fault detection
 *        using the 7Semi MAX31865 RTD interface board.
 *
 * This sketch uses hardware SPI to communicate with the MAX31865
 * and detects wiring or sensor faults while reading PT100 temperature.
 *
 * Key features demonstrated:
 * - Hardware SPI on default Arduino pins
 * - Configurable chip select (CS) pin
 * - Support for PT100 RTD (R0 = 100 Ω)
 * - Reference resistor = 430 Ω
 * - Low and high threshold detection (e.g., 20–40 Ω)
 * - Serial output of raw ADC code, resistance, temperature, and faults
 *
 * Connections:
 * - VCC   -> 3.3V or 5V
 * - GND   -> GND
 * - CS    -> user-defined (e.g., D10)
 * - SCK   -> D13
 * - MISO  -> D12
 * - MOSI  -> D11
 *
 * @section author Author
 * Written by 7Semi
 *
 * @section version Version
 * 1.0 - 15-Sep-2025
 *
 * @section license License
 * @license MIT
 * Copyright (c) 2025 7Semi
 *******************************************************/

#include <Arduino.h>
#include <SPI.h>
#include <7Semi_MAX31865.h>

// --------------------
// User configuration
// --------------------
#define CS_PIN    10      // Chip Select: change to any free GPIO
#define RREF_OHM  430.0f  // Reference resistor on your board (typ. 430 Ω)
#define R0_OHM    100.0f  // PT100 = 100 Ω (use 1000.0f for PT1000)

// --------------------
// SPI Pin Definitions (used only if switching to software SPI)
// --------------------
#define SCK_PIN   13;  // Default SCK 
#define MISO_PIN  12;  // Default MISO
#define MOSI_PIN  11;  // Default MOSI

// --------------------
// RTD Sensor Instance
// --------------------

// For hardware SPI (uses board's default SPI pins)
MAX31865_7Semi rtd(CS_PIN, SPI);

// For software SPI 
// MAX31865_7Semi rtd(CS_PIN, SCK_PIN, MISO_PIN, MOSI_PIN);

// Fault Helper
static void printFaultsAndClear() {
  Serial.println(F("FAULT detected!"));
  MAX31865_7Semi::FaultStatus f = rtd.readFaultStatus();
  if (f.rtdHigh)       Serial.println(F("  - RTD HIGH threshold"));
  if (f.rtdLow)        Serial.println(F("  - RTD LOW threshold"));
  if (f.refInHigh)     Serial.println(F("  - REFIN- > 0.85*Vbias"));
  if (f.refInLow)      Serial.println(F("  - REFIN- < 0.85*Vbias"));
  if (f.rtdInLow)      Serial.println(F("  - RTDIN- < 0.85*Vbias"));
  if (f.overUnderVolt) Serial.println(F("  - Over/Under Voltage"));
  rtd.clearFaults();
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { /* wait on boards that need it */ }
  delay(200);

  // Initialize MAX31865
  rtd.begin(WIRES_2,      // change to WIRES_3 if you have a 3-wire RTD
            FILTER_50HZ,  // or FILTER_60HZ
            true,         // autoConvert
            true,         // Vbias ON
            1000000);     // SPI = 1 MHz

  // Set reference resistor and RTD type
  rtd.setReferenceResistor(RREF_OHM);
  rtd.setR0(R0_OHM);

  // Fault window in OHMS (safe for PT100)
  rtd.setLowThreshold(20.0f);
  rtd.setHighThreshold(40.0f);
  rtd.clearFaults();

  Serial.println(F("MAX31865 ready."));
}
void loop() {
  // Check for faults
  if (rtd.readFault()) {
    printFaultsAndClear();
    delay(1000);
    return;
  }

  // Read sensor
  float ohms = rtd.readResistance();
  float temp = rtd.readTemperatureC();

  Serial.print(F("R="));
  Serial.print(ohms, 3);
  Serial.print(F(" Ω"));
  Serial.print(F("  T="));
  Serial.print(temp, 2);
  Serial.println(F(" °C"));

  delay(1000);
}


/*
  Notes:
  - To use 3-wire: change begin(...) to WIRES_3.
  - To use PT1000: rtd.setR0(1000.0f); and adjust thresholds (e.g., 80–160 Ω).
  - If you need software SPI (any pins), create the object like:
        MAX31865_7Semi rtd(CS_PIN, SCK_PIN, MISO_PIN, MOSI_PIN);
*/
