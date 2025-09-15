/*******************************************************
 * @file Basic.ino
 *
 * @brief Basic example demonstrating PT100/PT1000 RTD
 *        temperature reading using the 7Semi MAX31865 library.
 *
 * This sketch reads a PT100 or PT1000 RTD using software SPI
 * via the MAX31865 and prints raw ADC code, resistance (ohms),
 * and temperature (Celsius) to the Serial Monitor.
 *
 * Key features demonstrated:
 * - Software SPI on user-defined pins
 * - Configurable reference and nominal resistances (Rref/R0)
 * - Continuous temperature updates using auto-convert mode
 *
 * Connections:
 * - CS    -> D10
 * - SCK   -> D13
 * - MISO  -> D12
 * - MOSI  -> D11
 * - VCC   -> 3.3V or 5V
 * - GND   -> GND
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

#include <7Semi_MAX31865.h>

// =======================
// === Pin Configuration ===
// =======================
// Choose any available GPIO pins for software SPI
#define CS_PIN 10
#define SCK_PIN 13
#define MISO_PIN 12
#define MOSI_PIN 11

// =======================
// === RTD Configuration ===
// =======================
// Set your reference resistor and RTD nominal resistance
#define RREF 430.0  // Typical value on MAX31865 boards
#define R0 100.0    // PT100 = 100.0, PT1000 = 1000.0

// =======================
// === Object Creation ===
// =======================
// Create MAX31865 object using software SPI on specified pins
MAX31865_7Semi rtd(CS_PIN, SCK_PIN, MISO_PIN, MOSI_PIN);

void setup() {
  // Start Serial Monitor
  Serial.begin(115200);
  delay(200);

  // Initialize MAX31865 with default settings:
  // (2/4-wire mode, 50Hz filter, auto-convert, Vbias on)
  rtd.begin();

  // Set reference resistor (Rref) and nominal RTD resistance (R0)
  rtd.setReferenceResistor(RREF);
  rtd.setR0(R0);

  Serial.println(F("MAX31865 ready."));
}

void loop() {
  // Read raw ADC value (15-bit), resistance, and temperature
  float ohms = rtd.readResistance();
  float tempC = rtd.readTemperatureC();

  // Print results to Serial Monitor
  Serial.print(F("R: "));
  Serial.print(ohms, 3);
  Serial.print(F(" Ω"));
  Serial.print(F("  T: "));
  Serial.print(tempC, 2);
  Serial.println(F(" °C"));

  delay(1000);  // Wait 1 second
}
