# 7Semi-MAX31865-Arduino-Library

This Arduino library enables temperature sensing using the **7Semi MAX31865 breakout board**, compatible with **PT100 and PT1000 RTDs**. It communicates over SPI and provides high-accuracy readings for industrial and scientific use.

![Arduino](https://img.shields.io/badge/platform-arduino-blue.svg)
![License](https://img.shields.io/badge/license-MIT-green.svg)
![Status](https://img.shields.io/badge/status-active-brightgreen.svg)

---

## Hardware Required

- 7Semi MAX31865 Module  
- PT100 or PT1000 RTD sensor  
- Arduino-compatible board  
- SPI connection (SCK, MISO, MOSI, CS)  

---

## Getting Started

### 1. Installation via Arduino Library Manager

1. Open the **Arduino IDE**
2. Go to:
   - `Sketch > Include Library > Manage Libraries…` (IDE 1.x), or  
   - Use the **Library Manager icon** (IDE 2.x sidebar)
3. Search for:
   -7Semi MAX31865
4. Click **Install**

Then include in your sketch:
#include <7semi_max31865.h>

### 2.Wiring Example (SPI)

| MAX31865 Pin | Arduino Pin (Uno Example) |
| ------------ | ------------------------- |
| VCC          | 3.3V or 5V                |
| GND          | GND                       |
| CS           | D10                       |
| SCK          | D13                       |
| MISO         | D12                       |
| MOSI         | D11                       |


### 3.Applications

Industrial temperature sensing

Scientific data acquisition

Lab instrumentation

HVAC and process control



