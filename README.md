[![Designed by Sitron Labs](https://img.shields.io/badge/Designed_by-Sitron_Labs-FCE477)](https://www.sitronlabs.com/)
[![Join the Discord community](https://img.shields.io/discord/552242187665145866?logo=discord&logoColor=white&label=Discord&color=%237289da)](https://discord.gg/btnVDeWhfW)
![License](https://img.shields.io/github/license/sitronlabs/SitronLabs_Diodes_PI3USB9281C_Arduino_Library)
![Latest Release](https://img.shields.io/github/release/sitronlabs/SitronLabs_Diodes_PI3USB9281C_Arduino_Library)
[![Arduino Library Manager](https://www.ardu-badge.com/badge/Sitron%20Labs%20PI3USB9281C%20Arduino%20Library.svg?)](https://www.ardu-badge.com/Sitron%20Labs%20PI3USB9281C%20Arduino%20Library)
[![PlatformIO Registry](https://badges.registry.platformio.org/packages/sitronlabs/library/Sitron_Labs_PI3USB9281C_Arduino_Library.svg)](https://registry.platformio.org/libraries/sitronlabs/Sitron_Labs_PI3USB9281C_Arduino_Library)

# Sitron Labs Diodes PI3USB9281C Arduino Library

Arduino library for interfacing with the Diodes Incorporated (previously Pericom) PI3USB9281C USB device detection IC.

## Description

The PI3USB9281C is a USB charger and device detection IC that can identify various types of USB ports and charging devices. It can detect Standard Downstream Ports (SDP), Charging Downstream Ports (CDP), Dedicated Charging Ports (DCP), various charger types (1A, 2A, 2.4A), and CarKit accessories. The IC also provides control over D+/D- switch routing and monitors device attachment events. This library provides a simple interface to access these features via I2C.

## Installation

### Arduino IDE

Install via the Arduino Library Manager by searching for "Sitron Labs PI3USB9281C".

Alternatively, install manually:
1. Download or clone this repository
2. Place it in your Arduino `libraries` folder
3. Restart the Arduino IDE

### PlatformIO

Install via the PlatformIO Library Manager by searching for "Sitron Labs PI3USB9281C".

Alternatively, add the library manually to your `platformio.ini` file:

```ini
lib_deps = 
    https://github.com/sitronlabs/SitronLabs_Diodes_PI3USB9281C_Arduino_Library.git
```

## Hardware Connections

Connect the PI3USB9281C to your Arduino using I2C:

- VCC → 3.3V or 5V (check your board's specifications)
- GND → GND
- SDA → SDA (I2C Data)
- SCL → SCL (I2C Clock)
- ENB → Any digital pin (enable pin, active LOW)
- INT → NC (unused by this library)

The I2C address is fixed at 0x25.

## Usage

### Basic Device Detection

```cpp
#include <Wire.h>
#include <pi3usb9281c.h>

// Create device object
pi3usb9281c device;

// I2C address (fixed at 0x25)
const uint8_t I2C_ADDRESS = 0x25;

// Enable pin
const int PIN_ENB = 2;

void setup() {
  Serial.begin(9600);
  
  // Initialize I2C
  Wire.begin();
  
  // Setup the PI3USB9281C (I2C library, I2C address, enable pin)
  if (device.setup(Wire, I2C_ADDRESS, PIN_ENB) != 0) {
    Serial.println("Failed to setup PI3USB9281C");
    return;
  }
  
  // Detect the device
  if (!device.detect()) {
    Serial.println("PI3USB9281C not detected");
    return;
  }
  
  // Perform software reset
  device.reset();
  delay(100);
  
  // Set switch to automatic mode
  device.switch_state_set(PI3USB9281C_SWITCH_STATE_AUTO);
  
  Serial.println("PI3USB9281C initialized");
}

void loop() {
  // Check for device attachment (non-blocking)
  int attach_status = device.device_attach_get();
  
  if (attach_status == 1) {
    Serial.println("Device attached!");
    
    // Get device type
    enum pi3usb9281c_device_type type;
    if (device.device_type_get(&type) == 0) {
      switch (type) {
        case PI3USB9281C_DEVICE_TYPE_USB_SDP:
          Serial.println("USB Standard Downstream Port (up to 500mA)");
          break;
        case PI3USB9281C_DEVICE_TYPE_USB_CDP:
          Serial.println("USB Charging Downstream Port (up to 1.5A)");
          break;
        case PI3USB9281C_DEVICE_TYPE_USB_DCP:
          Serial.println("USB Dedicated Charging Port (beyond 1.5A)");
          break;
        case PI3USB9281C_DEVICE_TYPE_CHARGER_1A:
          Serial.println("1.0 Amps charger");
          break;
        case PI3USB9281C_DEVICE_TYPE_CHARGER_2A:
          Serial.println("2.0 Amps charger");
          break;
        case PI3USB9281C_DEVICE_TYPE_CHARGER_2_4A:
          Serial.println("2.4 Amps charger");
          break;
        case PI3USB9281C_DEVICE_TYPE_CARKIT1:
          Serial.println("CarKit type 1 accessory");
          break;
        case PI3USB9281C_DEVICE_TYPE_CARKIT2:
          Serial.println("CarKit type 2 accessory");
          break;
        default:
          Serial.println("Unknown device type");
          break;
      }
    }
  } else if (attach_status < 0) {
    Serial.println("Error reading device status");
  }
  
  delay(100);
}
```

### Waiting for Device Attachment

```cpp
#include <Wire.h>
#include <pi3usb9281c.h>

pi3usb9281c device;
const uint8_t I2C_ADDRESS = 0x25;
const int PIN_ENB = 2;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  
  device.setup(Wire, I2C_ADDRESS, PIN_ENB);
  device.detect();
  device.reset();
  delay(100);
  device.switch_state_set(PI3USB9281C_SWITCH_STATE_AUTO);
}

void loop() {
  Serial.println("Waiting for device attachment...");
  
  // Wait for device attachment (blocking, with timeout)
  if (device.device_attach_wait(5000) == 0) {
    Serial.println("Device attached!");
    
    enum pi3usb9281c_device_type type;
    if (device.device_type_get(&type) == 0) {
      // Process device type...
    }
  } else {
    Serial.println("Timeout waiting for device");
  }
  
  delay(1000);
}
```

## API Reference

### setup(TwoWire &i2c_library, uint8_t i2c_address, int pin_enb)

Initializes the PI3USB9281C device.

- `i2c_library`: I2C library instance to use (typically `Wire`)
- `i2c_address`: I2C address (must be 0x25)
- `pin_enb`: GPIO pin number connected to the device's enable pin (active LOW)

Returns 0 on success, or -EINVAL if the I2C address is invalid.

### detect(void)

Detects if a PI3USB9281C device is present by reading the device ID register.

Returns true if device is detected, false otherwise.

### reset(void)

Performs a software reset of the device.

Returns 0 on success, or a negative error code on I2C communication failure.

### device_attach_get(void)

Checks for device attachment (non-blocking). Reads the interrupt register once to check for device attachment and clears the interrupt flag if a device is attached.

Returns 1 if device attached, 0 if no device attached, or -EIO on I2C error.

### device_attach_wait(uint32_t timeout_ms)

Waits for a device attachment event (blocking). Polls the interrupt register until a device is attached or the timeout expires.

- `timeout_ms`: Maximum time to wait in milliseconds

Returns 0 on device attached, -ETIMEDOUT on timeout, or -EIO on I2C error.

### device_type_get(pi3usb9281c_device_type *type)

Gets the type of attached USB device or charger.

- `type`: Output parameter for the detected device type

Returns 0 on success, or -EIO on I2C communication error.

**Device types:**
- `PI3USB9281C_DEVICE_TYPE_USB_SDP`: Standard Downstream Port (up to 500mA)
- `PI3USB9281C_DEVICE_TYPE_USB_CDP`: Charging Downstream Port (up to 1.5A)
- `PI3USB9281C_DEVICE_TYPE_USB_DCP`: Dedicated Charging Port (beyond 1.5A)
- `PI3USB9281C_DEVICE_TYPE_CHARGER_1A`: 1.0 Amps charger
- `PI3USB9281C_DEVICE_TYPE_CHARGER_2A`: 2.0 Amps charger
- `PI3USB9281C_DEVICE_TYPE_CHARGER_2_4A`: 2.4 Amps charger
- `PI3USB9281C_DEVICE_TYPE_CARKIT1`: CarKit type 1 accessory
- `PI3USB9281C_DEVICE_TYPE_CARKIT2`: CarKit type 2 accessory
- `PI3USB9281C_DEVICE_TYPE_UNKNOWN`: Unknown device type

### switch_state_set(pi3usb9281c_switch_state state)

Controls the state of the D+/D- switch.

- `state`: Desired switch state
  - `PI3USB9281C_SWITCH_STATE_AUTO`: Automatic control based on detected device type
  - `PI3USB9281C_SWITCH_STATE_MANUAL_OPEN`: Force switch to disconnected state
  - `PI3USB9281C_SWITCH_STATE_MANUAL_CLOSED`: Force switch to connected state

Returns 0 on success, -EINVAL for invalid state, or -EIO on I2C error.

## Specifications

- I2C address: 0x25 (fixed)
- Communication interface: I2C
- Enable pin: Active LOW
- Detected USB port types: SDP, CDP, DCP
- Detected charger types: 1A, 2A, 2.4A
- Detected accessories: CarKit type 1 and type 2
- Switch control: Automatic or manual (open/closed)

