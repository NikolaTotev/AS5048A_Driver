# AS5048A Magnetic Rotary Encoder Driver for RP2040

This repository contains a C++ driver for the AS5048A magnetic rotary encoder, designed specifically for the Raspberry Pi Pico and other RP2040-based microcontrollers. The AS5048A is a high-resolution (14-bit) position sensor that uses magnetic sensing to determine rotational position.

## Features

- **High-resolution position sensing**: 14-bit resolution (16,384 positions per revolution)
- **Multiple interface options**:
  - SPI communication mode (primary implementation)
  - PWM mode support (infrastructure in place, requires application-level implementation)
- **Comprehensive sensing capabilities**:
  - Angle measurement (raw, degrees, and radians)
  - Magnetic field magnitude measurement
  - Automatic Gain Control (AGC) value reading
  - Field strength diagnostic information
- **Zero position configuration**:
  - Software-based zero position setting
  - One-time OTP (One-Time Programmable) zero position burning

## Hardware Connection

### SPI Mode

The default implementation uses the SPI interface with the following pin connections:

| AS5048A Pin | RP2040 Pin (Default) | Description |
|-------------|----------------------|-------------|
| CSN         | GPIO 5               | Chip Select |
| SCL/SCLK    | GPIO 2               | SPI Clock   |
| MOSI        | GPIO 3               | Master Out Slave In |
| MISO        | GPIO 4               | Master In Slave Out |
| VDD         | 3.3V                 | Power Supply |
| GND         | GND                  | Ground      |

### PWM Mode

The driver includes infrastructure for PWM mode, which would use a single GPIO pin:

| AS5048A Pin | RP2040 Pin (Example) | Description |
|-------------|----------------------|-------------|
| PWM         | GPIO 6               | PWM Output  |
| VDD         | 3.3V                 | Power Supply |
| GND         | GND                  | Ground      |

*Note: PWM mode requires additional implementation at the application level for capturing and processing the PWM signal.*

## Getting Started

### Prerequisites

- Raspberry Pi Pico or other RP2040-based board
- Raspberry Pi Pico SDK installed
- AS5048A magnetic rotary encoder
- Development environment with CMake

### Building the Project

1. Clone this repository
```bash
git clone https://github.com/yourusername/AS5048_Driver_rp2040.git
cd AS5048_Driver_rp2040
```

2. Create a build directory and run CMake
```bash
mkdir build
cd build
cmake ..
```

3. Build the project
```bash
make
```

4. Flash the binary to your RP2040 board
```bash
picotool load -x AS5048_Driver_rp2040.uf2
```

## Usage Example

The repository includes a sample usage file that demonstrates how to use the AS5048A driver:

```cpp
#include "as5048a.h"

// Define pin connections
#define SPI_PORT spi0
#define PIN_CS   5
#define PIN_SCK  2
#define PIN_MOSI 3
#define PIN_MISO 4

int main() {
    // Initialize stdio for serial output
    stdio_init_all();
    
    // Create encoder object with SPI interface
    AS5048A encoder(SPI_PORT, PIN_CS, PIN_SCK, PIN_MOSI, PIN_MISO);
    
    // Initialize the encoder
    if (!encoder.begin()) {
        printf("Failed to initialize encoder\n");
        return 1;
    }
    
    while (1) {
        // Read angle values
        uint16_t rawAngle = encoder.getRawAngle();
        float degrees = encoder.getAngleDegrees();
        float radians = encoder.getAngleRadians();
        
        // Print readings
        printf("Angle: %u raw, %.2f°, %.2f rad\n", rawAngle, degrees, radians);
        
        sleep_ms(500);  // Update every 500ms
    }
}
```

## API Reference

### Constructors

```cpp
// SPI mode constructor
AS5048A(spi_inst_t* spi_instance, uint8_t cs_pin, uint8_t sck_pin, uint8_t mosi_pin, uint8_t miso_pin, uint32_t speed_hz = 1000000);

// PWM mode constructor
AS5048A(uint8_t pwm_pin);
```

### Core Functions

```cpp
// Initialize the sensor
bool begin();

// Get the raw 14-bit angle
uint16_t getRawAngle();

// Get the angle in degrees (0-360)
float getAngleDegrees();

// Get the angle in radians (0-2π)
float getAngleRadians();
```

### Diagnostic Functions

```cpp
// Get the magnitude of the magnetic field
uint16_t getMagnitude();

// Get the automatic gain control value
uint8_t getAGC();

// Check magnetic field status (0: OK, 1: Too strong, 2: Too weak)
uint8_t getFieldStatus();
```

### Configuration Functions

```cpp
// Set the zero position
bool setZeroPosition(uint16_t rawAngle);

// Permanently program the zero position (one-time only!)
bool burnZeroPosition();
```

### Low-level Functions

```cpp
// Read from a register
uint16_t readRegister(uint16_t reg);

// Write to a register
bool writeRegister(uint16_t reg, uint16_t value);

// Check for errors and clear them
uint8_t clearErrors();

// Update angle from PWM input (for PWM mode)
bool updateFromPWM();
```

## Technical Details

The AS5048A provides:
- 14-bit angular position measurement (0-16383 values)
- Up to 1MHz SPI communication
- On-chip diagnostics and error detection
- Ability to set and permanently program a zero position
- PWM output option for simple interfaces

## Datasheet and Resources

- [AS5048A Datasheet](https://ams-osram.com/products/sensor-solutions/position-sensors/ams-as5048a-high-resolution-position-sensor)
- [RP2040 Documentation](https://www.raspberrypi.com/documentation/microcontrollers/rp2040.html)

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgments

- The AS5048A driver is designed specifically for the Raspberry Pi Pico SDK
- This driver was developed with the help of Claude Sonet 3.7.
- This driver has been tested by a human before release.
