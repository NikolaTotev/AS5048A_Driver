/**
 * @file as5048a.cpp
 * @brief Implementation of the AS5048A magnetic rotary encoder driver
 */

#include "as5048a.h"
#include <hardware/gpio.h>
#include <hardware/spi.h>
#include <hardware/pwm.h>
#include <hardware/clocks.h>
#include <math.h>
#include <pico/time.h>

#define M_PI 3.14159265358979323846

// SPI mode constructor
AS5048A::AS5048A(spi_inst_t* spi_instance, uint8_t cs_pin, uint8_t sck_pin, uint8_t mosi_pin, uint8_t miso_pin, uint32_t speed_hz) {
    _spi = spi_instance;
    _cs_pin = cs_pin;
    _sck_pin = sck_pin;
    _mosi_pin = mosi_pin;
    _miso_pin = miso_pin;
    _speed_hz = speed_hz;
    _use_pwm = false;
}

// PWM mode constructor
AS5048A::AS5048A(uint8_t pwm_pin) {
    _pwm_pin = pwm_pin;
    _use_pwm = true;
}

bool AS5048A::begin() {
    if (_use_pwm) {
        // Configure GPIO for PWM input
        gpio_init(_pwm_pin);
        gpio_set_dir(_pwm_pin, GPIO_IN);
        gpio_pull_up(_pwm_pin);

        // For PWM input, we would typically configure a capture mode
        // but RP2040 doesn't have direct PWM input capture
        // We could use PIO for this, but a simpler approach is to use
        // GPIO interrupt-based capture in the main application
        return true;
    } else {
        // SPI mode initialization
        // Initialize CS pin as GPIO output
        gpio_init(_cs_pin);
        gpio_set_dir(_cs_pin, GPIO_OUT);
        gpio_put(_cs_pin, 1); // CS inactive high
        
        // Initialize SPI pins
        gpio_set_function(_sck_pin, GPIO_FUNC_SPI);
        gpio_set_function(_mosi_pin, GPIO_FUNC_SPI);
        gpio_set_function(_miso_pin, GPIO_FUNC_SPI);
        
        // Initialize SPI with desired settings
        spi_init(_spi, _speed_hz);
        spi_set_format(_spi, 8, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST);
        
        // Wait a moment for the sensor to be ready
        sleep_ms(10);
        
        // Test communication by reading the NOP register
        uint16_t response = readRegister(REG_NOP);
        return (response != 0xFFFF); // If we got a valid response, init was successful
    }
}

uint16_t AS5048A::getRawAngle() {
    if (_use_pwm) {
        return _raw_angle;
    } else {
        _raw_angle = readRegister(REG_ANGLE) & 0x3FFF; // Mask to 14 bits
        return _raw_angle;
    }
}

float AS5048A::getAngleDegrees() {
    return rawToDegrees(getRawAngle());
}

float AS5048A::getAngleRadians() {
    return rawToRadians(getRawAngle());
}

uint16_t AS5048A::getMagnitude() {
    if (_use_pwm) {
        return 0; // Not available in PWM mode
    }
    return readRegister(REG_MAGNITUDE) & 0x3FFF; // Mask to 14 bits
}

uint8_t AS5048A::getAGC() {
    if (_use_pwm) {
        return 0; // Not available in PWM mode
    }
    return readRegister(REG_DIAG_AGC) & 0xFF; // AGC is in the lower 8 bits
}

uint8_t AS5048A::getFieldStatus() {
    if (_use_pwm) {
        return 0; // Not available in PWM mode
    }
    
    uint16_t diagValue = readRegister(REG_DIAG_AGC);
    bool comp_high = (diagValue >> 11) & 0x01;
    bool comp_low = (diagValue >> 10) & 0x01;
    
    if (comp_high) return 1; // Field too strong
    if (comp_low) return 2;  // Field too weak
    return 0; // Field OK
}

bool AS5048A::setZeroPosition(uint16_t rawAngle) {
    if (_use_pwm) {
        return false; // Not available in PWM mode
    }
    
    rawAngle &= 0x3FFF; // Ensure only the lower 14 bits are used
    
    // Writing the zero position requires writing to two registers
    uint16_t high_bits = (rawAngle >> 6) & 0xFF;
    uint16_t low_bits = rawAngle & 0x3F;
    
    bool success = true;
    success &= writeRegister(REG_OTP_ZERO_POS_HI, high_bits);
    success &= writeRegister(REG_OTP_ZERO_POS_LOW, low_bits);
    
    return success;
}

bool AS5048A::burnZeroPosition() {
    if (_use_pwm) {
        return false; // Not available in PWM mode
    }
    
    // WARNING: This operation is irreversible. The zero position can only be programmed once.
    
    // To program the OTP, follow the sequence from the datasheet:
    // 1. Enable programming
    bool success = writeRegister(REG_PROGRAMMING_CTRL, 0x0001);
    
    // 2. Set the burn bit
    success &= writeRegister(REG_PROGRAMMING_CTRL, 0x0008);
    
    // Wait for the burning to complete (typically 5ms)
    sleep_ms(10);
    
    // 3. Verify the programming by setting the verify bit
    success &= writeRegister(REG_PROGRAMMING_CTRL, 0x0040);
    
    // 4. Check that the zero position is correct
    uint16_t angle = getRawAngle();
    
    // Reset the programming control register
    writeRegister(REG_PROGRAMMING_CTRL, 0x0000);
    
    // Zero position should be close to zero after programming
    return success && (angle < 100);
}

uint16_t AS5048A::readRegister(uint16_t reg) {
    if (_use_pwm) {
        return 0; // Not available in PWM mode
    }
    
    // Set the read bit (bit 14)
    uint16_t command = reg | CMD_READ;
    
    // Add parity bit
    command |= (calculateParity(command) << 15);
    
    // Perform the transaction
    gpio_put(_cs_pin, 0); // CS active low
    uint16_t result = transfer16(command);
    gpio_put(_cs_pin, 1); // CS inactive high
    
    // Second transfer to get the data
    sleep_us(1); // Small delay
    gpio_put(_cs_pin, 0); // CS active low
    result = transfer16(REG_NOP | CMD_READ | (calculateParity(REG_NOP | CMD_READ) << 15));
    gpio_put(_cs_pin, 1); // CS inactive high
    
    // Check for errors
    if (result & (1 << 14)) { // Error flag
        clearErrors();
        return 0xFFFF; // Return error code
    }
    
    return result & 0x3FFF; // Mask to 14 bits of data
}

bool AS5048A::writeRegister(uint16_t reg, uint16_t value) {
    if (_use_pwm) {
        return false; // Not available in PWM mode
    }
    
    // Command without the read bit set (bit 14 = 0)
    uint16_t command = reg & ~CMD_READ;
    
    // Add parity bit
    command |= (calculateParity(command) << 15);
    
    // First transfer: Send the register address
    gpio_put(_cs_pin, 0); // CS active low
    uint16_t result = transfer16(command);
    gpio_put(_cs_pin, 1); // CS inactive high
    
    // Prepare the data frame
    uint16_t data_frame = value & 0x3FFF; // Mask to 14 bits
    // The "write" bit must be 0
    data_frame &= ~(1 << 14);
    // Add parity bit
    data_frame |= (calculateParity(data_frame) << 15);
    
    // Second transfer: Send the data
    sleep_us(1); // Small delay
    gpio_put(_cs_pin, 0); // CS active low
    result = transfer16(data_frame);
    gpio_put(_cs_pin, 1); // CS inactive high
    
    return !(result & (1 << 14)); // Return true if no error flag
}

uint8_t AS5048A::clearErrors() {
    if (_use_pwm) {
        return 0; // Not available in PWM mode
    }
    
    // The CLEAR_ERROR_FLAG register clears error flags
    uint16_t command = REG_CLEAR_ERROR_FLAG | CMD_READ;
    command |= (calculateParity(command) << 15);
    
    gpio_put(_cs_pin, 0); // CS active low
    uint16_t result = transfer16(command);
    gpio_put(_cs_pin, 1); // CS inactive high
    
    // Return the error flags that were cleared
    return (result >> 1) & 0x07; // Bits 1-3 contain the error flags
}

bool AS5048A::updateFromPWM() {
    if (!_use_pwm) {
        return false; // Not in PWM mode
    }
    
    // This would typically be implemented using a timer/counter to capture PWM pulses
    // For RP2040, a better approach would be using the PIO state machine
    // As a placeholder, we'll just indicate that this function needs to be implemented
    // in the application code using GPIO interrupts or PIO
    
    // In a real implementation, we would:
    // 1. Measure the PWM pulse width
    // 2. Calculate the angle from the pulse width according to the datasheet
    // 3. Update _raw_angle
    
    // Return false to indicate this is a placeholder
    return false;
}

// Helper function: Calculate the even parity bit for a given value
uint16_t AS5048A::calculateParity(uint16_t value) {
    uint16_t parity = 0;
    // The datasheet uses even parity over bits 0-14
    value &= 0x7FFF; // Mask off the parity bit if present
    
    while (value) {
        parity ^= (value & 1);
        value >>= 1;
    }
    
    return parity;
}

// Helper function: Perform a 16-bit SPI transfer
uint16_t AS5048A::transfer16(uint16_t data) {
    uint8_t tx_msb = (data >> 8) & 0xFF;
    uint8_t tx_lsb = data & 0xFF;
    uint8_t rx_msb, rx_lsb;
    
    spi_write_read_blocking(_spi, &tx_msb, &rx_msb, 1);
    spi_write_read_blocking(_spi, &tx_lsb, &rx_lsb, 1);
    
    return (rx_msb << 8) | rx_lsb;
}

// Helper function: Convert raw angle to radians
float AS5048A::rawToRadians(uint16_t raw) {
    return (raw * 2.0f * M_PI) / 16384.0f;
}

// Helper function: Convert raw angle to degrees
float AS5048A::rawToDegrees(uint16_t raw) {
    return (raw * 360.0f) / 16384.0f;
}