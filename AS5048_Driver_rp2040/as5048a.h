/**
 * @file as5048a.h
 * @brief Driver for the AS5048A magnetic rotary encoder
 * 
 * This driver supports the AS5048A magnetic rotary encoder using the SPI interface
 * on RP2040 microcontrollers. The AS5048A is a high-resolution (14-bit) position sensor.
 */

#ifndef AS5048A_H
#define AS5048A_H

#include <hardware/spi.h>
#include <hardware/gpio.h>
#include <hardware/pwm.h>

class AS5048A {
public:
    // Register addresses
    static constexpr uint16_t REG_NOP = 0x0000;
    static constexpr uint16_t REG_CLEAR_ERROR_FLAG = 0x0001;
    static constexpr uint16_t REG_PROGRAMMING_CTRL = 0x0003;
    static constexpr uint16_t REG_OTP_ZERO_POS_HI = 0x0016;
    static constexpr uint16_t REG_OTP_ZERO_POS_LOW = 0x0017;
    static constexpr uint16_t REG_DIAG_AGC = 0x3FFD;
    static constexpr uint16_t REG_MAGNITUDE = 0x3FFE;
    static constexpr uint16_t REG_ANGLE = 0x3FFF;
    
    // Command bit masks
    static constexpr uint16_t CMD_READ = 0x4000;    // Read command (bit 14 set)
    static constexpr uint16_t CMD_WRITE = 0x0000;   // Write command (bit 14 clear)
    
    // PWM interface definitions
    static constexpr uint16_t PWM_TOTAL_PERIOD = 4119; // Total number of PWM clock cycles

    /**
     * @brief Constructor for SPI mode
     * 
     * @param spi_instance SPI instance (spi0 or spi1)
     * @param cs_pin Chip select pin
     * @param sck_pin Clock pin
     * @param mosi_pin MOSI pin
     * @param miso_pin MISO pin
     * @param speed_hz SPI clock speed in Hz (default: 1MHz)
     */
    AS5048A(spi_inst_t* spi_instance, uint8_t cs_pin, uint8_t sck_pin, uint8_t mosi_pin, uint8_t miso_pin, uint32_t speed_hz = 1000000);

    /**
     * @brief Constructor for PWM mode
     * 
     * @param pwm_pin PWM input pin
     */
    AS5048A(uint8_t pwm_pin);
    
    /**
     * @brief Initialize the sensor
     * 
     * @return true if initialization successful
     */
    bool begin();
    
    /**
     * @brief Get the raw 14-bit angle
     * 
     * @return uint16_t Raw 14-bit angle value (0-16383)
     */
    uint16_t getRawAngle();
    
    /**
     * @brief Get the angle in degrees
     * 
     * @return float Angle in degrees (0-360)
     */
    float getAngleDegrees();
    
    /**
     * @brief Get the angle in radians
     * 
     * @return float Angle in radians (0-2π)
     */
    float getAngleRadians();
    
    /**
     * @brief Get the magnitude of the magnetic field
     * 
     * @return uint16_t Magnitude value
     */
    uint16_t getMagnitude();
    
    /**
     * @brief Get the automatic gain control value
     * 
     * @return uint8_t AGC value (0-255)
     */
    uint8_t getAGC();
    
    /**
     * @brief Check if there's a magnetic field issue
     * 
     * @return uint8_t 0: No error, 1: Field too strong, 2: Field too weak
     */
    uint8_t getFieldStatus();
    
    /**
     * @brief Set the zero position
     * 
     * @param rawAngle Raw angle to set as zero
     * @return true if successful
     */
    bool setZeroPosition(uint16_t rawAngle);
    
    /**
     * @brief Program the zero position to OTP (permanent, can only be done once!)
     * 
     * @return true if successful
     */
    bool burnZeroPosition();

    /**
     * @brief Read from a register
     * 
     * @param reg Register address
     * @return uint16_t Register value
     */
    uint16_t readRegister(uint16_t reg);
    
    /**
     * @brief Write to a register
     * 
     * @param reg Register address
     * @param value Value to write
     * @return true if successful
     */
    bool writeRegister(uint16_t reg, uint16_t value);
    
    /**
     * @brief Check for errors and clear them
     * 
     * @return uint8_t Error flags
     */
    uint8_t clearErrors();

    /**
     * @brief Update angle from PWM input (call regularly when using PWM mode)
     * 
     * @return bool true if a valid reading was obtained
     */
    bool updateFromPWM();

private:
    // SPI communication parameters
    spi_inst_t* _spi = nullptr;
    uint8_t _cs_pin = 0;
    uint8_t _sck_pin = 0;
    uint8_t _mosi_pin = 0;
    uint8_t _miso_pin = 0;
    uint32_t _speed_hz = 1000000;
    
    // PWM input parameters
    uint8_t _pwm_pin = 0;
    bool _use_pwm = false;
    uint32_t _pwm_pulse_width = 0;
    uint32_t _pwm_period = 0;
    
    // Current angle reading
    uint16_t _raw_angle = 0;
    
    // Helper functions
    uint16_t calculateParity(uint16_t value);
    uint16_t transfer16(uint16_t data);
    float rawToRadians(uint16_t raw);
    float rawToDegrees(uint16_t raw);
};

#endif // AS5048A_H