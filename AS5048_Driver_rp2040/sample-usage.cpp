/**
 * @file sample_usage.cpp
 * @brief Example showing how to use the AS5048A encoder with RP2040
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "as5048a.h"

// Define pin connections
#define SPI_PORT spi0
#define PIN_CS   5
#define PIN_SCK  2
#define PIN_MOSI 3
#define PIN_MISO 4

// For PWM mode
#define PIN_PWM  6

int main() {
    // Initialize stdio
    stdio_init_all();
    
    printf("AS5048A Magnetic Encoder Test\n");
    
    // Create the encoder object using SPI
    AS5048A encoder(SPI_PORT, PIN_CS, PIN_SCK, PIN_MOSI, PIN_MISO);
    
    // Alternative constructor for PWM mode
    // AS5048A encoder(PIN_PWM);
    
    // Initialize the encoder
    if (!encoder.begin()) {
        printf("Failed to initialize encoder\n");
        while (1) {
            tight_loop_contents();
        }
    }
    
    // Clear any existing errors
    encoder.clearErrors();
    
    while (1) {
        // Read angle values
        uint16_t rawAngle = encoder.getRawAngle();
        float degrees = encoder.getAngleDegrees();
        float radians = encoder.getAngleRadians();
        
        // Read diagnostic information
        uint16_t magnitude = encoder.getMagnitude();
        uint8_t agc = encoder.getAGC();
        uint8_t fieldStatus = encoder.getFieldStatus();
        
        // Print readings
        printf("Angle: %u raw, %.2f°, %.2f rad\n", rawAngle, degrees, radians);
        printf("Magnitude: %u\n", magnitude);
        printf("AGC: %u\n", agc);
        
        // Check magnetic field status
        switch (fieldStatus) {
            case 0:
                printf("Field status: OK\n");
                break;
            case 1:
                printf("Field status: Too strong\n");
                break;
            case 2:
                printf("Field status: Too weak\n");
                break;
        }
        
        printf("\n");
        sleep_ms(20);
    }
    
    return 0;
}