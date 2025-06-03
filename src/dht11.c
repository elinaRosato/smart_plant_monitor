#include "dht11.h"

// Initialize DHT11 sensor
void dht11_init(void) {
    DHT11_DDR &= ~(1 << DHT11_PIN);   // Set as input initially
    DHT11_PORT |= (1 << DHT11_PIN);   // Enable internal pull-up
}

// Read a single bit from DHT11
uint8_t dht11_read_bit(void) {
    // Wait for pin to go low
    while (DHT11_PINR & (1 << DHT11_PIN));
    
    // Wait for pin to go high
    while (!(DHT11_PINR & (1 << DHT11_PIN)));
    
    // Wait 30us and check if pin is still high
    _delay_us(30);
    
    if (DHT11_PINR & (1 << DHT11_PIN)) {
        return 1;  // Bit is 1
    } else {
        return 0;  // Bit is 0
    }
}

// Read a byte from DHT11
uint8_t dht11_read_byte(void) {
    uint8_t byte = 0;
    
    for (int i = 7; i >= 0; i--) {
        byte |= (dht11_read_bit() << i);
    }
    
    return byte;
}

// Main function to read DHT11 sensor
uint8_t dht11_read(uint8_t* humidity, uint8_t* temperature) {
    uint8_t data[5];
    uint8_t checksum;
    
    // Send start signal
    DHT11_DDR |= (1 << DHT11_PIN);    // Set pin as output
    DHT11_PORT &= ~(1 << DHT11_PIN);  // Pull pin low
    _delay_ms(18);                    // Wait 18ms
    
    DHT11_PORT |= (1 << DHT11_PIN);   // Pull pin high
    _delay_us(30);                    // Wait 30us
    
    DHT11_DDR &= ~(1 << DHT11_PIN);   // Set pin as input
    
    // Wait for DHT11 response
    // DHT11 should pull pin low for 80us, then high for 80us
    uint16_t timeout = 1000;
    
    // Wait for pin to go low (DHT11 response)
    while ((DHT11_PINR & (1 << DHT11_PIN)) && timeout--);
    if (timeout == 0) return 0;  // Timeout error
    
    // Wait for pin to go high
    timeout = 1000;
    while (!(DHT11_PINR & (1 << DHT11_PIN)) && timeout--);
    if (timeout == 0) return 0;  // Timeout error
    
    // Read 5 bytes of data
    for (int i = 0; i < 5; i++) {
        data[i] = dht11_read_byte();
    }
    
    // Calculate checksum
    checksum = data[0] + data[1] + data[2] + data[3];
    
    // Verify checksum
    if (checksum != data[4]) {
        return 0;  // Checksum error
    }
    
    // Extract humidity and temperature
    *humidity = data[0];
    *temperature = data[2];
    
    return 1;  // Success
}