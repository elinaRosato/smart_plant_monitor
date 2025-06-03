#ifndef DHT11_H
#define DHT11_H

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>

// Define the pin where DHT11 data pin is connected
#define DHT11_PIN PD5  // Digital pin 2 (Port D, bit 2)
#define DHT11_DDR DDRD
#define DHT11_PORT PORTD
#define DHT11_PINR PIND

// Function prototypes
void dht11_init(void);
uint8_t dht11_read(uint8_t* humidity, uint8_t* temperature);
uint8_t dht11_read_bit(void);
uint8_t dht11_read_byte(void);

#endif // DHT11_H
