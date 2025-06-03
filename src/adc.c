// adc.c
#include "adc.h"
#include <avr/io.h>
#include <util/delay.h>

void adc_init(void) {
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1); // Enable ADC, prescaler = 64
    ADMUX = (1 << REFS0);    // AVCC as reference
    DIDR0 = 0xFF;            // Disable digital input buffers on all ADC pins (optional)
}

uint16_t adc_read(uint8_t channel) {
    ADMUX = (ADMUX & 0xF0) | (channel & 0x0F); // Keep reference bits, set channel
    _delay_us(10); // Let the voltage settle
    ADCSRA |= (1 << ADSC);                     // Start dummy conversion
    while (ADCSRA & (1 << ADSC));
    
    ADCSRA |= (1 << ADSC);                     // Real conversion
    while (ADCSRA & (1 << ADSC));         // Wait for conversion to complete
    return ADC;
}