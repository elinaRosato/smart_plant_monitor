#include <avr/io.h>

// adc.h
#ifndef ADC_H
#define ADC_H

#include <avr/io.h>
#include <stdint.h>

void adc_init(void);
uint16_t adc_read(uint8_t channel);

#endif


