
/**
 * Plant Monitor System
 * Monitors soil humidity, ambient light, and temperature/humidity using DHT11
 * Features automatic cycling between sensors and error detection
 * Author: Elina Rosato
 * Last Updated: 29 May 2025
 */

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <string.h>

#include "lcd.h"
#include "adc.h"
#include "dht11.h"


// =============================================================================
// CONFIGURATION AND CONSTANTS
// =============================================================================

// UART settings for serial communication
#define F_CPU           16000000UL
#define BAUD            9600
#define UBRR_VALUE ((F_CPU / 16 / BAUD) - 1)

// Pin definitions
#define LIGHT_BUTTON   PD2
#define DHT_BUTTON     PD3
#define SOIL_BUTTON    PD4
#define LED_RED        PD6
#define LED_GREEN      PD7
#define BUZZER         PC2

// Sensor ADC channels
#define SENSOR_LIGHT   0    // ADC0
#define SENSOR_SOIL    1    // ADC1

// Sensor thresholds
#define SOIL_MIN        100      // Too dry threshold  
#define SOIL_MAX        700     // Too wet threshold  
#define LIGHT_MIN       200     // Too dark threshold  
#define LIGHT_MAX       1000    // Too bright threshold
#define TEMP_MAX        35      // Too hot threshold (°C)

// Timing constants (in timer ticks, 1 tick = 1s)
#define SENSOR_CYCLE_INTERVAL       4   // 4s - Auto cycle between sensors
#define DHT_READ_INTERVAL           4   // 4s - DHT11 reading interval
#define ERROR_CHECK_INTERVAL        2   // 2s - Sensor Reading and error checking interval

// Error codes
typedef enum {
    ERROR_NONE = 0,
    ERROR_TOO_BRIGHT = 1,
    ERROR_TOO_HOT = 2, 
    ERROR_TOO_WET = 3,
    ERROR_TOO_DRY = 4,
} error_code_t;

// Sensor types
typedef enum {
    SENSOR_TYPE_SOIL = 0,
    SENSOR_TYPE_DHT = 1,
    SENSOR_TYPE_LIGHT = 2,
    SENSOR_COUNT = 3
} sensor_type_t;

// =============================================================================
// GLOBAL STATE VARIABLES
// =============================================================================

// System state
volatile sensor_type_t current_sensor = SENSOR_TYPE_LIGHT;
volatile error_code_t current_error = ERROR_NONE;
volatile uint8_t auto_cycle_enabled = 1;

// Timer counters
volatile uint8_t cycle_timer = 0;
volatile uint8_t dht_timer = 0;
volatile uint8_t error_timer = 0;

// Flags for non-blocking operations
volatile uint8_t check_errors_flag = 0;
volatile uint8_t read_dht_flag = 0;
volatile uint8_t update_display_flag = 1;

// Cached sensor data
struct {
    uint8_t temperature;
    uint8_t humidity;
    uint8_t dht_valid;
    uint16_t light_value;
    uint16_t soil_value;
} sensor_data = {0, 0, 0, 0, 0};

// =============================================================================
// FUNCTION PROTOTYPES
// =============================================================================
void uart_init(void);
void uart_putchar(char c);
void uart_putstring(const char* str);
void uart_print_sensor_data(void);
void display_error(error_code_t error);
void display_soil_sensor(void);
void display_dht11_sensor(void);
void display_light_sensor(void);
void read_adc_sensors(void);
void read_dht11_sensor(void);
void check_sensor_errors(void);
void update_led_and_buzzer(void);
void buzzer_beep(void);
void update_display(void);
void timer1_init(void);
void gpio_init(void);
void interrupts_init(void);
void system_init(void);

// =============================================================================
// UART COMMUNICATION FUNCTIONS
// =============================================================================

// Initialize UART for serial communication
void uart_init(void) {
    // Set baud rate
    UBRR0H = (uint8_t)(UBRR_VALUE >> 8);
    UBRR0L = (uint8_t)UBRR_VALUE;
    UCSR0B = (1 << TXEN0);                  // Enable transmitter
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // Set frame format: 8 data bits, 1 stop bit
}

// Send a character via UART
void uart_putchar(char c) {
    // Wait for empty transmit buffer
    while (!(UCSR0A & (1 << UDRE0)));
    // Put data into buffer, sends the data
    UDR0 = c;
}

// Send a string via UART
void uart_putstring(const char* str) {
    while (*str) {
        uart_putchar(*str++);
    }
}

void uart_print_sensor_data(void) {
    char buffer[64];
    
    snprintf(buffer, sizeof(buffer), 
            "Light: %u%%, Soil: %u%%, Temp: %u deg C, Hum: %u%% | Error: %u\r\n",
            (uint16_t)((1023 - sensor_data.light_value) * 100UL / 1023),
            (uint16_t)((sensor_data.soil_value - SOIL_MIN) * 100UL / (SOIL_MAX - SOIL_MIN)),
            sensor_data.temperature,
            sensor_data.humidity,
            current_error);
    uart_putstring(buffer);
}

// =============================================================================
// TIMER SYSTEM
// =============================================================================

void timer1_init(void) {
    TCCR1B  = (1<<WGM12) | (1<<CS12)|(1<<CS10);  // CTC, prescaler 1024
    OCR1A   = F_CPU/1024/1 - 1;              // 1Hz (1s intervals)
    TIMSK1 |= (1<<OCIE1A);
}

// Timer ISR - Central timing coordinator: Runs every 1s and sets flags for various tasks
ISR(TIMER1_COMPA_vect) {
    // Sensor reading and error checking (every 2 seconds)
    if (++error_timer >= ERROR_CHECK_INTERVAL) {
        error_timer = 0;
        check_errors_flag = 1;
    }
    
    // DHT11 reading (every 4 seconds)
    if (++dht_timer >= DHT_READ_INTERVAL) {
        dht_timer = 0;
        read_dht_flag = 1;
    }
    
    // Auto sensor cycling (every 4 seconds, only when no errors)
    if (auto_cycle_enabled && current_error == ERROR_NONE) {
        if (++cycle_timer >= SENSOR_CYCLE_INTERVAL) {
            cycle_timer = 0;
            current_sensor = (current_sensor + 1) % SENSOR_COUNT;
            update_display_flag = 1;  // Trigger display update
        }
    }
}

// =============================================================================
// SENSOR READING FUNCTIONS
// =============================================================================

void read_adc_sensors(void) {
    sensor_data.light_value = adc_read(SENSOR_LIGHT);
    sensor_data.soil_value = adc_read(SENSOR_SOIL);
    // DHT11 reading is handled separately due to timing requirements
}

void read_dht11_sensor(void) {
    uint8_t temp, humidity;
    
    if (dht11_read(&humidity, &temp)) {
        sensor_data.temperature = temp;
        sensor_data.humidity = humidity;
        sensor_data.dht_valid = 1;
    } else {
        sensor_data.dht_valid = 0;
    }
}

// =============================================================================
// ERROR DETECTION SYSTEM
// =============================================================================

void check_sensor_errors(void) {
    error_code_t previous_error = current_error;  //Store previous error state
    
    if (sensor_data.soil_value > SOIL_MAX) {
        current_error = ERROR_TOO_WET;
    } else if (sensor_data.soil_value < SOIL_MIN) {
        current_error = ERROR_TOO_DRY;
    } else if (sensor_data.light_value < (1023 - LIGHT_MAX)) {
        current_error = ERROR_TOO_BRIGHT;
    } else if (sensor_data.dht_valid && sensor_data.temperature > TEMP_MAX) {
        current_error = ERROR_TOO_HOT;
    } else {
        current_error = ERROR_NONE; 
    }

    if (current_error != previous_error) update_display_flag = 1; // Trigger display update if error state changed
    
    // Disable auto-cycling when error occurs
    if (current_error != ERROR_NONE) {
        auto_cycle_enabled = 0;
        cycle_timer = 0;
    }
}

// =============================================================================
// LED AND BUZZER CONTROL SYSTEM  
// =============================================================================

void buzzer_beep(void) {
    for(uint16_t i = 0; i < 200; i++) {
        PORTC |= (1 << BUZZER);   // High
        _delay_us(10);           // 10 microseconds
        PORTC &= ~(1 << BUZZER);  // Low
        _delay_us(500);           // 500 microseconds
    }
}

void update_led_and_buzzer(void) {
    if (current_error != ERROR_NONE) {
        // Error state: Red ON, Blue OFF, Buzzer ON
        PORTD |= (1 << LED_RED);
        PORTD &= ~(1 << LED_GREEN);
        buzzer_beep();
    } else {
        // Normal state: Blue ON, Red OFF
        PORTD |= (1 << LED_GREEN);
        PORTD &= ~(1 << LED_RED);
    }
}

// =============================================================================
// LCD DISPLAY FUNCTIONS
// =============================================================================

void display_error(error_code_t error) {
    const char* error_messages[] = {
        "No Error",
        "Too Bright",
        "Too Hot", 
        "Too Wet",
        "Too Dry"
    };
    
    lcd_print_rows((uint8_t*)"Error Detected:", (uint8_t*)error_messages[error]);
}

void display_light_sensor(void) {
    uint16_t light_pct = (1023 - sensor_data.light_value) * 100UL / 1023;
    char second_row[17];
    snprintf(second_row, sizeof(second_row), "%u%%", light_pct);
    lcd_print_rows((uint8_t*)"Ambient Light:", (uint8_t*)second_row);
}

void display_soil_sensor(void) {
    uint16_t soil_value = sensor_data.soil_value;
    char second_row[17];
    if (soil_value < SOIL_MIN) {
        soil_value = SOIL_MIN;
    } else if (soil_value > SOIL_MAX) { 
        soil_value = SOIL_MAX;
    }
    uint16_t soil_pct = (soil_value - SOIL_MIN) * 100UL / (SOIL_MAX - SOIL_MIN);
    snprintf(second_row, sizeof(second_row), "%u%%", soil_pct);
    lcd_print_rows((uint8_t*)"Soil Moisture:", (uint8_t*)second_row);
}

void display_dht_sensor(void) {
    char first_row[17];
    char second_row[17];
    
    if (sensor_data.dht_valid) {
        snprintf(first_row, sizeof(first_row), "Temp: %u%cC", sensor_data.temperature, 223); // 223 is the degree symbol in ASCII
        snprintf(second_row, sizeof(second_row), "Humidity: %u%%", sensor_data.humidity);
        lcd_print_rows((uint8_t*)first_row, (uint8_t*)second_row);
    } else {
        lcd_print_rows((uint8_t*)"DHT11 Sensor:", (uint8_t*)"Not Available");
    }
}

void update_display(void) {
    // Display error if any
    if (current_error != ERROR_NONE) {
        display_error(current_error);
        return;
    }
    
    // Display current sensor data
    switch (current_sensor) {
        case SENSOR_TYPE_LIGHT:
            display_light_sensor();
            break;
        case SENSOR_TYPE_SOIL:
            display_soil_sensor();
            break;
        case SENSOR_TYPE_DHT:
            display_dht_sensor();
            break;
        default:
            break;
    }
}

// =============================================================================
// INTERRUPT HANDLERS (BUTTON INPUTS)
// =============================================================================

// Light sensor button (INT0)
ISR(INT0_vect) {
    current_sensor = SENSOR_TYPE_LIGHT;
    auto_cycle_enabled = 0;  // Disable auto-cycling when user selects
    cycle_timer = 0;
    update_display_flag = 1;
}

// DHT11 sensor button (INT1) 
ISR(INT1_vect) {
    current_sensor = SENSOR_TYPE_DHT;
    auto_cycle_enabled = 0;
    cycle_timer = 0;
    update_display_flag = 1;
}

// Soil sensor button (PCINT)
ISR(PCINT2_vect) {
    if (!(PIND & (1 << SOIL_BUTTON))) {  // Button pressed (active low)
        current_sensor = SENSOR_TYPE_SOIL;
        auto_cycle_enabled = 0;
        cycle_timer = 0;
        update_display_flag = 1;
    }
}

// =============================================================================
// SYSTEM INITIALIZATION
// =============================================================================

void gpio_init(void) {
    // LED pins as outputs
    DDRD |= (1 << LED_RED) | (1 << LED_GREEN);
    PORTD &= ~((1 << LED_RED) | (1 << LED_GREEN));
    
    // Button pins as inputs
    DDRD &= ~((1 << SOIL_BUTTON) | (1 << DHT_BUTTON) | (1 << LIGHT_BUTTON));

    // Buzzer pin as output and set it LOW
    DDRC |=  (1<<BUZZER);
    PORTC &= ~(1<<BUZZER);
}

void interrupts_init(void) {
    // External interrupts for INT0 and INT1 (falling edge)
    EICRA = (1 << ISC01) | (1 << ISC11);
    EIMSK = (1 << INT0) | (1 << INT1);
    
    // Pin change interrupt for PCINT20 (PD4)
    PCICR = (1 << PCIE2);
    PCMSK2 = (1 << PCINT20);
}

void system_init(void) {
    // Initialize all subsystems
    uart_init();
    lcd_init();
    adc_init();
    dht11_init();
    gpio_init();
    timer1_init();
    interrupts_init();
    
    sei();    // Enable global interrupts
    
    uart_putstring("\r\n=== Plant Monitor System Started ===\r\n");     // System startup message
    uart_putstring("System initialized. Starting sensor readings...\r\n");
    
    lcd_clear_and_home();
    lcd_print_rows((uint8_t*)"Smart Plant",(uint8_t*)"Monitor");

    PORTD |= (1 << LED_GREEN);  // Turn on green LED to indicate system is ready
    PORTD |= (1 << LED_RED);    // Turn on red LED to indicate system is ready
    _delay_ms(2000);
    PORTD &= ~((1 << LED_GREEN) | (1 << LED_RED)); // Turn off LEDs after initialization
}



// =============================================================================
// MAIN PROGRAM LOOP
// =============================================================================

int main(void) {
    system_init();      // Initialize system
    _delay_ms(2000);    // Give DHT11 time to stabilize
    read_adc_sensors();     // Initial soil and light sensors reading
    read_dht11_sensor();    // Initial dht11 sensor reading
    
    // Main event loop
    while (1) {
        // Handle DHT11 reading (every 4 seconds)
        if (read_dht_flag) {
            read_dht_flag = 0;
            read_dht11_sensor();
        }
        
        // Handle soil and light sensor reading and error checking (every 2 seconds)
        if (check_errors_flag) {
            check_errors_flag = 0;
            read_adc_sensors();     // Update sensor readings
            check_sensor_errors();
            update_led_and_buzzer();
            uart_print_sensor_data();  // Debug output
        }

        // Update display if needed
        if (update_display_flag) {
            update_display_flag = 0;
            update_display();
        }

        // Re-enable auto-cycling if error is cleared
        if (current_error == ERROR_NONE && !auto_cycle_enabled) {
            auto_cycle_enabled = 1;
            cycle_timer = 0;
        }
        
        // Small delay to prevent busy-waiting
        _delay_ms(10);
    }
    
    return 0;
}