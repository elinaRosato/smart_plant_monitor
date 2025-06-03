
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
#define SOIL_MIN        50
#define SOIL_MAX        700
#define LIGHT_MIN       200     // Too dark threshold  
#define LIGHT_MAX       1000     // Too bright threshold
#define TEMP_MAX        30      // Too hot threshold (°C)

// Timing constants (in timer ticks, 1 tick = 500ms)
#define DISPLAY_UPDATE_INTERVAL     1   // 500ms - LCD update rate
#define SENSOR_CYCLE_INTERVAL       8   // 4s - Auto cycle between sensors
#define DHT_READ_INTERVAL           8   // 4s - DHT11 reading interval
#define ERROR_CHECK_INTERVAL        4   // 2s - Error checking interval

// Error codes
typedef enum {
    ERROR_NONE = 0,
    ERROR_TOO_BRIGHT,
    ERROR_TOO_HOT, 
    ERROR_TOO_WET,
    ERROR_TOO_DRY,
    ERROR_DHT_FAIL
} error_code_t;

// Sensor types
typedef enum {
    SENSOR_TYPE_LIGHT = 0,
    SENSOR_TYPE_DHT,
    SENSOR_TYPE_SOIL,
    SENSOR_COUNT
} sensor_type_t;

// =============================================================================
// GLOBAL STATE VARIABLES
// =============================================================================

// System state
volatile sensor_type_t current_sensor = SENSOR_TYPE_LIGHT;
volatile error_code_t current_error = ERROR_NONE;
volatile uint8_t auto_cycle_enabled = 1;

// Timer counters
volatile uint8_t display_timer = 0;
volatile uint8_t cycle_timer = 0;
volatile uint8_t dht_timer = 0;
volatile uint8_t error_timer = 0;

// Flags for non-blocking operations
volatile uint8_t update_display_flag = 0;
volatile uint8_t check_errors_flag = 0;
volatile uint8_t read_dht_flag = 0;

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
void display_soil(void);
void display_temperature(void);
void display_light(void);
void show_selected_sensor(void);
void update_dht_data(void);

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
    OCR1A   = F_CPU/1024/2 - 1;              // 2Hz (500ms intervals)
    TIMSK1 |= (1<<OCIE1A);
}

/**
 * Timer ISR - Central timing coordinator
 * Runs every 500ms and sets flags for various tasks
 */
ISR(TIMER1_COMPA_vect) {
    // Display update (every 500ms)
    if (++display_timer >= DISPLAY_UPDATE_INTERVAL) {
        display_timer = 0;
        update_display_flag = 1;
    }
    
    // Error checking (every 2 seconds)
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
            update_display_flag = 1;  // Force display update
        }
    }
}

// =============================================================================
// SENSOR READING FUNCTIONS
// =============================================================================

void read_all_sensors(void) {
    // Read ADC sensors
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
        uart_putstring("DHT11: OK\r\n");
    } else {
        sensor_data.dht_valid = 0;
        uart_putstring("DHT11: FAIL\r\n");
    }
}

// =============================================================================
// ERROR DETECTION SYSTEM
// =============================================================================

error_code_t check_sensor_errors(void) {
    // Check soil moisture
    if (sensor_data.soil_value > SOIL_MAX) {
        return ERROR_TOO_WET;
    }
    if (sensor_data.soil_value < SOIL_MIN) {
        return ERROR_TOO_DRY;
    }
    
    // Check light levels
    if (sensor_data.light_value < (1023 - LIGHT_MAX)) {
        return ERROR_TOO_BRIGHT;
    }
    
    // Check temperature
    if (sensor_data.dht_valid && sensor_data.temperature > TEMP_MAX) {
        return ERROR_TOO_HOT;
    }
    
    // Check DHT11 sensor status
    if (!sensor_data.dht_valid) {
        return ERROR_DHT_FAIL;
    }
    
    return ERROR_NONE;
}

// =============================================================================
// LED CONTROL SYSTEM  
// =============================================================================

void update_led_and_buzzer_status(void) {
    if (current_error != ERROR_NONE) {
        // Error state: Red ON, Blue OFF
        PORTD |= (1 << LED_RED);
        PORTD &= ~(1 << LED_GREEN);
        // Beep if there’s a non‐zero error
        if (current_error != ERROR_NONE) {
            buzzer_beep_ms(); // 200 ms beep
        }
    } else {
        // Normal state: Blue ON, Red OFF
        PORTD |= (1 << LED_GREEN);
        PORTD &= ~(1 << LED_RED);
    }
}

// =============================================================================
// LCD DISPLAY FUNCTIONS
// =============================================================================

void lcd_clear_and_home(void) {
    lcd_putcmd(LCD_CLEAR);
    // Small delay for LCD processing (unavoidable for LCD commands)
    _delay_ms(2);
    lcd_putcmd(LCD_SET_CURSOR);
}

void display_error(error_code_t error) {
    const char* error_messages[] = {
        "No Error",
        "Too Bright",
        "Too Hot", 
        "Too Wet",
        "Too Dry",
        "DHT11 Fail"
    };
    
    lcd_clear_and_home();
    lcd_puts((uint8_t*)"ERROR");
    lcd_putcmd(LCD_SET_CURSOR | SECOND_ROW);
    
    if (error < sizeof(error_messages)/sizeof(error_messages[0])) {
        lcd_puts((uint8_t*)error_messages[error]);
    } else {
        lcd_puts((uint8_t*)"Unknown");
    }
}

void display_light_sensor(void) {
    uint16_t light_pct = (1023 - sensor_data.light_value) * 100UL / 1023;
    char buffer[17];
    
    lcd_clear_and_home();
    lcd_puts((uint8_t*)"Ambient Light:");
    lcd_putcmd(LCD_SET_CURSOR | SECOND_ROW);
    snprintf(buffer, sizeof(buffer), "%u%%", light_pct);
    lcd_puts((uint8_t*)buffer);
}

void display_soil_sensor(void) {
    uint16_t soil_val = sensor_data.soil_value;
    if (soil_val < SOIL_MIN) soil_val = SOIL_MIN;
    else if (soil_val > SOIL_MAX) soil_val = SOIL_MAX;
    
    uint16_t soil_pct = (soil_val - SOIL_MIN) * 100UL / (SOIL_MAX - SOIL_MIN);
    char buffer[17];
    
    lcd_clear_and_home();
    lcd_puts((uint8_t*)"Soil Humidity:");
    lcd_putcmd(LCD_SET_CURSOR | SECOND_ROW);
    snprintf(buffer, sizeof(buffer), "%u%%", soil_pct);
    lcd_puts((uint8_t*)buffer);
}

void display_dht_sensor(void) {
    char buffer[17];
    
    lcd_clear_and_home();
    if (sensor_data.dht_valid) {
        snprintf(buffer, sizeof(buffer), "Temp: %u%cC", sensor_data.temperature, 223); // 223 is the degree symbol in ASCII
        lcd_puts((uint8_t*)buffer);
        lcd_putcmd(LCD_SET_CURSOR | SECOND_ROW);
        snprintf(buffer, sizeof(buffer), "Humidity: %u%%", sensor_data.humidity);
        lcd_puts((uint8_t*)buffer);
    } else {
        lcd_puts((uint8_t*)"DHT11 Sensor:");
        lcd_putcmd(LCD_SET_CURSOR | SECOND_ROW);
        lcd_puts((uint8_t*)"Not Available");
    }
}

void update_display(void) {
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
    
    // Enable global interrupts
    sei();
    
    // System startup message
    uart_putstring("\r\n=== Plant Monitor System Started ===\r\n");
    uart_putstring("Features: Auto-cycle, Error detection, Multi-sensor\r\n");
    uart_putstring("Sensors: Light, Soil, DHT11 (Temp/Humidity)\r\n\r\n");
}

void buzzer_beep_ms(void) {
    for(uint16_t i = 0; i < 200; i++) {
        PORTC |= (1 << BUZZER);   // High
        _delay_us(10);           // 500 microseconds
        PORTC &= ~(1 << BUZZER);  // Low
        _delay_us(500);           // 500 microseconds
    }
}

// =============================================================================
// MAIN PROGRAM LOOP
// =============================================================================

int main(void) {
    // Initialize system
    system_init();
    
    // Initial sensor reading
    read_all_sensors();
    
    // Give DHT11 time to stabilize (unavoidable initial delay)
    _delay_ms(2000);
    read_dht11_sensor();
    
    // Main event loop
    while (1) {
        // Handle DHT11 reading (time-critical, has priority)
        if (read_dht_flag) {
            read_dht_flag = 0;
            read_dht11_sensor();
        }
        
        // Handle error checking
        if (check_errors_flag) {
            check_errors_flag = 0;
            read_all_sensors();  // Update sensor readings
            current_error = check_sensor_errors();
            update_led_and_buzzer_status();
            uart_print_sensor_data();  // Debug output
            
            // Re-enable auto-cycling if error is cleared
            if (current_error == ERROR_NONE && !auto_cycle_enabled) {
                auto_cycle_enabled = 1;
                cycle_timer = 0;
            }
        }
        
        // Handle display updates
        if (update_display_flag) {
            update_display_flag = 0;
            update_display();
        }
        
        // Small delay to prevent busy-waiting
        _delay_ms(10);
    }
    
    return 0;
}