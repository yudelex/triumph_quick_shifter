// created by yudelex 28.03.2026

// MCU is ATmega48
// PIN 23 (PC0) -> ADC input
// PIN 10 (PD6) -> PWM output

#define F_CPU 8000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>

volatile uint16_t adc_value = 0;
volatile uint16_t prev_adc_value = 0;
volatile uint8_t new_adc_ready = 0;
const uint8_t pwm_min = 5;
const uint8_t pwm_max = 250;

// =========================================================================
// Proportional mapping of a value from one range to another
// param x input value
// param in_min lower bound of input range
// param in_max upper bound of input range
// param out_min lower bound of output range
// param out_max upper bound of output range
// return mapped value

//Formula: (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
// =========================================================================
int16_t map_value(int16_t x, int16_t in_min, int16_t in_max, 
                  int16_t out_min, int16_t out_max) {
    // Check for division by zero
    if (in_min == in_max) {
        return out_min;
    }
    
    // Use int32_t for intermediate calculations to avoid overflow
    int32_t result;
    
    result = (int32_t)(x - in_min) * (out_max - out_min);
    result = result / (in_max - in_min);
    result = result + out_min;
    
    return (int16_t)result;
}

// ==============================================
// Mapping with output value limits
// ==============================================
int16_t map_value_limits(int16_t x, int16_t in_min, int16_t in_max, 
                        int16_t out_min, int16_t out_max) {
    int16_t result;
    
    // limit input value to range
    if (x <= in_min) {
        return out_min;
    }
    if (x >= in_max) {
        return out_max;
    }
    
    result = map_value(x, in_min, in_max, out_min, out_max);
    
    // Additional output value protection
    if (result < out_min) result = out_min;
    if (result > out_max) result = out_max;
    
    return result;
}

// ==============================================
// UART Initialization (9600 baud, 8N1)
// ==============================================
void uart_init(void) {
    UBRR0H = 0;
    UBRR0L = 51;  // 9600 baud @ 8MHz
    UCSR0B = (1 << TXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

// Send single character
void uart_putchar(char c) {
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = c;
}

// Send string
void uart_puts(const char *s) {
    while (*s) {
        uart_putchar(*s++);
    }
}

// Send 16-bit number as decimal
void uart_putnum(uint16_t num) {
    char buffer[6];
    char *ptr = buffer + 5;
    *ptr = '\0';
    
    if (num == 0) {
        uart_putchar('0');
        return;
    }
    
    while (num > 0) {
        *(--ptr) = '0' + (num % 10);
        num /= 10;
    }
    
    while (*ptr) {
        uart_putchar(*ptr++);
    }
}

// Send ADC value with voltage
void uart_print_adc(uint16_t adc_val) {
    uint16_t voltage_mv = ((uint32_t)adc_val * 5000) / 1023;
    
    uart_puts("ADC: ");
    uart_putnum(adc_val);
    uart_puts(" (");
    uart_putnum(voltage_mv / 1000);
    uart_puts(".");
    uart_putnum((voltage_mv % 1000) / 100);
    uart_puts("V)\r\n");
}

// ==============================================
// ADC Initialization
// ==============================================
void adc_init(void) {
    ADMUX = (1 << REFS0);  // AVcc reference
    ADMUX &= ~((1 << MUX3) | (1 << MUX2) | (1 << MUX1) | (1 << MUX0));  // ADC0
    ADCSRA = (1 << ADEN) | (1 << ADIE) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
    ADCSRA |= (1 << ADSC);
}

// ==============================================
// PWM Initialization
// ==============================================
void pwm_init(void) {
    DDRD |= (1 << PD6);
    TCCR0A = (1 << COM0A1) | (1 << WGM01) | (1 << WGM00);
    TCCR0B = (1 << CS01) | (1 << CS00);
    OCR0A = 0;
}

// ==============================================
// ADC Interrupt Handler
// ==============================================
ISR(ADC_vect) {
    adc_value = ADC;
    new_adc_ready = 1;
    if (adc_value >> 2 <= pwm_min) {
        OCR0A = (uint8_t)(adc_value = pwm_min);  // Update PWM
    }
    else OCR0A = (uint8_t)(adc_value >> 2);
    
    if (adc_value >> 2 >= pwm_max) {
        OCR0A = (uint8_t)(adc_value = pwm_max);
    }
    else OCR0A = (uint8_t)(adc_value >> 2);  // Update PWM
    ADCSRA |= (1 << ADSC);
}

// ==============================================
// Main
// ==============================================
int main(void) {
    pwm_init();
    adc_init();
    uart_init();
    
    sei();  // Enable interrupts
    
    uart_puts("\r\nADC->PWM System Ready\r\n");
    
    while (1) {
        if (new_adc_ready && prev_adc_value != adc_value) {
            new_adc_ready = 0;
            uart_print_adc(adc_value);
        }
        prev_adc_value = adc_value;
    }
}