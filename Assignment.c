#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// LCD
#include <macros.h>
#include <lcd_model.h>
#include <ascii_font.h>
#include <graphics.h>

#include "bh1750.h"


#define SET_BIT(reg, pin)           (reg) |= (1 << (pin))
#define CLEAR_BIT(reg, pin)         (reg) &= ~(1 << (pin))
#define WRITE_BIT(reg, pin, value)  (reg) = (((reg) & ~(1 << (pin))) | ((value) << (pin)))
#define BIT_VALUE(reg, pin)         (((reg) >> (pin)) & 1)
#define BIT_IS_SET(reg, pin)        (BIT_VALUE((reg),(pin))==1)

// UART Definitions
#define BAUD 9600 
#define MYUBRR 16000000UL / 16 / BAUD - 1

// Declaration of functions 
void setup_lcd(void);
void process(void);


void init_uart(unsigned int ubrr);
char uart_getchar(void);
void uart_putchar(unsigned char data);
void uart_putstring(unsigned char* s);

volatile uint32_t overflow_count = 0;
volatile uint8_t state_count = 0;
volatile uint8_t switch_state = 0;


// init for lcd screen
void new_lcd_init(uint8_t contrast) {
    // Set up the pins connected to the LCD as outputs
    SET_OUTPUT(DDRD, SCEPIN); // Chip select -- when low, tells LCD we're sending data
    SET_OUTPUT(DDRD, RSTPIN); // Chip Reset
    SET_OUTPUT(DDRD, DCPIN);  // Data / Command selector
    SET_OUTPUT(DDRD, DINPIN); // Data input to LCD
    SET_OUTPUT(DDRD, SCKPIN); // Clock input to LCD
    SET_OUTPUT(DDRB, 4);     
 
    CLEAR_BIT(PORTD, RSTPIN); // Reset LCD
    SET_BIT(PORTD, SCEPIN);   // Tell LCD we're not sending data.
    SET_BIT(PORTD, RSTPIN);   // Stop resetting LCD
 
    LCD_CMD(lcd_set_function, lcd_instr_extended);
    LCD_CMD(lcd_set_contrast, contrast);
    LCD_CMD(lcd_set_temp_coeff, 0);
    LCD_CMD(lcd_set_bias, 3);
 
    LCD_CMD(lcd_set_function, lcd_instr_basic);
    LCD_CMD(lcd_set_display_mode, lcd_display_normal);
    LCD_CMD(lcd_set_x_addr, 0);
    LCD_CMD(lcd_set_y_addr, 0);
}

// inititalises the led and the switches for the prototype

void setup_led(){
	CLEAR_BIT(DDRB, 0);
	CLEAR_BIT(DDRB, 2);
	SET_OUTPUT(DDRB, 1);
	
	bh1750_setup();
}
void setup_lcd(void) {     
    new_lcd_init(LCD_DEFAULT_CONTRAST);
    clear_screen();
    show_screen();
}

void setup_ADC(){
	// Enable ADC with a prescaler of 128
	// 0b 1000 0111
	ADCSRA = 0b10000111;
	ADMUX = 0b01000000;
}

void setup_debouncing(){
	TCCR0A = 0x00;
    TCCR0B = 0x04;
    TIMSK0 = 0x01;
    CLEAR_BIT(DDRB, 0);
    CLEAR_BIT(DDRB, 2);
}

void setup_timer(){
	init_uart(MYUBRR);
	TCCR1A = 0b00000000;
	TCCR1B = 0b00000101;
	TIMSK1 = 0b00000001;
	sei();
}

void process()
{
	// turns on first led and LCD when pressed and turn off led when secon button is pressed
	if (BIT_IS_SET(PINB, 0))
	{
		SET_BIT(PORTB, 1);
		draw_string( 0, 5, "Light", FG_COLOUR );
    	draw_string( 0, 15,"Sensor Active", FG_COLOUR);
		char temp_buf[64];
		// ADC implementation 
		ADCSRA |= (1 << ADSC);
		while(ADCSRA & (1 << ADSC)){}
		uint16_t light = ADC;
		itoa(light, (char*)temp_buf,10);
		if (light > 100)
		{
			DDRB |= (1 << PB4);
			PORTB |= (1 << PB4); 
		}else{
			DDRB |= (0 << PB4);
			PORTB |= (0 << PB4);
		}

		// light sensor reading
		char src[20];
		char dest[20];
		uint16_t light_level;
		bh1750_read(&light_level);
		dtostrf(light_level,7, 3, src);
		strcpy(dest, "L = ");
		strcat(dest, src);
		draw_string(2, 40, dest, FG_COLOUR);
		show_screen();
		
		uart_putstring((unsigned char *)dest);
		uart_putchar('l');
		uart_putchar('x');
		uart_putchar('\n');
		for (int i = 0; i < 13; ++i)
		{
			uart_putchar('\b');
		}
		
		// get the time of the timer in seconds
		char temp_buf_time[64];
		double time = (overflow_count * 65536.0 + TCNT1 * 1024.0) / 16000000.0;
		char message[100] = "time:";	
		dtostrf(time, 7,3,temp_buf_time);
		uart_putstring((unsigned char *)message);
		uart_putstring((unsigned char *)temp_buf_time);
		uart_putchar('s');	
		uart_putchar('\n');

		for (int i = 0; i < 13; ++i)
		{
			uart_putchar('\b');
		}
		} 
	if (BIT_IS_SET(PINB, 2))
	{
		CLEAR_BIT(PORTB, 1);
	}
	
	

}

int main(void)
{
	setup_ADC();			
	setup_lcd();
	setup_timer();
	setup_led();
	// main loop
	while(1)
	{
		process();	
	}
}

// initialise uart 
void init_uart(unsigned int ubrr)
{
	UBRR0H = (unsigned char)(ubrr >> 8);
	UBRR0L = (unsigned char)(ubrr);
	UCSR0B = (1 << RXEN0) | (1 << TXEN0);
	UCSR0C = (3 << UCSZ00);
}

// transmit the data from sensor
void uart_putchar(unsigned char data)
{
	while (!(UCSR0A & (1 << UDRE0)));
		UDR0 = data;	
}

// recieve data
char uart_getchar(void)
{
	while (!(UCSR0A & (1 << UDRE0)));
	return UDR0;
}

void uart_putstring(unsigned char* s){
	while(*s > 0) uart_putchar(*s++);
}

ISR(TIMER1_OVF_vect){
	overflow_count++;
}

ISR(TIMER0_OVF_vect){
    state_count = state_count << 1 ;
    state_count &= 0b00000111;
    state_count |= BIT_VALUE(PINB, 0);
    state_count |= BIT_VALUE(PINB, 2);
    if (state_count == 0b00000111){
        switch_state = 1;
    }
    if (state_count == 0){
        switch_state = 0;
    }
}