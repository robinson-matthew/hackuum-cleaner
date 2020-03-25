/** Vacuum cleaner hack.
**/

/** 
Notes:
 - When the photointerrupter gets blocked, the voltage goes high ~4.75 V. When
   the channel isn't blocked, the voltage is around 0.2 V.
 - Timer 0 is being used in normal mode to determine the speed of the motor.
 - Timer 1 is being used to PWM the motor.
 - TODO: Timer 2 is system time.  
 - Using pins PC2 through PC5 did not work for the Blue LED. It seems that tehy are tied up in
   the JTAG interface. This probably needs to be disabled. Follow up: yes it does! By default
   the JTAG interface is on and needs to be turned off if we want to use these pins. Quick way
   was throught the fuses. It's a high fuse. To disable it, I used: avrdude -p m1284 -c usbtiny -U hfuse:w:0xd9:m
**/

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include "lcd/lcd.h"

/** LED Interface Macros
**/

#define RED_LED PORTC0
#define GREEN_LED PORTC1
#define BLUE_LED PORTC6

#define RED_LED_ON() (PORTC |= (1 << RED_LED))
#define RED_LED_OFF() (PORTC &= ~(1 << RED_LED))

#define BLUE_LED_ON() (PORTC |= (1 << BLUE_LED))
#define BLUE_LED_OFF() (PORTC &= ~(1 << BLUE_LED))

#define GREEN_LED_ON() (PORTC |= (1 << GREEN_LED))
#define GREEN_LED_OFF() (PORTC &= ~(1 << GREEN_LED))

uint8_t splash_message[]  = "Hackuum";
char output_string[30];

volatile uint16_t count = 0;
uint16_t previous_count = 0;

volatile uint8_t rpm_ticks = 0;
uint8_t previous_rpm_ticks = 0;
uint16_t frequency_hz = 0; // The frequency of the motor in Hz. 

// Initialize int0 to happen on a rising edge.
void interrupt_int0_init(void)
{
    EIMSK |= (1 << INT0); // Turn the interrupt on.
    EICRA |= ((1 << ISC01)|(1 << ISC00)); // Rising edge interrupt only.
    sei(); // Turn on all interrupts.
}

// Inititalize Timer 0 to determine the RPMs.
void speed_timer_init(void)
{
    TCCR0B |= (1 << CS11) | (1 << CS10); // Prescale w/ 64.
    TCNT0 = 0; // Set counter to 0.
}

// Inititalize Timer 1 to PWM the H bridge enable pin to adjust the speed..
void motor_pwm_init(void)
{
    TCCR1A |= (1 << WGM11); // Fast PWM pt 1.
    TCCR1B |= (1 << WGM12); // Fast PWM pt 2.
    TCCR1B |= (1 << WGM13); // Counter max in ICR1.
    TCCR1B |= (1 << CS10);  // Set prescalar to 1.
    TCCR1A |= (1 << COM1A1); // Output on OC1A.
    ICR1 = 24; // Set the upper count.    
    // Set PD4 and PD6 as output pins to control the direction of the motor.
    // These are inputs on the motor driver IC on port A: PD4 is connected to 1A, PD5 to 2A. 
    // Set PD5, OC1A, to be an output for the PWM signal.
    DDRD |= ((1 << PD5)|(1<<PD4)|(1<<PD6)); 
}

// Inititalize Timer 2 to be a system clock.
void system_clock_init(void)
{
    
}


ISR(INT0_vect)
{
    rpm_ticks = TCNT0;
    count++;
    // Reset the timer.
    TCNT0 = 0;
}


int main(void)
{
    interrupt_int0_init();
    lcd_init();
    lcd_write_string_4f(splash_message);
    speed_timer_init();
    uint8_t duty_cycle = 0;

    // PD6 is enable, PD4 is 1A, PD5 is 2A. 
    //DDRD |= ((1 << PORTD6)|(1<<PORTD5)|(1<<PORTD4));
    //motor_pwm_init();

    // To trun the motor, one of the As must be high and the other low.
    // Set PD4 high, PD6 low.
    #if 1
    PORTD &= ~(1 << PORTD6);
    PORTD |= (1 << PORTD4);

    // Wait a software second and then try and turn it on.
    _delay_ms(1000);
    // Turn the enable on to try and turn the motor.
    //PORTD |= (1 << PORTD6);
    #endif
    // LED on PC0
    DDRC |= (1 << RED_LED)|(1<<GREEN_LED)|(1<<BLUE_LED);
    //PORTC |= (1 << PORTC0);
    GREEN_LED_OFF();
    BLUE_LED_OFF();
    RED_LED_OFF();
    BLUE_LED_ON();
    // Clear the screen.
    lcd_check_BF_4();
    lcd_write_instruction_4f(lcd_Clear);
    // Write to the screen.
    sprintf(output_string,"%d Hz",frequency_hz);
    lcd_check_BF_4();
    lcd_write_string_4f(output_string);


    // endless loop
    while(1)
    {
        //PORTC &= (1 << PORTC2);
        #if 0
        OCR1A = 24;
        // Update the LCD count.
        frequency_hz = 1000000/(rpm_ticks*64);
        if (previous_rpm_ticks != rpm_ticks)
        {
            // Clear the screen.
            lcd_check_BF_4();
            lcd_write_instruction_4f(lcd_Clear);
            // Write to the screen.
            sprintf(output_string,"%d Hz",frequency_hz);
            lcd_check_BF_4();
            lcd_write_string_4f(output_string);

            // Update the previous count.
            previous_count = count;
            previous_rpm_ticks = rpm_ticks;
        }
        #endif
        #if 0
        duty_cycle++;
        if (24 == duty_cycle)
        {
            duty_cycle = 0;
        }
        //_delay_ms(5000);
        #endif
        #if 0
        
        //PORTC ^= (1 << PORTC0);
        //_delay_ms(100);
        PORTD &= ~(1 << PORTD5);
        PORTD |= (1 << PORTD4);

        // Wait a software second and then try and turn it on.
        PORTD |= (1 << PORTD6);
        _delay_ms(2000);

        PORTD &= ~(1 << PORTD6);
        // Turn the enable on to try and turn the motor.
        _delay_ms(2000);
        PORTD &= ~(1 << PORTD4);
        PORTD |= (1 << PORTD5);
        
        PORTD |= (1 << PORTD6);
        _delay_ms(2000);
        PORTD &= ~(1 << PORTD6);
        _delay_ms(2000);
        #endif
    }

    return 0;
}
