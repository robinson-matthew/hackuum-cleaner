/** Vacuum cleaner hack.
**/

/** 
Notes:
 - When the photointerrupter gets blocked, the voltage goes high ~4.75 V. When
   the channel isn't blocked, the voltage is around 0.2 V.
 - Timer 0 is being used in normal mode to determine the speed of the motor.
 - Timer 1 is being used to PWM the motor.
 - Timer 2 is system time.  
 - Using pins PC2 through PC5 did not work for the Blue LED. It seems that tehy are tied up in
   the JTAG interface. This probably needs to be disabled. Follow up: yes it does! By default
   the JTAG interface is on and needs to be turned off if we want to use these pins. Quick way
   was throught the fuses. It's a high fuse. To disable it, I used: avrdude -p m1284 -c usbtiny -U hfuse:w:0xd9:m
 - BUGS: Overflow on timer hits sporadically causing the RPM value to go to 0. This appears to be the photointerrupt
         "bouncing" out of the way casuing the MCU to miss the interrupt. Holding the photinterrupt with my finger causes
         the problem to go away. We need to add in some filtering and/or some averaging to help this.   
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
#define BLUE_LED PORTC2

#define RED_LED_ON() (PORTC |= (1 << RED_LED))
#define RED_LED_OFF() (PORTC &= ~(1 << RED_LED))
#define RED_LED_TOGGLE() (PORTC ^= (1 << RED_LED))

#define BLUE_LED_ON() (PORTC |= (1 << BLUE_LED))
#define BLUE_LED_OFF() (PORTC &= ~(1 << BLUE_LED))
#define BLUE_LED_TOGGLE() (PORTC ^= (1 << BLUE_LED))

#define GREEN_LED_ON() (PORTC |= (1 << GREEN_LED))
#define GREEN_LED_OFF() (PORTC &= ~(1 << GREEN_LED))
#define GREEN_LED_TOGGLE() (PORTC ^= (1 << GREEN_LED))

// Button Interface Macros
#define BUTTON_ONE PORTD3
#define BUTTON_ONE_PIN PIND3
#define BUTTON_TWO PORTD1
#define BUTTON_TWO_PIN PIND1

// States for the button state machine.
#define NoPush 1
#define MaybePush 2
#define Pushed 3
#define MaybeNoPush 4
unsigned char PushState; // The state of the button state machine.

// Define task times up at the top so it is easy to change. Remember
// our base is about 1 ms.
#define t1 1000 // For the LED pulse.
#define t2 250 // For updating the LCD
#define t3 42 // For checking the buttons on the debounce.

uint8_t splash_message[]  = "Hackuum";
char output_string[30];

volatile uint16_t count = 0;
uint16_t previous_count = 0;

volatile uint8_t rpm_ticks = 0;
volatile uint8_t motor_moving_flag = 0; // Set this high when it is moving and low when it's not.
uint8_t previous_rpm_ticks = 0;
uint16_t frequency_hz = 0; // The frequency of the motor in Hz.
uint16_t previous_frequency_hz = 0; // The frequency of the motor in Hz.
uint8_t lcd_update_flag = 0;
uint8_t pressed_button = 0; // Global variable for which button was pressed.


// Times for tasks called by scheduler.
volatile unsigned int time1, time2, time3;
 

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
    // Turn on the interrupt to catch when an overflow occurs. Hopefully this can help us when the motor isn't moving.
    TIMSK0 |= (1 << TOIE0); 
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
    DDRD |= ((1<<PD4)|(1<<PD6)); 
    // Update! I am trying a test here. I don't want the motor to start rigth away so I'm going
    // to make the pin that is the outout to the enable pin on the H brigde IC, PD5, an input.
    // If the user presses the on button it turns it to an output. If they turn it off, it goes back
    // to being an input.

    // Above didn't work.

    // Begin with the motor braked.
    PORTD &= ~(1 << PORTD6);
    PORTD &= ~(1 << PORTD4);

}

// Inititalize Timer 2 to be a system clock with a base of 1 ms.
// This is going to set an interrupt when the compare regsiter hits
// value is OCR2A. Make sure to handle this
void system_clock_init(void)
{
    // Set the time base to 1 ms.
    TIMSK2 = (1 << OCIE2A); // Turn on timer 1 output compare A, ISR.
    OCR2A = 128; // Set the compare register to 250 time ticks. 
    TCCR2B |= (1 << CS21); // Set a prescalar of 8.
    TCCR2A = (1 << WGM01); //Turn on clear-on-match.  
}

/** We have buttons on: PD3, 
    Set the to inputs (0 on DDR) and set the internal pull ups. This will cause revser logic.
**/
void button_init(void)
{
    DDRD &= ~((1 << BUTTON_ONE)|(1 << BUTTON_TWO)); // Make an input.
    PORTD |= ((1 << BUTTON_ONE)|(1 << BUTTON_TWO)); // Set internal pull-up resistor.
}


/** Main tasks to be done in the evnt loop.
**/
// Toggle the LED so we have an idea that the system is active.
void main_task_1(void)
{
    // Recall we have reverse logic going here.
    // If the button is pressed the bit is clear.
    if (3 == pressed_button)
    {   
        RED_LED_OFF();
        GREEN_LED_OFF();
        BLUE_LED_ON();
        // Brake the motor.
        PORTD &= ~(1 << PORTD6);
        PORTD &= ~(1 << PORTD4);
    }
    else if(1 == pressed_button)
    {
        BLUE_LED_OFF();
        RED_LED_OFF();
        GREEN_LED_ON();
        // Turn the motor on
        PORTD |= (1 << PORTD4);
        PORTD &= ~(1 << PORTD6);
    }else
    {
        BLUE_LED_OFF();
        GREEN_LED_OFF();
        RED_LED_ON();
    }
}

// Update the LCD 
void main_task_2(void)
{
    if (lcd_update_flag)
    {
        lcd_check_BF_4();
        lcd_write_instruction_4f(lcd_Clear);
        // Write to the screen.
        sprintf(output_string,"%d Hz",frequency_hz); 
        //sprintf(output_string,"%d Hz",TCNT0); 
        lcd_write_string_4f(output_string);
        lcd_update_flag = 0;
    }
}

// Check the buttons for a push.
void main_task_3(void)
{
    
    switch(PushState)
    {
        case NoPush:
            if(bit_is_clear(PIND,PORTD3) || bit_is_clear(PIND,PORTD1)) 
            {
                PushState=MaybePush;    
            }
            else 
            {
                PushState=NoPush;    
            }
            break;
        case MaybePush:
        
            if(bit_is_clear(PIND,PORTD3) || bit_is_clear(PIND,PORTD1)) 
            {
                 // Note: Bruce Land's code sets the PushFlag here so I'm finding the button that was pressed here.
                 PushState = Pushed;
                 //button_push_counter++;
                 //LED_ON();
                 
                 if(bit_is_clear(PIND,PORTD3))
                 {
                     pressed_button = 3;
                     
                 }else if(bit_is_clear(PIND,PORTD1))
                 {
                     pressed_button = 1;
                
                 }else
                 {
                     pressed_button = 0x00;
                 }
            }
            else PushState=NoPush;
            
            break;
        case Pushed:
            //PORTC |= (1 << LED);
            if(bit_is_clear(PIND,PORTD3) || bit_is_clear(PIND,PORTD1)) 
            {
                PushState = Pushed;
                //button_push_counter++;
                //pressed_button = 0x00;
            }
            else 
            {
                //pressed_button = 0x00;
                PushState = MaybeNoPush;
            }
            break;
        case MaybeNoPush:
            if(bit_is_clear(PIND,PORTD3) || bit_is_clear(PIND,PORTD1))
            {
                PushState=Pushed;  
            } 
            else 
            {
                PushState=NoPush;    
            }
            
            //pressed_button = 0x00;
           // button_push_counter = 0; // Reset the counter when we go back to the no push state.
            break;
    }
    //button_push_counter += 1;
    //return(pressed_button);
}


/** ISR()s!
**/

ISR(INT0_vect)
{
    rpm_ticks = TCNT0;
    count++;
    // Reset the timer.
    TCNT0 = 0;
    motor_moving_flag = 1;
}

// System clock interrupt.
ISR(TIMER2_COMPA_vect)
{
    if (time1 > 0) --time1;
    if (time2 > 0) --time2;
    if (time3 > 0) --time3;
}

// Raise a flag when the motor hasn't moved.
ISR(TIMER0_OVF_vect)
{
    motor_moving_flag = 0;
} 

int main(void)
{
    interrupt_int0_init();
    lcd_init();
    lcd_write_string_4f(splash_message);
    // Move to the second line.
    lcd_check_BF_4();
    lcd_write_instruction_4f(lcd_SetCursor | lcd_LineTwo);
    lcd_write_string_4f("Cleaner");
    speed_timer_init();
    system_clock_init();
    button_init();
    //uint8_t duty_cycle = 0;

    // Initialize the the schedules.
    time1 = t1; 
    time2 = t2;
    time3 = t3;
    // Initialize the push state for the buttons.
    PushState = NoPush;


    // PD6 is enable, PD4 is 1A, PD5 is 2A. 
    //DDRD |= ((1 << PORTD6)|(1<<PORTD5)|(1<<PORTD4));
    //motor_pwm_init();

    // To trun the motor, one of the As must be high and the other low.
    // Set PD4 high, PD6 low.
    #if 1
    //PORTD &= ~(1 << PORTD6);
    //PORTD |= (1 << PORTD4);
    PORTD &= ~(1 << PORTD6);
    PORTD &= ~(1 << PORTD4);
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
    BLUE_LED_OFF();
    // Clear the screen.
    //lcd_check_BF_4();
    //lcd_write_instruction_4f(lcd_Clear);
    // Write to the screen.
    //sprintf(output_string,"%d Hz",frequency_hz);
    //lcd_check_BF_4();
    //lcd_write_string_4f(output_string);


    // endless loop
    while(1)
    {

        if (0 == time1){time1=t1;main_task_1();}
        if (0 == time2){time2=t2;main_task_2();}
        if (0 == time3){time3=t3;main_task_3();}
        #if 1
        OCR1A = 24;

        // Compute the frequency.
        if (motor_moving_flag)
        {
            // Handle the case when the timer has expired, meaning the motor is not moving.
            previous_rpm_ticks = rpm_ticks;
            frequency_hz = 1000000/(rpm_ticks*64);
        }else
        {
            frequency_hz = 0;
        }
        if (previous_frequency_hz != frequency_hz)
        {
            lcd_update_flag= 1;
            previous_frequency_hz = frequency_hz;
        }
        #endif
        #if 0
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
