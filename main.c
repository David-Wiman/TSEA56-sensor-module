/*
 * Hallsensor.c
 *
 * Created: 2022-04-04 10:04:44
 * Author : alvgu648
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>

volatile long int time_var = 0;
volatile long int left_hall_time = 0;
volatile long int right_hall_time = 0;
volatile long int pre_left_time_var = 0;
volatile long int pre_right_time_var = 0;
volatile int test = 0;


void init_timer() {
	
	TCCR0A = 0;
	TCCR0B = (1<<CS01) | (0<<CS00);
	TIMSK0 = (1<<TOIE0);
	sei();
}

void init_interupts() {
	EICRA = (1<<ISC00) | (1<<ISC01) | (1<<ISC10) | (1<<ISC11);
	EIMSK = (1<<INT1) | (1<<INT0);
	
	
}

 // One time_var = 0.256ms
ISR (TIMER0_OVF_vect) {
	time_var = time_var + 1;
	};
	
// Vänster	
ISR (INT0_vect) { 
	left_hall_time = time_var - pre_left_time_var;
	pre_left_time_var = time_var;
}

// Höger
ISR (INT1_vect) {
	test +=1;
	right_hall_time = time_var - pre_right_time_var;
	pre_right_time_var = time_var;
}

int main(void)
{
	init_timer();
	init_interupts();
    /* Replace with your application code */
    while (1) 
    {
    }
}

