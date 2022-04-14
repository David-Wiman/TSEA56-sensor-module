/*
 * main.c
 *
 * Created: 2022-04-04 10:04:44
 * Author : alvgu648
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>

#include "communication-module/common/avr_i2c.h"
#include "communication-module/common/i2c_common.h"

volatile uint16_t time_var = 0;
volatile uint16_t left_hall_time = 0;
volatile uint16_t right_hall_time = 0;
volatile uint16_t pre_left_time_var = 0;
volatile uint16_t pre_right_time_var = 0;

volatile int right_hall_prescaler = 0;
volatile int left_hall_prescaler = 0;
uint16_t IR_distance_mean = 0;


//volatile int count_r = 0;  // actually not really used
//volatile int count_l = 0;  // actually not really used
volatile uint16_t driven_distance_right = 0;  // In dm
volatile uint16_t driven_distance_left = 0;  // In dm

volatile uint16_t left_speed;  // In mm/s
volatile uint16_t right_speed;  //In mm/s

volatile uint16_t IR_output = 0;
volatile uint16_t IR_distance = 0;
int IR_buffer_size = 10;
int IR_buffer[10];
int IR_buffer_index = 0;



void init_hall_timer() {
	
	TCCR0A = 0;
	TCCR0B = (1<<CS01) | (0<<CS00);	//Set prescale to 8
	TIMSK0 = (1<<TOIE0); // Set interrupts on overflow
}

void init_ir_timer() {
	
	TCCR1A = 0;
	TCCR1B = (1<<WGM12) | (1<<CS01) | (1<<CS00);  // Set CTC and prescale = 64
	TIMSK1 = (1<<OCIE1A); // Set OCR1A as top-value
	OCR1A = 5000;
}

void init_interrupts_INT0_1() {
	EICRA = (1<<ISC00) | (1<<ISC01) | (1<<ISC10) | (1<<ISC11); // Set interrupts on rising edge
	EIMSK = (1<<INT1) | (1<<INT0);	//Enable interrupts on INT0 and INT1
}

void init_ADC() {
	ADMUX = (1<<REFS1) | (1<<REFS0);  // Set referense-voltage to internal 2.56V.
	ADCSRA = (1<<ADEN) | (1<<ADIE);	//Enable ADC and ADC Interrupt
}

// This function should only be used in the same block as IR_buffer is already accessed, to prevent multiple accesses at the same time
// Note that this function updates IR_distance_mean directly rather than returning it
uint16_t calc_IR_mean() {
	int sum = 0;
	for (int i=0; i<IR_buffer_size; i++) {
		sum += IR_buffer[i];
		}
	return sum/IR_buffer_size;
}

ISR (TIMER1_COMPA_vect) {
	ADCSRA |= (1<<ADSC);	// Start ADC conversion.
}

 // One time_var = 0.256ms
ISR (TIMER0_OVF_vect) {
	time_var++;
}
	
	// IR distance calculation
ISR (ADC_vect)  {
	IR_output = ADCL;  // Read bit 1-8.
	IR_output |= (ADCH<<8);  // Read bit 9-10
	if (IR_output < 700) {
		IR_distance = 400*63/IR_output;  // (1024 steps)/(2.56 V) = 400 steps/V, taking inverse to get distance
	}  else {
		IR_distance = 71 - 20*IR_output/400 ;  // See documentation for details about this calculation
	}
	if (IR_distance < 150) {
		IR_buffer[IR_buffer_index] = IR_distance;  // Put in buffer and rotate index of buffer
		IR_buffer_index++;
		if (IR_buffer_index >= IR_buffer_size) {
			IR_buffer_index = 0;
		}	
		IR_distance_mean = calc_IR_mean();
	} else {
		IR_distance_mean = 0;
	}
	
	if(IR_distance_mean<10){
		printf("hej");
    }
	I2C_pack_one(SENSOR_OBSTACLE_DISTANCE, IR_distance_mean);
}
	
// Left hall sensor
ISR (INT0_vect) {
	// count_l += 1;
	left_hall_time = time_var - pre_left_time_var;
	pre_left_time_var = time_var;
	left_speed = 98175/left_hall_time; // 1000*0.0080*pi*1000/0.256 = 98175
	left_hall_prescaler++;
	if (left_hall_prescaler >= 4) {
		driven_distance_left++;
		
		uint16_t message_names[] = {SENSOR_LEFT_SPEED, SENSOR_LEFT_DRIVING_DISTANCE};
		uint16_t messages[] = {left_speed, driven_distance_left};
		I2C_pack(message_names, messages, 2);
		left_hall_prescaler = 0;

	}

}

// Right hall sensor
ISR (INT1_vect) {
	// count_r +=1;
	right_hall_time = time_var - pre_right_time_var;
	pre_right_time_var = time_var;
	right_speed = 98175/right_hall_time; // 1000*0.0080*pi*1000/0.256 = 98175

	right_hall_prescaler++;
	if (right_hall_prescaler >= 4) {
		driven_distance_right++;
		
		uint16_t message_names[] = {SENSOR_RIGHT_SPEED, SENSOR_RIGHT_DRIVING_DISTANCE};
		uint16_t messages[] = {right_speed, driven_distance_right};
		I2C_pack(message_names, messages, 2);
		right_hall_prescaler = 0;

	}
}



int main() {
	init_hall_timer();
	init_interrupts_INT0_1();
	init_ADC();
	init_ir_timer();

	I2C_init(SENSOR_MODULE_SLAVE_ADDRESS);
	sei();
    /* Replace with your application code */
    while (1) 
    {
	}
	return 1;
}

